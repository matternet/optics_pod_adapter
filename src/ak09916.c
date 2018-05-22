#include <modules/worker_thread/worker_thread.h>
#include <hal.h>

#include <uavcan.equipment.ahrs.MagneticFieldStrength.h>
#include <modules/uavcan_debug/uavcan_debug.h>

#define AK09916_I2C_ADDRESS 0x0C

#define WT hpwork_thread
WORKER_THREAD_DECLARE_EXTERN(WT)

static const I2CConfig i2cconfig = {
    STM32_TIMINGR_PRESC(11U) |
    STM32_TIMINGR_SCLDEL(3U) | STM32_TIMINGR_SDADEL(3U) |
    STM32_TIMINGR_SCLH(3U)  | STM32_TIMINGR_SCLL(9U),
    0,
    0
};

static struct worker_thread_timer_task_s ak09916_task;
static void ak09916_task_func(struct worker_thread_timer_task_s* task);

static bool ak09916_init(void) {
    for (uint8_t retries = 0; retries<50; retries++) {
        uint16_t whoami;
        if (i2cMasterTransmitTimeout(&I2CD1, AK09916_I2C_ADDRESS, (uint8_t*)"\x00", 1, (uint8_t*)&whoami, 2, LL_MS2ST(10)) != MSG_OK) {
            goto fail;
        }

        if (whoami != 0x0948) {
            goto fail;
        }

        // Reset AK09916
        if (i2cMasterTransmitTimeout(&I2CD1, AK09916_I2C_ADDRESS, (uint8_t*)"\x32\x01", 2, NULL, 0, LL_MS2ST(10)) != MSG_OK) {
            goto fail;
        }

        chThdSleep(LL_MS2ST(100));

        // Place AK09916 in continuous measurement mode 4 (100hz)
        if (i2cMasterTransmitTimeout(&I2CD1, AK09916_I2C_ADDRESS, (uint8_t*)"\x31\x08", 2, NULL, 0, LL_MS2ST(10)) != MSG_OK) {
            goto fail;
        }

        // Readback and verify
        uint8_t byte;
        if (i2cMasterTransmitTimeout(&I2CD1, AK09916_I2C_ADDRESS, (uint8_t*)"\x31", 1, &byte, 1, LL_MS2ST(10)) != MSG_OK || byte != 0x08) {
            goto fail;
        }

        return true;
fail:
        chThdSleep(LL_MS2ST(10));
        continue;
    }

    return false;
}

static bool ak09916_read_data(void) {
    uint8_t st1;
    if (i2cMasterTransmitTimeout(&I2CD1, AK09916_I2C_ADDRESS, (uint8_t*)"\x10", 1, &st1, 1, LL_MS2ST(10)) != MSG_OK) {
        return false;
    }

    if ((st1&1) == 0) {
        return false;
    }

    uint8_t data[8];

    if (i2cMasterReceiveTimeout(&I2CD1, AK09916_I2C_ADDRESS, data, 8, LL_MS2ST(10)) != MSG_OK) {
        return false;
    }

    int16_t x = (uint16_t)data[0] | (uint16_t)data[1]<<8;
    int16_t y = (uint16_t)data[2] | (uint16_t)data[3]<<8;
    int16_t z = (uint16_t)data[4] | (uint16_t)data[5]<<8;

    struct uavcan_equipment_ahrs_MagneticFieldStrength_s msg;
    msg.magnetic_field_ga[0] = x*49120.0/32752.0;
    msg.magnetic_field_ga[1] = y*49120.0/32752.0;
    msg.magnetic_field_ga[2] = z*49120.0/32752.0;
    msg.magnetic_field_covariance_len = 0;

    uavcan_broadcast(0, &uavcan_equipment_ahrs_MagneticFieldStrength_descriptor, CANARD_TRANSFER_PRIORITY_MEDIUM, &msg);

    return true;
}

RUN_AFTER(INIT_END) {

    i2cAcquireBus(&I2CD1);
    i2cStart(&I2CD1, &i2cconfig);
    bool success = ak09916_init();
    i2cReleaseBus(&I2CD1);

    if (success) {
        worker_thread_add_timer_task(&WT, &ak09916_task, ak09916_task_func, NULL, LL_US2ST(1000), true);
    }
}




static void ak09916_task_func(struct worker_thread_timer_task_s* task) {
    (void)task;
    i2cAcquireBus(&I2CD1);
    i2cStart(&I2CD1, &i2cconfig);
    ak09916_read_data();
    i2cReleaseBus(&I2CD1);
}
