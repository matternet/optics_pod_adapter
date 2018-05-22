#include <string.h>
#include <modules/worker_thread/worker_thread.h>
#include <hal.h>

#include <modules/uavcan_debug/uavcan_debug.h>

#include <com.matternet.equipment.pixycam.Observation.h>

#define PIXY_I2C_ADDRESS 0x54
#define PIXY_FRAME_SYNC 0xAA55
#define PIXY_MAX_BLOBS 64

#define WT hpwork_thread
WORKER_THREAD_DECLARE_EXTERN(WT)

static const I2CConfig i2cconfig = {
    STM32_TIMINGR_PRESC(11U) |
    STM32_TIMINGR_SCLDEL(3U) | STM32_TIMINGR_SDADEL(3U) |
    STM32_TIMINGR_SCLH(3U)  | STM32_TIMINGR_SCLL(9U),
    0,
    0
};

struct __attribute__((packed)) pixycam_blob_s {
    uint16_t checksum;
    uint16_t signature;
    uint16_t x_px;
    uint16_t y_px;
    uint16_t w_px;
    uint16_t h_px;
};

static struct worker_thread_timer_task_s pixycam_task;
static void pixycam_task_func(struct worker_thread_timer_task_s* task);


RUN_AFTER(INIT_END) {
    worker_thread_add_timer_task(&WT, &pixycam_task, pixycam_task_func, NULL, LL_US2ST(1000), true);
}

static void pixycam_pixel_to_1M_plane(float pix_x, float pix_y, float *ret_x, float *ret_y) {
    *ret_x = (-0.00293875727162397f*pix_x + 0.470201163459835f)/(4.43013552642296e-6f*((pix_x - 160.0f)*(pix_x - 160.0f)) +
    4.79331390531725e-6f*((pix_y - 100.0f)*(pix_y - 100.0f)) - 1.0f);
    *ret_y = (-0.003056843086277f*pix_y + 0.3056843086277f)/(4.43013552642296e-6f*((pix_x - 160.0f)*(pix_x - 160.0f)) +
    4.79331390531725e-6f*((pix_y - 100.0f)*(pix_y - 100.0f)) - 1.0f);
}

static void pixycam_publish(struct pixycam_blob_s* blob) {
    float corner1_x_px = blob->x_px-blob->w_px*0.5f;
    float corner1_y_px = blob->y_px-blob->h_px*0.5f;

    float corner2_x_px = blob->x_px+blob->w_px*0.5f;
    float corner2_y_px = blob->y_px+blob->h_px*0.5f;

    float corner1_x;
    float corner1_y;
    pixycam_pixel_to_1M_plane(corner1_x_px, corner1_y_px, &corner1_x, &corner1_y);

    float corner2_x;
    float corner2_y;
    pixycam_pixel_to_1M_plane(corner2_x_px, corner2_y_px, &corner2_x, &corner2_y);

    float center_x = (corner1_x+corner2_x)/2;
    float center_y = (corner1_y+corner2_y)/2;

    float size_x = corner2_x-corner1_x;
    float size_y = corner2_y-corner1_y;

    struct com_matternet_equipment_pixycam_Observation_s msg;
    msg.center_x = center_x;
    msg.center_y = center_y;
    msg.size_x = size_x;
    msg.size_y = size_y;

    uavcan_broadcast(0, &com_matternet_equipment_pixycam_Observation_descriptor, CANARD_TRANSFER_PRIORITY_MEDIUM, &msg);
}

static bool pixycam_search_frame_sync(void) {
    union {
        uint16_t words[2];
        uint8_t bytes[4];
    } buf;

    if (i2cMasterReceiveTimeout(&I2CD1, PIXY_I2C_ADDRESS, &buf.bytes[0], 4, LL_MS2ST(10)) != MSG_OK) {
        return false;
    }

    if (buf.words[0] == PIXY_FRAME_SYNC && buf.words[1] == PIXY_FRAME_SYNC) {
        return true;
    }

    for (uint8_t i=0; i<40; i++) {
        memmove(&buf.bytes[0], &buf.bytes[1], 3);

        if (i2cMasterReceiveTimeout(&I2CD1, PIXY_I2C_ADDRESS, &buf.bytes[3], 1, LL_MS2ST(10)) != MSG_OK) {
            return false;
        }

        if (buf.words[0] == PIXY_FRAME_SYNC && buf.words[1] == PIXY_FRAME_SYNC) {
            return true;
        }
    }

    return false;
}

static void pixycam_task_func(struct worker_thread_timer_task_s* task) {
    (void)task;
    i2cAcquireBus(&I2CD1);
    i2cStart(&I2CD1, &i2cconfig);

    if (!pixycam_search_frame_sync()) {
        i2cReleaseBus(&I2CD1);
        return;
    }

    uint32_t count = 0;

    {
        uint16_t word;
        do {
            // retrieve blob
            struct pixycam_blob_s blob;
            if (i2cMasterReceiveTimeout(&I2CD1, PIXY_I2C_ADDRESS, (uint8_t*)&blob, sizeof(blob), LL_MS2ST(10)) != MSG_OK) {
                i2cReleaseBus(&I2CD1);
                return;
            }

            if (blob.signature + blob.x_px + blob.y_px + blob.w_px + blob.h_px != blob.checksum) {
                i2cReleaseBus(&I2CD1);
                return;
            }

            if (count == 0) {
                pixycam_publish(&blob);
            }

            count++;


            if (i2cMasterReceiveTimeout(&I2CD1, PIXY_I2C_ADDRESS, (uint8_t*)&word, 2, LL_MS2ST(10)) != MSG_OK) {
                i2cReleaseBus(&I2CD1);
                return;
            }
        } while(word == PIXY_FRAME_SYNC);
    }

    i2cReleaseBus(&I2CD1);
}
