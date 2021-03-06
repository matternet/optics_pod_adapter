#include <hal.h>

void boardInit(void) {
    // SPI3
    palSetLineMode(BOARD_PAL_LINE_SPI_3_SCK, PAL_MODE_ALTERNATE(6) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUPDR_PULLDOWN);
    palSetLineMode(BOARD_PAL_LINE_SPI_3_MISO, PAL_MODE_ALTERNATE(6) | PAL_STM32_OSPEED_HIGHEST);
    palSetLineMode(BOARD_PAL_LINE_SPI_3_MOSI, PAL_MODE_ALTERNATE(6) | PAL_STM32_OSPEED_HIGHEST);

    // I2C2
    palSetLineMode(BOARD_PAL_LINE_I2C_2_SCL, PAL_MODE_ALTERNATE(4) | PAL_STM32_OSPEED_HIGHEST);
    palSetLineMode(BOARD_PAL_LINE_I2C_2_SDA, PAL_MODE_ALTERNATE(4) | PAL_STM32_OSPEED_HIGHEST);

    // outputs
    palSetLineMode(BOARD_PAL_LINE_SPI_CS_ICM, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
    palSetLineMode(BOARD_PAL_LINE_SPI_CS_FLOW, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
    palSetLineMode(BOARD_PAL_LINE_LED_TRIG, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
    palSetLineMode(BOARD_PAL_LINE_ICM_FSYNC, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
    palSetLineMode(BOARD_PAL_LINE_AMBER_LED, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
    palClearLine(BOARD_PAL_LINE_AMBER_LED);

    // inputs
    palSetLineMode(BOARD_PAL_LINE_FLOW_INT, PAL_STM32_MODE_INPUT | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUPDR_PULLDOWN);
    palSetLineMode(BOARD_PAL_LINE_ICM_IRQ, PAL_STM32_MODE_INPUT | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUPDR_PULLDOWN);

    // CAN
    palSetLineMode(BOARD_PAL_LINE_CAN_RX, PAL_MODE_ALTERNATE(9) | PAL_STM32_OSPEED_HIGHEST);
    palSetLineMode(BOARD_PAL_LINE_CAN_TX, PAL_MODE_ALTERNATE(9) | PAL_STM32_OSPEED_HIGHEST);
}
