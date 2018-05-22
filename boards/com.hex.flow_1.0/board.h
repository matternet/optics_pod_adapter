#pragma once

#include <modules/platform_stm32f302x8/platform_stm32f302x8.h>

#define BOARD_PAL_LINE_I2C_1_SCL PAL_LINE(GPIOB,6) // AF4
#define BOARD_PAL_LINE_I2C_1_SDA PAL_LINE(GPIOB,7) // AF4

#define BOARD_PAL_LINE_CAN_RX PAL_LINE(GPIOA,11)
#define BOARD_PAL_LINE_CAN_TX PAL_LINE(GPIOA,12)
