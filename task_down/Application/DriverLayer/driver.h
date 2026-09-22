/* driver.h - 驱动层统一初始化 */
#ifndef __DRIVER_H
#define __DRIVER_H
#include "stm32h7xx_hal.h"
#include "stdbool.h"

#include "drv_can.h"
#include "drv_flash.h"
#include "drv_gpio.h"
#include "drv_spi.h"
#include "drv_tick.h"
#include "drv_tim.h"
#include "drv_uart.h"
#include "DWT.h"
void DRIVER_Init(void);

#endif

