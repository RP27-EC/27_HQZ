/* drv_tick.h - 系统时基与延时 */

#ifndef __TICK_DRV_H
#define __TICK_DRV_H
#include "stm32f4xx_hal.h"
uint32_t micros(void);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

#endif


