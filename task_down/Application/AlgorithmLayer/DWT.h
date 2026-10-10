/* DWT.h - DWT 微秒计数 */

#ifndef _DWT_H
#define _DWT_H
#include "stm32h7xx_hal.h"
void DWT_Init(void);
uint32_t DWT_GetCycleCount(void);

#endif

