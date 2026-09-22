/* drv_tim.h - 定时器驱动 */
#ifndef __DRV_TIM_H
#define __DRV_TIM_H
#include "stm32h7xx_hal.h"
void TIM1_Init(void);
void TIM4_Init(void);
void TIM3_Set_PWM(uint16_t compare);
/* Servo functions */
#endif

