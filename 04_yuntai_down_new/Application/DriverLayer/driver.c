/**
 ******************************************************************************
 * @file        driver.c
 * @brief       Minimal driver initialization for the gimbal down board.
 ******************************************************************************
 */

#include "driver.h"

void DRIVER_Init(void)
{
    USART5_Init();
    CAN2_Filter_Init();
}
