/**
 ******************************************************************************
 * @file        driver.c
 * @author      RobotPilots@2020
 * @brief       Drivers' Manager.
 ******************************************************************************
 * @attention   
 * 
 * Copyright 2020 RobotPilots
 *  
 * @Version     V1.0
 * @date        9-September-2020
 ****************************************************************************
 */
 
/* Includes ------------------------------------------------------------------*/
#include "driver.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

void DRIVER_Init(void)
{
	CAN_Filter_Init();
	/* Legacy/non-used peripherals for the current gimbal board. */
	/* USART1: external XRobot IMU is not in the active build. */
	/* USART3: local DBUS RC is disabled; manual input comes from D5. */
	/* USART6: current RX handler is empty. */
	/* TIM1: no active PWM consumer found. */
	/* TIM4: interrupt callback is empty. */
	// USART1_Init();
	// USART3_Init();
	// USART6_Init();
	// TIM1_Init();
	// TIM4_Init();
}
