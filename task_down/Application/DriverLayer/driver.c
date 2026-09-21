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
#include "board_comm_config.h"
#include "chassis_config.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

void DRIVER_Init(void)
{
#if BOARD_COMM_DEBUG
    USART5_Init();
    CAN2_Filter_Init();
#if CHASSIS_BRINGUP_ENABLE
    CAN1_Filter_Init();
#endif
#else
	USART1_Init();
	USART5_Init();
	USART8_Init();
	USART9_Init();
    USART10_Init();
	CAN1_Filter_Init();
	CAN2_Filter_Init();
	CAN3_Filter_Init();
#endif
}
