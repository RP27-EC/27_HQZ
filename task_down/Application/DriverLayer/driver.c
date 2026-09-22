/* driver.c - 驱动层统一初始化 */
#include "driver.h"
#include "board_comm_config.h"
#include "chassis_config.h"
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

