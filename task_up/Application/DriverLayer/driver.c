/* driver.c - 驱动层统一初始化 */

#include "driver.h"
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


