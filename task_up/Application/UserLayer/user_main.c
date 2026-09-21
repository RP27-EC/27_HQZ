/* user_main.c - 用户任务入口 */

#include "stm32f4xx_hal.h"
#include "tim.h"

#include "user_main.h"
void user_main(void);
/**
 *	@brief	用户设备初始化
 */
void USER_Init(void)
{
	motor_all_init();
	HAL_TIM_Base_Init(&htim4);
	HAL_TIM_Base_Start_IT(&htim4);
	launcher.init();
}


/**
 *	@brief	用户应用层, 1ms 执行一次
 */
void user_main(void)
{
  // 电机心跳, 判断是否失联
	motor[FRIC_R].heartbeat(&motor[FRIC_R]);
	motor[FRIC_L].heartbeat(&motor[FRIC_L]);
	motor[DIAL].heartbeat(&motor[DIAL]);
	rc_dev.heart_beat(&rc_dev);
	 
	if (launcher.info->rc_work_state == DEV_ONLINE)
	{
		launcher.ctrl();
	}
	else
	{
		launcher.self_protect();
	}
}



/**
 *	@brief	定时器中断回调, 1ms 一次
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
 if (htim->Instance == TIM4)
 {
	 static uint16_t i = 0;
	 
	 if (++i == 60000)
	 {
		 i = 0;
	 }
	 
	 user_main();
	
 }
}

 
 
 
 
 
 
 
 
 
 
 
 
 
 
