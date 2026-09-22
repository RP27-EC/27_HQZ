/* user_main.c - 用户任务入口 */
#include "stm32f4xx_hal.h"
#include "tim.h"

#include "user_main.h"
void user_main(void);
/* 用户设备初始化 */
void USER_Init(void)
{
	motor_all_init();
	HAL_TIM_Base_Init(&htim4);
	HAL_TIM_Base_Start_IT(&htim4);
	launcher.init();
}


/* 用户应用层 */
void user_main(void)
{

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



/* 定时器中断回调 */
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

 
 
 
 
 
 
 
 
 
 
 
 
 
 
///*	

//*/
//int16_t send_buff[4];
//float tar;
//void StartControlTask(void const * argument)
//{
//	

//	motor[GIMB_Y].init(&motor[GIMB_Y]);

//	motor[GIMB_Y].pid_init(&motor[GIMB_Y].pid.speed,gimb_y_speed_pid_param);
//	
//  for(;;)
//  {

//		motor[GIMB_Y].heartbeat(&motor[GIMB_Y]);


//		send_buff[motor[GIMB_Y].id.buff_p] = motor[GIMB_Y].c_speed(&motor[GIMB_Y],tar);



//		CAN1_Send_With_int16_to_uint8(motor[GIMB_Y].id.tx_id,send_buff);
////		
////		
//		
//		
//    osDelay(1);
//  }

//}






