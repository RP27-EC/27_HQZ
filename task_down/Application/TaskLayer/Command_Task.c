/**
  ******************************************************************************
  * @file    Command_task.c
  * @brief   指令更新任务
  *          更新整车标志位和模式
  ******************************************************************************
  */
#include "Command_Task.h"
//决策层，遥控器输入然后去做
void StartCommandTask(void const * argument)
{
	for(;;)
	{
		rc_interrupt_update(&rc_sensor);//遥控器数据解析
    keyboard_update(rc_sensor.info); // 键鼠状态检测
	
	

    osDelay(1);
	}
}  

