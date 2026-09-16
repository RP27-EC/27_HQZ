/**
 ******************************************************************************
 * @file    monitor_task.c
 * @brief   监控任务
 *     
 *     
 ******************************************************************************
 */
#include "monitor_task.h"

void StartMonitorTask(void const *argument)
{
	for (;;)
	{

		/* 检查陀螺仪心跳 */
		imu_sensor.heart_beat(&imu_sensor.work_state);

		/* 检查 DM 电机与遥控器心跳 */
		dm_motor_list_heart_beat();
		rc_sensor.heart_beat(&rc_sensor);

		if (imu_sensor.work_state.dev_state == DEV_OFFLINE)
		{
		}

		osDelay(1);
	}
}
