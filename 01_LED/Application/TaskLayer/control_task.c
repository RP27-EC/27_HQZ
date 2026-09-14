/**
  ******************************************************************************
  * @file    control_task.c
  * @brief   陀螺仪任务
  ******************************************************************************
  */
#include "control_task.h"
#include "imu_sensor.h"


volatile imu_debug_t imu_dbg;

/**
  * @brief  把 imu_sensor 里的数据搬到 imu_dbg
  */
static void imu_debug_update(void)
{
	imu_info_t *info = imu_sensor.info;

	imu_dbg.acc_x = info->raw_info.acc_x;
	imu_dbg.acc_y = info->raw_info.acc_y;
	imu_dbg.acc_z = info->raw_info.acc_z;

	imu_dbg.gyro_x = info->raw_info.gyro_x;
	imu_dbg.gyro_y = info->raw_info.gyro_y;
	imu_dbg.gyro_z = info->raw_info.gyro_z;

	imu_dbg.yaw   = info->base_info.yaw;
	imu_dbg.pitch = info->base_info.pitch;
	imu_dbg.roll  = info->base_info.roll;

	imu_dbg.rate_yaw   = info->base_info.rate_yaw;
	imu_dbg.rate_pitch = info->base_info.rate_pitch;
	imu_dbg.rate_roll  = info->base_info.rate_roll;

	imu_dbg.ave_rate_yaw   = info->base_info.ave_rate_yaw;
	imu_dbg.ave_rate_pitch = info->base_info.ave_rate_pitch;
	imu_dbg.ave_rate_roll  = info->base_info.ave_rate_roll;

	imu_dbg.accx = info->base_info.accx;
	imu_dbg.accy = info->base_info.accy;
	imu_dbg.accz = info->base_info.accz;

	imu_dbg.temperature = info->base_info.temperature;

	imu_dbg.dev_state = (uint8_t)imu_sensor.work_state.dev_state;
	imu_dbg.cali_end  = imu_sensor.work_state.cali_end;
	imu_dbg.err_code  = (uint8_t)imu_sensor.work_state.err_code;

}

void StartControlTask(void const * argument)
{
	for(;;)
	{
		if ((imu_sensor.work_state.err_code == IMU_NONE_ERR) || \
				(imu_sensor.work_state.err_code == IMU_DATA_CALI))
		{
			imu_sensor.update(&imu_sensor);
		}

		imu_debug_update();

		osDelay(1);	
	}
}
