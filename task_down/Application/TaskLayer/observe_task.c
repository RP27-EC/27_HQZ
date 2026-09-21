#include "observe_task.h"
#include "board_comm_config.h"

// 姿态解算任务

void StartUpdataTask(void const * argument)
{
	for(;;)
	{
		
		
#if !BOARD_COMM_DEBUG
		if(imu_sensor.work_state.err_code == IMU_NONE_ERR ||
			imu_sensor.work_state.err_code == IMU_DATA_CALI)
		{
			imu_sensor.update(&imu_sensor);//正常工作时才更新IMU
		}
#endif
		

		 osDelay(1);
	}
}
