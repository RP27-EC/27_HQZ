/* observe_task.c - 监测任务 */

#include "observe_task.h"
#include "board_comm_config.h"

// 姿态解算任务

void StartUpdataTask(void const * argument)
{
	for(;;)
	{
		
		
#if !BOARD_COMM_DEBUG
		if(imu_dev.work_state.err_code == IMU_E_NONE ||
			imu_dev.work_state.err_code == IMU_E_CALI)
		{
			imu_dev.update(&imu_dev);//正常工作时才更新IMU
		}
#endif
		

		 osDelay(1);
	}
}

