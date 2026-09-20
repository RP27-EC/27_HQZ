#include "observe_task.h"
#include "board_comm_config.h"

extern osSemaphoreId_t semTaskObserveToCtrl;
extern osSemaphoreId_t semTaskCtrlToObserve;
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
		

		osSemaphoreRelease(semTaskObserveToCtrl);//释放信号量，通知控制任务可以运行
		osSemaphoreAcquire(semTaskCtrlToObserve, osWaitForever); //等待控制任务运行完毕，释放信号量
		//这里使用了信号量
		 osDelay(1);
	}
}
