#ifndef __CONTROL_TASK
#define __CONTROL_TASK

#include "cmsis_os.h"
#include "main.h"
#include "device.h"

void StartControlTask(void const * argument);

typedef struct
{
	/* 原始数据 */
	float acc_x, acc_y, acc_z;				
	float gyro_x, gyro_y, gyro_z;			

	/* 解算后的姿态角 */
	float yaw, pitch, roll;					

	/* 角速度 */
	float rate_yaw, rate_pitch, rate_roll;			
	float ave_rate_yaw, ave_rate_pitch, ave_rate_roll;	

	/* 世界坐标系加速度 */
	float accx, accy, accz;					

	/* 温度与运行状态 */
	float temperature;					/* 摄氏度 */
	uint8_t dev_state;				
	uint8_t cali_end;					/* 1 = 零偏标定完成 */
	uint8_t err_code;				
} imu_debug_t;


#endif
