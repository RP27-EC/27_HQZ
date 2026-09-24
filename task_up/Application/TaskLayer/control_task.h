/* control_task.h - 控制任务 */

#ifndef __CONTROL_TASK
#define __CONTROL_TASK

#include "cmsis_os.h"
#include "main.h"
#include "device.h"

void StartControlTask(void const * argument);

/* 上板 IMU 调试快照 */
typedef struct
{
	/* 原始数据 */
	float acc_x, acc_y, acc_z;  /* 机体系加速度，m/s^2 */
	float gyro_x, gyro_y, gyro_z; /* 机体系角速度，rad/s */

	/* 解算后的姿态角 */
	float yaw, pitch, roll;     /* 欧拉角，deg */

	/* 角速度 */
	float rate_yaw, rate_pitch, rate_roll; /* 瞬时角速度，deg/s */
	float ave_rate_yaw, ave_rate_pitch, ave_rate_roll; /* 滤波角速度，deg/s */

	/* 世界坐标系加速度 */
	float accx, accy, accz;     /* m/s^2 */

	/* 温度与运行状态 */
	float temperature;          /* 摄氏度 */
	uint8_t dev_state;          /* IMU 设备状态 */
	uint8_t cali_end;           /* 1 = 零偏标定完成 */
	uint8_t err_code;           /* IMU 错误码 */
} imu_debug_t;


#endif

