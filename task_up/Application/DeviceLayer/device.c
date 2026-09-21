/* device.c - 设备统一初始化 */

#include "device.h"
void DEVICE_Init(void)
{
	imu_dev.init(&imu_dev);
	rc_dev.init(&rc_dev);
	dm_motor_list_init();
	

}




