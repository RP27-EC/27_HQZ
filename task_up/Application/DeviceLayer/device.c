/* device.c - 设备统一初始化 */

#include "device.h"
#include "launcher.h"
void DEVICE_Init(void)
{
	imu_dev.init(&imu_dev);
	rc_dev.init(&rc_dev);
	rm_motor_list_init();
	kt_motor_list_init();
	dm_motor_list_init();
	Launcher_Init();
	

}




