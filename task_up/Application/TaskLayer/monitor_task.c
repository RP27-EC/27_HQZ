/* monitor_task.c - 设备监控任务 */

#include "monitor_task.h"
#include "communicate.h"
#include "imu_sensor.h"
#include "motor.h"
#include "rc_sensor.h"

void StartMonitorTask(void const *argument)
{
    (void)argument;

    for (;;)
    {
        imu_dev.heart_beat(&imu_dev.work_state);
        dm_motor_list_heart_beat();
        rm_motor_list_heart_beat();
        kt_motor_list_heart_beat();
        rc_dev.heart_beat(&rc_dev);
        C_Board_Communicate_HeartBeat();

        osDelay(1);
    }
}



