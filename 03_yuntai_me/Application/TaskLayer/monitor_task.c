/**
  ******************************************************************************
  * @file    monitor_task.c
  * @brief   Device and board communication heartbeat task
  ******************************************************************************
  */
#include "monitor_task.h"
#include "communicate.h"
#include "imu_sensor.h"
#include "motor.h"

void StartMonitorTask(void const *argument)
{
    (void)argument;

    for (;;)
    {
        imu_sensor.heart_beat(&imu_sensor.work_state);
        dm_motor_list_heart_beat();
        C_Board_Communicate_HeartBeat();

        osDelay(1);
    }
}
