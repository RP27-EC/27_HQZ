/**
 ******************************************************************************
 * @file    monitor_task.c
 * @brief   Device heartbeat monitor task.
 ******************************************************************************
 */
#include "monitor_task.h"
#include "board_protocol.h"
#include "infantry.h"
#include "rc_sensor.h"
#include "imu_sensor.h"
#include "cap.h"
#include "motor.h"
#include "judge.h"
#include "iwdg.h"
#include "board_comm_config.h"

void StartMonitorTask(void const *argument)
{
    (void)argument;

    for (;;)
    {
#if !BOARD_COMM_DEBUG
        rm_motor_list_heart_beat();
#endif
        rc_sensor.heart_beat(&rc_sensor);
#if !BOARD_COMM_DEBUG
        imu_sensor.heart_beat(&imu_sensor.work_state);
#endif
        board.heartbeat(&board);

#if BOARD_CAP_ENABLE
        cap.heartbeat(&cap);
#endif

#if BOARD_JUDGE_ENABLE
        judge.heartbeat(&judge);
#endif

#if !BOARD_COMM_DEBUG
        infantry.heart_beat(&infantry);
#endif

//      HAL_IWDG_Refresh(&hiwdg1);

        osDelay(1);
    }
}
