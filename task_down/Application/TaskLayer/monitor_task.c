/* monitor_task.c - 设备监控任务 */

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
#include "chassis_config.h"
#include "launch.h"

void StartMonitorTask(void const *argument)
{
    (void)argument;

    for (;;)
    {
#if (!BOARD_COMM_DEBUG) || CHASSIS_BRINGUP_ENABLE
        rm_motor_list_heart_beat();
#endif
        rc_dev.heart_beat(&rc_dev);
        launch.heart_beat(&launch);
#if !BOARD_COMM_DEBUG
        imu_dev.heart_beat(&imu_dev.work_state);
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

