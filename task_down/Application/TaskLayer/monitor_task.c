/* monitor_task.c - 设备监控任务 */

#include "monitor_task.h"
#include "board_protocol.h"
#include "rc_sensor.h"
#include "motor.h"
#include "launch.h"
#include "supercap.h"

void StartMonitorTask(void const *argument)
{
    (void)argument;

    for (;;)
    {
        rm_motor_list_heart_beat();
        rc_dev.heart_beat(&rc_dev);
        launch.heart_beat(&launch);
        board.heartbeat(&board);
        SuperCap_Heartbeat();

        osDelay(1);
    }
}
