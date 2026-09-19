/**
 ******************************************************************************
 * @file    monitor_task.c
 * @brief   Minimal down-board heartbeat task.
 ******************************************************************************
 */
#include "monitor_task.h"
#include "board_protocol.h"
#include "rc_sensor.h"

void StartMonitorTask(void const *argument)
{
    (void)argument;

    for (;;)
    {
        rc_sensor.heart_beat(&rc_sensor);
        board.heartbeat(&board);
        osDelay(1);
    }
}
