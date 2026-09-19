/**
  ******************************************************************************
  * @file    control_task.c
  * @brief   Remote-control board link task.
  ******************************************************************************
  */
#include "control_task.h"
#include "board_protocol.h"
#include "rc_sensor.h"

void StartCtrlTask(void const *argument)
{
    (void)argument;

    for (;;)
    {
        if ((rc_sensor.work_state == DEV_ONLINE) &&
            (rc_sensor.info->s1.value == RC_SW_UP))
        {
            board.tx_pkt->car_pkt.car_state = 1u;
            board.tx_pkt->car_pkt.gimbal_mode = 1u;
        }
        else
        {
            board.tx_pkt->car_pkt.car_state = 0u;
            board.tx_pkt->car_pkt.gimbal_mode = 0u;
            board.tx_pkt->gimbal_target_pkt.yaw_mec_tar = 0.0f;
            board.tx_pkt->gimbal_target_pkt.pitch_mec_tar = 0.0f;
            board.tx_pkt->gimbal_target_pkt.yaw_imu_tar = 0.0f;
            board.tx_pkt->gimbal_target_pkt.pitch_imu_tar = 0.0f;
        }

        osDelay(1);
    }
}
