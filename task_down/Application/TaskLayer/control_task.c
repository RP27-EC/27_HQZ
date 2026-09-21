/**
  ******************************************************************************
  * @file    control_task.c
  * @brief   Vehicle control task.
  ******************************************************************************
  */
#include "control_task.h"
#include "cap.h"
#include "ui.h"
#include "priority_ui.h"
#include "infantry.h"
#include "board_protocol.h"
#include "rc_sensor.h"
#include "board_comm_config.h"
#include "chassis_config.h"
#include "chassis_input.h"
#include "chassis_control.h"

uint8_t open_ui = 0;

void StartCtrlTask(void const *argument)
{
    (void)argument;

    for (;;)
    {
#if BOARD_COMM_DEBUG
        /* S1 up: arm; middle/down: disarm and keep the board link alive. */
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

#if CHASSIS_BRINGUP_ENABLE
        Chassis_Input_Update();
        Chassis_Control_Update(&chassis_input_cmd);
#endif
#else
        infantry.work(&infantry);

#if BOARD_CAP_ENABLE
        cap.tx();
#endif

#if BOARD_UI_ENABLE
        if (open_ui == 0)
        {
            open_ui = 1;
        }
        else
        {
            Ui_Info_Update();
            Ui_Send();
        }
#endif
#endif
        osDelay(1);
    }
}
