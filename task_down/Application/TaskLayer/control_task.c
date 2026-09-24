/* control_task.c - 控制任务 */

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
#include "chassis_follow.h"
#include "chassis_spin.h"
#include "launch.h"
#include "rp_math.h"

#if BOARD_COMM_DEBUG
/* 调试模式下用遥控右摇杆生成云台机械角目标 */
static void Board_Debug_Gimbal_Command(void)
{
    static uint8_t mec_mode_active = 0u; /* 机械角模式已激活 */
    static float yaw_mec_target = 0.0f;  /* Yaw 机械目标角，rad */
    static float pitch_mec_target = 0.0f;/* Pitch 机械目标角，rad */
    rc_data_t *rc_info = rc_dev.info;     /* 遥控数据源 */

    if (rc_dev.work_state != DEV_ONLINE)
    {
        mec_mode_active = 0u;
        board.tx_pkt->car_pkt.car_state = 0u;
        board.tx_pkt->car_pkt.gimbal_mode = 0u;
        board.tx_pkt->gimbal_target_pkt.yaw_mec_tar = 0.0f;
        board.tx_pkt->gimbal_target_pkt.pitch_mec_tar = 0.0f;
        board.tx_pkt->gimbal_target_pkt.yaw_imu_tar = 0.0f;
        board.tx_pkt->gimbal_target_pkt.pitch_imu_tar = 0.0f;
        return;
    }

    board.tx_pkt->car_pkt.car_state = 1u;
    /* S1 下位保留控制使能，仅切换机械环 */
    /* S1 下拨：只切换云台机械环，底盘仍由底盘分支控制 */
    if (rc_info->s1.value == RC_SW_DOWN)
    {
        board.tx_pkt->car_pkt.gimbal_mode = 0u;
        /* 进入机械模式时从当前角度起调，避免跳变 */
        if (mec_mode_active == 0u)
        {
            yaw_mec_target = board.rx_meg->gimbal_meg.yaw_mec;
            pitch_mec_target = board.rx_meg->gimbal_meg.pitch_mec;
            mec_mode_active = 1u;
        }

        yaw_mec_target += BOARD_MEC_YAW_SIGN * (float)rc_info->ch0 /
                          BOARD_RC_AXIS_MAX * BOARD_MEC_YAW_STEP_RAD;
        yaw_mec_target = motor_half_cycle(yaw_mec_target, 2.0f * 3.14159265358979323846f);
        board.tx_pkt->gimbal_target_pkt.yaw_mec_tar = yaw_mec_target;

        pitch_mec_target += (float)rc_info->ch1 / BOARD_RC_AXIS_MAX *
                            BOARD_MEC_PITCH_STEP_RAD;
        if (pitch_mec_target > BOARD_MEC_PITCH_MAX_RAD)
        {
            pitch_mec_target = BOARD_MEC_PITCH_MAX_RAD;
        }
        else if (pitch_mec_target < BOARD_MEC_PITCH_MIN_RAD)
        {
            pitch_mec_target = BOARD_MEC_PITCH_MIN_RAD;
        }
        board.tx_pkt->gimbal_target_pkt.pitch_mec_tar = pitch_mec_target;
    }
    /* 机械模式退出后交回 IMU 角度环 */
    else
    {
        board.tx_pkt->car_pkt.gimbal_mode = 1u;
        mec_mode_active = 0u;
    }
}
#endif

uint8_t open_ui = 0; /* UI 首次发送延迟标志 */

/* 下板 1 kHz 控制任务，按调试阶段切换控制链路 */

void StartCtrlTask(void const *argument)
{
    (void)argument;

    for (;;)
    {
#if BOARD_COMM_DEBUG
        Board_Debug_Gimbal_Command();

#endif

    /* 第一阶段底盘调试链路 */
#if CHASSIS_BRINGUP_ENABLE
        Chassis_Follow_UpdateMode();
        Chassis_Spin_UpdateMode();
        Chassis_Input_Update();
        Chassis_Follow_Update(&chassis_input_cmd);
        Chassis_Spin_Update(&chassis_input_cmd);
        Chassis_Control_Update(&chassis_input_cmd);
        launch.work(&launch);
#elif !BOARD_COMM_DEBUG
        infantry.work(&infantry);

#if BOARD_CAP_ENABLE
        cap.tx();
#endif

#if BOARD_UI_ENABLE
    /* 第一阶段 UI 首帧跳过后再持续刷新 */
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

