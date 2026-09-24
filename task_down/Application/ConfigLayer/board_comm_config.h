/* board_comm_config.h - 板间通信配置 */

#ifndef __BOARD_COMM_CONFIG_H
#define __BOARD_COMM_CONFIG_H

/* Board-to-board bring-up switches. */
#define BOARD_COMM_DEBUG                1u
#define BOARD_COMM_TX_ENABLE            1u
#define BOARD_COMM_D1D2_PERIOD_MS       1u
#define BOARD_COMM_D3D4_PERIOD_MS       10u
#define BOARD_COMM_D3D4_ENABLE          0u
#define BOARD_COMM_D5_ENABLE            1u

/* Temporarily bypassed vehicle modules during bring-up. */
#define BOARD_CAP_ENABLE                0u
#define BOARD_JUDGE_ENABLE              0u
#define BOARD_UI_ENABLE                 0u

/* S1 下位机械模式：右摇杆目标步长与 Pitch 限位。 */
#define BOARD_MEC_YAW_STEP_RAD          0.002f
#define BOARD_MEC_YAW_SIGN              (-1.0f)
#define BOARD_MEC_PITCH_STEP_RAD        0.002f
#define BOARD_MEC_PITCH_MIN_RAD         (-7.5f * 0.01745329251994329577f)
#define BOARD_MEC_PITCH_MAX_RAD         (30.0f * 0.01745329251994329577f)

#endif

