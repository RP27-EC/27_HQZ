/* board_comm_config.h - 板间通信配置 */

#ifndef __BOARD_COMM_CONFIG_H
#define __BOARD_COMM_CONFIG_H

/* Board-to-board bring-up switches. */
#define BOARD_COMM_DEBUG                1u
#define BOARD_COMM_TX_ENABLE            1u
#define BOARD_COMM_D1D2_PERIOD_MS       1u
#define BOARD_COMM_D3_PERIOD_MS         10u // 热量报文间隔，ms
#define BOARD_COMM_D3_ENABLE             1u // 热量发送使能，0/1
#define BOARD_COMM_D4_PERIOD_MS         10u // 血量报文间隔，ms
#define BOARD_COMM_D4_ENABLE             0u // 血量发送使能，0/1
#define BOARD_HEAT_LIMIT_TIMEOUT_MS   1500u // 裁判参数超时，ms
#define BOARD_HEAT_VALUE_TIMEOUT_MS    300u // 裁判热量超时，ms
#define BOARD_COMM_D5_ENABLE            1u

/* S2 档位软件消抖，单位：1 ms 控制周期。 */
#define BOARD_LAUNCH_S2_DEBOUNCE_TICKS  15u

/* Temporarily bypassed vehicle modules during bring-up. */
#define BOARD_LIFT_ENABLE               1u
#define BOARD_HOLE_WHEEL_REVERSE        1u
#define BOARD_HOLE_EXIT_TIMEOUT_MS      5000u
#define BOARD_HOLE_PITCH_TARGET_RAD     0.0f

#define BOARD_CAP_ENABLE                0u
#define BOARD_JUDGE_ENABLE              0u
#define BOARD_UI_ENABLE                 0u

/* S1 下位机械模式：右摇杆目标步长与 Pitch 限位。 */
#define BOARD_MEC_YAW_FRONT_RAD         0.0f
#define BOARD_MEC_YAW_REAR_RAD          3.14159265358979323846f
#define BOARD_MEC_PITCH_STEP_RAD        0.002f
#define BOARD_MEC_PITCH_MIN_RAD         (-7.5f * 0.01745329251994329577f)
#define BOARD_MEC_PITCH_MAX_RAD         (30.0f * 0.01745329251994329577f)

#endif

