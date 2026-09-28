/* board_comm_config.h - 板间通信配置 */

#ifndef __BOARD_COMM_CONFIG_H
#define __BOARD_COMM_CONFIG_H

#define BOARD_COMM_TX_ENABLE            1u
#define BOARD_COMM_D1D2_PERIOD_MS       1u
#define BOARD_COMM_D5_ENABLE            1u

/* S2 档位软件消抖，单位：1 ms 控制周期。 */
#define BOARD_LAUNCH_S2_DEBOUNCE_TICKS  15u

#define BOARD_LIFT_ENABLE               1u
#define BOARD_HOLE_WHEEL_REVERSE        1u
#define BOARD_HOLE_EXIT_TIMEOUT_MS      5000u
#define BOARD_HOLE_PITCH_TARGET_RAD     0.0f

/* S1 下位机械模式：右摇杆目标步长与 Pitch 限位。 */
#define BOARD_MEC_YAW_FRONT_RAD         0.0f
#define BOARD_MEC_PITCH_STEP_RAD        0.002f
#define BOARD_MEC_PITCH_MIN_RAD         (-7.5f * 0.01745329251994329577f)
#define BOARD_MEC_PITCH_MAX_RAD         (30.0f * 0.01745329251994329577f)

#endif
