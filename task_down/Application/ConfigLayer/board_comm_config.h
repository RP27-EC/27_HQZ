/* board_comm_config.h - 板间通信配置 */

#ifndef __BOARD_COMM_CONFIG_H
#define __BOARD_COMM_CONFIG_H

#define BOARD_COMM_TX_ENABLE            1u  // 板间发送总开关
#define BOARD_COMM_D1D2_PERIOD_MS       1u  // D1/D2 发送周期
#define BOARD_COMM_D5_ENABLE            1u  // D5 遥控指令包开关

/* S2 档位软件消抖，单位：1 ms 控制周期。 */
#define BOARD_LAUNCH_S2_DEBOUNCE_TICKS  15u  // S2 档位消抖节拍

#define BOARD_LIFT_ENABLE               1u  // 升降模块开关
#define BOARD_HOLE_WHEEL_REVERSE        1u  // 打符轮反转
#define BOARD_HOLE_EXIT_TIMEOUT_MS      5000u  // 打符退出超时
#define BOARD_HOLE_PITCH_TARGET_RAD     0.0f  // 打符 Pitch 目标角

/* S1 下位机械模式：右摇杆目标步长与 Pitch 限位。 */
#define BOARD_MEC_YAW_FRONT_RAD         0.0f  // 机械模式 Yaw 前向角
#define BOARD_MEC_PITCH_STEP_RAD        0.002f  // 机械模式 Pitch 步进
#define BOARD_MEC_PITCH_MIN_RAD         (-7.5f * 0.01745329251994329577f)
#define BOARD_MEC_PITCH_MAX_RAD         (30.0f * 0.01745329251994329577f)

#endif
