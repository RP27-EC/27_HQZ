#ifndef __GIMBAL_MEC_CONFIG_H
#define __GIMBAL_MEC_CONFIG_H

#define GIMBAL_MEC_YAW_MAX_RATE_DEG_S      1000.0f // 回正速度上限，deg/s
#define GIMBAL_MEC_YAW_NEAR_RATE_KP        20.0f   // 回正近区增益，s^-1
#define GIMBAL_MEC_YAW_HOLD_ENTER_DEG      0.3f    // 保持进入角误差，deg
#define GIMBAL_MEC_YAW_HOLD_EXIT_DEG       0.5f    // 保持退出角误差，deg
#define GIMBAL_MEC_YAW_HOLD_ENTER_DPS      3.0f    // 保持进入相对速度，deg/s
#define GIMBAL_MEC_YAW_HOLD_EXIT_DPS       6.0f    // 保持退出相对速度，deg/s
#define GIMBAL_MEC_YAW_HOLD_STABLE_MS      50u     // 保持进入稳定时间，ms
#define GIMBAL_MEC_HOLD_RATE_DEADBAND_DPS  0.5f    // 保持速度误差软死区，deg/s

#endif
