#ifndef __GIMBAL_RATE_CONFIG_H
#define __GIMBAL_RATE_CONFIG_H

#define GIMBAL_YAW_RELEASE_STOP_DEG_S 3.0f  // 停稳角速度阈值，deg/s
#define GIMBAL_YAW_RELEASE_STABLE_MS 20u    // 连续停稳时间，ms
#define GIMBAL_YAW_RELEASE_TIMEOUT_MS 300u  // 制动最长时间，ms
#define GIMBAL_YAW_RECOVER_MAX_DPS    360.0f // 受扰回正上限，deg/s
#define GIMBAL_YAW_RECOVER_DECEL_RAD_S2 25.0f // 回正减速度，rad/s²
#define GIMBAL_MEC_YAW_RATE_TAU_MS    3.0f // 速度滤波常数，ms
#define GIMBAL_MEC_YAW_BRAKE_STEP_NM  0.16f // 制动力矩斜坡，N·m/ms
#define GIMBAL_PITCH_SPEED_TAU_MS     3.0f // 速度滤波常数，ms
#define GIMBAL_PITCH_HOLD_I_ENABLE    0u // 保持积分使能，0/1
#define GIMBAL_PITCH_HOLD_I_GAIN      5.0f // 积分增益，s^-2
#define GIMBAL_PITCH_HOLD_I_MIN_DEG   0.2f // 积分误差下限，deg
#define GIMBAL_PITCH_HOLD_I_MAX_DEG   2.0f // 积分误差上限，deg
#define GIMBAL_PITCH_HOLD_I_STABLE_MS 100u // 积分稳定等待，ms
#define GIMBAL_PITCH_HOLD_I_RAMP_MS   200.0f // 积分缓入时间，ms
#define GIMBAL_PITCH_HOLD_I_MAX_DPS   5.0f // 积分修正上限，deg/s

#endif
