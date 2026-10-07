/* gimbal_init_config.h - 云台归中参数 */

#ifndef __GIMBAL_INIT_CONFIG_H
#define __GIMBAL_INIT_CONFIG_H

#define GIMBAL_INIT_YAW_HOME_DEG              0.0f   // Yaw归中目标，deg
#define GIMBAL_INIT_PITCH_HOME_DEG            0.0f   // Pitch归中目标，deg
#define GIMBAL_INIT_SPEED_LIMIT_ENABLE        0u     // 速度规划开关，0/1
#define GIMBAL_INIT_YAW_MAX_RATE_DEG_S        200.0f // Yaw角速度上限，deg/s
#define GIMBAL_INIT_PITCH_MAX_RATE_DEG_S      45.0f  // Pitch角速度上限，deg/s
#define GIMBAL_INIT_YAW_DECEL_RAD_S2          4.0f   // Yaw制动减速度，rad/s^2
#define GIMBAL_INIT_PITCH_DECEL_RAD_S2        3.0f   // Pitch制动减速度，rad/s^2
#define GIMBAL_INIT_YAW_RAMP_DEG_PER_MS       0.25f  // Yaw目标斜坡，deg/ms
#define GIMBAL_INIT_PITCH_RAMP_DEG_PER_MS     0.25f  // Pitch目标斜坡，deg/ms
#define GIMBAL_INIT_TIMEOUT_MS                6000u  // 归中超时，ms
#define GIMBAL_INIT_STABLE_MS                 30u    // 到位稳定时长，ms
#define GIMBAL_INIT_YAW_TOL_DEG               2.0f   // Yaw到位误差，deg
#define GIMBAL_INIT_PITCH_TOL_DEG             2.0f   // Pitch到位误差，deg
#define GIMBAL_INIT_YAW_SPEED_TOL_RAD_S       0.5f   // Yaw到位速度，rad/s
#define GIMBAL_INIT_PITCH_SPEED_TOL_RAD_S     0.5f   // Pitch到位速度，rad/s
#define GIMBAL_INIT_YAW_TORQUE_LIMIT_NM       6.0f   // Yaw力矩上限，N·m
#define GIMBAL_INIT_PITCH_TORQUE_LIMIT_NM     6.0f   // Pitch力矩上限，N·m
#define GIMBAL_INIT_D_FILTER_ALPHA            0.0f   // 微分滤波系数，0~1
#define GIMBAL_INIT_YAW_OUTER_KP              0.1f   // Yaw位置环比例增益
#define GIMBAL_INIT_YAW_OUTER_KI              0.0f   // Yaw位置环积分增益
#define GIMBAL_INIT_YAW_OUTER_KD              1.0f   // Yaw位置环微分增益
#define GIMBAL_INIT_YAW_OUTER_INTEGRAL_MAX    0.0f   // Yaw位置积分限幅
#define GIMBAL_INIT_YAW_OUTER_OUT_MAX         500.0f // Yaw位置环输出限幅
#define GIMBAL_INIT_YAW_INNER_KP              1.5f   // Yaw速度环比例增益
#define GIMBAL_INIT_YAW_INNER_KI              0.0f   // Yaw速度环积分增益
#define GIMBAL_INIT_YAW_INNER_KD              0.2f   // Yaw速度环微分增益
#define GIMBAL_INIT_YAW_INNER_INTEGRAL_MAX    0.0f   // Yaw速度积分限幅
#define GIMBAL_INIT_YAW_INNER_OUT_MAX         100.0f // Yaw速度环输出限幅
#define GIMBAL_INIT_PITCH_OUTER_KP            1.6f   // Pitch位置环比例增益
#define GIMBAL_INIT_PITCH_OUTER_KI            0.0f   // Pitch位置环积分增益
#define GIMBAL_INIT_PITCH_OUTER_KD            0.0f   // Pitch位置环微分增益
#define GIMBAL_INIT_PITCH_OUTER_INTEGRAL_MAX  500.0f // Pitch位置积分限幅
#define GIMBAL_INIT_PITCH_OUTER_OUT_MAX       10.0f  // Pitch位置环输出限幅
#define GIMBAL_INIT_PITCH_INNER_KP            1.2f   // Pitch速度环比例增益
#define GIMBAL_INIT_PITCH_INNER_KI            0.0f   // Pitch速度环积分增益
#define GIMBAL_INIT_PITCH_INNER_KD            0.0f   // Pitch速度环微分增益
#define GIMBAL_INIT_PITCH_INNER_INTEGRAL_MAX  0.0f   // Pitch速度积分限幅
#define GIMBAL_INIT_PITCH_INNER_OUT_MAX       10.0f  // Pitch速度环输出限幅
#define GIMBAL_INIT_PITCH_GRAVITY_ENABLE      1u     // Pitch重力补偿，0/1
#define GIMBAL_INIT_PITCH_GRAVITY_K_NM        1.1f   // 重力补偿幅值，N·m
#define GIMBAL_INIT_PITCH_GRAVITY_B_NM        0.0f   // 重力补偿偏置，N·m
#define GIMBAL_INIT_PITCH_GRAVITY_SIGN        1.0f   // 重力补偿方向，仅±1
#define GIMBAL_INIT_PITCH_GRAVITY_MIDDLE_DEG  0.0f   // 重力补偿中点，deg

#endif
