/* gimbal_turn_config.h - 机械模式掉头参数 */

#ifndef __GIMBAL_TURN_CONFIG_H
#define __GIMBAL_TURN_CONFIG_H

#include "gimbal_init_config.h" // 掉头参数默认取归中值

#define GIMBAL_TURN_ENTER_ERR_DEG             90.0f  // 可选掉头串级阈值，deg
#define GIMBAL_MEC_YAW_USE_TURN_PATH          0      // 掉头串级路径开关，0/1
#define GIMBAL_TURN_YAW_RAMP_DEG_PER_MS        0.5f // 目标斜坡，deg/ms
#define GIMBAL_R_TURN_YAW_RAMP_DEG_PER_MS      1.5f // R目标斜坡，deg/ms
#define GIMBAL_R_TURN_YAW_MAX_RATE_DEG_S       1000.0f // R角速度上限，deg/s
#define GIMBAL_TURN_SPEED_LIMIT_ENABLE        GIMBAL_INIT_SPEED_LIMIT_ENABLE // 速度规划开关，0/1
#define GIMBAL_TURN_YAW_MAX_RATE_DEG_S        GIMBAL_INIT_YAW_MAX_RATE_DEG_S // 最大角速度，deg/s
#define GIMBAL_TURN_YAW_DECEL_RAD_S2          GIMBAL_INIT_YAW_DECEL_RAD_S2 // 制动减速度，rad/s^2
#define GIMBAL_TURN_YAW_TORQUE_LIMIT_NM       GIMBAL_INIT_YAW_TORQUE_LIMIT_NM // 力矩上限，N·m
#define GIMBAL_TURN_D_FILTER_ALPHA            GIMBAL_INIT_D_FILTER_ALPHA // 微分滤波系数，0~1

#define GIMBAL_TURN_YAW_OUTER_KP              GIMBAL_INIT_YAW_OUTER_KP // 位置比例增益，1/s
#define GIMBAL_TURN_YAW_OUTER_KI              GIMBAL_INIT_YAW_OUTER_KI // 位置积分增益，1/s/周期
#define GIMBAL_TURN_YAW_OUTER_KD              GIMBAL_INIT_YAW_OUTER_KD // 位置微分增益，1/s·周期
#define GIMBAL_TURN_YAW_OUTER_INTEGRAL_MAX    GIMBAL_INIT_YAW_OUTER_INTEGRAL_MAX // 位置积分限幅，deg·周期
#define GIMBAL_TURN_YAW_OUTER_OUT_MAX         GIMBAL_INIT_YAW_OUTER_OUT_MAX // 位置环速度限幅，deg/s

#define GIMBAL_TURN_YAW_INNER_KP              GIMBAL_INIT_YAW_INNER_KP // 速度比例增益，N·m/(deg/s)
#define GIMBAL_TURN_YAW_INNER_KI              GIMBAL_INIT_YAW_INNER_KI // 速度积分增益，按周期累加
#define GIMBAL_TURN_YAW_INNER_KD              GIMBAL_INIT_YAW_INNER_KD // 速度微分增益，按周期差分
#define GIMBAL_TURN_YAW_INNER_INTEGRAL_MAX    GIMBAL_INIT_YAW_INNER_INTEGRAL_MAX // 速度积分限幅，deg/s·周期
#define GIMBAL_TURN_YAW_INNER_OUT_MAX         GIMBAL_INIT_YAW_INNER_OUT_MAX // 速度环力矩限幅，N·m

#endif
