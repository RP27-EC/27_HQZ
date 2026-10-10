/* gimbal_turn_config.h - 机械模式掉头参数 */

#ifndef __GIMBAL_TURN_CONFIG_H
#define __GIMBAL_TURN_CONFIG_H

#include "gimbal_init_config.h" // 掉头参数默认沿用归中配置

#define GIMBAL_TURN_ENTER_ERR_DEG             90.0f  // 旧路径切换阈值，deg
#define GIMBAL_MEC_YAW_USE_TURN_PATH          0      // 旧掉头路径开关，0/1
#define GIMBAL_TURN_YAW_RAMP_DEG_PER_MS        0.5f // 目标斜坡，deg/ms
#define GIMBAL_R_TURN_YAW_RAMP_DEG_PER_MS      1.5f // R目标斜坡，deg/ms
#define GIMBAL_R_TURN_YAW_MAX_RATE_DEG_S       1000.0f // R角速度上限，deg/s
#define GIMBAL_TURN_SPEED_LIMIT_ENABLE        GIMBAL_INIT_SPEED_LIMIT_ENABLE // 速度规划开关，0/1
#define GIMBAL_TURN_YAW_MAX_RATE_DEG_S        GIMBAL_INIT_YAW_MAX_RATE_DEG_S // 最大角速度，deg/s
#define GIMBAL_TURN_YAW_DECEL_RAD_S2          GIMBAL_INIT_YAW_DECEL_RAD_S2 // 制动减速度，rad/s^2
#define GIMBAL_TURN_YAW_TORQUE_LIMIT_NM       GIMBAL_INIT_YAW_TORQUE_LIMIT_NM // 力矩上限，N·m
#define GIMBAL_TURN_D_FILTER_ALPHA            GIMBAL_INIT_D_FILTER_ALPHA // 微分滤波系数，0~1

#define GIMBAL_TURN_YAW_OUTER_KP              GIMBAL_INIT_YAW_OUTER_KP // 位置环比例增益
#define GIMBAL_TURN_YAW_OUTER_KI              GIMBAL_INIT_YAW_OUTER_KI // 位置环积分增益
#define GIMBAL_TURN_YAW_OUTER_KD              GIMBAL_INIT_YAW_OUTER_KD // 位置环微分增益
#define GIMBAL_TURN_YAW_OUTER_INTEGRAL_MAX    GIMBAL_INIT_YAW_OUTER_INTEGRAL_MAX // 位置积分限幅
#define GIMBAL_TURN_YAW_OUTER_OUT_MAX         GIMBAL_INIT_YAW_OUTER_OUT_MAX // 位置环输出限幅

#define GIMBAL_TURN_YAW_INNER_KP              GIMBAL_INIT_YAW_INNER_KP // 速度环比例增益
#define GIMBAL_TURN_YAW_INNER_KI              GIMBAL_INIT_YAW_INNER_KI // 速度环积分增益
#define GIMBAL_TURN_YAW_INNER_KD              GIMBAL_INIT_YAW_INNER_KD // 速度环微分增益
#define GIMBAL_TURN_YAW_INNER_INTEGRAL_MAX    GIMBAL_INIT_YAW_INNER_INTEGRAL_MAX // 速度积分限幅
#define GIMBAL_TURN_YAW_INNER_OUT_MAX         GIMBAL_INIT_YAW_INNER_OUT_MAX // 速度环输出限幅

#endif
