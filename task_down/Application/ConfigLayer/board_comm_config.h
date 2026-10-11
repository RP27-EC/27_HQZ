/* board_comm_config.h - 板间通信配置 */

#ifndef __BOARD_COMM_CONFIG_H
#define __BOARD_COMM_CONFIG_H

/* 板间控制链路开关 */
#define BOARD_COMM_TX_ENABLE            1u // 板间发送使能，0/1
#define BOARD_COMM_D1D2_PERIOD_MS       1u // 控制报文发送周期，ms
#define BOARD_COMM_D3_PERIOD_MS         10u // 热量报文间隔，ms
#define BOARD_COMM_D3_ENABLE             1u // 热量发送使能，0/1
#define BOARD_COMM_D4_PERIOD_MS         10u // 血量报文间隔，ms
#define BOARD_COMM_D4_ENABLE             0u // 血量发送使能，0/1
#define BOARD_HEAT_LIMIT_TIMEOUT_MS   1500u // 裁判参数超时，ms
#define BOARD_HEAT_VALUE_TIMEOUT_MS    300u // 裁判热量超时，ms
#define BOARD_COMM_D5_ENABLE            1u // 操作输入发送使能，0/1
#define BOARD_COMM_D6_ENABLE            1u // 弹速发送使能，0/1
#define BOARD_COMM_D6_PERIOD_MS        100u // 弹速心跳间隔，ms
#define BOARD_SPEED_BULLET_TYPE          1u // 首发心跳17mm，值1
#define BOARD_SPEED_SHOOTER_NUMBER       1u // 首发第一机构，值1
#define BOARD_SPEED_OFFLINE_AGE_MS   65535u // 裁判离线哨兵，ms
#define BOARD_SPEED_MAX_AGE_MS       65534u // 在线年龄饱和值，ms

/* S2 档位软件消抖，单位：1 ms 控制周期。 */
#define BOARD_LAUNCH_S2_DEBOUNCE_TICKS  15u // S2消抖连续周期数，1ms/周期

/* 升降请求开关 */
#define BOARD_LIFT_ENABLE               1u // 升降请求处理使能，0/1
#define BOARD_HOLE_EXIT_TIMEOUT_MS      5000u // 退出过洞等待上限，ms
#define BOARD_HOLE_PITCH_TARGET_RAD     0.0f // 过洞Pitch目标，rad

/* S1 下位机械模式：右摇杆目标步长与 Pitch 限位。 */
#define BOARD_MEC_YAW_FRONT_RAD         0.0f // 前向Yaw目标，rad
#define BOARD_MEC_YAW_REAR_RAD          3.14159265358979323846f // 后向Yaw目标，rad
#define BOARD_MEC_PITCH_STEP_RAD        0.002f // 满杆Pitch步长，rad/周期
#define BOARD_MEC_PITCH_MIN_RAD         (-7.5f * 0.01745329251994329577f) // Pitch机械下限，rad
#define BOARD_MEC_PITCH_MAX_RAD         (30.0f * 0.01745329251994329577f) // Pitch机械上限，rad

#endif
