/* power_limit.h - 底盘功率限制 */
#ifndef __POWER_LIMIT_H
#define __POWER_LIMIT_H

#include <stdint.h>
#include "judge.h"
#include "motor.h"
#include "power_limit_config.h"

/* P=k0+k1*I+k2*w+k3*I*w+k4*I²+k5*w² */
/* I为C620计数，w为转子rpm */
typedef struct
{
    float k[6]; // 系数量纲使各项为W
} power_coeff_t;

/* 非法输入的估算值为NaN */
typedef struct
{
    float target_power; // 生效预算，W
    float requested_power; // 限幅前正功率估算，W
    float estimate_power; // 下发正功率估算，W
    float buffer_energy; // 在线有效缓冲，0~60J
    float scale; // 公共力矩比例，0~1
    float wheel_predict[WHEEL_CNT]; // 下发单轮功率估算，W
    float wheel_raw[WHEEL_CNT]; // 下发电流计数，±14745
    float wheel_torque_limited[WHEEL_CNT]; // 下发输出轴力矩，N·m
    int16_t cap_power_raw; // 超电功率计数，量纲未知
    uint8_t judge_online; // 裁判心跳在线，0/1
    uint8_t fallback_used; // 本轮固定回退，0/1
    uint8_t limited; // 本轮力矩缩小，0/1
    uint8_t target_unreachable; // 预算不可达或输入非法，0/1
    uint8_t cap_online; // 超电反馈在线，0/1
} power_limit_state_t;

extern power_limit_state_t power_limit_state;

void Power_Limit_Init(void);
float Power_Limit_GetTarget(dev_work_state_t judge_status, uint8_t power_data_valid,
                            uint16_t limit_w, uint16_t buffer_j);
void Power_Limit_SetCapFeedback(int16_t raw, uint8_t online);
void Power_Limit_Apply(float torque_out[WHEEL_CNT], rm_motor_t *const motor[WHEEL_CNT]);

#endif
