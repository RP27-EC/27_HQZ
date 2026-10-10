/* chassis_follow.h - 底盘跟随 */

#ifndef __CHASSIS_FOLLOW_H
#define __CHASSIS_FOLLOW_H

#include <stdint.h>

#include "chassis_control.h"

/* 底盘跟随状态 */
typedef struct
{
    float yaw_mec_rad;    /* 云台机械角，rad */
    float yaw_error_rad;  /* 相对跟随中心误差，rad */
    float center_rad;     /* 跟随中心，rad */
    float wz_target;      /* 自动旋转目标，控制量 */
    float wz_output;      /* 旋转输出，控制量 */
    float manual_yaw_rate;/* 统一 Yaw 操作量，deg/s */
    float command_ff;     /* 指令前馈，转速控制量 */
    float recovery_ff;    /* 回正前馈，转速控制量 */
    float blend;          /* 接管融合比例，0~1 */
    int8_t turn_direction;/* -1/0/1，防抖转向 */

    uint8_t selected;     /* 1 = 跟随档位选中 */
    uint8_t data_valid;   /* 1 = 云台反馈有效 */
    uint8_t active;       /* 1 = 跟随环已接管 */
    uint8_t fault_latched;/* 1 = 跟随故障锁定 */
    uint8_t correction_stopped; /* 1 = 停止角度纠偏 */
    uint8_t disturbance_armed;  /* 1 = 已确认对齐停稳 */
    uint8_t recovery_pending;   /* 1 = 等待旋转交接 */
    uint8_t recovery_active;    /* 1 = 回正前馈生效 */
} chassis_follow_state_t;

extern chassis_follow_state_t chassis_follow;

void Chassis_Follow_Init(void);
void Chassis_Follow_UpdateMode(void);
void Chassis_Follow_Update(chassis_cmd_t *cmd);
uint8_t Chassis_Follow_IsSelected(void);
uint8_t Chassis_Follow_IsActive(void);
uint8_t Chassis_Follow_HasFault(void);

#endif

