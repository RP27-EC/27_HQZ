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
    float wz_target;      /* 自动旋转目标，rad/s */
    float wz_output;      /* 融合后旋转输出，rad/s */
    float blend;          /* 自动控制融合系数 */
    int8_t turn_direction;/* -1/0/1，防抖转向 */

    uint8_t selected;     /* 1 = 跟随档位选中 */
    uint8_t data_valid;   /* 1 = 云台反馈有效 */
    uint8_t active;       /* 1 = 跟随环已接管 */
    uint8_t fault_latched;/* 1 = 跟随故障锁定 */
} chassis_follow_state_t;

extern chassis_follow_state_t chassis_follow;

void Chassis_Follow_Init(void);
void Chassis_Follow_UpdateMode(void);
void Chassis_Follow_Update(chassis_cmd_t *cmd);
uint8_t Chassis_Follow_IsSelected(void);
uint8_t Chassis_Follow_IsActive(void);
uint8_t Chassis_Follow_HasFault(void);

#endif

