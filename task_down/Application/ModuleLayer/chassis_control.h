#ifndef __CHASSIS_CONTROL_H
#define __CHASSIS_CONTROL_H

#include <stdint.h>

#include "chassis_config.h"
#include "motor.h"

//遥控  键鼠
typedef enum
{
    CHASSIS_SRC_NONE = 0,
    CHASSIS_SRC_RC,
    CHASSIS_SRC_RC_FOLLOW,
    CHASSIS_SRC_KEYBOARD,
} chassis_source_e;

//指令结构体
typedef struct
{
    float vx;
    float vy;
    float wz;
    uint8_t valid;
    chassis_source_e source;
} chassis_cmd_t;

// 四个电机对应的一些值
typedef struct
{
    float wheel_target[WHEEL_CNT];
    float wheel_speed[WHEEL_CNT];
    float wheel_torque_out[WHEEL_CNT];

    uint8_t wheel_online[WHEEL_CNT];//在线
    uint8_t all_online;

    chassis_cmd_t cmd;
    uint8_t enabled;
    uint8_t fault;
} chassis_control_state_t;


typedef struct
{
    Motor_RM_Group_t *wheel;
    chassis_control_state_t state;
} chassis_control_t;

extern chassis_control_t chassis_ctrl;

void Chassis_Control_Init(void);
void Chassis_Control_SetEnable(uint8_t enable);
void Chassis_Control_Stop(void);
void Chassis_Control_Update(const chassis_cmd_t *cmd);

#endif
