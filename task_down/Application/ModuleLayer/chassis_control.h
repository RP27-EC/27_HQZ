/* chassis_control.h - 底盘控制 */

#ifndef __CHASSIS_CONTROL_H
#define __CHASSIS_CONTROL_H

#include <stdint.h>

#include "chassis_config.h"
#include "motor.h"

/* 底盘控制来源 */
typedef enum
{
    CHASSIS_SRC_NONE = 0,   /* 无有效输入 */
    CHASSIS_SRC_RC,         /* 遥控直控 */
    CHASSIS_SRC_RC_FOLLOW,  /* 遥控跟随云台 */
    CHASSIS_SRC_SPIN,       /* 小陀螺 */
    CHASSIS_SRC_KEYBOARD,   /* 键鼠直控 */
} chassis_source_e;

/* 底盘速度指令 */
typedef struct
{
    float vx;              /* 纵向速度 */
    float vy;              /* 横向速度 */
    float wz;              /* 旋转角速度 */
    uint8_t valid;         /* 1 = 指令有效 */
    chassis_source_e source; /* 指令来源 */
} chassis_cmd_t;

/* 四轮底盘状态 */
typedef struct
{
    float wheel_target[WHEEL_CNT];      /* 轮速目标 */
    float wheel_speed[WHEEL_CNT];       /* 轮速反馈 */
    float wheel_torque_out[WHEEL_CNT];  /* 力矩输出 */

    uint8_t wheel_online[WHEEL_CNT];    /* 单轮在线标志 */
    uint8_t all_online;                 /* 1 = 四轮均在线 */

    chassis_cmd_t cmd;                  /* 最近一次指令 */
    uint8_t enabled;                    /* 1 = 控制使能 */
    uint8_t fault;                      /* 1 = 底盘故障 */
} chassis_control_state_t;


/* 底盘控制对象 */
typedef struct
{
    rm_group_t *wheel;            /* 四轮电机组 */
    chassis_control_state_t state;/* 运行状态 */
} chassis_control_t;

extern chassis_control_t chassis_ctrl;

void Chassis_Control_Init(void);
void Chassis_Control_SetEnable(uint8_t enable);
void Chassis_Control_Stop(void);
void Chassis_Control_Update(const chassis_cmd_t *cmd);

#endif

