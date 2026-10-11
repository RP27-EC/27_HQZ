/* chassis_control.h - 底盘控制 */

#ifndef __CHASSIS_CONTROL_H
#define __CHASSIS_CONTROL_H

#include <stdint.h>

#include "chassis_config.h"
#include "motor.h"

/* 底盘控制来源 */
typedef enum
{
    CHASSIS_SRC_NONE = 0, // 无有效输入，来源0
    CHASSIS_SRC_RC, // 遥控直控，来源1
    CHASSIS_SRC_RC_FOLLOW, // 遥控跟随，来源2
    CHASSIS_SRC_SPIN, // 小陀螺，来源3
    CHASSIS_SRC_KEYBOARD, // 键鼠直控，来源4
} chassis_source_e;

/* 底盘速度指令 */
typedef struct
{
    float vx; // 纵向轮速分量，rad/s
    float vy; // 横向轮速分量，rad/s
    float wz; // 旋转轮速分量，rad/s
    uint8_t valid; // 速度指令有效，0/1
    chassis_source_e source; // 速度指令来源，0~4
} chassis_cmd_t;

/* 四轮底盘状态 */
typedef struct
{
    float wheel_target[WHEEL_CNT]; // 四轮输出轴目标，rad/s
    float wheel_speed[WHEEL_CNT]; // 四轮输出轴反馈，rad/s
    float wheel_torque_out[WHEEL_CNT]; // 四轮力矩输出，N·m

    uint8_t wheel_online[WHEEL_CNT]; // 四轮在线，每项0/1
    uint8_t all_online; // 四轮均在线，0/1

    chassis_cmd_t cmd; // 当前速度指令，单位见类型
    uint8_t enabled; // 底盘控制使能，0/1
    uint8_t fault; // 底盘故障锁定，0/1
} chassis_control_state_t;


/* 底盘控制对象 */
typedef struct
{
    rm_group_t *wheel; // 四轮电机组对象指针
    chassis_control_state_t state; // 运行状态，单位见类型
} chassis_control_t;

extern chassis_control_t chassis_ctrl;

void Chassis_Control_Init(void);
void Chassis_Control_SetEnable(uint8_t enable);
void Chassis_Control_Stop(void);
void Chassis_Control_Update(const chassis_cmd_t *cmd);

#endif

