/* chassis_spin.h - 底盘小陀螺 */

#ifndef __CHASSIS_SPIN_H
#define __CHASSIS_SPIN_H

#include <stdint.h>

#include "chassis_control.h"

/* 小陀螺状态 */
typedef struct
{
    float target_wz; /* 目标旋转角速度，rad/s */
    float output_wz; /* 斜坡后旋转输出，rad/s */

    uint8_t selected;/* 1 = 小陀螺档位选中 */
    uint8_t active;  /* 1 = 小陀螺控制生效 */
} chassis_spin_state_t;

extern chassis_spin_state_t chassis_spin;

void Chassis_Spin_Init(void);
void Chassis_Spin_UpdateMode(void);
void Chassis_Spin_Update(chassis_cmd_t *cmd);
uint8_t Chassis_Spin_IsSelected(void);
uint8_t Chassis_Spin_IsActive(void);

#endif

