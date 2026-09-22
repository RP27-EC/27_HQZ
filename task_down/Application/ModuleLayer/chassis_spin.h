/* chassis_spin.h - 底盘小陀螺 */

#ifndef __CHASSIS_SPIN_H
#define __CHASSIS_SPIN_H

#include <stdint.h>

#include "chassis_control.h"

typedef struct
{
    float target_wz;
    float output_wz;

    uint8_t selected;
    uint8_t active;
} chassis_spin_state_t;

extern chassis_spin_state_t chassis_spin;

void Chassis_Spin_Init(void);
void Chassis_Spin_UpdateMode(void);
void Chassis_Spin_Update(chassis_cmd_t *cmd);
uint8_t Chassis_Spin_IsSelected(void);
uint8_t Chassis_Spin_IsActive(void);

#endif

