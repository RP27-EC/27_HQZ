/* chassis_follow.h - 底盘跟随 */

#ifndef __CHASSIS_FOLLOW_H
#define __CHASSIS_FOLLOW_H

#include <stdint.h>

#include "chassis_control.h"

typedef struct
{
    float yaw_mec_rad;
    float yaw_error_rad;
    float wz_target;
    float wz_output;
    float blend;
    int8_t turn_direction;

    uint8_t selected;
    uint8_t data_valid;
    uint8_t active;
    uint8_t fault_latched;
} chassis_follow_state_t;

extern chassis_follow_state_t chassis_follow;

void Chassis_Follow_Init(void);
void Chassis_Follow_UpdateMode(void);
void Chassis_Follow_Update(chassis_cmd_t *cmd);
uint8_t Chassis_Follow_IsSelected(void);
uint8_t Chassis_Follow_IsActive(void);
uint8_t Chassis_Follow_HasFault(void);

#endif

