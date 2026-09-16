#ifndef __MOTOR_H
#define __MOTOR_H

#include "rp_config.h"
#include "drv_can.h"
#include "DM_Motor.h"

/* DM4310 feedback IDs. Command IDs are configured in motor.c. */
#define ID_GIMB_P 0x11
#define ID_GIMB_Y 0x12

extern Motor_DM_t Pitch_Motor;
extern Motor_DM_t Yaw_Motor;

void dm_motor_list_init(void);
void dm_motor_list_heart_beat(void);

#endif
