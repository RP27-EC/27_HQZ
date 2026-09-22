/* motor.h - 电机对象管理 */

#ifndef __MOTOR_H
#define __MOTOR_H

#include "rp_config.h"
#include "can_protocol.h"
#include "rm_motor.h"
#include "KT_motor.h"
#include "HT_motor.h"
#include "DM_motor.h"
#include "motor_def.h"
#include "drv_can.h"

#define ID_GIMB_P 0x11
#define ID_GIMB_Y 0x12
#define ID_FRIC_L 0x201
#define ID_FRIC_R 0x202
#define ID_DIAL   0x141

typedef enum
{
    YAW = 0,
    PITCH,
} dev_dm_motor_list_e;

typedef enum
{
    SHOOT_FRIC_L = 0,
    SHOOT_FRIC_R,
    SHOOT_FRIC_NUM,
} dev_shoot_rm_motor_list_e;

extern  KT_motor_t dail_motor;
extern  ht_motor_t L_Wheel;
extern dm_motor_t dm_motor[2];
extern dm_group_t DM_Group;
extern  rm_motor_t rm_motor[SHOOT_FRIC_NUM];
extern  rm_group_t RM_Group;
void rm_motor_list_init(void);
void rm_motor_list_heart_beat(void);
void rm_motor_list_sleep(void);
void kt_motor_list_init(void);
void kt_motor_list_heart_beat(void);
void kt_motor_list_sleep(void);
void ht_motor_list_init(void);
void dm_motor_list_init(void);
void dm_motor_list_heart_beat(void);
uint8_t rm_motor_list_workstate(void);

#endif




