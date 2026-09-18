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

/*锟斤拷锟斤拷锟斤拷宀斤拷锟�------------------------------------------------*/
//锟斤拷锟揭拷锟缴撅拷锟絉M锟斤拷锟�
//1.rm_motor_driver锟斤拷锟接碉拷锟絀D锟斤拷CAN锟斤拷锟斤拷 
//2.dev_rm_motor_list_e锟斤拷拥锟斤拷锟斤拷锟斤拷
//3.CAN1_rxDataHandler锟斤拷CAN2_rxDataHandler锟斤拷锟斤拷锟接伙拷取锟斤拷锟斤拷锟较拷暮锟斤拷锟�
//4.rm_motor_t rm_motor[]锟斤拷锟斤拷锟斤拷拥锟斤拷锟杰结构锟斤拷
//5.锟斤拷锟斤拷pid锟结构锟斤拷锟皆硷拷锟斤拷rm_motor_list_init锟斤拷锟斤拷mo tor_pid_init锟斤拷始锟斤拷pid锟结构锟斤拷
//锟斤拷锟揭拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷motor_out锟斤赋值锟斤拷锟斤拷CAN_Send统一锟斤拷锟斤拷
/*锟斤拷锟絀D锟疥定锟斤拷------------------------------------------------*/
#define ID_GIMB_P 0x11
#define ID_GIMB_Y 0x12

typedef enum
{
    YAW = 0,
    PITCH,
} dev_dm_motor_list_e;

extern  KT_motor_t kt_motor[1];
extern  Motor_HT_t L_Wheel;
extern Motor_DM_t dm_motor[2];
extern Motor_DM_Group_t DM_Group;
extern  Motor_RM_t R_Fric;
extern  Motor_RM_Group_t RM_Group;
/* Exported functions --------------------------------------------------------*/
void rm_motor_list_init(void);
void rm_motor_list_heart_beat(void);
void kt_motor_list_init(void);
void ht_motor_list_init(void);
void dm_motor_list_init(void);
void dm_motor_list_heart_beat(void);
uint8_t rm_motor_list_workstate(void);

#endif

