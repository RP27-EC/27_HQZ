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

/*������岽��------------------------------------------------*/
//���Ҫ��ɾ��RM���
//1.rm_motor_driver���ӵ��ID��CAN���� 
//2.dev_rm_motor_list_e��ӵ������
//3.CAN1_rxDataHandler��CAN2_rxDataHandler�����ӻ�ȡ�����Ϣ�ĺ���
//4.rm_motor_t rm_motor[]������ӵ���ܽṹ��
//5.����pid�ṹ���Լ���rm_motor_list_init����mo tor_pid_init��ʼ��pid�ṹ��
//���Ҫ�����������motor_out�︳ֵ����CAN_Sendͳһ����
/*���ID�궨��------------------------------------------------*/
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

