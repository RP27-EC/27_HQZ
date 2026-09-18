#ifndef __DM_MOTOR_H
#define __DM_MOTOR_H

#include "rp_config.h"
#include "arm_math.h"
#include "HT_Motor.h"
#include "drv_can.h"
#include "drv_tick.h"
#include "motor_def.h"
#include "rp_math.h"
#ifndef __HT_MOTOR_H
/*���ָ�*/
typedef enum Motor_MIT_Command_enum_e
{
	Enter_Motor_Mode,//ʹ�ܵ������(ָʾ�Ʊ���)
	Exit_Motor_Mode,//ʧ�ܵ������(ָʾ�Ʊ��)
	Zero_Position_Sensor,//�趨��ǰ����Ƕ�Ϊ��
	
}Motor_MIT_Command_e;
#endif

#define P_MIN -PI    // Radians
#define P_MAX PI        
#define V_MIN -30.0f    // Rad/s
#define V_MAX 30.0f
#define KP_MIN 0.0f     // N-m/rad
#define KP_MAX 500.0f
#define KD_MIN 0.0f     // N-m/rad/s
#define KD_MAX 5.0f
#define T_MIN -10.0f    // N.m
#define T_MAX 10.0f
#define C_MIN -10.0f    // A
#define C_MAX 10.0f


/*���ʹ��״̬*/
typedef enum Motor_HT_Work_state_enum_e
{
	Motor_Enable,//����ɿ�
	Motor_Unenable,//������ɿ�
	Over_Voltage,//��ѹ
	Lack_Voltage,//Ƿѹ
	Over_Current,//����
	MOS_OverTemp,//������MOS����
	Motor_OverTemp,//�������
	Commun_Loss,//ͨ�Ŷ�ʧ
	Unknow_Err,//δ֪����
}Motor_DM_Work_state_e;


/*�����ʼ������*/
typedef struct Motor_DM_Born_Info_struct_t
{	
    uint32_t stdId;//������Ʊ���ID


#ifdef __STM32F4xx_HAL_H
    CAN_HandleTypeDef *hcan;//can��ѡ��
#endif
	
#ifdef STM32H7xx_HAL_H
    FDCAN_HandleTypeDef *hcan;//can��ѡ��
#endif
	
	  int8_t order_correction;//Ť��������涨
}Motor_DM_Born_Info_t;

/*���յ��������Ϣ�ṹ��*/
typedef struct Motor_DM_Rx_Info_struct_t
{
	float speed;//����ٶ�(��λrad/s)
	
	float torque;//���ת��(��λN.m)
	
	float motor_angle_sum;

  float motor_angle;//��������ƾ��ԽǶȣ�-PI~PI

	float motor_angle_last;
	
	uint8_t num;//��������˳���
}Motor_DM_Rx_Info_t;

/*���͵��������Ϣ�ṹ��*/
typedef struct Motor_DM_Tx_Info_struct_t
{
	float torque;//��Ҫ���͵�ת��(��λN.m)
	
	float target_speed;//Ŀ���ٶ�(��λrad/s)
	
	float target_angle;//Ŀ��Ƕ�(��λrad)
	
	float Kp;//λ������
	
	float Kd;//�ٶ�����
	
	uint8_t single_tx_buff[8];//ʹ�õ������ʱ�ĸ�������
}Motor_DM_Tx_Info_t;
/* �����ο����� = (torque + Kp*err_angle + Kd*err_speed) */

/*���״̬�ṹ��*/
typedef struct Motor_DM_State_struct_t
{
    uint32_t offline_cnt;

    uint32_t offline_cnt_max;

    dev_work_state_t status;
	
		Motor_DM_Work_state_e motor_state;
	
		Motor_DM_Work_state_e last_motor_state;
}Motor_DM_State_t;

/*������ܽṹ��*/
typedef struct Motor_DM_struct_t
{
	Motor_DM_Born_Info_t* born_info;
	
	Motor_DM_Rx_Info_t* rx_info;
	
	Motor_DM_Tx_Info_t* tx_info;
	
	Motor_DM_State_t* state;


	
	void (*single_init)(struct Motor_DM_struct_t *motor);
	
	void (*single_sleep)(struct Motor_DM_struct_t *motor);
	
	void (*zero_position)(struct Motor_DM_struct_t *motor);
	
	void (*single_set_torque)(struct Motor_DM_struct_t *motor);
	
	void (*single_set_speed)(struct Motor_DM_struct_t *motor);
	
	void (*single_set_angle)(struct Motor_DM_struct_t *motor);
	
	void (*rx)(struct Motor_DM_struct_t *motor, uint8_t *rxBuf);
	
	void (*single_heart_beat)(struct Motor_DM_struct_t *motor);
}Motor_DM_t;

/*�����ṹ�壬����ʹ��MIT��������ƣ��ýṹ��ֻ�ǰѵ������һЩͨ�õĹ�������������������ƣ������������Ķ���ģʽ*/
typedef struct Motor_DM_Group_struct_t
{
	Motor_DM_t* motor[4];
	
	uint8_t motor_num;//ʵ�ʵ������
	
	void (*group_set_torque)(struct Motor_DM_Group_struct_t *group);
	
	void (*group_sleep)(struct Motor_DM_Group_struct_t *group);
	
	void (*group_init)(struct Motor_DM_Group_struct_t *group);
	
	void (*group_heartbeat)(struct Motor_DM_Group_struct_t *group);
}Motor_DM_Group_t;

void DM_Single_Motor_Init(Motor_DM_t *motor);
void Group_Motor_Init(Motor_DM_Group_t *group);

#endif
