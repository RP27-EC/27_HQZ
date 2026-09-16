/**
 * @file        DM_Motor.c
 * @author      2025_YZJ
 * @Version     V1.0
 * @date        8-Febraruary-2025
 * @brief       ����mit���Ƶ����
 */
 
/* Includes ------------------------------------------------------------------*/
#include "DM_Motor.h"

static uint8_t Motor_Command[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};

static void Motor_Send_Data(Motor_DM_t *motor, uint8_t* buf);
static void Motor_SetControlPara(Motor_DM_t *motor);
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits);
static float uint_to_float(uint16_t x_int, float x_min, float x_max, uint8_t bits);
static void Motor_Send_Command(Motor_DM_t *motor, Motor_MIT_Command_e Command);
static void Angle_Sum_Cal(Motor_DM_t *motor);
static void Motor_ERR_Check(Motor_DM_t *motor, uint8_t err_word);
static void Group_Motor_Heartbeat(Motor_DM_Group_t *group);

/*..........................................�����..........................................*/
/**
  * @brief          �����ж��
  * @param[in]      Motor_DM_t *motor     �������
  * @retval         none
  */
void DM_Single_Motor_Sleep(Motor_DM_t *motor)
{
	if(motor != NULL)
	{
		motor->tx_info->torque = 0;
		motor->tx_info->Kd = 0;
		motor->tx_info->Kp = 0;
	}
}

/**
  * @brief          ��������������
  * @param[in]      Motor_DM_t *motor     �������
  * @retval         none
  */
void DM_Single_Motor_ZeroPosSensor(Motor_DM_t *motor)
{
	if(motor != NULL)
	{
		motor->single_sleep(motor);//�ȶԵ��ж��
		
		Motor_Send_Command(motor, Zero_Position_Sensor);
	}
}

/**
  * @brief          ������������ת��,����CAN���Ͳ���
  * @param[in]      Motor_DM_t *motor     �������
  * @retval         none
  */
void DM_Single_Motor_Set_Torque(Motor_DM_t *motor)
{
	  if(motor != NULL)
		{
			if(motor->state->motor_state == Motor_Unenable)
			{
				motor->state->motor_state = Motor_Enable;
				Motor_Send_Command(motor, Enter_Motor_Mode);
			}
			else
			{
				Motor_DM_Tx_Info_t* motor_tx_info = motor->tx_info;
				motor_tx_info->Kp = 0;
				motor_tx_info->Kd = 0;
				Motor_SetControlPara(motor);
				motor->tx_info->torque = 0;
			}
		}
}

/**
  * @brief          ����������ٶ�,��Ҫ��������kd\target_speed\torque,����CAN���Ͳ���
  * @param[in]      Motor_DM_t *motor     �������
  * @retval         none
  */
void DM_Single_Motor_Set_Speed(Motor_DM_t *motor)
{
	  if(motor != NULL)
		{
			if(motor->state->motor_state == Motor_Unenable)
			{
				motor->state->motor_state = Motor_Enable;
				Motor_Send_Command(motor, Enter_Motor_Mode);
			}
			else
			{
				Motor_DM_Tx_Info_t* motor_tx_info = motor->tx_info;
				motor_tx_info->Kp = 0;
				Motor_SetControlPara(motor);
			}

		}
}

/**
  * @brief          ��������ƽǶ�,��Ҫ��������kp\target_angle\kd\target_speed\torque,����CAN���Ͳ���
  * @param[in]      Motor_DM_t *motor     �������
  * @retval         none
  */
void DM_Single_Motor_Set_Angle(Motor_DM_t *motor)
{
	  if(motor != NULL)
		{
			if(motor->state->motor_state == Motor_Unenable)
			{
				motor->state->motor_state = Motor_Enable;
				Motor_Send_Command(motor, Enter_Motor_Mode);
			}
			else
			{
				motor->tx_info->target_speed = 0;
				Motor_SetControlPara(motor);
			}
		}
}

/**
  * @brief          ���CAN�жϽ������ݴ���
  * @param[in]      Motor_DM_t *motor      �������
  * @param[in]      uint8_t *rxBuf						CAN�������ݰ�
  * @retval         none
  */
static void Motor_ReceiveData(Motor_DM_t *motor, uint8_t *rxBuf)
{
	Motor_DM_Rx_Info_t* motor_rx_info = motor->rx_info;
	Motor_ERR_Check(motor, rxBuf[0] >> 4);
	motor_rx_info->motor_angle = uint_to_float((uint16_t)((rxBuf[1] << 8) | rxBuf[2]), P_MIN, P_MAX, 16);
	motor_rx_info->speed = uint_to_float((uint16_t)((rxBuf[3] << 4) | (rxBuf[4] >> 4)), V_MIN, V_MAX, 12);
	if(abs(motor_rx_info->speed) == 0.0109901428f)
	{
		motor_rx_info->speed = 0;
	}
	motor_rx_info->torque = uint_to_float((uint16_t)(((rxBuf[4]&0x0F) << 8) | rxBuf[5]), C_MIN, C_MAX, 12);
	Angle_Sum_Cal(motor);
	motor->state->offline_cnt = 0;
	motor->state->status = DEV_ONLINE;
}

/**
  * @brief          �����������
  * @param[in]      Motor_HT_t *motor    �������
  * @retval         none
  */
static void DM_Motor_Hearbeat(Motor_DM_t *motor)
{
	motor->state->offline_cnt++;
	
	if(motor->state->offline_cnt > motor->state->offline_cnt_max) 
	{
		motor->state->offline_cnt = motor->state->offline_cnt_max;
		motor->state->status = DEV_OFFLINE;
		motor->state->motor_state = Motor_Unenable;
	}
	else 
	{
		if(motor->state->status == DEV_OFFLINE)
			motor->state->status = DEV_ONLINE;
	}
}

/**
  * @brief          �������ʼ��
  * @param[in]      Motor_DM_t *motor     �������
  * @retval         none
  */
void DM_Single_Motor_Init(Motor_DM_t *motor)
{
	motor->single_sleep = DM_Single_Motor_Sleep;
	motor->single_set_torque = DM_Single_Motor_Set_Torque;
	motor->single_set_speed  = DM_Single_Motor_Set_Speed;
	motor->single_set_angle  = DM_Single_Motor_Set_Angle;
	motor->rx = Motor_ReceiveData;
	motor->single_heart_beat = DM_Motor_Hearbeat;
	/*�����������*/
	motor->state->motor_state = Motor_Unenable;
	motor->state->last_motor_state = Motor_Unenable;
	motor->state->offline_cnt_max = 100;
	motor->state->offline_cnt = motor->state->offline_cnt_max;
	motor->state->status = DEV_OFFLINE;
	motor->rx_info->motor_angle_sum = 0;
}
/*..........................................�����..........................................*/
/**
  * @brief          ������1~4��������������ת��
  * @param[in]      Motor_DM_Group_t *group     �����
  * @retval         none
  */
static void Group_Motor_Set_Torque(Motor_DM_Group_t *group)
{
	static uint8_t rx_num = 0;
	
	if(group->motor[rx_num] != NULL)
	{
		group->motor[rx_num]->single_set_torque(group->motor[rx_num]);
		rx_num ++;
	}
	if(rx_num >= group->motor_num)
	{
		rx_num = 0;
	}

}

/**
  * @brief          �����ж��
  * @param[in]      Motor_DM_Group_t *group     �����
  * @retval         none
  */
static void Group_Motor_Sleep(Motor_DM_Group_t *group)
{	
	if(group->motor[0] != NULL)
	{
		group->motor[0]->single_sleep(group->motor[0]);
	}
	if(group->motor[1] != NULL)
	{
		group->motor[1]->single_sleep(group->motor[1]);
	}
	if(group->motor[2] != NULL)
	{
		group->motor[2]->single_sleep(group->motor[2]);
	}
	if(group->motor[3] != NULL)
	{
		group->motor[3]->single_sleep(group->motor[3]);
	}
}

/**
  * @brief          �����������
  * @param[in]      Motor_DM_Group_t *group     �����
  * @retval         none
  */
static void Group_Motor_Heartbeat(Motor_DM_Group_t *group)
{
	if(group->motor[0] != NULL)
	{
		group->motor[0]->single_heart_beat(group->motor[0]);
	}
	  
	if(group->motor[1] != NULL)
	{
		group->motor[1]->single_heart_beat(group->motor[1]);
	}

  if(group->motor[2] != NULL)
	{
		group->motor[2]->single_heart_beat(group->motor[2]);
	}
	  
	if(group->motor[3] != NULL)
	{
		group->motor[3]->single_heart_beat(group->motor[3]);
	}
}

/**
  * @brief          ������ʼ��
  * @param[in]      Motor_DM_Group_t *group     �����
  * @retval         none
  */
void Group_Motor_Init(Motor_DM_Group_t *group)
{
	uint8_t num_init = 0;
	if(group->motor[0] != NULL)
	{
		group->motor[0]->single_init = DM_Single_Motor_Init;
		group->motor[0]->single_init(group->motor[0]);
		num_init++;
	}
	
	if(group->motor[1] != NULL)
	{
		group->motor[1]->single_init = DM_Single_Motor_Init;
		group->motor[1]->single_init(group->motor[1]);
		num_init++;
	}

  if(group->motor[2] != NULL)
	{
		group->motor[2]->single_init = DM_Single_Motor_Init;
		group->motor[2]->single_init(group->motor[2]);
		num_init++;
	}
	  
	if(group->motor[3] != NULL)
	{
		group->motor[3]->single_init = DM_Single_Motor_Init;
		group->motor[3]->single_init(group->motor[3]);
		num_init++;
	}
	  
		group->motor_num = num_init;
	  group->group_set_torque = Group_Motor_Set_Torque;
		group->group_heartbeat = Group_Motor_Heartbeat;
	  group->group_sleep = Group_Motor_Sleep;
}

/*..........................................���ߺ���..........................................*/
/**
  * @brief          ���ϲ����͵�������
  * @param          Motor_DM_t *motor
  * @param[in]      Motor_DM_Command_e Command
  * @retval         none
  */
static void Motor_Send_Command(Motor_DM_t *motor, Motor_MIT_Command_e Command)
{
	switch(Command)
	{
		case Enter_Motor_Mode:
		Motor_Command[7] = 0xFC;
		break;
		case Exit_Motor_Mode:
		Motor_Command[7] = 0xFD;
		break;
		case Zero_Position_Sensor:
		Motor_Command[7] = 0xFE;
		break;
		default:
		break;
	}
	Motor_Send_Data(motor, Motor_Command);
}

/**
  * @brief          ���ݽṹ����Ϣ���ͱ���
  * @param          Motor_DM_t *motor
  * @param          uint8_t* buf Ҫ���͵ı�����Ϣ
  * @retval         none
  */
static void Motor_Send_Data(Motor_DM_t *motor, uint8_t* buf)
{
	Motor_DM_Born_Info_t* motor_born_info = motor->born_info;
	
	CAN_SendData(motor_born_info->hcan, motor_born_info->stdId, buf);
}

/**
  * @brief          ���ݷ��͵ı�����Ϣ���ñ��Ĳ�����
  * @param          Motor_DM_t *motor
  * @retval         none
  */
static void Motor_SetControlPara(Motor_DM_t *motor)
{
	Motor_DM_Tx_Info_t* motor_tx_info = motor->tx_info;
	uint16_t p, v, kp, kd, t;
  uint8_t* buf = motor_tx_info->single_tx_buff;
	
	/* ��������Ĳ����ڶ���ķ�Χ�� */
	motor_tx_info->target_angle = constrain(motor_tx_info->target_angle, P_MIN, P_MAX);
	motor_tx_info->target_speed = constrain(motor_tx_info->target_speed, V_MIN, V_MAX);
	motor_tx_info->Kp = constrain(motor_tx_info->Kp, KP_MIN, KP_MAX);
	motor_tx_info->Kd = constrain(motor_tx_info->Kd, KD_MIN, KD_MAX);
	motor_tx_info->torque = constrain(motor_tx_info->torque, T_MIN, T_MAX);
	
	/* ����Э�飬��float��������ת�� */
	p = float_to_uint(motor_tx_info->target_angle,      P_MIN,  P_MAX,  16);            
	v = float_to_uint(motor_tx_info->target_speed,      V_MIN,  V_MAX,  12);
	kp = float_to_uint(motor_tx_info->Kp,    KP_MIN, KP_MAX, 12);
	kd = float_to_uint(motor_tx_info->Kd,    KD_MIN, KD_MAX, 12);
	t = float_to_uint(motor_tx_info->torque,      T_MIN,  T_MAX,  12);
	
	/* ���ݴ���Э�飬������ת��ΪCAN���������ֶ� */
	buf[0] = p>>8;
	buf[1] = p&0xFF;
	buf[2] = v>>4;
	buf[3] = ((v&0xF)<<4)|(kp>>8);
	buf[4] = kp&0xFF;
	buf[5] = kd>>4;
	buf[6] = ((kd&0xF)<<4)|(t>>8);
	buf[7] = t&0xff;
	
	Motor_Send_Data(motor, buf);
}

/**
  * @brief  ��floatתΪuint����������������,��ͨ��Э�鱣��һ��
  * @param
  * @retval 
  */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    
    return (uint16_t) ((x-offset)*((float)((1<<bits)-1))/span);
}

/**
  * @brief  ��uintתΪfloat����������������
  * @param
  * @retval 
  */
static float uint_to_float(uint16_t x_int, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/**
  * @brief          ��������ת�ǶȺ�
  * @param[in]      Motor_DM_t *motor     �������
  * @retval         none
  */
static void Angle_Sum_Cal(Motor_DM_t *motor)
{
	float err = 0.f;
	
	float order_correction = 0.f;
	
	if(motor->born_info->order_correction == 1 || motor->born_info->order_correction == -1)
	{
		order_correction = (float)motor->born_info->order_correction;
	}
	else
	{
		order_correction = 1.f;
	}
	
	if(!motor->rx_info->motor_angle_last && !motor->rx_info->motor_angle_sum)//��һ�Ƕ�ֵΪ0�ҽǶȺ�Ϊ��ʱ����������������������
	{
		err = 0.f;
	}
	else
	{
		err = motor->rx_info->motor_angle - motor->rx_info->motor_angle_last;
	}
	
	if(abs(err) > (float)PI)//�����
	{
		if(err > 0.f)
		{
			motor->rx_info->motor_angle_sum += (-(float)PI * 2.f + err) * order_correction;
		}
		else
		{
			motor->rx_info->motor_angle_sum += ((float)PI * 2.f + err) * order_correction;
		}
	}
	else
	{
		motor->rx_info->motor_angle_sum += err * order_correction;
	}
	
	motor->rx_info->motor_angle_last = motor->rx_info->motor_angle;
}

/**
  * @brief          �жϵ��������
  * @param[in]      Motor_DM_t *motor     �������
  * @retval         none
  */
static void Motor_ERR_Check(Motor_DM_t *motor, uint8_t err_word)
{
	Motor_DM_State_t* my_state = motor->state;
//	static Motor_DM_Work_state_e temp_state = Motor_Unenable;
	switch(err_word)
	{
		case 0:
		my_state->motor_state = Motor_Unenable;
		break;
		case 1:
		my_state->motor_state = Motor_Enable;
		break;
		case 8:
		my_state->motor_state = Over_Voltage;
		break;
		case 9:
		my_state->motor_state = Lack_Voltage;
		break;
		case 10:
		my_state->motor_state = Over_Current;
		break;
		case 11:
		my_state->motor_state = MOS_OverTemp;
		break;
		case 12:
		my_state->motor_state = Motor_OverTemp;
		break;
		case 13:
		my_state->motor_state = Commun_Loss;
		break;
		default:
		my_state->motor_state = Unknow_Err;
		break;
	};
	/*��ȡ��һ�β�ͬ�ڵ�ǰ�ĵ��״̬*/
//	if(temp_state != my_state->motor_state)
//	{
//		if(temp_state != my_state->last_motor_state)
//		{
//			my_state->last_motor_state = temp_state;
//		}
//	}
//	temp_state = my_state->motor_state;
	if(my_state->motor_state != Motor_Enable &&
		my_state->motor_state != Motor_Unenable)
	{
		my_state->last_motor_state = my_state->motor_state;
	}
}

/*ʾ������*/

/*-------------�����������-------------*/
/*
Motor_DM_Born_Info_t Yaw_Born_Info =
{
	.stdId = 0x001,//������Ʊ���ID
	
	.hcan = &hfdcan2,//ʹ�õ�Can����

};

Motor_DM_Rx_Info_t Yaw_Rx_Info_t;

Motor_DM_Tx_Info_t Yaw_Tx_Info_t;

Motor_DM_State_t Yaw_State_t;

Motor_DM_t Yaw_Motor = 
{
	.born_info = &Yaw_Born_Info,
	
	.rx_info = &Yaw_Rx_Info_t,
	
	.tx_info = &Yaw_Tx_Info_t,
	
	.state = &Yaw_State_t,
	
	.single_init = &DM_Single_Motor_Init,
};
*/

/*-------------��ʼ��-------------*/
/*
Yaw_Motor.single_init(&Yaw_Motor);
*/

/*-------------���պ���-------------*/
/*
void CAN2_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	switch (rxId)
	{
		case 0x000://����ID
		Yaw_Motor.rx(&Yaw_Motor, rxBuf);
		break;
		default:
			break;
	}
}
*/

/*-------------����ִ��-------------*/
/*
  * @file    monitor_task.c
  * @brief   �������
  *          1. ��ģ������ʧ�����
  *          2. ���ң����״̬��������λ
void StartMonitorTask(void const * argument)//
{
	
	for(;;)
	{
		Yaw_Motor.heartbeat(&Yaw_Motor);
		
		osDelay(1);
	}
}

  * @file    monitor_task.c
  * @brief   �����������
  *          1. ��������Ϳ��Ʊ���
  *          2. ��״̬��־λ������Ӧ
void StartMonitorTask(void const * argument)//
{
	
	for(;;)
	{
		//���Ϳ��Ʊ��ģ����Ƶ�����Ť��Ϊ0.5N*m
		Yaw_Motor.tx_info->torque = 0.5f;
		Yaw_Motor.single_set_torque(&Yaw_Motor);
		
		osDelay(1);
	}
}

*/

