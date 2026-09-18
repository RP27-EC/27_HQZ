/**
 * @file        DM_Motor.c
 * @author      2025_YZJ
 * @Version     V1.0
 * @date        8-Febraruary-2025
 * @brief       闁跨喐鏋婚幏鐑芥晸閺傘倖瀚筸it闁跨喐鏋婚幏鐑芥晸閻欙紕顣幏鐑芥晸閺傘倖瀚归柨鐕傛嫹
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

/*..........................................闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐕傛嫹..........................................*/
/**
  * @brief          闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔峰建鐠佽瀚归柨鐕傛嫹
  * @param[in]      Motor_DM_t *motor     闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻敓锟�
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
  * @brief          闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻弬銈嗗闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹凤拷
  * @param[in]      Motor_DM_t *motor     闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻敓锟�
  * @retval         none
  */
void DM_Single_Motor_ZeroPosSensor(Motor_DM_t *motor)
{
	if(motor != NULL)
	{
		motor->single_sleep(motor);//闁跨喖銈虹€靛湱顣幏鐑芥晸閸欘偉顔愰幏鐑芥晸閿燂拷
		
		Motor_Send_Command(motor, Zero_Position_Sensor);
	}
}

/**
  * @brief          闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻弬銈嗗闁跨喐鏋婚幏鐑芥晸閺傘倖瀚规潪顒勬晸閺傘倖瀚�,闁跨喐鏋婚幏鐑芥晸閺傘倖瀚笴AN闁跨喐鏋婚幏鐑芥晸闁颁絻顕滈幏鐑芥晸閺傘倖瀚�
  * @param[in]      Motor_DM_t *motor     闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻敓锟�
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
  * @brief          闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻弬銈嗗闁跨喐鏋婚幏铚傛禋闁跨噦鎷�,闁跨喐鏋婚幏鐤洣闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻弬銈嗗kd\target_speed\torque,闁跨喐鏋婚幏鐑芥晸閺傘倖瀚笴AN闁跨喐鏋婚幏鐑芥晸闁颁絻顕滈幏鐑芥晸閺傘倖瀚�
  * @param[in]      Motor_DM_t *motor     闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻敓锟�
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
  * @brief          闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻弬銈嗗楠炲啿绁甸柨鐕傛嫹,闁跨喐鏋婚幏鐤洣闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻弬銈嗗kp\target_angle\kd\target_speed\torque,闁跨喐鏋婚幏鐑芥晸閺傘倖瀚笴AN闁跨喐鏋婚幏鐑芥晸闁颁絻顕滈幏鐑芥晸閺傘倖瀚�
  * @param[in]      Motor_DM_t *motor     闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻敓锟�
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
  * @brief          闁跨喐鏋婚幏鐑芥晸缁辩瘓N闁跨喎褰ㄩ弬顓熸灮閹风兘鏁撻弬銈嗗闁跨喐鏋婚幏鐑芥晸閹圭柉鎻幏鐑芥晸閺傘倖瀚�
  * @param[in]      Motor_DM_t *motor      闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻敓锟�
  * @param[in]      uint8_t *rxBuf						CAN闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻幑宄板簻閹凤拷
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
  * @brief          闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻弬銈嗗闁跨喐鏋婚幏鐑芥晸閿燂拷
  * @param[in]      Motor_HT_t *motor    闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻敓锟�
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
  * @brief          闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撶紒鐐点€嬮幏鐑芥晸閿燂拷
  * @param[in]      Motor_DM_t *motor     闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻敓锟�
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
	/*闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻弬銈嗗闁跨喐鏋婚幏鐑芥晸閿燂拷*/
	motor->state->motor_state = Motor_Unenable;
	motor->state->last_motor_state = Motor_Unenable;
	motor->state->offline_cnt_max = 100;
	motor->state->offline_cnt = motor->state->offline_cnt_max;
	motor->state->status = DEV_OFFLINE;
	motor->rx_info->motor_angle_sum = 0;
}
/*..........................................闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐕傛嫹..........................................*/
/**
  * @brief          闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹凤拷1~4闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻弬銈嗗闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风柉娴嗛柨鐔告灮閹凤拷
  * @param[in]      Motor_DM_Group_t *group     闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐕傛嫹
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
  * @brief          闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔峰建鐠佽瀚归柨鐕傛嫹
  * @param[in]      Motor_DM_Group_t *group     闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐕傛嫹
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
  * @brief          闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻弬銈嗗闁跨喐鏋婚幏鐑芥晸閿燂拷
  * @param[in]      Motor_DM_Group_t *group     闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐕傛嫹
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
  * @brief          闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹峰嘲顫愰柨鐔告灮閹凤拷
  * @param[in]      Motor_DM_Group_t *group     闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐕傛嫹
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

/*..........................................闁跨喐鏋婚幏鐑芥晸缁旑厾灏ㄩ幏鐑芥晸閺傘倖瀚�..........................................*/
/**
  * @brief          闁跨喐鏋婚幏鐑芥晸鏉堝啳顕滈幏鐑芥晸閺傘倖瀚归柨鐔煎徍绾板瀚归柨鐔告灮閹风兘鏁撻弬銈嗗閻ｅ鐓夐幏鐑芥晸閿燂拷
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
  * @brief          闁跨喐鏋婚幏鐑芥晸閹归绮ㄩ弸鍕晸閺傘倖瀚归柨鐔告灮閹烽攱浼呴柨鐔告灮閹风兘鏁撻柊鍨敾閹风兘鏁撻弬銈嗗
  * @param          Motor_DM_t *motor
  * @param          uint8_t* buf 鐟曚線鏁撻弬銈嗗闁跨喖鍙洪惃鍕敾閹风兘鏁撻弬銈嗗闁跨喐鏋婚幏閿嬩紖
  * @retval         none
  */
static void Motor_Send_Data(Motor_DM_t *motor, uint8_t* buf)
{
	Motor_DM_Born_Info_t* motor_born_info = motor->born_info;
	
	CAN_SendData(motor_born_info->hcan, motor_born_info->stdId, buf);
}

/**
  * @brief          闁跨喐鏋婚幏鐑芥晸閹瑰嘲鍤栭幏鐑芥晸闁扮數娈戦幉瀣闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归幁顖炴晸閺傘倖瀚归柨鐔虹叓閹插瀚归柨鐔惰寧鐠囇勫闁跨喐鏋婚幏鐑芥晸閺傘倖瀚�
  * @param          Motor_DM_t *motor
  * @retval         none
  */
static void Motor_SetControlPara(Motor_DM_t *motor)
{
	Motor_DM_Tx_Info_t* motor_tx_info = motor->tx_info;
	uint16_t p, v, kp, kd, t;
  uint8_t* buf = motor_tx_info->single_tx_buff;
	
	/* 闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻弬銈嗗閻楃喖鏁撻弬銈嗗闁跨喐鏋婚幏鐤樂闁跨喐鏋婚幏鐑芥晸娓氥儱鍤栭幏宄版纯闁跨喐鏋婚幏锟� */
	motor_tx_info->target_angle = constrain(motor_tx_info->target_angle, P_MIN, P_MAX);
	motor_tx_info->target_speed = constrain(motor_tx_info->target_speed, V_MIN, V_MAX);
	motor_tx_info->Kp = constrain(motor_tx_info->Kp, KP_MIN, KP_MAX);
	motor_tx_info->Kd = constrain(motor_tx_info->Kd, KD_MIN, KD_MAX);
	motor_tx_info->torque = constrain(motor_tx_info->torque, T_MIN, T_MAX);
	
	/* 闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归崡蹇涙晸娴犲绱濋柨鐔告灮閹风⿴loat闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻弬銈嗗鏉烆剟鏁撻弬銈嗗 */
	p = float_to_uint(motor_tx_info->target_angle,      P_MIN,  P_MAX,  16);            
	v = float_to_uint(motor_tx_info->target_speed,      V_MIN,  V_MAX,  12);
	kp = float_to_uint(motor_tx_info->Kp,    KP_MIN, KP_MAX, 12);
	kd = float_to_uint(motor_tx_info->Kd,    KD_MIN, KD_MAX, 12);
	t = float_to_uint(motor_tx_info->torque,      T_MIN,  T_MAX,  12);
	
	/* 闁跨喐鏋婚幏鐑芥晸閹圭柉鎻幏鐑芥晸閺傘倖瀚归崡蹇涙晸娴犲绱濋柨鐔告灮閹风兘鏁撻弬銈嗗闁跨喐鏋婚幏鐤祮闁跨喐鏋婚幏铚傝礋CAN闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻弬銈嗗闁跨喕顢滅拋瑙勫 */
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
  * @brief  闁跨喐鏋婚幏绌巐oat鏉烆兛璐焨int闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻弬銈嗗闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻弬銈嗗,闁跨喐鏋婚幏鐑解偓姘舵晸閺傘倖瀚归崡蹇涙晸娴犲绻氶柨鐔告灮閹疯渹绔撮柨鐔告灮閹凤拷
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
  * @brief  闁跨喐鏋婚幏绌焛nt鏉烆兛璐焒loat闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻弬銈嗗闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻弬銈嗗
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
  * @brief          闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻弬銈嗗鏉烆剟鏁撶憴鎺戝閻氬瓨瀚�
  * @param[in]      Motor_DM_t *motor     闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻敓锟�
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
	
	if(!motor->rx_info->motor_angle_last && !motor->rx_info->motor_angle_sum)//闁跨喐鏋婚幏铚傜闁跨喕顫楃拋瑙勫閸婇棿璐�0闁跨喐褰憴鎺戝閻氬瓨瀚规稉娲晸閺傘倖瀚归弮鍫曟晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻弬銈嗗闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻弬銈嗗闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻弬銈嗗
	{
		err = 0.f;
	}
	else
	{
		err = motor->rx_info->motor_angle - motor->rx_info->motor_angle_last;
	}
	
	if(abs(err) > (float)PI)//闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐕傛嫹
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
  * @brief          闁跨喎褰ㄩ弬顓狀暜閹风兘鏁撻弬銈嗗闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐕傛嫹
  * @param[in]      Motor_DM_t *motor     闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻敓锟�
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
	/*闁跨喐鏋婚幏宄板絿闁跨喐鏋婚幏铚傜闁跨喕濞囩拠褎瀚归崥宀勬晸閼哄倻顣幏宄板闁跨喍鑼庣喊澶嬪闁跨喖妯侀崚鐚存嫹*/
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

/*缁€娲晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻弬銈嗗*/

/*-------------闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻弬銈嗗闁跨喐鏋婚幏鐑芥晸閿燂拷-------------*/
/*
Motor_DM_Born_Info_t Yaw_Born_Info =
{
	.stdId = 0x001,//闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹烽銈ㄩ柨鐔告灮閹风兘鏁撶徊鈧珼
	
	.hcan = &hfdcan2,//娴ｅ潡鏁撻惌顐ゎ暜閹风áan闁跨喐鏋婚幏鐑芥晸閺傘倖瀚�

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

/*-------------闁跨喐鏋婚幏宄邦潗闁跨喐鏋婚幏锟�-------------*/
/*
Yaw_Motor.single_init(&Yaw_Motor);
*/

/*-------------闁跨喐鏋婚幏鐑芥晸缁夊摜灏ㄩ幏鐑芥晸閺傘倖瀚�-------------*/
/*
void CAN2_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	switch (rxId)
	{
		case 0x000://闁跨喐鏋婚幏鐑芥晸閺傘倖瀚笽D
		Yaw_Motor.rx(&Yaw_Motor, rxBuf);
		break;
		default:
			break;
	}
}
*/

/*-------------闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归幍褔鏁撻弬銈嗗-------------*/
/*
  * @file    monitor_task.c
  * @brief   闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻敓锟�
  *          1. 闁跨喐鏋婚幏閿嬆侀柨鐔告灮閹风兘鏁撻弬銈嗗闁跨喐鏋婚幏宄般亼闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐕傛嫹
  *          2. 闁跨喐鏋婚幏鐑芥晸閹活叏缍囬幏鐑芥晸閺傘倖瀚归柨鐔兼▉閸掍警鍓ㄩ幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻弬銈嗗闁跨喕濞囬敓锟�
void StartMonitorTask(void const * argument)//
{
	
	for(;;)
	{
		Yaw_Motor.heartbeat(&Yaw_Motor);
		
		osDelay(1);
	}
}

  * @file    monitor_task.c
  * @brief   闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻弬銈嗗闁跨喐鏋婚幏鐑芥晸閿燂拷
  *          1. 闁跨喐鏋婚幏鐑芥晸閺傘倖瀚归柨鐔告灮閹风兘鏁撻弬銈嗗濞戝倿鏁撻弬銈嗗缁併劑鏁撻弬銈嗗闁跨噦鎷�
  *          2. 闁跨喐鏋婚幏椋庡Ц閹線鏁撻弬銈嗗韫囨ぞ缍呴柨鐔告灮閹风兘鏁撻弬銈嗗闁跨喐鏋婚幏宄扮安
void StartMonitorTask(void const * argument)//
{
	
	for(;;)
	{
		//闁跨喐鏋婚幏鐑芥晸闁伴潧灏呴幏鐑芥晸閻欌剝鍞婚幏鐑芥晸娓氥儻缍囬幏鐑芥晸閺傘倖瀚归柨鐔哄珱绾板瀚归柨鐔告灮閹风兘鏁撻弬銈嗗閹殿參鏁撻弬銈嗗娑擄拷0.5N*m
		Yaw_Motor.tx_info->torque = 0.5f;
		Yaw_Motor.single_set_torque(&Yaw_Motor);
		
		osDelay(1);
	}
}

*/

