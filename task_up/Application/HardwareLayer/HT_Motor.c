/* HT_Motor.c - HT 电机驱动 */

#if 0 /* Legacy HT motor driver disabled: current gimbal board uses DM motors only. */
/**
 * @file        Ht_Motor.c
 * @author      2025_YZJ
 * @Version     V1.0
 * @date        8-Febraruary-2025
 * @brief       HT 电机驱动(适配型号 HT-03)
 * @brief       尚未完善, 目前仅作轮毂电机使用
 */
#include "HT_Motor.h"

static uint8_t Motor_Command[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
static void Motor_Send_Data(ht_motor_t *motor, uint8_t* buf);
static void Motor_Send_Command(ht_motor_t *motor, mit_cmd_t Command);
static void HT_Motor_SetControlPara(ht_motor_t *motor);
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits);
static float uint_to_float(uint16_t x_int, float x_min, float x_max, uint8_t bits);
static void Encoder_to_Motor_Angle(ht_motor_t *motor);
uint8_t flag_rx;
/* 电机卸力 */
void HT_Single_Motor_Sleep(ht_motor_t *motor)
{
	if(motor != NULL)
	{
//		if(motor->state->mode == Motor_UnControl)
//		{
//			Motor_Send_Command(motor, Enter_Motor_Mode);// 发送使能命令
//			motor->state->mode = Motor_Control;
//		}
		
		ht_tx_t* motor_tx_info = motor->tx_info;
		motor_tx_info->torque = 0;
		motor_tx_info->Kd = 0;
		motor_tx_info->Kp = 0;
	}
}

/* 把当前位置设为零点 */
void HT_Single_Motor_ZeroPosSensor(ht_motor_t *motor)
{
	if(motor != NULL)
	{
		motor->single_sleep(motor);  // 先卸力
		
		Motor_Send_Command(motor, Zero_Position_Sensor);
	}
}

/* 力矩模式输出 */
void HT_Single_Motor_Set_Torque(ht_motor_t *motor)
{
	  if(motor != NULL)
		{
			if(motor->state->mode == Motor_UnControl)
			{
				Motor_Send_Command(motor, Enter_Motor_Mode);  // 发送使能命令
				motor->state->mode = Motor_Control;
			}
			else
			{
				ht_tx_t* motor_tx_info = motor->tx_info;
				motor_tx_info->target_angle = 0;
				motor_tx_info->target_speed = 0;
				motor_tx_info->Kp = 0;
				motor_tx_info->Kd = 0;
				HT_Motor_SetControlPara(motor);
				Motor_Send_Data(motor, motor_tx_info->single_tx_buff);
			}
		}
}

/* 速度模式输出 */
void HT_Single_Motor_Set_Speed(ht_motor_t *motor)
{
	  if(motor != NULL)
		{
			if(motor->state->mode == Motor_UnControl)
			{
				Motor_Send_Command(motor, Enter_Motor_Mode);  // 发送使能命令
				motor->state->mode = Motor_Control;
			}
			
			ht_tx_t* motor_tx_info = motor->tx_info;
			motor_tx_info->target_angle = 0;
			motor_tx_info->Kp = 0;
			HT_Motor_SetControlPara(motor);
		
			Motor_Send_Data(motor, motor_tx_info->single_tx_buff);
		}
}

/* 位置模式输出 */
void HT_Single_Motor_Set_Angle(ht_motor_t *motor)
{
	  if(motor != NULL)
		{
			if(motor->state->mode == Motor_UnControl)
			{
				Motor_Send_Command(motor, Enter_Motor_Mode);  // 发送使能命令
				motor->state->mode = Motor_Control;
			}
			
			ht_tx_t* motor_tx_info = motor->tx_info;
			HT_Motor_SetControlPara(motor);
			Motor_Send_Data(motor, motor_tx_info->single_tx_buff);
		}
}

/* 解析电机反馈帧 */
void static Motor_ReceiveData(ht_motor_t *motor, uint8_t *rxBuf)
{
	ht_rx_t* motor_rx_info = motor->rx_info;
	motor_rx_info->encoder = uint_to_float((uint16_t)((rxBuf[1] << 8) | rxBuf[2]), HT_P_MIN, HT_P_MAX, 16);
	motor_rx_info->speed = uint_to_float((uint16_t)((rxBuf[3] << 4) | (rxBuf[4] >> 4)), HT_V_MIN, HT_V_MAX, 12) * motor->born_info->order_correction;
	motor_rx_info->torque_current = uint_to_float((uint16_t)(((rxBuf[4]&0x0F) << 8) | rxBuf[5]), HT_C_MIN, HT_C_MAX, 12);
	motor_rx_info->torque = motor_rx_info->torque_current * HT_TORQUE_CONSTANT;
	Encoder_to_Motor_Angle(motor);
	motor_rx_info->motor_angle_sum_vi += motor_rx_info->speed * TIME_STEP;
	motor->state->offline_cnt = 0;
}

/* 离线计数检测 */
void HT_Motor_Hearbeat(ht_motor_t *motor)
{
	motor->state->offline_cnt++;
	
	if(motor->state->offline_cnt > motor->state->offline_cnt_max) 
	{
//		motor->state->offline_cnt = motor->state->offline_cnt_max;
		motor->state->status = DEV_OFFLINE;
		motor->state->mode = Motor_UnControl;
	}
	else 
	{
		if(motor->state->status == DEV_OFFLINE)
			motor->state->status = DEV_ONLINE;
	}
}

/* 绑定接口并复位状态 */
void ht_motor_init(ht_motor_t *motor)
{
	motor->state->mode = Motor_UnControl;
	motor->single_sleep = HT_Single_Motor_Sleep;
	motor->zero_position = HT_Single_Motor_ZeroPosSensor;
	motor->single_set_torque = HT_Single_Motor_Set_Torque;
	motor->single_set_speed  = HT_Single_Motor_Set_Speed;
	motor->single_set_angle  = HT_Single_Motor_Set_Angle;
	motor->rx = Motor_ReceiveData;
	motor->single_heart_beat = HT_Motor_Hearbeat;
/* 先卸力 */
	motor->single_sleep(motor);
	motor->state->offline_cnt_max = 500;
	motor->rx_info->motor_angle_sum = 0;
}

/* 组控制 */

/* 组内依次发送力矩 */
static void Group_Motor_Set_Torque(ht_group_t *group)
{	
	
		if(group->motor[0] != NULL)
		{
			group->motor[0]->single_set_angle(group->motor[0]);
		}
		if(group->motor[1] != NULL)
		{
			group->motor[1]->single_set_angle(group->motor[1]);
		}
		if(group->motor[2] != NULL)
		{
			group->motor[2]->single_set_angle(group->motor[2]);
		}
		if(group->motor[3] != NULL)
		{
			group->motor[3]->single_set_angle(group->motor[3]);
		}
}

/* 组内全部卸力 */
static void Group_Motor_Sleep(ht_group_t *group)
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

/* 组内心跳检测 */
static void Group_Motor_Heartbeat(ht_group_t *group)
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


/* 电机组初始化 */
void HT_Group_Motor_Init(ht_group_t *group)
{
	  if(group->motor[0] != NULL)
		{
			group->motor[0]->single_init = ht_motor_init;
	    group->motor[0]->single_init(group->motor[0]);
		}
	  
		if(group->motor[1] != NULL)
		{
			group->motor[1]->single_init = ht_motor_init;
	    group->motor[1]->single_init(group->motor[1]);
		}

    if(group->motor[2] != NULL)
		{
			group->motor[2]->single_init = ht_motor_init;
	    group->motor[2]->single_init(group->motor[2]);
		}
	  
		if(group->motor[3] != NULL)
		{
			group->motor[3]->single_init = ht_motor_init;
	    group->motor[3]->single_init(group->motor[3]);
		}
	  
	
	  group->group_set_torque = Group_Motor_Set_Torque;
		group->group_heartbeat = Group_Motor_Heartbeat;
	  group->group_sleep = Group_Motor_Sleep;
}

/* 底层辅助 */
/* 下发命令帧 */
static void Motor_Send_Command(ht_motor_t *motor, mit_cmd_t Command)
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

/* 组装并发送控制帧 */
static void Motor_Send_Data(ht_motor_t *motor, uint8_t* buf)
{
	ht_cfg_t* motor_born_info = motor->born_info;
	
	CAN_SendData(motor_born_info->hcan, motor_born_info->stdId, buf);
}

/* 计算控制量 */
static void HT_Motor_SetControlPara(ht_motor_t *motor)
{
	ht_tx_t* motor_tx_info = motor->tx_info;
	uint16_t p, v, kp, kd, t;
  uint8_t* buf = motor_tx_info->single_tx_buff;
	
/* 参数约束在定义范围内 */
	motor_tx_info->target_angle = constrain(motor_tx_info->target_angle, HT_P_MIN, HT_P_MAX);
	motor_tx_info->target_speed = constrain(motor_tx_info->target_speed, HT_V_MIN, HT_V_MAX);
	motor_tx_info->Kp = constrain(motor_tx_info->Kp, HT_KP_MIN, HT_KP_MAX);
	motor_tx_info->Kd = constrain(motor_tx_info->Kd, HT_KD_MIN, HT_KD_MAX);
	motor_tx_info->torque = constrain(motor_tx_info->torque, -8, 8);
	
/* 按协议把浮点转成整型 */
	p = float_to_uint(motor_tx_info->target_angle,      HT_P_MIN,  HT_P_MAX,  16);            
	v = float_to_uint(motor_tx_info->target_speed,      HT_V_MIN,  HT_V_MAX,  12);
	kp = float_to_uint(motor_tx_info->Kp,    HT_KP_MIN, HT_KP_MAX, 12);
	kd = float_to_uint(motor_tx_info->Kd,    HT_KD_MIN, HT_KD_MAX, 12);
	t = float_to_uint(motor_tx_info->torque,      HT_T_MIN,  HT_T_MAX,  12);
	
/* 按协议把数据打包进 CAN 报文 */
	buf[0] = p>>8;
	buf[1] = p&0xFF;
	buf[2] = v>>4;
	buf[3] = ((v&0xF)<<4)|(kp>>8);
	buf[4] = kp&0xFF;
	buf[5] = kd>>4;
	buf[6] = ((kd&0xF)<<4)|(t>>8);
	buf[7] = t&0xff;
}

/* 浮点转定长整型 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    
    return (uint16_t) ((x-offset)*((float)((1<<bits)-1))/span);
}

/* 定长整型转浮点 */
static float uint_to_float(uint16_t x_int, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}


/* 编码器角度换算 */
static void Encoder_to_Motor_Angle(ht_motor_t *motor)
{
	float order_correction = 0.f;
	
	if(motor->born_info->order_correction == 1 || motor->born_info->order_correction == -1)
	{
		order_correction = (float)motor->born_info->order_correction;
	}
	else
	{
		order_correction = 1.f;
	}
	
	// 圈数处理
	if(!motor->rx_info->encoder_last && !motor->rx_info->motor_angle_sum)  // 首次上电角度累计值为0, 特殊处理
	{
		motor->rx_info->encoder_err = 0.f;
	}
	else
	{
		motor->rx_info->encoder_err = -motor->rx_info->encoder_last + motor->rx_info->encoder;
	}
	 
	if(motor->rx_info->encoder_err > 180.f)
	{
		motor->rx_info->encoder_err -= 2*HT_P_MAX;
	}
	else if(motor->rx_info->encoder_err < -180.f)
	{
		motor->rx_info->encoder_err += 2*HT_P_MAX;
	}
	
	motor->rx_info->motor_angle_sum += motor->rx_info->encoder_err * order_correction;
	motor->rx_info->motor_angle = motor->rx_info->motor_angle_sum - (int16_t)(motor->rx_info->motor_angle_sum / (2.f*PI)) * 2.f * PI;
	
	motor->rx_info->encoder_last = motor->rx_info->encoder;
	
}


#endif
