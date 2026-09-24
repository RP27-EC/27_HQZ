/* DM_Motor.c - 达妙电机驱动 */
#include "DM_Motor.h"

static uint8_t Motor_Command[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC}; /* 达妙模式命令帧 */

static void Motor_Send_Data(dm_motor_t *motor, uint8_t* buf);
static void Motor_SetControlPara(dm_motor_t *motor);
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits);
static float uint_to_float(uint16_t x_int, float x_min, float x_max, uint8_t bits);
static void Motor_Send_Command(dm_motor_t *motor, mit_cmd_t Command);
static void Angle_Sum_Cal(dm_motor_t *motor);
static void Motor_ERR_Check(dm_motor_t *motor, uint8_t err_word);
static void Group_Motor_Heartbeat(dm_group_t *group);

/* 电机卸力并清空输出 */
/* 卸力：清零 Kp/Kd 和力矩 */
void DM_Single_Motor_Sleep(dm_motor_t *motor)
{
	if(motor != NULL)
	{
		motor->tx_info->torque = 0; /* 清力矩 */
		motor->tx_info->Kd = 0; /* 清阻尼 */
		motor->tx_info->Kp = 0; /* 清刚度 */
	}
}

/* 把当前位置设为零点 */
void DM_Single_Motor_ZeroPosSensor(dm_motor_t *motor)
{
	if(motor != NULL)
	{
		motor->single_sleep(motor); /* 先卸力 */
		
		Motor_Send_Command(motor, Zero_Position_Sensor);
	}
}

/* 力矩模式输出 */
/* 力矩模式：Kp/Kd 置零，仅保留力矩 */
void DM_Single_Motor_Set_Torque(dm_motor_t *motor)
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
				dm_tx_t* motor_tx_info = motor->tx_info;
				motor_tx_info->Kp = 0; /* 关闭位置刚度 */
				motor_tx_info->Kd = 0; /* 关闭速度阻尼 */
				Motor_SetControlPara(motor);
				motor->tx_info->torque = 0;
			}
		}
}

/* 速度模式输出 */
/* 速度模式：关闭位置刚度，保留速度环 */
void DM_Single_Motor_Set_Speed(dm_motor_t *motor)
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
				dm_tx_t* motor_tx_info = motor->tx_info;
				motor_tx_info->Kp = 0; /* 速度模式不设位置刚度 */
				Motor_SetControlPara(motor);
			}

		}
}

/* 位置模式输出 */
/* 位置模式：速度目标置零，交给 MIT 位置环 */
void DM_Single_Motor_Set_Angle(dm_motor_t *motor)
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

/* 解析电机反馈帧 */
/* 解析达妙反馈帧：错误码、位置、速度、力矩 */
static void Motor_ReceiveData(dm_motor_t *motor, uint8_t *rxBuf)
{
	dm_rx_t* motor_rx_info = motor->rx_info;
	Motor_ERR_Check(motor, rxBuf[0] >> 4); /* 高 4 位为状态 */
	motor_rx_info->motor_angle = uint_to_float((uint16_t)((rxBuf[1] << 8) | rxBuf[2]), P_MIN, P_MAX, 16); /* 位置，rad */
	motor_rx_info->speed = uint_to_float((uint16_t)((rxBuf[3] << 4) | (rxBuf[4] >> 4)), V_MIN, V_MAX, 12);
	if(abs(motor_rx_info->speed) == 0.0109901428f)
	{
		motor_rx_info->speed = 0;
	}
	motor_rx_info->torque = uint_to_float((uint16_t)(((rxBuf[4]&0x0F) << 8) | rxBuf[5]), C_MIN, C_MAX, 12); /* 力矩，N*m */
	Angle_Sum_Cal(motor); /* 累计多圈角度 */
	motor->state->offline_cnt = 0;
	motor->state->status = DEV_ONLINE;
}

/* 离线计数检测 */
/* 离线计数：超限后锁离线并禁止使能 */
static void DM_Motor_Hearbeat(dm_motor_t *motor)
{
	motor->state->offline_cnt++; /* 周期递增 */
	
	if(motor->state->offline_cnt > motor->state->offline_cnt_max) 
	{
		motor->state->offline_cnt = motor->state->offline_cnt_max;
		motor->state->status = DEV_OFFLINE;
		motor->state->motor_state = Motor_Unenable; /* 离线必须卸力 */
	}
	else 
	{
		if(motor->state->status == DEV_OFFLINE)
			motor->state->status = DEV_ONLINE;
	}
}

/* 绑定接口并复位状态 */
void dm_motor_init(dm_motor_t *motor)
{
	motor->single_sleep = DM_Single_Motor_Sleep;
	motor->single_set_torque = DM_Single_Motor_Set_Torque;
	motor->single_set_speed  = DM_Single_Motor_Set_Speed;
	motor->single_set_angle  = DM_Single_Motor_Set_Angle;
	motor->rx = Motor_ReceiveData;
	motor->single_heart_beat = DM_Motor_Hearbeat;
/* 复位状态 */
	motor->state->motor_state = Motor_Unenable;
	motor->state->last_motor_state = Motor_Unenable;
	motor->state->offline_cnt_max = 100; /* 离线阈值 */
	motor->state->offline_cnt = motor->state->offline_cnt_max;
	motor->state->status = DEV_OFFLINE;
	motor->rx_info->motor_angle_sum = 0;
}
/* 组内依次发送力矩 */
static void Group_Motor_Set_Torque(dm_group_t *group)
{
	static uint8_t rx_num = 0; /* 轮询槽位 */
	
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

/* 组内全部卸力 */
static void Group_Motor_Sleep(dm_group_t *group)
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
static void Group_Motor_Heartbeat(dm_group_t *group)
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
void dm_group_init(dm_group_t *group)
{
	uint8_t num_init = 0;
	if(group->motor[0] != NULL)
	{
		group->motor[0]->single_init = dm_motor_init;
		group->motor[0]->single_init(group->motor[0]);
		num_init++;
	}
	
	if(group->motor[1] != NULL)
	{
		group->motor[1]->single_init = dm_motor_init;
		group->motor[1]->single_init(group->motor[1]);
		num_init++;
	}

  if(group->motor[2] != NULL)
	{
		group->motor[2]->single_init = dm_motor_init;
		group->motor[2]->single_init(group->motor[2]);
		num_init++;
	}
	  
	if(group->motor[3] != NULL)
	{
		group->motor[3]->single_init = dm_motor_init;
		group->motor[3]->single_init(group->motor[3]);
		num_init++;
	}
	  
		group->motor_num = num_init;
	  group->group_set_torque = Group_Motor_Set_Torque;
		group->group_heartbeat = Group_Motor_Heartbeat;
	  group->group_sleep = Group_Motor_Sleep;
}

/* 组控制 */
/* 下发命令帧 */
/* 发送达妙模式切换命令 */
static void Motor_Send_Command(dm_motor_t *motor, mit_cmd_t Command)
{
	switch(Command)
	{
		case Enter_Motor_Mode:
		Motor_Command[7] = 0xFC; /* 进入电机模式 */
		break;
		case Exit_Motor_Mode:
		Motor_Command[7] = 0xFD; /* 退出电机模式 */
		break;
		case Zero_Position_Sensor:
		Motor_Command[7] = 0xFE; /* 设置零位 */
		break;
		default:
		break;
	}
	Motor_Send_Data(motor, Motor_Command);
}

/* 组装并发送控制帧 */
static void Motor_Send_Data(dm_motor_t *motor, uint8_t* buf)
{
	dm_cfg_t* motor_born_info = motor->born_info;
	
	CAN_SendData(motor_born_info->hcan, motor_born_info->stdId, buf);
}

/* 计算控制量 */
static void Motor_SetControlPara(dm_motor_t *motor)
{
	dm_tx_t* motor_tx_info = motor->tx_info;
	uint16_t p, v, kp, kd, t; /* MIT 协议压缩值 */
  uint8_t* buf = motor_tx_info->single_tx_buff;
	
/* 参数在定义范围内 */
	motor_tx_info->target_angle = constrain(motor_tx_info->target_angle, P_MIN, P_MAX); /* 位置限幅 */
	motor_tx_info->target_speed = constrain(motor_tx_info->target_speed, V_MIN, V_MAX);
	motor_tx_info->Kp = constrain(motor_tx_info->Kp, KP_MIN, KP_MAX);
	motor_tx_info->Kd = constrain(motor_tx_info->Kd, KD_MIN, KD_MAX);
	motor_tx_info->torque = constrain(motor_tx_info->torque, T_MIN, T_MAX);
	
/* 按协议把浮点转成整型 */
	p = float_to_uint(motor_tx_info->target_angle,      P_MIN,  P_MAX,  16);            
	v = float_to_uint(motor_tx_info->target_speed,      V_MIN,  V_MAX,  12);
	kp = float_to_uint(motor_tx_info->Kp,    KP_MIN, KP_MAX, 12);
	kd = float_to_uint(motor_tx_info->Kd,    KD_MIN, KD_MAX, 12);
	t = float_to_uint(motor_tx_info->torque,      T_MIN,  T_MAX,  12);
	
/* 按协议把数据打包进 CAN 报文 */
	buf[0] = p>>8;
	buf[1] = p&0xFF;
	buf[2] = v>>4;
	buf[3] = ((v&0xF)<<4)|(kp>>8); /* 速度低 4 位 + Kp 高 4 位 */
	buf[4] = kp&0xFF;
	buf[5] = kd>>4;
	buf[6] = ((kd&0xF)<<4)|(t>>8); /* Kd 低 4 位 + 力矩高 4 位 */
	buf[7] = t&0xff;
	
	Motor_Send_Data(motor, buf);
}

/* 浮点转定长整型 */
/* 按量程把浮点压缩为定长整数 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    
    return (uint16_t) ((x-offset)*((float)((1<<bits)-1))/span);
}

/* 定长整型转浮点 */
/* 按量程把定长整数还原为浮点 */
static float uint_to_float(uint16_t x_int, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/* 累计角度换算 */
/* 累计多圈角度，处理 ±pi 跨越 */
static void Angle_Sum_Cal(dm_motor_t *motor)
{
	float err = 0.f; /* 相邻角度差 */
	
	float order_correction = 0.f; /* 方向修正 */
	
	if(motor->born_info->order_correction == 1 || motor->born_info->order_correction == -1)
	{
		order_correction = (float)motor->born_info->order_correction;
	}
	else
	{
		order_correction = 1.f;
	}
	
	if(!motor->rx_info->motor_angle_last && !motor->rx_info->motor_angle_sum)  // 首次上电角度累计值为0, 特殊处理
	{
		err = 0.f;
	}
	else
	{
		err = motor->rx_info->motor_angle - motor->rx_info->motor_angle_last; /* 本拍角度增量 */
	}
	
	if(abs(err) > (float)PI)  // 圈数处理
	{
		if(err > 0.f)
		{
			motor->rx_info->motor_angle_sum += (-(float)PI * 2.f + err) * order_correction; /* 正过零补偿 */
		}
		else
		{
			motor->rx_info->motor_angle_sum += ((float)PI * 2.f + err) * order_correction; /* 负过零补偿 */
		}
	}
	else
	{
		motor->rx_info->motor_angle_sum += err * order_correction;
	}
	
	motor->rx_info->motor_angle_last = motor->rx_info->motor_angle; /* 保存上拍角度 */
}

/* 电机错误码解析 */
/* 将达妙错误码映射为电机状态 */
static void Motor_ERR_Check(dm_motor_t *motor, uint8_t err_word)
{
	dm_state_t* my_state = motor->state;
//	static dm_err_t temp_state = Motor_Unenable;
	switch(err_word)
	{
		case 0:
		my_state->motor_state = Motor_Unenable; /* 未使能 */
		break;
		case 1:
		my_state->motor_state = Motor_Enable; /* 已使能 */
		break;
		case 8:
		my_state->motor_state = Over_Voltage; /* 过压 */
		break;
		case 9:
		my_state->motor_state = Lack_Voltage; /* 欠压 */
		break;
		case 10:
		my_state->motor_state = Over_Current; /* 过流 */
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
/* 取与上一拍不同的电机状态 */
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

