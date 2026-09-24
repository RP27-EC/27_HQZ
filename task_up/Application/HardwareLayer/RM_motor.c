/* RM_motor.c - RM 电机驱动 */

/**
  ******************************************************************************
  * @file    RM_motor.c
 * @brief   电机卸力
  * @version 
  * @date    
  ******************************************************************************
  * @attention
  * 
  ******************************************************************************
  */
#include "rm_motor.h"
#include "pid.h"
#include "arm_math.h"
#include "rp_math.h"
static uint16_t CAN_01_GetMotorAngle(uint8_t *rxData);
static int16_t CAN_23_GetMotorSpeed(uint8_t *rxData);
static int16_t CAN_45_GetMotorCurrent(uint8_t *rxData);
static uint8_t CAN_6_GetMotorTemperature(uint8_t *rxData);
static void Torque_to_Raw_Current(rm_motor_t *motor);
static void Angle_Sum_Cal(rm_motor_t *motor);
static void Encoder_to_Motor_Angle(rm_motor_t *motor);
static float RPM_to_Rads(rm_motor_t *motor);
static void Raw_Current_to_Torque(rm_motor_t* motor);
static void Encoder_Sum_Cal(rm_motor_t *motor);
/* 单电机函数 */
/* 单电机力矩模式：力矩转电流后组帧发送 */
static void Motor_Set_Torque(rm_motor_t *motor)
{
	uint8_t Id = motor->born_info->rxId*2; /* CAN 电流字节偏移 */
	Torque_to_Raw_Current(motor);
	motor->tx_info->tx_buff[Id] = (uint8_t)(motor->tx_info->torque_current_raw >> 8); /* 电流高字节 */
	motor->tx_info->tx_buff[Id+1] = (uint8_t)(motor->tx_info->torque_current_raw);
	CAN_SendData(motor->born_info->hcan, motor->born_info->stdId, motor->tx_info->tx_buff);
}

/* 串级控制计算 */
static void Motor_Ctrl(rm_motor_t *motor)
{
	uint8_t Id = motor->born_info->rxId*2;
	motor->tx_info->tx_buff[Id] = (uint8_t)(motor->tx_info->torque_current_raw >> 8);
	motor->tx_info->tx_buff[Id+1] = (uint8_t)(motor->tx_info->torque_current_raw);
	CAN_SendData(motor->born_info->hcan, motor->born_info->stdId, motor->tx_info->tx_buff);
}

/* 单电机卸力 */
/* 单电机卸力：清零力矩并发送 */
static void Single_Motor_Sleep(rm_motor_t *motor)
{
	motor->tx_info->torque = 0;
	motor->single_set_torque(motor);
}

/* 速度环输出 */
static void Motor_Set_Speed(rm_motor_t *motor)
{
	pid_ctrl_t* my_speed_ctrl = motor->ctrl->speed_ctrl; /* 速度环 */
	my_speed_ctrl->measure = motor->rx_info->encoder_speed; /* 反馈转速 */
	my_speed_ctrl->err=my_speed_ctrl->target-my_speed_ctrl->measure; /* 速度误差 */
	single_pid_ctrl(my_speed_ctrl);
	motor->tx_info->torque = my_speed_ctrl->out;
}


/* 角度环输出 */
static void Motor_Set_Angle(rm_motor_t *motor)
{
	pid_ctrl_t* my_angle_ctrl = motor->ctrl->angle_ctrl_outer; /* 外环角度 */
	pid_ctrl_t* my_speed_ctrl = motor->ctrl->angle_ctrl_inner; /* 内环速度 */
/* 外环参数 */
	if(motor->ctrl->Angle_Input_Flag == false)
	{
	my_angle_ctrl->measure = motor->rx_info->encoder;
	}
	my_angle_ctrl->err=my_angle_ctrl->target-my_angle_ctrl->measure; /* 角度误差 */
	if(motor->ctrl->Nearest_Return == true)
	{
		if(motor->ctrl->Angle_Input_Flag == false)
		{
			if(my_angle_ctrl->err < 0)
			{
				my_angle_ctrl->err += 8192;
			}
			if(my_angle_ctrl->err > 4096)
			{
				my_angle_ctrl->err -= 8192;
			}
		}
		else
		{
			if(my_angle_ctrl->err < -180.f)
			{
				my_angle_ctrl->err += 360.f;
			}
			if(my_angle_ctrl->err > 180.f)
			{
				my_angle_ctrl->err -= 360.f;
			}
		}
	}
	single_pid_ctrl(my_angle_ctrl);
	
	my_speed_ctrl->target = my_angle_ctrl->out; /* 外环输出作速度目标 */
	if(motor->ctrl->Speed_Input_Flag == false)
	my_speed_ctrl->measure = motor->rx_info->encoder_speed;
	my_speed_ctrl->err=my_speed_ctrl->target-my_speed_ctrl->measure; /* 速度误差 */
	single_pid_ctrl(my_speed_ctrl);
	motor->tx_info->torque = my_speed_ctrl->out;
}

/* 离线计数检测 */
static void rm_motor_heart_beat(rm_motor_t *motor)
{
    rm_state_t *motor_state = motor->state;
    motor_state->offline_cnt++;
    if(motor_state->offline_cnt > motor_state->offline_cnt_max) 
	{
        motor_state->offline_cnt = motor_state->offline_cnt_max;
        motor_state->status = DEV_OFFLINE;
    }
    else 
	{
        if(motor_state->status == DEV_OFFLINE)
            motor_state->status = DEV_ONLINE;
    }
}

/* 解算反馈数据 */
static void rm_motor_update(rm_motor_t *rm_motor, uint8_t *rxBuf)
{
    rm_rx_t *motor_info = rm_motor->rx_info;
    
    motor_info->encoder = CAN_01_GetMotorAngle(rxBuf);
		Encoder_Sum_Cal(rm_motor);
		Encoder_to_Motor_Angle(rm_motor);
		motor_info->encoder_speed = CAN_23_GetMotorSpeed(rxBuf);
		motor_info->speed = RPM_to_Rads(rm_motor);
		motor_info->torque_current_raw = CAN_45_GetMotorCurrent(rxBuf);
		Raw_Current_to_Torque(rm_motor);
    motor_info->temperature = CAN_6_GetMotorTemperature(rxBuf);
    rm_motor->state->offline_cnt = 0;
}

/* 绑定接口并复位状态 */
void rm_motor_init(rm_motor_t *motor)
{
	motor->single_set_torque = Motor_Set_Torque;
	motor->single_heart_beat = rm_motor_heart_beat;
	motor->single_sleep = Single_Motor_Sleep;
	motor->single_set_speed = Motor_Set_Speed;
	motor->single_set_angle = Motor_Set_Angle;
	motor->single_ctrl = Motor_Ctrl;
	motor->rx = rm_motor_update;
	motor->state->offline_cnt_max = 100;
}

/* 组控制 */
/* 组内依次发送力矩 */
static void Group_Motor_Set_Torque(rm_group_t *group)
{	
		int16_t torque_current = 0;
		uint8_t Id;
		for(uint8_t i = 0; i < 4; i ++)
		{
			if(group->motor[i] != NULL)
			{
				Id = group->motor[i]->born_info->rxId*2;
				Torque_to_Raw_Current(group->motor[i]);
				torque_current = group->motor[i]->tx_info->torque_current_raw;
				group->tx_buff[Id] = (uint8_t)(torque_current >> 8);
				group->tx_buff[Id+1] = (uint8_t)(torque_current);
//				group->motor[i]->tx_info->torque = 0;
			}
		}
		
		CAN_SendData(group->hcan, group->stdId, group->tx_buff);
		
		memset(group->tx_buff, 0, 8);
}

/* 组内串级控制 */
static void Group_Motor_Ctrl(rm_group_t *group)
{	
		int16_t torque_current = 0;
		uint8_t Id;
		for(uint8_t i = 0; i < 4; i ++)
		{
			if(group->motor[i] != NULL)
			{
				Id = group->motor[i]->born_info->rxId*2;
				torque_current = group->motor[i]->tx_info->torque_current_raw;
				group->tx_buff[Id] = (uint8_t)(torque_current >> 8);
				group->tx_buff[Id+1] = (uint8_t)(torque_current);
//				group->motor[i]->tx_info->torque = 0;
			}
		}
		
		CAN_SendData(group->hcan, group->stdId, group->tx_buff);
		
		memset(group->tx_buff, 0, 8);
}

/* 组内全部卸力 */
static void Group_Motor_Sleep(rm_group_t *group)
{		
		
	for(uint8_t i = 0; i < 4; i ++)
		{
			if(group->motor[i] != NULL)
			{
				group->motor[i]->tx_info->torque = 0;
			}
		}
		
}


/* 组内心跳检测 */
static void Group_Motor_Heartbeat(rm_group_t *group)
{
	for(uint8_t i = 0; i < 4; i ++)
	{
		if(group->motor[i] != NULL)
		{
			group->motor[i]->single_heart_beat(group->motor[i]);
		}
	}
}

/* 电机组初始化 */
void rm_group_init(rm_group_t *group)
{
		for(uint8_t i = 0; i < 4; i ++)
		{
			if(group->motor[i] != NULL)
			{
				group->motor[i]->single_init = rm_motor_init;
				group->motor[i]->single_init(group->motor[i]);
			}
		}
	
	  group->group_set_torque = Group_Motor_Set_Torque;
		group->group_heartbeat = Group_Motor_Heartbeat;
	  group->group_sleep = Group_Motor_Sleep;
		group->group_ctrl = Group_Motor_Ctrl;
}

/* 底层辅助 */
/* 解析角度反馈帧 */
static uint16_t CAN_01_GetMotorAngle(uint8_t *rxData)
{
	uint16_t angle; /* 0-8191 编码器值 */
	angle = (uint16_t)(rxData[0] << 8| rxData[1]); /* 大端拼接 */
	return angle;
}

/* 解析速度反馈帧 */
static int16_t CAN_23_GetMotorSpeed(uint8_t *rxData)
{
	int16_t speed; /* 转速，rpm */
	speed = (int16_t)(rxData[2] << 8| rxData[3]); /* 大端拼接 */
	return speed;
}

/* 解析电流反馈帧 */
static int16_t CAN_45_GetMotorCurrent(uint8_t *rxData)
{
	int16_t current; /* 原始电流 */
	current = (int16_t)(rxData[4] << 8 | rxData[5]); /* 大端拼接 */
	return current;
}


/* 解析温度反馈帧 */
static uint8_t CAN_6_GetMotorTemperature(uint8_t *rxData)
{
	uint8_t temp; /* 摄氏度 */
	temp = rxData[6]; /* 温度字节 */
	return temp;
}

/* 力矩转原始电流值 */
static void Torque_to_Raw_Current(rm_motor_t *motor)
{
		switch(motor->born_info->type)
		{
			// 3508 用恒定转矩输出, 实际力矩由 motor->tx_info->torque 决定
			case _3508_Single:
			motor->tx_info->torque_current = motor->tx_info->torque;
			motor->tx_info->torque_current = constrain(motor->tx_info->torque_current, -16384, 16384);  // 限幅
/* 3508 转矩电流转成整型值 */
			motor->tx_info->torque_current_raw = (int16_t)((motor->tx_info->torque_current));
			break;
			case _3508_Reduction:
			motor->tx_info->torque_current = motor->tx_info->torque / _3508_TORQUE_CONSTANT;
			motor->tx_info->torque_current = constrain(motor->tx_info->torque_current, -_3508_MAX_CURRENT*0.9f, _3508_MAX_CURRENT*0.9f);  // 限幅
/* 3508 减速箱转矩电流转成整型值 */
			motor->tx_info->torque_current_raw = (int16_t)((motor->tx_info->torque_current / _3508_MAX_CURRENT) * 16384.f);
			break;
			case _2006_Single:
			motor->tx_info->torque_current = motor->tx_info->torque;
			motor->tx_info->torque_current = constrain(motor->tx_info->torque_current, -10000, 10000);  // 限幅
/* 2006 转矩电流转成整型值 */
			motor->tx_info->torque_current_raw = (int16_t)((motor->tx_info->torque_current));
			break;
			case _6020_Single:
			motor->tx_info->torque_current = motor->tx_info->torque / _6020_TORQUE_CONSTANT;
			motor->tx_info->torque_current = constrain(motor->tx_info->torque_current, -_6020_MAX_CURRENT, _6020_MAX_CURRENT);  // 限幅
/* 6020 转矩电流转成整型值 */
			motor->tx_info->torque_current_raw = (int16_t)((motor->tx_info->torque_current));
			break;
		}
		

}


/* 编码器累计圈数 */
static void Encoder_Sum_Cal(rm_motor_t *motor)
{
	int16_t err; /* 相邻编码器差值 */
	rm_rx_t *motor_info = motor->rx_info;
	
/* 未初始化 */
	if(motor_info->motor_angle_last == 0 && motor_info->encoder_sum == 0)
	{
		err = 0;
	}
	else
	{
		err = motor_info->encoder - motor_info->encoder_last; /* 编码器增量 */
	}
	
/* 过零点 */
	if(abs(err) > 4095)
	{
/* 0 -> 8191 */
		if(err >= 0)
			motor_info->encoder_sum += -8191 + err; /* 跨零点累加 */
/* 8191 -> 0 */
		else
			motor_info->encoder_sum += 8191 + err; /* 跨零点累加 */
	}
/* 未跨零点 */
	else
	{
		motor_info->encoder_sum += err; /* 普通累加 */
	}
	
	motor_info->encoder_last = motor_info->encoder; /* 保存本拍编码器 */
}


/* 编码器角度换算 */
static void Encoder_to_Motor_Angle(rm_motor_t *motor)
{
	if(motor->born_info->type == _3508_Reduction)
	motor->rx_info->motor_angle = ((float)motor->rx_info->encoder / 8191.f) * (float)PI * 2.f / _3508_REDUCT_RATIO;
	else if(motor->born_info->type == _2006_Single)
	motor->rx_info->motor_angle = ((float)motor->rx_info->encoder / 8191.f) * (float)PI * 2.f / _2006_REDUCT_RATIO;
	else
	motor->rx_info->motor_angle = ((float)motor->rx_info->encoder / 8191.f) * (float)PI * 2.f;
	
	Angle_Sum_Cal(motor);
}

/* 累计角度换算 */
static void Angle_Sum_Cal(rm_motor_t *motor)
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
	
	if(!motor->rx_info->motor_angle_last && !motor->rx_info->motor_angle_sum)  // 首次上电角度累计值为0, 特殊处理
	{
		err = 0.f;
	}
	else
	{
		err = motor->rx_info->motor_angle - motor->rx_info->motor_angle_last;
	}
	
	if((abs(err) > ((float)PI)) || (abs(err) > ((float)PI/_3508_REDUCT_RATIO) && motor->born_info->type == _3508_Reduction))  // 圈数处理
	{
		if(err > 0.f)
		{
			if(motor->born_info->type == _3508_Reduction)
			motor->rx_info->motor_angle_sum += (-(float)PI * 2.f / _3508_REDUCT_RATIO + err) * order_correction;
			else
			motor->rx_info->motor_angle_sum += (-(float)PI * 2.f + err) * order_correction;
		}
		else
		{
			if(motor->born_info->type == _3508_Reduction)
			motor->rx_info->motor_angle_sum += ((float)PI * 2.f / _3508_REDUCT_RATIO + err) * order_correction;
			else
			motor->rx_info->motor_angle_sum += ((float)PI * 2.f + err) * order_correction;
		}
	}
	else
	{
		motor->rx_info->motor_angle_sum += err * order_correction;
	}
	
	motor->rx_info->motor_angle_last = motor->rx_info->motor_angle;
}

/* rpm 转 rad/s */
/* rpm 转 rad/s，并按型号换算到输出轴 */
static float RPM_to_Rads(rm_motor_t *motor)
{
	float ret; /* rad/s */
	if(motor->born_info->type == _3508_Reduction)
	ret= motor->rx_info->encoder_speed/60.f*2*PI/_3508_REDUCT_RATIO;
	else if(motor->born_info->type == _2006_Single)
	ret= motor->rx_info->encoder_speed/60.f*2*PI/_2006_REDUCT_RATIO;
	else
	ret= motor->rx_info->encoder_speed/60.f*2*PI;
	return ret;
}


/* 原始电流转力矩 */
static void Raw_Current_to_Torque(rm_motor_t* motor)
{
		motor->rx_info->torque_current = (motor->rx_info->torque_current_raw / 16384.f)*20.f; /* 原始值转电流 */
		motor->rx_info->torque = motor->rx_info->torque_current * _3508_TORQUE_CONSTANT; /* 电流转力矩 */
}
