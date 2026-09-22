/* BRT_code.c - 外设扩展模块驱动 */

#include "brt_code.h"
#include "rp_math.h"
#include "arm_math.h"
static void Code_Set_Command(Code_BRT_t* code, Code_BRT_Command_e command);
static void Code_Send_data(Code_BRT_t* code);
/* 读角度 */
static void Code_Read_Angle(Code_BRT_t* code)
{
	static uint8_t over_time = 0;

	if(code->tx_info->command_flag[Read_encoder] != 1)
	{
		Code_Set_Command(code, Read_encoder);
		code->tx_info->tx_buff[3] = 0x00;
		code->tx_info->command_flag[Read_encoder] = 1;
		Code_Send_data(code);
	}
	else
	{
		over_time ++;
		if(over_time > 100)
		{
			over_time = 0;
			code->tx_info->command_flag[Read_encoder] = 0;
		}
	}
}

/* 读速度 */
static void Code_Read_Speed(Code_BRT_t* code)
{
	static uint8_t over_time = 0;

	if(code->tx_info->command_flag[Read_speed] != 1)
	{
		Code_Set_Command(code, Read_speed);
		code->tx_info->tx_buff[3] = 0x00;
		code->tx_info->command_flag[Read_speed] = 1;
		Code_Send_data(code);
	}
	else
	{
		over_time ++;
		if(over_time > 100)
		{
			over_time = 0;
			code->tx_info->command_flag[Read_speed] = 0;
		}
	}
}

/* 读累计角度 */
static void Code_Read_Sum_Angle(Code_BRT_t* code)
{
	static uint8_t over_time = 0;

	if(code->tx_info->command_flag[Read_sum_encoder] != 1)
	{
		Code_Set_Command(code, Read_sum_encoder);
		code->tx_info->tx_buff[3] = 0x00;
		code->tx_info->command_flag[Read_sum_encoder] = 1;
		Code_Send_data(code);
	}
	else
	{
		over_time ++;
		if(over_time > 100)
		{
			over_time = 0;
			code->tx_info->command_flag[Read_sum_encoder] = 0;
		}
	}
}


/* 读圈数 */
static void Code_Read_Turn(Code_BRT_t* code)
{
	static uint8_t over_time = 0;

	if(code->tx_info->command_flag[Read_Turn] != 1)
	{
		Code_Set_Command(code, Read_Turn);
		code->tx_info->tx_buff[3] = 0x00;
		code->tx_info->command_flag[Read_Turn] = 1;
		Code_Send_data(code);
	}
	else
	{
		over_time ++;
		if(over_time > 100)
		{
			over_time = 0;
			code->tx_info->command_flag[Read_Turn] = 0;
		}
	}
}

/* 设置回传周期 */
static void Code_Set_Receive_Time(Code_BRT_t* code, uint16_t Time)
{
	static uint8_t over_time = 0;

	if(code->tx_info->command_flag[Set_Receive_Time] == 0)
	{
		Code_Set_Command(code, Set_Receive_Time);
		Time = constrain(Time, 50, 65535);
		code->tx_info->tx_buff[3] = (uint8_t)(Time);
		code->tx_info->tx_buff[4] = (uint8_t)(Time >> 8);
		code->tx_info->command_flag[Set_Receive_Time] = 1;
		Code_Send_data(code);
	}
	else if(code->tx_info->command_flag[Set_Receive_Time] == 1)
	{
		over_time ++;
		if(over_time > 100)
		{
			over_time = 0;
			code->tx_info->command_flag[Set_Receive_Time] = 0;
		}
	}
}


/* 设置编码器 ID */
static void Code_Set_ID(Code_BRT_t* code, uint8_t id)
{
	static uint8_t over_time = 0;

	if(code->tx_info->command_flag[Set_Id] == 0)
	{
		Code_Set_Command(code, Set_Id);
		id = constrain(id, 1, 255);
		code->tx_info->tx_buff[3] = (uint8_t)(id);
		code->tx_info->command_flag[Set_Id] = 1;
		code->tx_info->new_id = id;
		Code_Send_data(code);
	}
	else if(code->tx_info->command_flag[Set_Id] == 1)
	{
		over_time ++;
		if(over_time > 100)
		{
			over_time = 0;
			code->tx_info->command_flag[Set_Id] = 0;
		}
	}
	else if(code->tx_info->command_flag[Set_Id] == 2)
	{
		code->born_info->stdId = code->tx_info->new_id;
	}
	else
	{
		code->tx_info->command_flag[Set_Id] = 3;
	}
}

/* 设置波特率 */
static void Code_Set_Baud(Code_BRT_t* code, uint8_t Baud)
{
	static uint8_t over_time = 0;

	if(code->tx_info->command_flag[Set_Baud] == 0)
	{
		Code_Set_Command(code, Set_Baud);
		Baud = constrain(Baud, 0, 4);
		code->tx_info->tx_buff[3] = Baud;
		code->tx_info->command_flag[Set_Baud] = 1;
		Code_Send_data(code);
	}
	else if(code->tx_info->command_flag[Set_Baud] == 1)
	{
		over_time ++;
		if(over_time > 100)
		{
			over_time = 0;
			code->tx_info->command_flag[Set_Baud] = 0;
		}
	}
}

/* 设置零点 */
static void Code_Set_Zero_Point(Code_BRT_t* code)
{
	static uint8_t over_time = 0;

	if(code->tx_info->command_flag[Set_Zero_Pole] == 0)
	{
		Code_Set_Command(code, Set_Zero_Pole);
		code->tx_info->tx_buff[3] = 0;
		code->tx_info->command_flag[Set_Zero_Pole] = 1;
		Code_Send_data(code);
	}
	else if(code->tx_info->command_flag[Set_Zero_Pole] == 1)
	{
		over_time ++;
		if(over_time > 100)
		{
			over_time = 0;
			code->tx_info->command_flag[Set_Zero_Pole] = 0;
		}
	}
}

/* 设置中点 */
static void Code_Set_Mid_Point(Code_BRT_t* code)
{
	static uint8_t over_time = 0;

	if(code->tx_info->command_flag[Set_Mid_Pole] == 0)
	{
		Code_Set_Command(code, Set_Mid_Pole);
		code->tx_info->tx_buff[3] = 0;
		code->tx_info->command_flag[Set_Mid_Pole] = 1;
		Code_Send_data(code);
	}
	else if(code->tx_info->command_flag[Set_Mid_Pole] == 1)
	{
		over_time ++;
		if(over_time > 100)
		{
			over_time = 0;
			code->tx_info->command_flag[Set_Mid_Pole] = 0;
		}
	}
}

/* Code_Set_Num_Point */
static void Code_Set_Num_Point(Code_BRT_t* code, uint16_t encoder)
{
	static uint8_t over_time = 0;

	if(code->tx_info->command_flag[Set_Num_Pole] == 0)
	{
		Code_Set_Command(code, Set_Num_Pole);
		encoder = constrain(encoder, 0, 65535);
		code->tx_info->tx_buff[6] = (uint8_t)encoder;
		code->tx_info->tx_buff[5] = (uint8_t)(encoder >> 8);
		code->tx_info->command_flag[Set_Num_Pole] = 1;
		Code_Send_data(code);
	}
	else if(code->tx_info->command_flag[Set_Num_Pole] == 1)
	{
		over_time ++;
		if(over_time > 100)
		{
			over_time = 0;
			code->tx_info->command_flag[Set_Num_Pole] = 0;
		}
	}
}

/* 设置工作模式 */
static void Code_Set_Mode(Code_BRT_t* code, uint8_t Mode)
{
	static uint8_t over_time = 0;

	if(code->tx_info->command_flag[Set_Code_Mode] == 0)
	{
		Code_Set_Command(code, Set_Code_Mode);
		Mode = constrain(Mode, 0, 2);
		switch(Mode)
		{
			case 0:
			Mode = 0x00;
			break;
			case 1:
			Mode = 0x01;
			break;
			case 2:
			Mode = 0xAA;
			break;
		}
		code->tx_info->tx_buff[3] = Mode;
		code->tx_info->command_flag[Set_Code_Mode] = 1;
		Code_Send_data(code);
	}
	else if(code->tx_info->command_flag[Set_Code_Mode] == 1)
	{
		over_time ++;
		if(over_time > 100)
		{
			over_time = 0;
			code->tx_info->command_flag[Set_Code_Mode] = 0;
		}
	}
}

/* 设置方向 */
static void Code_Set_Dire(Code_BRT_t* code, uint8_t dire)
{
	static uint8_t over_time = 0;

	if(code->tx_info->command_flag[Set_Dire] == 0)
	{
		Code_Set_Command(code, Set_Dire);
		code->tx_info->tx_buff[3] = dire;
		code->tx_info->command_flag[Set_Dire] = 1;
		Code_Send_data(code);
	}
	else if(code->tx_info->command_flag[Set_Dire] == 1)
	{
		over_time ++;
		if(over_time > 100)
		{
			over_time = 0;
			code->tx_info->command_flag[Set_Dire] = 0;
		}
	}
}

/* 设置采样时间 */
static void Code_Set_Sample_Time(Code_BRT_t* code, uint16_t time)
{
	static uint8_t over_time = 0;

	if(code->tx_info->command_flag[Set_Sample_Time] == 0)
	{
		Code_Set_Command(code, Set_Sample_Time);
		code->tx_info->tx_buff[3] = (uint8_t)(time);
		code->tx_info->tx_buff[4] = (uint8_t)(time >> 8);
		code->tx_info->command_flag[Set_Sample_Time] = 1;
		Code_Send_data(code);
	}
	else if(code->tx_info->command_flag[Set_Sample_Time] == 1)
	{
		over_time ++;
		if(over_time > 100)
		{
			over_time = 0;
			code->tx_info->command_flag[Set_Sample_Time] = 0;
		}
	}
}

/* 编码器发送数据 */
static void Code_Send_data(Code_BRT_t* code)
{
	CAN_SendData(code->born_info->hcan, code->born_info->stdId, code->tx_info->tx_buff);
	memset(code->tx_info->tx_buff, 0, 8);
}


/* 刷新编码器数据 */
static void Code_Update(Code_BRT_t* code, uint8_t *rxBuf)
{
	switch(rxBuf[2])
	{
		case Read_encoder:
		code->rx_info->angle_raw = (uint16_t)((rxBuf[4] << 8) | rxBuf[3]);
		code->tx_info->command_flag[rxBuf[2]] = 2;
		break;
		case Read_sum_encoder:
		memcpy(&code->rx_info->sum_angle_raw, &rxBuf[3], 4);
		if(code->rx_info->sum_angle_raw > 1073741823)
		code->rx_info->sum_encoder = code->rx_info->sum_angle_raw-2147483647;
		else
		code->rx_info->sum_encoder = code->rx_info->sum_angle_raw;
		code->rx_info->sum_angle = code->rx_info->sum_encoder*2.f*PI/4096;
		code->tx_info->command_flag[rxBuf[2]] = 2;
		break;
		case Read_Turn:
		memcpy(&code->rx_info->sum_turn, &rxBuf[3], 4);
		if(code->rx_info->sum_turn > 262143)
		code->rx_info->sum_encoder = code->rx_info->sum_turn-524287;
		else
		code->rx_info->sum_encoder = code->rx_info->sum_turn;
		code->tx_info->command_flag[rxBuf[2]] = 2;
		break;
		case Read_speed:
		memcpy(&code->rx_info->speed_raw, &rxBuf[3], 4);
		code->tx_info->command_flag[rxBuf[2]] = 2;
		break;
		case Set_Id:
		if(rxBuf[1] == code->tx_info->new_id)
		code->tx_info->command_flag[rxBuf[2]] = 2;
		else
		code->tx_info->command_flag[rxBuf[2]] = 3;
		break;
		default:
		if(rxBuf[3] == 0)
		{
			code->tx_info->command_flag[rxBuf[2]] = 2;
		}
		else
		{
			code->tx_info->command_flag[rxBuf[2]] = 3;
		}
		break;
	}
	
	code->state->offline_cnt = 0;
}

/* BRT 编码器心跳 */
static void BRT_Code_Heart_Beat(Code_BRT_t *code)
{
    Code_BRT_State_t *code_state = code->state;
    code_state->offline_cnt++;
    if(code_state->offline_cnt > code_state->offline_cnt_max) 
	{
        code_state->offline_cnt = code_state->offline_cnt_max;
        code_state->status = DEV_OFFLINE;
    }
    else 
	{
        if(code_state->status == DEV_OFFLINE)
            code_state->status = DEV_ONLINE;
    }
}

/* BRT 编码器初始化 */
void BRT_Code_Init(Code_BRT_t* code)
{ 
	code->rx = Code_Update;
	code->receive_encoder = Code_Read_Angle;
	code->set_receive_time = Code_Set_Receive_Time;
	code->set_mode = Code_Set_Mode;
	code->receive_sum_encoder = Code_Read_Sum_Angle;
	code->set_id = Code_Set_ID;
	code->receive_speed = Code_Read_Speed;
	code->set_baud = Code_Set_Baud;
	code->set_zero = Code_Set_Zero_Point;
	code->receive_sum_turn = Code_Read_Turn;
	code->set_dire = Code_Set_Dire;
	code->set_sample_time = Code_Set_Sample_Time;
	code->set_encoder = Code_Set_Num_Point;
	code->set_mid = Code_Set_Mid_Point;
	code->state->offline_cnt_max = 100;
	code->single_heart_beat = BRT_Code_Heart_Beat;
}




/* 下发编码器命令 */
static void Code_Set_Command(Code_BRT_t* code, Code_BRT_Command_e command)
{
	switch(command)
	{
		case Read_encoder:
		code->tx_info->tx_buff[0] = 0x04;
		code->tx_info->tx_buff[1] = 0x01;
		code->tx_info->tx_buff[2] = 0x01;
		break;
		case Set_Id:
		code->tx_info->tx_buff[0] = 0x04;
		code->tx_info->tx_buff[1] = 0x01;
		code->tx_info->tx_buff[2] = 0x02;
		break;
		case Set_Baud:
		code->tx_info->tx_buff[0] = 0x04;
		code->tx_info->tx_buff[1] = 0x01;
		code->tx_info->tx_buff[2] = 0x03;
		break;
		case Set_Code_Mode:
		code->tx_info->tx_buff[0] = 0x04;
		code->tx_info->tx_buff[1] = 0x01;
		code->tx_info->tx_buff[2] = 0x04;
		break;
		case Set_Receive_Time:
		code->tx_info->tx_buff[0] = 0x05;
		code->tx_info->tx_buff[1] = 0x01;
		code->tx_info->tx_buff[2] = 0x05;
		break;
		case Set_Zero_Pole:
		code->tx_info->tx_buff[0] = 0x04;
		code->tx_info->tx_buff[1] = 0x01;
		code->tx_info->tx_buff[2] = 0x06;
		break;
		case Set_Dire:
		code->tx_info->tx_buff[0] = 0x04;
		code->tx_info->tx_buff[1] = 0x01;
		code->tx_info->tx_buff[2] = 0x07;
		break;
		case Read_sum_encoder:
		code->tx_info->tx_buff[0] = 0x04;
		code->tx_info->tx_buff[1] = 0x01;
		code->tx_info->tx_buff[2] = 0x08;
		break;
		case Read_Turn:
		code->tx_info->tx_buff[0] = 0x04;
		code->tx_info->tx_buff[1] = 0x01;
		code->tx_info->tx_buff[2] = 0x09;
		break;
		case Read_speed:
		code->tx_info->tx_buff[0] = 0x04;
		code->tx_info->tx_buff[1] = 0x01;
		code->tx_info->tx_buff[2] = 0x0A;
		break;
		case Set_Sample_Time:
		code->tx_info->tx_buff[0] = 0x05;
		code->tx_info->tx_buff[1] = 0x01;
		code->tx_info->tx_buff[2] = 0x0B;
		break;
		case Set_Mid_Pole:
		code->tx_info->tx_buff[0] = 0x04;
		code->tx_info->tx_buff[1] = 0x01;
		code->tx_info->tx_buff[2] = 0x0C;
		break;
		case Set_Num_Pole:
		code->tx_info->tx_buff[0] = 0x07;
		code->tx_info->tx_buff[1] = 0x01;
		code->tx_info->tx_buff[2] = 0x0D;
		break;
		default:
		break;
	};
}


