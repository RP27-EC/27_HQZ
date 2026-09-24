/* board_protocol.c - 板间通信协议 */

#include "board_protocol.h"
#include "judge.h"
#include "rc_sensor.h"
#include "string.h"
#include <stdbool.h>
#include "drv_can.h"
#include "rp_device_config.h"
#include "rp_math.h"
#include "main.h"
#include "chassis_config.h"
#include "chassis_follow.h"
#include "chassis_spin.h"


Board_Tx_Pkt_t    board_tx_pkt;
Board_Rx_Meg_t    board_rx_meg;

Board_Status_t board_status = 
{
	.offline_cnt_max = BOARD_OFFLINE_CNT_MAX,
};

Board_t board = 
{
	.tx_pkt = &board_tx_pkt,
	.rx_meg = &board_rx_meg,
	.status = &board_status,
	
	.init = Board_Init,
};



void Board_Init(Board_t* board)
{
	board->status->offline_cnt = board->status->offline_cnt_max;
	board->status->status = DEV_OFFLINE;
	board->status->gimbal_rx_time_ms = 0u;
	board->status->gimbal_data_valid = 0u;
	
	board->tx_01 = Board_Tx_Pkt_01;
	board->tx_02 = Board_Tx_Pkt_02;
	board->tx_03 = Board_Tx_Pkt_03;
	board->tx_04 = Board_Tx_Pkt_04;
	board->tx_05 = Board_Tx_Pkt_05;
	
	board->rx_01 = Board_Rx_Meg_01;
	board->rx_02 = Board_Rx_Meg_02;
	
	board->heartbeat = Board_Heart_Beat;
}


void Board_Heart_Beat(Board_t* board)
{
	board->status->offline_cnt ++;
	
	if(board->status->offline_cnt >= board->status->offline_cnt_max)
	{
		board->status->offline_cnt = board->status->offline_cnt_max;
		board->status->status = DEV_OFFLINE;
	}
	else
	{
		board->status->status = DEV_ONLINE;
	}
}



uint8_t  pkt_01[8];
uint8_t  pkt_02[8];
uint8_t  pkt_03[8];
uint8_t  pkt_04[8];
uint8_t  pkt_05[8];

void Board_Tx_Pkt_01(Board_t* board)
{
	memset(pkt_01, 0, 8);
	
	pkt_01[0] |= (board->tx_pkt->car_pkt.car_state & 0x03) << 0;
	pkt_01[0] |= (board->tx_pkt->car_pkt.gimbal_mode & 0x01) << 2;
	pkt_01[0] |= (board->tx_pkt->car_pkt.vision_mode & 0x07) << 3;
	pkt_01[0] |= (board->tx_pkt->car_pkt.game_start & 0x01) << 6;
	pkt_01[0] |= (board->tx_pkt->car_pkt.my_color & 0x01) << 7;
	
	uint16_t t1,t2;
	
	t1 = float_to_uint(board->tx_pkt->car_pkt.v_x,-8000.f,8000.f,16);      //pitch陀螺仪模式目标角度
	t2 = float_to_uint(board->tx_pkt->car_pkt.v_y,-8000.f,8000.f,16);  
	
	pkt_01[1] = t1>>8;
	pkt_01[2] = t1;
	pkt_01[3] = t2>>8;
	pkt_01[4] = t2;

									 
	pkt_01[5] |= (board->tx_pkt->shoot_pkt.launch_state & 0x01) << 0;
	pkt_01[5] |= (board->tx_pkt->shoot_pkt.shoot_mode & 0x01) << 1;
	pkt_01[5] |= (board->tx_pkt->shoot_pkt.shoot_level & 0x01) << 2;
	pkt_01[5] |= (board->tx_pkt->gimbal_target_pkt.is_hole & 0x01) << 3;
	

	CAN_SendData(&hfdcan2, ID_PKT_01, pkt_01);
	
//	board->status->offline_cnt ++;
	
}

void Board_Tx_Pkt_02(Board_t* board)
{
	uint16_t t1,t2,t3,t4;
	
	t1 = float_to_uint(board->tx_pkt->gimbal_target_pkt.pitch_imu_tar,-360.f,360.f,16);      //pitch陀螺仪模式目标角度
	t2 = float_to_uint(board->tx_pkt->gimbal_target_pkt.yaw_imu_tar,-360.f,360.f,16);  
	t3 = float_to_uint(board->tx_pkt->gimbal_target_pkt.pitch_mec_tar,-4.f,4.f,16);     //pitch机械模式目标角度
	t4 = float_to_uint(board->tx_pkt->gimbal_target_pkt.yaw_mec_tar,-4.f,4.f,16);        //ywa轴发射后角度偏移
	  
	pkt_02[0] = t1>>8;
	pkt_02[1] = t1;
	pkt_02[2] = t2>>8;
	pkt_02[3] = t2;
	pkt_02[4] = t3>>8;
	pkt_02[5] = t3;
	pkt_02[6] = t4>>8;
	pkt_02[7] = t4;
	

		CAN_SendData(&hfdcan2, ID_PKT_02, pkt_02);

	
//	board->status->offline_cnt ++;
	
}

void Board_Tx_Pkt_03(Board_t* board)
{
	uint16_t t1,t2;
	
	t1 = float_to_uint(board->tx_pkt->judge_shoot_pkt.shoot_speed,-50.f,50.f,16);    
	t2 = float_to_uint(board->tx_pkt->judge_shoot_pkt.shoot_freq,-50.f,50.f,16);  
	
	board->tx_pkt->judge_shoot_pkt.shoot_heat_err = judge.pkt->shooter_barrel_heat_limit - judge.pkt->shooter_17mm_1_barrel_heat;
	
	pkt_03[0] = t1>>8;
	pkt_03[1] = t1;
	pkt_03[2] = t2>>8;
	pkt_03[3] = t2;
	pkt_03[4] = board->tx_pkt->judge_shoot_pkt.shoot_heat_err>>8;
	pkt_03[5] = board->tx_pkt->judge_shoot_pkt.shoot_heat_err;
	pkt_03[6] = judge.info->robot_status.shooter_barrel_cooling_value>>8;
	pkt_03[7] = judge.info->robot_status.shooter_barrel_cooling_value;
	

	CAN_SendData(&hfdcan2, ID_PKT_03, pkt_03);
	
//	board->status->offline_cnt ++;
	
}


void Board_Tx_Pkt_04(Board_t* board)
{
	for(uint8_t i = 0;i<8;i++)
	{
	  pkt_04[i] = board->tx_pkt->blood_pkt.blood[i];
	}

	CAN_SendData(&hfdcan2, ID_PKT_04, pkt_04);
	
//	board->status->offline_cnt ++;
	
}


static float Board_Remote_Axis_To_Rate(int16_t axis, float max_rate)
{
    float value = (float)axis;

    if ((value > -BOARD_RC_AXIS_DEADBAND) && (value < BOARD_RC_AXIS_DEADBAND))
    {
        return 0.0f;
    }

    value -= (value > 0.0f) ? BOARD_RC_AXIS_DEADBAND : -BOARD_RC_AXIS_DEADBAND;
    value /= (BOARD_RC_AXIS_MAX - BOARD_RC_AXIS_DEADBAND);

    if (value > 1.0f)
    {
        value = 1.0f;
    }
    else if (value < -1.0f)
    {
        value = -1.0f;
    }

    return value * max_rate;
}

void Board_Tx_Pkt_05(Board_t* board)
{
    float yaw_rate = 0.0f;
    float pitch_rate = 0.0f;
    int16_t yaw_raw;
    int16_t pitch_raw;
    uint8_t valid = 0u;

    if (rc_dev.work_state == DEV_ONLINE)
    {
        valid = 1u;
#if CHASSIS_BRINGUP_ENABLE
        if (rc_dev.info->s1.value == RC_SW_DOWN)
        {
            yaw_rate = Board_Remote_Axis_To_Rate(rc_dev.info->ch0,
                                                 BOARD_D5_YAW_RATE_MAX_DEG_S);
        }
        else if (Chassis_Follow_IsSelected() != 0u)
        {
            yaw_rate = Board_Remote_Axis_To_Rate(rc_dev.info->ch0,
                                                 BOARD_D5_YAW_RATE_MAX_DEG_S);
        }
        else if (Chassis_Spin_IsSelected() != 0u)
        {
            yaw_rate = Board_Remote_Axis_To_Rate(rc_dev.info->ch0,
                                                 BOARD_D5_YAW_RATE_MAX_DEG_S);
        }
        else
        {
#if CHASSIS_OWNS_RC_YAW
            yaw_rate = 0.0f;
#else
            yaw_rate = Board_Remote_Axis_To_Rate(rc_dev.info->ch0,
                                                 BOARD_D5_YAW_RATE_MAX_DEG_S);
#endif
        }
#else
        yaw_rate = Board_Remote_Axis_To_Rate(rc_dev.info->ch0,
                                             BOARD_D5_YAW_RATE_MAX_DEG_S);
#endif
        pitch_rate = Board_Remote_Axis_To_Rate(rc_dev.info->ch1,
                                               BOARD_D5_PITCH_RATE_MAX_DEG_S);
    }

    yaw_raw = (int16_t)(yaw_rate / BOARD_D5_RATE_LSB_DEG_S);
    pitch_raw = (int16_t)(pitch_rate / BOARD_D5_RATE_LSB_DEG_S);

    memset(pkt_05, 0, 8);
    pkt_05[0] = valid & 0x01u;
    pkt_05[1] = 0u;
    pkt_05[2] = (uint8_t)((uint16_t)yaw_raw >> 8);
    pkt_05[3] = (uint8_t)yaw_raw;
    pkt_05[4] = (uint8_t)((uint16_t)pitch_raw >> 8);
    pkt_05[5] = (uint8_t)pitch_raw;
    pkt_05[6] = 0u;
    pkt_05[7] = 0u;

    CAN_SendData(&hfdcan2, ID_PKT_05, pkt_05);
}

void Board_Rx_Meg_01(Board_t* board,uint8_t* rxbuf)
{
	board->rx_meg->state_meg.yaw_motor_state= (rxbuf[0] >> 0) & 0x01;
	board->rx_meg->state_meg.pitch_motor_state= (rxbuf[0] >> 1) & 0x01;
	board->rx_meg->state_meg.height_motor_state= (rxbuf[0] >> 2) & 0x01;
	board->rx_meg->state_meg.r_fric_state= (rxbuf[0] >> 3) & 0x01;
	board->rx_meg->state_meg.l_fric_state= (rxbuf[0] >> 4) & 0x01;
	board->rx_meg->state_meg.dial_motor_state= (rxbuf[0] >> 5) & 0x01;
	board->rx_meg->state_meg.vision_state= (rxbuf[0] >> 6) & 0x01;
	board->rx_meg->state_meg.is_down= rxbuf[1];
	
	uint16_t t1 = ((uint16_t)rxbuf[2] << 8) | rxbuf[3];  
  uint16_t t2 = ((uint16_t)rxbuf[4] << 8) | rxbuf[5];

  board->rx_meg->vision_meg.vision_yaw_tar = uint_to_float(t1, -360.0f, 360.0f,16);
  board->rx_meg->vision_meg.vision_pitch_tar = uint_to_float(t2, -360.0f, 360.0f,16);
	board->rx_meg->vision_meg.is_find_target = (rxbuf[6] >> 0) & 0x01;
	

 
	
	board->status->offline_cnt = 0;
}


void Board_Rx_Meg_02(Board_t* board,uint8_t* rxbuf)
{
  uint16_t t1 = ((uint16_t)rxbuf[0] << 8) | rxbuf[1];  
  uint16_t t2 = ((uint16_t)rxbuf[2] << 8) | rxbuf[3];
  uint16_t t3 = ((uint16_t)rxbuf[4] << 8) | rxbuf[5];
  uint16_t t4 = ((uint16_t)rxbuf[6] << 8) | rxbuf[7];
    
  board->rx_meg->gimbal_meg.yaw_mec     = uint_to_float(t1, -4.f, 4.f,16);
  board->rx_meg->gimbal_meg.pitch_mec   = uint_to_float(t2, -4.f, 4.f,16);
  board->rx_meg->gimbal_meg.yaw_imu     = uint_to_float(t3, -360.0f, 360.0f,16);
  board->rx_meg->gimbal_meg.pitch_imu   = uint_to_float(t4, -360.0f, 360.0f,16);

  board->status->gimbal_rx_time_ms = HAL_GetTick();
  board->status->gimbal_data_valid = 1u;

	board->status->offline_cnt = 0;
}




