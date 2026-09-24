/* gimbal.c - 云台控制 */

#include "gimbal.h"
#include "vision.h"
#include "infantry.h"
#include "board_protocol.h"
#include "rp_math.h"
#include "rc_sensor.h"

static void Gimbal_Init(Gimbal_t* gimbal);
static void Gimbal_Data_Update(Gimbal_t* gimbal);
static void Gimbal_Status_Update(Gimbal_t* gimbal);
static void Gimbal_Slave_Update(Gimbal_t* gimbal);
static void Gimbal_Boss_Update(Gimbal_t* gimbal);
static void Gimbal_Offline_Update(Gimbal_t* gimbal);
static void Gimbal_Offline_Process(Gimbal_t* gimbal);
static void Gimbal_Cmd_Transmit(Gimbal_t* gimbal);
static void Gimbal_Work(Gimbal_t* gimbal);


/* 云台控制对象：默认休眠，标定四个朝向的 Yaw 零点 */
Gimbal_t gimbal = {
  .mode = G_SLEEP,
  .gimbal_reset_flag= false,
		
	.config = {
		.yaw_zero[FRONT] = YAW_MEC_ZERO_ANGLE,
		.yaw_zero[BEHIND] = 2.77879,
		.yaw_zero[RIGHT] = -1.95565724,
		.yaw_zero[LEFT] = 1.19649029,
	  .rc_yaw_imu_step = 0.2f,
	  .rc_yaw_mec_step = 0.02f,
	  .rc_pitch_mec_step = 0.002f,
	  .rc_pitch_imu_step = 0.05f,
	  .key_yaw_mec_step = 0.003f,
	  .key_pitch_mec_step = 0.0003f,
	  .key_yaw_imu_step = 0.003f,
	  .key_pitch_imu_step = 0.003f,
	  
	},
	
	.init = Gimbal_Init,
};



/* 绑定周期任务与心跳接口 */
static void Gimbal_Init(Gimbal_t* gimbal)
{
  gimbal->work = Gimbal_Work;
	gimbal->heart_beat = Gimbal_Offline_Update;
}

/* 将整车模式映射为云台模式，狗洞场景按头/底盘状态仲裁 */
static void Gimbal_Status_Update(Gimbal_t* gimbal)
{
	switch (infantry.mode)
	{
	  case I_SLEEP:
			gimbal->mode = G_SLEEP;
			break;
		
		case I_INIT:
			gimbal->mode = G_INIT;
			break;
		
		case I_MEC:
			gimbal->mode = G_SLAVE;
			break;
		
		//狗洞特殊处理，需根据头是否到位和狗洞标志位综合考虑其工作模式
		case I_HOLE:
			 if(infantry.flag.hole_flag == true && infantry.flag.chassis_reset.value == true)
			{
				gimbal->mode = G_BOSS;
			}
			else if(infantry.flag.hole_flag == true && infantry.flag.chassis_reset.value == false)
			{
				gimbal->mode = G_SLAVE;
			}
			else if(infantry.flag.hole_flag == false && board.rx_meg->state_meg.is_down == 2)
			{
				//退出狗洞
				gimbal->mode = G_BOSS;
				infantry.mode = I_IMU;
			}
			else if(infantry.flag.hole_flag == false && board.rx_meg->state_meg.is_down != 2)
			{
				gimbal->mode = G_SLAVE;
			}
			break;
			
		case I_IMU:
		case I_TURN:
			gimbal->mode = G_BOSS;
			break;
		
		default:
			break;
	}
	
}

/* 刷新云台反馈并计算机械零点误差，Yaw IMU 可选本地或上板 */
static void Gimbal_Data_Update(Gimbal_t* gimbal)
{
  gimbal->info.yaw_mec = board.rx_meg->gimbal_meg.yaw_mec;

	#if GIMBAL_SWITCH == 0
	  gimbal->info.yaw_imu = imu_dev.info->base_info.yaw;
	#else
	  gimbal->info.yaw_imu = board.rx_meg->gimbal_meg.yaw_imu;
	#endif
	
	gimbal->info.pitch_mec = board.rx_meg->gimbal_meg.pitch_mec;
	gimbal->info.pitch_imu = board.rx_meg->gimbal_meg.pitch_imu;

	gimbal->info.yaw_mec_err_raw = motor_half_cycle(YAW_MEC_ZERO_ANGLE - gimbal->info.yaw_mec,2*PI);
	
	gimbal->info.pitch_mec_err_raw = motor_half_cycle(PITCH_MEC_ZERO_ANGLE - gimbal->info.pitch_mec ,2*PI);
	
//	gimbal->info.yaw_mec_err_act = motor_half_cycle(gimbal->info.yaw_mec - gimbal->target.yaw_mec_tar,2*PI);
};
	

static uint16_t reset_tick = 0;
/* 上电回中：机械误差进入容差或超时后置复位完成标志 */
static void Gimbal_Init_Process(Gimbal_t* gimbal)
{
	gimbal->target.yaw_mec_tar = YAW_MEC_ZERO_ANGLE;
	gimbal->target.pitch_mec_tar = PITCH_MEC_ZERO_ANGLE;
	
	gimbal->target.yaw_imu_tar = gimbal->info.yaw_imu;
	gimbal->target.pitch_imu_tar = gimbal->info.pitch_imu;
	
	reset_tick ++;
	
	if(abs(gimbal->info.yaw_mec_err_raw) <= 0.3f/180.f*PI && abs(gimbal->info.pitch_mec_err_raw) <= 0.3f/180.f*PI\
		&& board.rx_meg->state_meg.is_down == 2)
	{
		gimbal->gimbal_reset_flag = true;
//		board.tx_pkt->gimbal_target_pkt.is_hole = true;
		reset_tick = 0;
	}
//	else if(board.rx_meg->state_meg.is_down == 1)
//	{
////		gimbal->gimbal_reset_flag = true;
////		reset_tick = 0;
//	}
	else if(reset_tick >= 8000)
	{
		gimbal->gimbal_reset_flag = true;
		reset_tick = 0;
	}
}


static void Gimbal_Direct_Update(Gimbal_t* gimbal)
{
	if(abs(gimbal->info.yaw_mec_err_raw) <= PI/4)
	{
		
	}
	else if(abs(gimbal->info.yaw_mec_err_raw) >= 3*PI/4)
	{
		
	}
	else if(gimbal->info.yaw_mec_err_raw > PI/4 && gimbal->info.yaw_mec_err_raw < 3*PI/4)
	{
		
	}
	else if(gimbal->info.yaw_mec_err_raw < -PI/4 && gimbal->info.yaw_mec_err_raw > -3*PI/4)
	{
		
	}
}



/* 从控模式：锁机械角，过洞压低，掉头切换前后零点 */
static void  Gimbal_Slave_Update(Gimbal_t* gimbal)
{
	
	if(gimbal->gimbal_reset_flag == false || gimbal->mode == G_INIT)
	{
		gimbal->target.yaw_mec_tar = YAW_MEC_ZERO_ANGLE;
		gimbal->target.pitch_mec_tar = PITCH_MEC_ZERO_ANGLE;
	
	}
	
	else{
		//过洞处理
		if(board.rx_meg->state_meg.is_down != 2 || infantry.mode == I_HOLE)
		{
			gimbal->target.yaw_mec_tar = YAW_MEC_ZERO_ANGLE;
			gimbal->target.pitch_mec_tar = PITCH_MEC_ZERO_ANGLE + 5 / 180 * PI;
		}
		//掉头处理
		else if(infantry.flag.U_turn_flag.value == true)
		{
			if(infantry.flag.U_turn_flag.form == RISING)
			{
				if(gimbal->target.yaw_mec_tar == gimbal->config.yaw_zero[FRONT])
			  {
				  gimbal->target.yaw_mec_tar = gimbal->config.yaw_zero[BEHIND];
			  }
			  else if(gimbal->target.yaw_mec_tar == gimbal->config.yaw_zero[BEHIND])
			  {
				  gimbal->target.yaw_mec_tar = gimbal->config.yaw_zero[FRONT];
			  }
			}
			
			if(abs(motor_half_cycle(gimbal->target.yaw_mec_tar - board.rx_meg->gimbal_meg.yaw_mec, 2*PI)) <= 3.f/180.f * PI)
			{
				infantry.flag.U_turn_flag.value = false;
			}
		}
		else{
			//头在哪哪边为正
			if(abs(gimbal->info.yaw_mec_err_raw) <= PI/2)
		  {
			  gimbal->target.yaw_mec_tar = gimbal->config.yaw_zero[FRONT];
		  }
		  else{
		    gimbal->target.yaw_mec_tar = gimbal->config.yaw_zero[BEHIND];
		
		  }
			
		  if(infantry.ctrl == RC_CTRL)
		  {
			  gimbal->target.pitch_mec_tar += rc_dev.info->ch1/660.f * gimbal->config.rc_pitch_mec_step;
		  }
		  else if(infantry.ctrl == KEY_CTRL)
		  {
			  gimbal->target.pitch_mec_tar += rc_dev.info->mouse_y * gimbal->config.key_pitch_mec_step;
		  }
		
		}
		
		gimbal->target.pitch_mec_tar = motor_half_cycle(gimbal->target.pitch_mec_tar,2*PI);
		
		gimbal->target.pitch_mec_tar = constrain(gimbal->target.pitch_mec_tar,PITCH_MEC_MIN_ANGLE,PITCH_MEC_MAX_ANGLE);
	}
	
	//实时更新陀螺仪目标值
	gimbal->target.yaw_imu_tar = gimbal->info.yaw_imu;
	gimbal->target.pitch_imu_tar = gimbal->info.pitch_imu;
 
//	gimbal->info.yaw_mec_err_act = motor_half_cycle(gimbal->info.yaw_mec - gimbal->target.yaw_mec_tar,2*PI);

}


/* 主控模式：视觉/掉头/操作手更新 IMU 目标并处理底盘复位 */
static void  Gimbal_Boss_Update(Gimbal_t* gimbal)
{
	//视觉模式上板直接用视觉包目标值，下板需要实时更新目标值防止退出视觉时目标值衔接错误导致头动
	if(vision.mode != V_NORMAL && board.rx_meg->state_meg.vision_state == true && board.rx_meg->vision_meg.is_find_target == true)  
	{
		gimbal->target.yaw_imu_tar = board.rx_meg->vision_meg.vision_yaw_tar;
	  gimbal->target.pitch_imu_tar = board.rx_meg->vision_meg.vision_pitch_tar;
	}
	//掉头处理
  else if(infantry.flag.U_turn_flag.value == true)
	{
		if(infantry.flag.U_turn_flag.form == RISING)
		{
			gimbal->target.yaw_imu_tar += 180.f;
			gimbal->target.yaw_imu_tar = motor_half_cycle(gimbal->target.yaw_imu_tar,360.f);
		}
		if(abs(motor_half_cycle(gimbal->target.yaw_imu_tar - board.rx_meg->gimbal_meg.yaw_imu,360.f)) <= 3.f)
		{
			infantry.flag.U_turn_flag.value = false;
		}
	}
	else
	{
	  if(infantry.ctrl == RC_CTRL)
    {
		  gimbal->target.yaw_imu_tar -= rc_dev.info->ch0/660.f * gimbal->config.rc_yaw_imu_step;
		  gimbal->target.pitch_imu_tar += rc_dev.info->ch1/660.f * gimbal->config.rc_pitch_imu_step;
			
	  }
	  else if(infantry.ctrl == KEY_CTRL)
	  {
		  gimbal->target.yaw_imu_tar -= rc_dev.info->mouse_x * gimbal->config.key_yaw_imu_step;
		  gimbal->target.pitch_imu_tar += rc_dev.info->mouse_y * gimbal->config.key_pitch_imu_step;
			
	  }

	}
	
	//角度限幅，动态限位
	gimbal->target.yaw_imu_tar = motor_half_cycle(gimbal->target.yaw_imu_tar,360.f);
	gimbal->target.pitch_imu_tar = constrain(gimbal->target.pitch_imu_tar,PITCH_IMU_MIN_ANGLE,PITCH_IMU_MAX_ANGLE);
	
	//底盘复位
	if(infantry.flag.chassis_reset.value == true)
	{
		gimbal->target.yaw_mec_tar = gimbal->config.yaw_zero[FRONT];
		
		if(abs(motor_half_cycle(gimbal->target.yaw_mec_tar - gimbal->info.yaw_mec,2 * PI)) <= 4.f/180.f * PI)
		{
      infantry.flag.chassis_reset.value = false;
		}
	}
	else{
		if(abs(gimbal->info.yaw_mec_err_raw) <= PI/2)
		{
			gimbal->target.yaw_mec_tar = gimbal->config.yaw_zero[FRONT];
		}
		else{
			gimbal->target.yaw_mec_tar = gimbal->config.yaw_zero[BEHIND];
		}
	}
			
//	gimbal->target.yaw_mec_tar = gimbal->info.yaw_mec;
	
	gimbal->target.pitch_mec_tar = gimbal->info.pitch_mec;
	
//	gimbal->info.yaw_mec_err_act = motor_half_cycle(gimbal->info.yaw_mec - gimbal->target.yaw_mec_tar,2*PI);
}


/* 从上板反馈同步云台设备在线状态 */
static void Gimbal_Offline_Update(Gimbal_t* gimbal)
{
	gimbal->state.yaw_heart = board.rx_meg->state_meg.yaw_motor_state;
	gimbal->state.pitch_heart = board.rx_meg->state_meg.pitch_motor_state;
	gimbal->state.left_heart = board.rx_meg->state_meg.height_motor_state;
	
}

/* 失联时清复位流程，并将目标回正 */
static void Gimbal_Offline_Process(Gimbal_t* gimbal)
{
	gimbal->gimbal_reset_flag = false;
	reset_tick = 0;
	
	//机械目标值回正
	gimbal->target.pitch_mec_tar = PITCH_MEC_ZERO_ANGLE;
	gimbal->target.yaw_mec_tar = YAW_MEC_ZERO_ANGLE;
	gimbal->target.yaw_imu_tar = gimbal->info.yaw_imu;
	gimbal->target.pitch_imu_tar = 0;

}

/* 下发云台目标，计算底盘跟随误差与特殊姿态抑制 */
static void Gimbal_Cmd_Transmit(Gimbal_t* gimbal)
{
  board.tx_pkt->gimbal_target_pkt.yaw_mec_tar = gimbal->target.yaw_mec_tar;
	board.tx_pkt->gimbal_target_pkt.yaw_imu_tar = gimbal->target.yaw_imu_tar;
	board.tx_pkt->gimbal_target_pkt.pitch_mec_tar = gimbal->target.pitch_mec_tar;
	board.tx_pkt->gimbal_target_pkt.pitch_imu_tar = gimbal->target.pitch_imu_tar;
	
	
	#if GIMBAL_SWITCH == 0
	 gimbal->info.yaw_mec_err_act = motor_half_cycle(gimbal->info.yaw_imu - gimbal->target.yaw_imu_tar,360.f) / 180.f * PI;
	#else 
	 if(infantry.flag.U_turn_flag.value == true || infantry.flag.R_turn_flag.value == true || infantry.flag.L_turn_flag.value == true)
	 {
		 gimbal->info.yaw_mec_err_act = 0;    //头特殊转时底盘不跟
	 }
	 else{
		gimbal->info.yaw_mec_err_act = motor_half_cycle(gimbal->info.yaw_mec - gimbal->target.yaw_mec_tar,2*PI);
	 }
	#endif
		
	
	if(gimbal->mode == G_SLEEP || gimbal->mode == G_INIT || gimbal->mode == G_SLAVE)
	{
		board.tx_pkt->car_pkt.gimbal_mode = 0;
	}
	else{
	  board.tx_pkt->car_pkt.gimbal_mode = 1;
	}
	
	
	if(infantry.mode == I_INIT){
//	  board.tx_pkt->gimbal_target_pkt.is_hole = true;    //只有开狗洞标志位和底盘不复位才能给上板发压低标志位
	}
	else if(infantry.flag.hole_flag == false)
	{
		board.tx_pkt->gimbal_target_pkt.is_hole = false;
	}
	else if(infantry.flag.chassis_reset.value == true)
	{
		board.tx_pkt->gimbal_target_pkt.is_hole = false;
	}	
	else{
	   board.tx_pkt->gimbal_target_pkt.is_hole = true; 
	
	} 
  

}


/* 周期顺序：刷新反馈 -> 选择模式 -> 执行模式 -> 下发 */
static void Gimbal_Work(Gimbal_t* gimbal)
{
	Gimbal_Data_Update(gimbal);
	Gimbal_Offline_Update(gimbal);
	Gimbal_Status_Update(gimbal);
	switch (gimbal->mode)
	{
		case G_SLEEP:
			Gimbal_Offline_Process(gimbal);
			break;
		
		case G_INIT:
			Gimbal_Init_Process(gimbal);
		  break;

		case G_SLAVE:
			Gimbal_Slave_Update(gimbal);
		  break;
		
		case G_BOSS:
			Gimbal_Boss_Update(gimbal);
			break;
	
	  default:
			break;
	}
	
	Gimbal_Cmd_Transmit(gimbal);

}



