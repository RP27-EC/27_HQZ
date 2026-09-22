#include "judge.h"
#include "judge_protocol.h"
#include "Board_protocol.h"
#include "imu_sensor.h"
#include "cap.h"
#include "drv_tick.h"

Judge_Info_t Judge_Info;
Judge_Pkt_t Judge_Pkt;
Judge_Status_t Judge_Status = 
{
	.offline_cnt_max = JUDGE_OFFLINE_CNT_MAX,
};
Judge_t judge = 
{
	.info = &Judge_Info,
	.pkt = &Judge_Pkt,
	.status = &Judge_Status,
	.init = Judge_Init,
};


/* 裁判系统初始化 */
void Judge_Init(Judge_t* judge)
{
	judge->status->offline_cnt = judge->status->offline_cnt_max;
	judge->status->status = DEV_OFFLINE;
	
	judge->rx = Judge_Data_Update;
	
	judge->heartbeat = Judge_Heart_Beat;
}

/* 裁判系统心跳 */
void Judge_Heart_Beat(Judge_t* judge)
{
	judge->status->offline_cnt++;
	if(judge->status->offline_cnt >= judge->status->offline_cnt_max)
	{
		judge->status->offline_cnt = judge->status->offline_cnt_max;
		judge->status->status = DEV_OFFLINE;
	}
	else{
	  judge->status->status = DEV_ONLINE;
	
	}
}


void Judge_Data_Update(uint16_t id, uint8_t *rxBuf)
{
    switch (id)
    {
    case ID_game_status:
      memcpy(&judge.info->game_status, rxBuf, LEN_game_status);
				
		  judge.pkt->game_progress = judge.info->game_status.game_progress;

      if(judge.pkt->game_progress == 4)
      {
				board.tx_pkt->car_pkt.game_start = 1;
			}				
		  else{
			  board.tx_pkt->car_pkt.game_start = 0;
			}
			
		  judge.status->offline_cnt = 0;
      judge.status->status = DEV_ONLINE;    
		  break;

    case ID_game_result:
      memcpy(&judge.info->game_result, rxBuf, LEN_game_result);
		  judge.status->offline_cnt = 0;
      judge.status->status = DEV_ONLINE;    
      break;

    case ID_game_robot_HP:
      memcpy(&judge.info->game_robot_HP, rxBuf, LEN_game_robot_HP);
	
		  judge.pkt->blood[J_HERO] = judge.info->game_robot_HP.ally_1_robot_HP;
		  judge.pkt->blood[J_ENGINEER] = judge.info->game_robot_HP.ally_2_robot_HP;
		  judge.pkt->blood[J_INFANTRY_3] = judge.info->game_robot_HP.ally_3_robot_HP;
		  judge.pkt->blood[J_INFANTRY_4] = judge.info->game_robot_HP.ally_4_robot_HP;
		  judge.pkt->blood[J_SENTRY] = judge.info->game_robot_HP.ally_7_robot_HP;
		  judge.pkt->blood[J_OUTPOST] = judge.info->game_robot_HP.ally_outpost_HP;
		  judge.pkt->blood[J_BASE] = judge.info->game_robot_HP.ally_base_HP;   
		 
		  for(uint8_t i = 0; i< J_ROBOT_CNT;i++)
		  {
			  board.tx_pkt->blood_pkt.blood[i] = judge.pkt->blood[i];
			}
		
		  judge.status->offline_cnt = 0;
      judge.status->status = DEV_ONLINE;    
      break;

    case ID_event_data:
      memcpy(&judge.info->event_data, rxBuf, LEN_event_data);
		  judge.status->offline_cnt = 0;
      judge.status->status = DEV_ONLINE;     
      break;

    case ID_referee_warning:
      memcpy(&judge.info->referee_warning, rxBuf, LEN_referee_warning);
		  judge.status->offline_cnt = 0;
      judge.status->status = DEV_ONLINE;    
      break;

    case ID_dart_info:
      memcpy(&judge.info->dart_info, rxBuf, LEN_dart_info);
		  judge.status->offline_cnt = 0;
      judge.status->status = DEV_ONLINE;    
      break;

    case ID_robot_status:
      memcpy(&judge.info->robot_status, rxBuf, LEN_robot_status);
		
		  judge.pkt->robot_id = judge.info->robot_status.robot_id;
		  judge.pkt->shooter_barrel_heat_limit = judge.info->robot_status.shooter_barrel_heat_limit; 
      judge.pkt->chassis_power_limit = judge.info->robot_status.chassis_power_limit;          
		
		  if(judge.pkt->robot_id < 10)
		  {
				board.tx_pkt->car_pkt.my_color = 0;
			}
		  else{
			 board.tx_pkt->car_pkt.my_color = 1;
		  }
		
		  judge.status->offline_cnt = 0;
      judge.status->status = DEV_ONLINE;    
      break;

    case ID_power_heat_data:
      memcpy(&judge.info->power_heat_data, rxBuf, LEN_power_heat_data);
		
		  judge.pkt->buffer_energy = judge.info->power_heat_data.buffer_energy;
		  judge.pkt->shooter_17mm_1_barrel_heat = judge.info->power_heat_data.shooter_17mm_1_barrel_heat;
		   
		  judge.status->offline_cnt = 0;
      judge.status->status = DEV_ONLINE;    
      break;

    case ID_robot_pos:
      memcpy(&judge.info->robot_pos, rxBuf, LEN_robot_pos);
		  judge.status->offline_cnt = 0;
      judge.status->status = DEV_ONLINE;    
      break;

    case ID_buff:
      memcpy(&judge.info->buff, rxBuf, LEN_buff);
		  judge.status->offline_cnt = 0;
      judge.status->status = DEV_ONLINE;    
      break;

    case ID_hurt_data:
      memcpy(&judge.info->hurt_data, rxBuf, LEN_hurt_data);
		  judge.status->offline_cnt = 0;
      judge.status->status = DEV_ONLINE;    
      break;

    case ID_shoot_data:
      memcpy(&judge.info->shoot_data, rxBuf, LEN_shoot_data);
		
		  judge.pkt->initial_speed = judge.info->shoot_data.initial_speed;
		  judge.pkt->launching_frequency = judge.info->shoot_data.launching_frequency;
		    
		  board.tx_pkt->judge_shoot_pkt.shoot_speed = judge.pkt->initial_speed;
		  board.tx_pkt->judge_shoot_pkt.shoot_freq = judge.pkt->launching_frequency;
		
		  Shooting_Cmd_Excute_Tick_Calculating(1);
		  Speed_Statistic();
		
		  if(shoot_statistics.shoot_mode == 1)
		  {
			  shoot_statistics.shooting_flag = 1;
				Shooting_Cmd_Excute_Tick_Calculating(0);
				
			}
		
		  judge.status->offline_cnt = 0;
      judge.status->status = DEV_ONLINE;    
      break;

    case ID_projectile_allowance:
      memcpy(&judge.info->projectile_allowance, rxBuf, LEN_projectile_allowance);
		
		  judge.pkt->projectile_allowance_17mm = judge.info->projectile_allowance.projectile_allowance_17mm;  
		
		  board.tx_pkt->judge_shoot_pkt.allowance_max = judge.pkt->projectile_allowance_17mm;
		
		  judge.status->offline_cnt = 0;
      judge.status->status = DEV_ONLINE;    
      break;

    case ID_rfid_status:
      memcpy(&judge.info->rfid_status, rxBuf, LEN_rfid_status);
		  judge.status->offline_cnt = 0;
      judge.status->status = DEV_ONLINE;    
      break;

    case ID_robot_interaction_data:
      memcpy(&judge.info->robot_interaction_data, rxBuf, LEN_robot_interaction_data);
			if (judge.info->robot_interaction_data.data_cmd_id == 0x211)
			{
				memcpy(&judge.info->radar_information_status, &judge.info->robot_interaction_data.user_data, sizeof(radar_information_status_t));
			}
		  judge.status->offline_cnt = 0;
      judge.status->status = DEV_ONLINE;    
      break;

    case ID_map_command:
      memcpy(&judge.info->map_command, rxBuf, LEN_map_command);
		  judge.status->offline_cnt = 0;
      judge.status->status = DEV_ONLINE;    
      break;

    case ID_map_robot_data:
      memcpy(&judge.info->map_robot_data, rxBuf, LEN_map_robot_data);
		  judge.status->offline_cnt = 0;
      judge.status->status = DEV_ONLINE;    
      break;

    case ID_map_data:
      memcpy(&judge.info->map_data, rxBuf, LEN_map_data);
		  judge.status->offline_cnt = 0;
      judge.status->status = DEV_ONLINE;    
      break;

    case ID_custom_info:
      memcpy(&judge.info->custom_info, rxBuf, LEN_custom_info);
		  judge.status->offline_cnt = 0;
      judge.status->status = DEV_ONLINE;    
      break;

    case ID_set_video_channel:
      memcpy(&judge.info->set_video_channel, rxBuf, LEN_set_video_channel);
		  judge.status->offline_cnt = 0;
      judge.status->status = DEV_ONLINE;    
      break;

    case ID_query_video_channel:
      memcpy(&judge.info->query_video_channel, rxBuf, LEN_query_video_channel);
		  judge.status->offline_cnt = 0;
      judge.status->status = DEV_ONLINE;    
      break;
    
    default:
      break;
    }
}


/* bullet_data_t */
bullet_data_t shoot_statistics = {
	.shooting_flag = 0,

};
uint8_t cali_flag = 1; //是否计算平均值和平方差
void Speed_Statistic(void)
{
//	shoot_statistics.temperature_L=shoot.fric.thr_fric[FRIC_L]->rx_info->temperature;
	shoot_statistics.speed_now=judge.info->shoot_data.initial_speed;
  float s_speed =judge.info->shoot_data.initial_speed;
  shoot_statistics.num++;
	
    // 统计速度区间
    if (s_speed < 23.7f)
    {
        shoot_statistics.lower_237++;
        shoot_statistics. num--; //弹速太离谱不统计
    }
    else if (s_speed >= 23.7f && s_speed <= 23.8f)
    {
        shoot_statistics.speed_237++;
    }
    else if (s_speed > 23.8f && s_speed <= 23.9f)
    {
        shoot_statistics.speed_238++;
    }
    else if (s_speed > 23.9f && s_speed <= 24.0f)
    {
        shoot_statistics.speed_239++;
    }
    else if (s_speed > 24.0f && s_speed <= 24.1f)
    {
        shoot_statistics.speed_240++;
    }
    else if (s_speed > 24.1f && s_speed <= 24.2f)
    {
        shoot_statistics.speed_241++;
    }
    else if (s_speed > 24.2f && s_speed <= 24.3f)
    {
        shoot_statistics.speed_242++;
    }
    else if (s_speed > 24.3f && s_speed <= 24.4f)
    {
        shoot_statistics.speed_243++;
    }
    else if (s_speed > 24.4f && s_speed <= 24.5f)
    {
        shoot_statistics.speed_244++;
    }
    else if (s_speed > 24.5f && s_speed <= 24.6f)
    {
        shoot_statistics.speed_245++;
    }
    else if (s_speed > 24.6f && s_speed <= 24.7f)
    {
        shoot_statistics.speed_246++;
    }
	  else if (s_speed > 24.7f && s_speed <= 24.8f)
    {
        shoot_statistics.speed_247++;
    }
	  else if (s_speed > 24.8f && s_speed <= 24.9f)
    {
        shoot_statistics.speed_248++;
    }
	  else if (s_speed > 24.9f && s_speed <= 25.0f)
    {
        shoot_statistics.speed_249++;
    }
    else if (s_speed > 25.0f)
    {
        shoot_statistics.higher_250++;
        shoot_statistics. num--; //弹速太离谱不统计
    }

    // 统计弹速平均值和方差
    if (cali_flag == 1)
    {
        // 计算平均值
        shoot_statistics.mean = (
            shoot_statistics.speed_237 * 23.7f +
            shoot_statistics.speed_238 * 23.8f +
            shoot_statistics.speed_239 * 23.9f +
            shoot_statistics.speed_240 * 24.0f +
            shoot_statistics.speed_241 * 24.1f +
            shoot_statistics.speed_242 * 24.2f +
            shoot_statistics.speed_243 * 24.3f +
            shoot_statistics.speed_244 * 24.4f +
            shoot_statistics.speed_245 * 24.5f +
            shoot_statistics.speed_246 * 24.6f +
			      shoot_statistics.speed_247 * 24.7f +
			      shoot_statistics.speed_248 * 24.8f +
			      shoot_statistics.speed_249 * 24.9f ) / (float)shoot_statistics.num;
                                         
        // 计算方差
        float sum_of_squares = (
            shoot_statistics.speed_237 * ((23.7f - shoot_statistics.mean) * (23.7f - shoot_statistics.mean)) +
            shoot_statistics.speed_238 * ((23.8f - shoot_statistics.mean) * (23.8f - shoot_statistics.mean)) +
            shoot_statistics.speed_239 * ((23.9f - shoot_statistics.mean) * (23.9f - shoot_statistics.mean)) +
            shoot_statistics.speed_240 * ((24.0f - shoot_statistics.mean) * (24.0f - shoot_statistics.mean)) +
            shoot_statistics.speed_241 * ((24.1f - shoot_statistics.mean) * (24.1f - shoot_statistics.mean)) +
            shoot_statistics.speed_242 * ((24.2f - shoot_statistics.mean) * (24.2f - shoot_statistics.mean)) +
            shoot_statistics.speed_243 * ((24.3f - shoot_statistics.mean) * (24.3f - shoot_statistics.mean)) +
            shoot_statistics.speed_244 * ((24.4f - shoot_statistics.mean) * (24.4f - shoot_statistics.mean)) +
            shoot_statistics.speed_245 * ((24.5f - shoot_statistics.mean) * (24.5f - shoot_statistics.mean)) +
            shoot_statistics.speed_246 * ((24.6f - shoot_statistics.mean) * (24.6f - shoot_statistics.mean)) +
			      shoot_statistics.speed_247 * ((24.7f - shoot_statistics.mean) * (24.7f - shoot_statistics.mean)) +
			      shoot_statistics.speed_248 * ((24.8f - shoot_statistics.mean) * (24.8f - shoot_statistics.mean)) +
			      shoot_statistics.speed_249 * ((24.9f - shoot_statistics.mean) * (24.9f - shoot_statistics.mean)));
                                           
            shoot_statistics.variance = sum_of_squares / (float)shoot_statistics.num;
						
    }
}


/* 射击指令执行计时 */
void Shooting_Cmd_Excute_Tick_Calculating(uint8_t flag)
{
	static uint32_t cmd_start_tick = 0;
	static uint32_t rx_bullet_tick = 0;
	static uint8_t rx_bullet_cnt = 0;
	static uint8_t reset_cnt_flag = 0;
	

	const uint8_t buf_length = 100;
	if (flag == 0)//命令开始执行
	{
		cmd_start_tick = HAL_GetTick();
	}
	else if (flag == 1)//接收到弹速
	{
		rx_bullet_tick = HAL_GetTick();
		shoot_statistics.shooting_cmd_excute_tick=rx_bullet_tick - cmd_start_tick;
		shoot_statistics.shooting_cmd_excute_tick_buf[rx_bullet_cnt]=shoot_statistics.shooting_cmd_excute_tick;
		
		//移动指针
		rx_bullet_cnt++;
		//回归零点
		if(rx_bullet_cnt>buf_length-1)
		{
			rx_bullet_cnt=0;
			reset_cnt_flag=1;
		}
		//计算平均数
		float shooting_cmd_excute_tick_sum = 0;
		float sum_of_squares = 0;
		
		if(reset_cnt_flag==1)//如果回到原点过，直接遍历
		{
			
			for(uint8_t i=0;i<buf_length;i++)
			{
				shooting_cmd_excute_tick_sum+=shoot_statistics.shooting_cmd_excute_tick_buf[i];
			}
			shoot_statistics.shooting_cmd_excute_tick_mean=shooting_cmd_excute_tick_sum/buf_length;
			
			for(uint8_t i=0;i<buf_length;i++)
			{
			  sum_of_squares+=(shoot_statistics.shooting_cmd_excute_tick_buf[i]-shoot_statistics.shooting_cmd_excute_tick_mean)*(shoot_statistics.shooting_cmd_excute_tick_buf[i]-shoot_statistics.shooting_cmd_excute_tick_mean);
			}
			shoot_statistics.shooting_cmd_excute_tick_variance=sum_of_squares/buf_length;
			
			reset_cnt_flag= 0;
		}
		else//多少个就多少个
		{
			for(uint8_t i=0;i<rx_bullet_cnt;i++)
			{
				shooting_cmd_excute_tick_sum+=shoot_statistics.shooting_cmd_excute_tick_buf[i];
			}
			shoot_statistics.shooting_cmd_excute_tick_mean=shooting_cmd_excute_tick_sum/rx_bullet_cnt;
			
			for(uint8_t i=0;i<rx_bullet_cnt;i++)
			{
			  sum_of_squares+=(shoot_statistics.shooting_cmd_excute_tick_buf[i]-shoot_statistics.shooting_cmd_excute_tick_mean)*(shoot_statistics.shooting_cmd_excute_tick_buf[i]-shoot_statistics.shooting_cmd_excute_tick_mean);
			}
			shoot_statistics.shooting_cmd_excute_tick_variance=sum_of_squares/rx_bullet_cnt;
		}
		
		
	}
}




