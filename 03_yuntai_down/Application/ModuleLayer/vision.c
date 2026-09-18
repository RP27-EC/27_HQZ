#include "vision.h"
#include "infantry.h"
#include "board_protocol.h"

static void Vision_Init(Vision_t* vision);
static void Vision_Status_Update(Vision_t* vision);
static void Vision_Offline_Update(Vision_t* vision);
static void Vision_Data_Update(Vision_t* vision);
static void Vision_Cmd_Transmit(Vision_t* vision);
static void Vision_Work(Vision_t* vision);


Vision_t vision = {
  .mode = V_NORMAL,
	
	.init = Vision_Init,
};


static void Vision_Init(Vision_t* vision)
{
	vision->work = Vision_Work;
	vision->heart_beat = Vision_Offline_Update;
}

/**
 * @brief  视觉模式状态更新
 * @note   
 */
static void Vision_Status_Update(Vision_t* vision)
{
	switch (infantry.flag.vision_flag)
	{
	  case 0:
			vision->mode = V_NORMAL;
			break;
		
		case 1:
			vision->mode = V_AUTO;
			break;
		
		case 2:
			vision->mode = V_S_BUFF;
			break;
		
		case 3:
			vision->mode = V_B_BUFF;
			break;
		
		case 4:
			vision->mode = V_OUTPOST;
			break;
		
		case 5:
			vision->mode = V_HERO;
			break;
	
	  default:
			break;
		
	}
}

/**
 * @brief  视觉失联检测
 * 
 */
static void Vision_Offline_Update(Vision_t* vision)
{
	vision->info.vision_heart = board.rx_meg->state_meg.vision_state;
}


static void Vision_Data_Update(Vision_t* vision)
{
	vision->info.vision_yaw_tar = board.rx_meg->vision_meg.vision_yaw_tar;
	vision->info.vision_pitch_tar = board.rx_meg->vision_meg.vision_pitch_tar;
  vision->info.is_find_target = board.rx_meg->vision_meg.is_find_target;

}

/**
 * @brief   视觉命令发送
 * @note    更新到板间
 */
static void Vision_Cmd_Transmit(Vision_t* vision)
{
	switch (vision->mode)
	{
	  case V_NORMAL:
			board.tx_pkt->car_pkt.vision_mode = 0;
			break;
		
		case V_AUTO:
			board.tx_pkt->car_pkt.vision_mode = 1;
			break;
		
		case V_S_BUFF:
			board.tx_pkt->car_pkt.vision_mode = 2;
			break;
		
		case V_B_BUFF:
			board.tx_pkt->car_pkt.vision_mode = 3;
			break;
		
		case V_OUTPOST:
			board.tx_pkt->car_pkt.vision_mode = 4;
			break;
		
		case V_HERO:
			board.tx_pkt->car_pkt.vision_mode = 5;
			break;
	
	  default:
			break;
		
	}
}


void Vision_Work(Vision_t* vision)
{
	Vision_Data_Update(vision);
	Vision_Status_Update(vision);
	Vision_Cmd_Transmit(vision);
	
}
