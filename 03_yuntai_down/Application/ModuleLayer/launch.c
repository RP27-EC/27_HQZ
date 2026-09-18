#include "launch.h"
#include "board_protocol.h"
/*
上供弹关系，底盘发射机构模块只需要向上传输
                 状态，
                 模式，
                 电平，
三样东西，太过简单就不在这里做文章
*/

static void Launch_Init(Launch_t* launch);
static void Launch_Data_Update(Launch_t* launch);
static void Launch_Offline_Update(Launch_t* launch);
static void Launch_Cmd_Transmit(Launch_t* launch);
static void Launch_Work(Launch_t* launch);

Launch_t  launch = {
	.state = L_LOCK,
	.mode = SINGLE_SHOT,
	.shoot_lock = 1,
	.shoot_level = 0,
	
	.init = Launch_Init,

};


static void Launch_Init(Launch_t* launch)
{
  launch->work = Launch_Work;
	launch->heart_beat = Launch_Offline_Update;
}


static void Launch_Data_Update(Launch_t* launch)
{
  

}


/**
 * @brief  发射机构失联检测
 * @note  
 */
static void Launch_Offline_Update(Launch_t* launch)
{
	launch->heart.r_fric_heart = board.rx_meg->state_meg.r_fric_state;
	launch->heart.l_fric_heart = board.rx_meg->state_meg.l_fric_state;
	launch->heart.dial_heart = board.rx_meg->state_meg.dial_motor_state;
	
}

/**
 * @brief  发射机构命令发送
 * @note   更新到板间
 */
static void Launch_Cmd_Transmit(Launch_t* launch)
{
  board.tx_pkt->shoot_pkt.launch_state = launch->state;
	board.tx_pkt->shoot_pkt.shoot_mode = launch->mode;
	board.tx_pkt->shoot_pkt.shoot_level = launch->shoot_level;
 
}


static void Launch_Work(Launch_t* launch)
{
  Launch_Cmd_Transmit(launch);
}