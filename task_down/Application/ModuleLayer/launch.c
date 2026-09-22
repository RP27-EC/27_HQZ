/* launch.c - 发射机构控制 */

#include "launch.h"
#include "board_protocol.h"
#include "rc_sensor.h"
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
    uint8_t rc_online;
    int16_t thumbwheel;

    rc_online = (rc_dev.work_state == DEV_ONLINE) ? 1u : 0u;

    if ((rc_online != 0u) &&
        (rc_dev.info->s1.value == RC_SW_MID))
    {
        launch->state = L_UNLOCK;

        thumbwheel = rc_dev.info->thumbwheel.value;
        if (thumbwheel <= -200)
        {
            launch->mode = SINGLE_SHOT;
            launch->shoot_level = 1u;
        }
        else if (thumbwheel >= 200)
        {
            launch->mode = REPEAT_SHOT;
            launch->shoot_level = 1u;
        }
        else
        {
            launch->mode = SINGLE_SHOT;
            launch->shoot_level = 0u;
        }
    }
    else
    {
        launch->state = L_LOCK;
        launch->mode = SINGLE_SHOT;
        launch->shoot_level = 0u;
    }
}


/* 发射离线刷新 */
static void Launch_Offline_Update(Launch_t* launch)
{
	launch->heart.r_fric_heart = board.rx_meg->state_meg.r_fric_state;
	launch->heart.l_fric_heart = board.rx_meg->state_meg.l_fric_state;
	launch->heart.dial_heart = board.rx_meg->state_meg.dial_motor_state;
	
}

/* 下发发射控制量 */
static void Launch_Cmd_Transmit(Launch_t* launch)
{
  board.tx_pkt->shoot_pkt.launch_state = launch->state;
	board.tx_pkt->shoot_pkt.shoot_mode = launch->mode;
	board.tx_pkt->shoot_pkt.shoot_level = launch->shoot_level;
 
}


static void Launch_Work(Launch_t* launch)
{
  Launch_Data_Update(launch);
  Launch_Cmd_Transmit(launch);
}

