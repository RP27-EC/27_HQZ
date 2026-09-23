/* launch.c - 发射机构控制 */

#include "launch.h"
#include "board_protocol.h"
#include "rc_sensor.h"

#define LAUNCH_INPUT_DEBOUNCE_MS 20u

static void Launch_Init(Launch_t* launch);
static void Launch_Data_Update(Launch_t* launch);
static void Launch_Offline_Update(Launch_t* launch);
static void Launch_Cmd_Transmit(Launch_t* launch);
static void Launch_Work(Launch_t* launch);

Launch_t launch = {
	.state = L_LOCK,
	.mode = SINGLE_SHOT,
	.shoot_lock = 1,
	.shoot_level = 0,
	.shoot_raw = 0,
	.mode_raw = SINGLE_SHOT,
	.shoot_debounce_cnt = 0,

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
    uint8_t shoot_raw;
    Launch_Mode_e mode_raw;

    rc_online = (rc_dev.work_state == DEV_ONLINE) ? 1u : 0u;

    if ((rc_online != 0u) &&
        (rc_dev.info->s1.value == RC_SW_MID))
    {
        launch->state = L_UNLOCK;

        thumbwheel = rc_dev.info->thumbwheel.value;
        if (thumbwheel <= -200)
        {
            shoot_raw = 1u;
            mode_raw = SINGLE_SHOT;
        }
        else if (thumbwheel >= 200)
        {
            shoot_raw = 1u;
            mode_raw = REPEAT_SHOT;
        }
        else
        {
            shoot_raw = 0u;
            mode_raw = SINGLE_SHOT;
        }
    }
    else
    {
        launch->state = L_LOCK;
        launch->mode = SINGLE_SHOT;
        launch->shoot_level = 0u;
        launch->shoot_raw = 0u;
        launch->mode_raw = SINGLE_SHOT;
        launch->shoot_debounce_cnt = 0u;
        return;
    }

    // NOTE: 抑制拨轮阈值抖动，防止重复单发
    if ((shoot_raw != launch->shoot_raw) ||
        (mode_raw != launch->mode_raw))
    {
        launch->shoot_raw = shoot_raw;
        launch->mode_raw = mode_raw;
        launch->shoot_debounce_cnt = 0u;
    }
    else if (launch->shoot_debounce_cnt < LAUNCH_INPUT_DEBOUNCE_MS)
    {
        launch->shoot_debounce_cnt++;
    }
    else
    {
        launch->mode = launch->mode_raw;
        launch->shoot_level = launch->shoot_raw;
    }
}

static void Launch_Offline_Update(Launch_t* launch)
{
	launch->heart.r_fric_heart = board.rx_meg->state_meg.r_fric_state;
	launch->heart.l_fric_heart = board.rx_meg->state_meg.l_fric_state;
	launch->heart.dial_heart = board.rx_meg->state_meg.dial_motor_state;
}

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
