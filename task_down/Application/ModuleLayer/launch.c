/* launch.c - 发射机构控制 */

#include "launch.h"
#include "board_protocol.h"
#include "rc_sensor.h"
#include "chassis_input.h"

static void Launch_Init(Launch_t *launch);
static void Launch_Work(Launch_t *launch);
static void Launch_Offline_Update(Launch_t *launch);

static uint8_t launch_shoot_switch_seen;
static uint8_t launch_shoot_previous_switch;
static uint8_t launch_shoot_armed;

Launch_t launch =
{
    .state = L_LOCK,
    .mode = SINGLE_SHOT,
    .shoot_lock = 1,
    .shoot_level = 0,
    .init = Launch_Init,
};

static void Launch_Reset_Shoot_Arm(void)
{
    launch_shoot_switch_seen = 0u;
    launch_shoot_previous_switch = 0u;
    launch_shoot_armed = 0u;
}

static void Launch_Init(Launch_t *launch)
{
    launch->work = Launch_Work;
    launch->heart_beat = Launch_Offline_Update;
    Launch_Reset_Shoot_Arm();
}

static void Launch_Data_Update(Launch_t *launch)
{
    uint8_t s1;
    uint8_t s2;

    if (Chassis_Input_IsKeyboardMode() != 0u)
    {
        Launch_Reset_Shoot_Arm();
        launch->state = L_UNLOCK;

        if ((rc_dev.info->mouse_btn_l.value & 0x01u) != 0u)
        {
            launch->mode = (rc_dev.info->mouse_btn_l.status == long_press) ?
                           REPEAT_SHOT : SINGLE_SHOT;
            launch->shoot_level = 1u;
        }
        else
        {
            launch->mode = SINGLE_SHOT;
            launch->shoot_level = 0u;
        }

        return;
    }

    if (rc_dev.work_state != DEV_ONLINE)
    {
        Launch_Reset_Shoot_Arm();
        launch->state = L_LOCK;
        launch->mode = SINGLE_SHOT;
        launch->shoot_level = 0u;
        return;
    }

    s1 = (uint8_t)rc_dev.info->s1.value;
    s2 = (uint8_t)rc_dev.info->s2.value;

    /* 现有小陀螺组合：S1 上 + S2 下，禁止发射。 */
    if ((s1 == RC_SW_UP) && (s2 == RC_SW_DOWN))
    {
        Launch_Reset_Shoot_Arm();
        launch->state = L_LOCK;
        launch->mode = SINGLE_SHOT;
        launch->shoot_level = 0u;
        return;
    }

    if ((s1 != RC_SW_UP) && (s1 != RC_SW_MID))
    {
        Launch_Reset_Shoot_Arm();
        launch->state = L_LOCK;
        launch->mode = SINGLE_SHOT;
        launch->shoot_level = 0u;
        return;
    }

    /* 上电后必须先变化 S2，避免开机停在上位直接发射。 */
    if ((launch_shoot_switch_seen != 0u) &&
        (s2 != launch_shoot_previous_switch))
    {
        launch_shoot_armed = 1u;
    }
    launch_shoot_switch_seen = 1u;
    launch_shoot_previous_switch = s2;

    if ((launch_shoot_armed != 0u) && (s2 == RC_SW_UP))
    {
        launch->state = L_UNLOCK;
        launch->mode = (s1 == RC_SW_UP) ? REPEAT_SHOT : SINGLE_SHOT;
        launch->shoot_level = 1u;
    }
    else if ((launch_shoot_armed != 0u) && (s2 == RC_SW_MID))
    {
        launch->state = L_UNLOCK;
        launch->mode = SINGLE_SHOT;
        launch->shoot_level = 0u;
    }
    else
    {
        launch->state = L_LOCK;
        launch->mode = SINGLE_SHOT;
        launch->shoot_level = 0u;
    }
}

static void Launch_Offline_Update(Launch_t *launch)
{
    launch->heart.r_fric_heart = board.rx_meg->state_meg.r_fric_state;
    launch->heart.l_fric_heart = board.rx_meg->state_meg.l_fric_state;
    launch->heart.dial_heart = board.rx_meg->state_meg.dial_motor_state;
}

static void Launch_Cmd_Transmit(Launch_t *launch)
{
    board.tx_pkt->shoot_pkt.launch_state = launch->state;
    board.tx_pkt->shoot_pkt.shoot_mode = launch->mode;
    board.tx_pkt->shoot_pkt.shoot_level = launch->shoot_level;
}

static void Launch_Work(Launch_t *launch)
{
    Launch_Data_Update(launch);
    Launch_Cmd_Transmit(launch);
}
