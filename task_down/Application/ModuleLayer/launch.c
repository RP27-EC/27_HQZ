/* launch.c - 发射机构控制 */

#include "launch.h"
#include "board_protocol.h"
#include "rc_sensor.h"
#include "chassis_input.h"
#include "board_comm_config.h"

static void Launch_Init(Launch_t *launch);
static void Launch_Work(Launch_t *launch);
static void Launch_Offline_Update(Launch_t *launch);

static uint8_t launch_shoot_switch_seen;     /* S2 已完成首次采样 */
static uint8_t launch_shoot_previous_switch; /* 上拍 S2 原始值 */
static uint8_t launch_remote_feed_ready; // 互锁后释放确认，0/1
static uint8_t launch_shoot_armed;           /* 发射已由拨杆动作解锁 */

static uint8_t launch_s2_filter_initialized;
static uint8_t launch_s2_raw_last;
static uint8_t launch_s2_filtered;
static uint8_t launch_s2_stable_ticks;
static uint8_t launch_keyboard_active; // 上拍键鼠模式，0/1
static uint8_t launch_keyboard_fric_on; // 键鼠摩擦轮使能，0/1
static uint8_t launch_keyboard_last_g; // 上拍G键电平，0/1
static uint8_t launch_keyboard_mouse_ready; // 新按下已解锁，0/1
static uint8_t launch_keyboard_last_mouse; // 上拍左键电平，0/1

/* 发射机构全局对象 */

Launch_t launch =
{
    .state = L_LOCK,       /* 默认锁定 */
    .mode = SINGLE_SHOT,   /* 默认单发 */
    .shoot_lock = 1,       /* 默认锁定发射 */
    .shoot_level = 0,      /* 默认无触发 */
    .init = Launch_Init,   /* 初始化接口 */
};

/* 清空拨杆解锁序列 */
static void Launch_Reset_Shoot_Arm(void)
{
    launch_shoot_switch_seen = 0u;     /* 重新等待首次采样 */
    launch_shoot_previous_switch = 0u;
    launch_shoot_armed = 0u;           /* 清除解锁 */
}

/* 清除 S2 消抖状态，重新上线时先接受当前档位。 */
static void Launch_Reset_S2_Filter(void)
{
    launch_s2_filter_initialized = 0u;
    launch_s2_raw_last = 0u;
    launch_s2_filtered = RC_SW_MID;
    launch_s2_stable_ticks = 0u;
}

/* S2 连续稳定后才承认新档位，滤除回中位时的接触弹跳。 */
static uint8_t Launch_Filter_S2(uint8_t raw)
{
    if (launch_s2_filter_initialized == 0u)
    {
        launch_s2_filter_initialized = 1u;
        launch_s2_raw_last = raw;
        launch_s2_filtered = raw;
        launch_s2_stable_ticks = BOARD_LAUNCH_S2_DEBOUNCE_TICKS;
        return launch_s2_filtered;
    }

    if (raw != launch_s2_raw_last)
    {
        launch_s2_raw_last = raw;
        launch_s2_stable_ticks = 0u;
    }
    else if (launch_s2_stable_ticks < BOARD_LAUNCH_S2_DEBOUNCE_TICKS)
    {
        launch_s2_stable_ticks++;
    }

    if (launch_s2_stable_ticks >= BOARD_LAUNCH_S2_DEBOUNCE_TICKS)
    {
        launch_s2_filtered = raw;
    }

    return launch_s2_filtered;
}

/* 绑定周期任务与心跳接口 */
static void Launch_Init(Launch_t *launch)
{
    launch->work = Launch_Work;                 /* 绑定周期任务 */
    launch->heart_beat = Launch_Offline_Update; /* 绑定心跳任务 */
    Launch_Reset_S2_Filter();
    Launch_Reset_Shoot_Arm();
    launch_keyboard_active = 0u;
    launch_keyboard_fric_on = 0u;
    launch_keyboard_last_g = 0u;
    launch_keyboard_mouse_ready = 0u;
    launch_keyboard_last_mouse = 0u;
    launch_remote_feed_ready = 0u;
    launch->feed_permit = 0u;
    launch->s2_mid = 0u;
    launch->ctrl_source = 0u;
    launch->block_reason = 1u;
}

/* 根据键鼠或遥控拨杆更新发射状态与模式 */
static void Launch_Data_Update(Launch_t *launch)
{
    uint8_t s1;
    uint8_t s2;
    uint8_t g_now;
    uint8_t mouse_down;
    uint8_t feed_safe = 1u;
    uint8_t switch_changed;

    launch->state = L_LOCK;
    launch->mode = SINGLE_SHOT;
    launch->shoot_level = 0u;
    launch->feed_permit = 0u;
    launch->s2_mid = 0u;
    launch->ctrl_source = 0u;
    launch->block_reason = 1u;
    if ((rc_dev.work_state != DEV_ONLINE) ||
        (board.status->status != DEV_ONLINE))
    {
        Launch_Reset_Shoot_Arm();
        Launch_Reset_S2_Filter();
        launch_keyboard_active = 0u;
        launch_keyboard_fric_on = 0u;
        launch_keyboard_mouse_ready = 0u;
        launch_keyboard_last_g = 0u;
        launch_keyboard_last_mouse = 0u;
        launch_remote_feed_ready = 0u;
        launch->block_reason = 3u;
        return;
    }
#if BOARD_LIFT_ENABLE
    feed_safe = ((board.tx_pkt->gimbal_target_pkt.is_hole == 0u) &&
                 (board.rx_meg->state_meg.is_down == 2u)) ? 1u : 0u;
#endif
    launch->feed_permit = feed_safe;
    launch->ctrl_source = (Chassis_Input_IsKeyboardMode() != 0u) ? 1u : 0u;
    if (launch->ctrl_source != 0u)
    {
        Launch_Reset_Shoot_Arm();
        Launch_Reset_S2_Filter();
        launch_remote_feed_ready = 0u;
        g_now = ((rc_dev.info->key_v & KEY_PRESSED_OFFSET_G) != 0u) ? 1u : 0u;
        mouse_down = (uint8_t)(rc_dev.info->mouse_btn_l.value & 0x01u);
        if (launch_keyboard_active == 0u)
        {
            launch_keyboard_active = 1u;
            launch_keyboard_fric_on = 0u;
            launch_keyboard_last_g = g_now;
            launch_keyboard_mouse_ready = 0u;
            launch_keyboard_last_mouse = mouse_down;
        }
        if (feed_safe == 0u) launch_keyboard_mouse_ready = 0u;
        else if (mouse_down == 0u) launch_keyboard_mouse_ready = 1u;
        if ((g_now != 0u) && (launch_keyboard_last_g == 0u))
        {
            launch_keyboard_fric_on ^= 1u;
            launch_keyboard_mouse_ready = 0u;
        }
        else if ((mouse_down != 0u) && (launch_keyboard_last_mouse == 0u) &&
                 (launch_keyboard_mouse_ready != 0u))
        {
            launch_keyboard_fric_on = 1u;
        }
        launch_keyboard_last_g = g_now;
        launch_keyboard_last_mouse = mouse_down;
        launch->state = (launch_keyboard_fric_on != 0u) ? L_UNLOCK : L_LOCK;
        if ((feed_safe != 0u) && (mouse_down != 0u) &&
            (launch_keyboard_mouse_ready != 0u) && (launch_keyboard_fric_on != 0u))
        {
            launch->mode = (rc_dev.info->mouse_btn_l.status == long_press) ?
                           REPEAT_SHOT : SINGLE_SHOT;
            launch->shoot_level = 1u;
        }
        launch->block_reason = (feed_safe == 0u) ? 2u :
            ((launch->state == L_LOCK) ? 1u :
             ((launch_keyboard_mouse_ready == 0u) ? 5u : 0u));
        return;
    }

    if (launch_keyboard_active != 0u)
    {
        Launch_Reset_Shoot_Arm();
        Launch_Reset_S2_Filter();
        launch_remote_feed_ready = 0u;
    }
    launch_keyboard_active = 0u;
    launch_keyboard_fric_on = 0u;
    launch_keyboard_mouse_ready = 0u;
    launch_keyboard_last_g = 0u;
    s1 = (uint8_t)rc_dev.info->s1.value;
    s2 = Launch_Filter_S2((uint8_t)rc_dev.info->s2.value);
    launch->s2_mid = (s2 == RC_SW_MID) ? 1u : 0u;
    switch_changed = ((launch_shoot_switch_seen != 0u) &&
                      (s2 != launch_shoot_previous_switch)) ? 1u : 0u;
    launch_shoot_switch_seen = 1u;
    launch_shoot_previous_switch = s2;

    if (((s1 != RC_SW_UP) && (s1 != RC_SW_MID)) ||
        ((s1 == RC_SW_UP) && (s2 == RC_SW_DOWN)))
    {
        launch_shoot_armed = 0u;
        launch_remote_feed_ready = 0u;
        launch->feed_permit = 0u;
        launch->block_reason = 4u;
        return;
    }
    if (switch_changed != 0u) launch_shoot_armed = 1u;
    if (feed_safe == 0u) launch_remote_feed_ready = 0u;
    else if (s2 != RC_SW_UP) launch_remote_feed_ready = 1u;

    if ((launch_shoot_armed != 0u) &&
        ((s2 == RC_SW_UP) || (s2 == RC_SW_MID)))
    {
        launch->state = L_UNLOCK;
        if ((feed_safe != 0u) && (launch_remote_feed_ready != 0u) &&
            (s2 == RC_SW_UP))
        {
            launch->mode = (s1 == RC_SW_UP) ? REPEAT_SHOT : SINGLE_SHOT;
            launch->shoot_level = 1u;
        }
    }
    launch->block_reason = (feed_safe == 0u) ? 2u :
        ((launch->state == L_LOCK) ? 1u :
         ((launch_remote_feed_ready == 0u) ? 5u : 0u));
}

/* 从板间状态同步发射子设备在线信息 */
static void Launch_Offline_Update(Launch_t *launch)
{
    launch->heart.r_fric_heart = board.rx_meg->state_meg.r_fric_state; /* 右轮 */
    launch->heart.l_fric_heart = board.rx_meg->state_meg.l_fric_state; /* 左轮 */
    launch->heart.dial_heart = board.rx_meg->state_meg.dial_motor_state; /* 拨盘 */
}

/* 将许可、模式和触发电平写入板间报文 */
static void Launch_Cmd_Transmit(Launch_t *launch)
{
    board.tx_pkt->shoot_pkt.launch_state = launch->state;  /* 发射许可 */
    board.tx_pkt->shoot_pkt.shoot_mode = launch->mode;    /* 单发/连发 */
    board.tx_pkt->shoot_pkt.shoot_level = launch->shoot_level; /* 触发 */
    board.tx_pkt->shoot_pkt.feed_permit = launch->feed_permit;
    board.tx_pkt->shoot_pkt.s2_mid = launch->s2_mid;
    board.tx_pkt->shoot_pkt.ctrl_source = launch->ctrl_source;
}

/* 发射机构周期任务 */
static void Launch_Work(Launch_t *launch)
{
    Launch_Data_Update(launch);
    Launch_Cmd_Transmit(launch);
}
