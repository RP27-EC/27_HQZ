/* launch.c - 发射机构控制 */

#include "launch.h"
#include "board_protocol.h"
#include "rc_sensor.h"
#include "chassis_input.h"
#include "board_comm_config.h"
#include <stddef.h>

static void Launch_Init(Launch_t *launch);
static void Launch_Work(Launch_t *launch);
static void Launch_Offline_Update(Launch_t *launch);

static uint8_t launch_shoot_switch_seen;
static uint8_t launch_shoot_previous_switch;
static uint8_t launch_remote_fric_armed;
static uint8_t launch_shoot_armed;
static uint8_t launch_last_source;
static uint8_t launch_last_g_pressed;
static uint8_t launch_keyboard_fric_on;

typedef enum
{
    LAUNCH_SOURCE_NONE = 0, // 无有效输入，0
    LAUNCH_SOURCE_RC_MID, // 遥控S1中位，1
    LAUNCH_SOURCE_RC_UP, // 遥控S1上位，2
    LAUNCH_SOURCE_KEYBOARD // 键鼠控制，3
} launch_source_e;

static uint8_t launch_s2_filter_initialized;
static uint8_t launch_s2_raw_last;
static uint8_t launch_s2_filtered;
static uint8_t launch_s2_stable_ticks;

/* 发射机构全局对象 */

Launch_t launch =
{
    .state = L_LOCK,       /* 摩擦轮使能，0/1 */
    .mode = SINGLE_SHOT,   /* 单发/连发，0/1 */
    .shoot_lock = 1u,      /* 供弹禁止，0/1 */
    .shoot_level = 0u,     /* 供弹触发，0/1 */
    .feed_permit = 0u,     /* 供弹许可，0/1 */
    .init = Launch_Init,   /* 初始化对象，无量纲 */
};

/* 清空拨杆解锁序列 */
static void Launch_Reset_Shoot_Arm(void)
{
    launch_shoot_switch_seen = 0u;     /* 重新等待首次采样 */
    launch_shoot_previous_switch = 0u;
    launch_remote_fric_armed = 0u;
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
    launch_last_source = LAUNCH_SOURCE_NONE;
    launch_last_g_pressed = 0u;
    launch_keyboard_fric_on = 1u;
    launch->state = L_LOCK;
    launch->mode = SINGLE_SHOT;
    launch->shoot_lock = 1u;
    launch->shoot_level = 0u;
    launch->feed_permit = 0u;
}

/* 根据键鼠或遥控拨杆更新发射状态与模式 */
static void Launch_Data_Update(Launch_t *launch)
{
    uint8_t s1;
    uint8_t s2;
    uint8_t g_pressed;
    uint8_t mouse_pressed;
    launch_source_e source;

    launch->state = L_LOCK;
    launch->mode = SINGLE_SHOT;
    launch->shoot_lock = 1u;
    launch->shoot_level = 0u;
    launch->feed_permit = 0u;

    if ((rc_dev.work_state != DEV_ONLINE) || (rc_dev.info == NULL))
    {
        Launch_Reset_Shoot_Arm();
        Launch_Reset_S2_Filter();
        launch_last_source = LAUNCH_SOURCE_NONE;
        launch_keyboard_fric_on = 1u;
        return;
    }

    s1 = (uint8_t)rc_dev.info->s1.value;
    g_pressed = ((rc_dev.info->key_v & KEY_PRESSED_OFFSET_G) != 0u) ? 1u : 0u;
    if (Chassis_Input_IsKeyboardMode() != 0u)
    {
        source = LAUNCH_SOURCE_KEYBOARD;
    }
    else if (s1 == RC_SW_MID)
    {
        source = LAUNCH_SOURCE_RC_MID;
    }
    else if (s1 == RC_SW_UP)
    {
        source = LAUNCH_SOURCE_RC_UP;
    }
    else
    {
        source = LAUNCH_SOURCE_NONE;
    }

    if (source != launch_last_source)
    {
        Launch_Reset_Shoot_Arm();
        Launch_Reset_S2_Filter();
        launch_keyboard_fric_on = 1u;
        launch_last_g_pressed = g_pressed;
        launch_last_source = (uint8_t)source;
    }

    if ((source == LAUNCH_SOURCE_KEYBOARD) &&
        (g_pressed != 0u) && (launch_last_g_pressed == 0u))
    {
        launch_keyboard_fric_on ^= 1u;
        launch_shoot_armed = 0u;
    }
    launch_last_g_pressed = g_pressed;

#if BOARD_LIFT_ENABLE
    /* 上升期间允许发射 */
    if ((board.tx_pkt->gimbal_target_pkt.is_hole != 0u) ||
        (board.rx_meg->state_meg.is_down == 0u) ||
        (board.rx_meg->state_meg.is_down == 3u))
    {
        launch_shoot_armed = 0u;
        return;
    }
#endif

    if (source == LAUNCH_SOURCE_KEYBOARD)
    {
        if (launch_keyboard_fric_on == 0u)
        {
            launch_shoot_armed = 0u;
            return;
        }
        launch->state = L_UNLOCK;
        mouse_pressed = rc_dev.info->mouse_btn_l.value & 0x01u;
        if (mouse_pressed == 0u)
        {
            launch_shoot_armed = 1u;
        }
        if (launch_shoot_armed != 0u)
        {
            launch->feed_permit = 1u;
            launch->shoot_lock = 0u;
            launch->shoot_level = mouse_pressed;
            launch->mode = ((mouse_pressed != 0u) &&
                           (rc_dev.info->mouse_btn_l.status == long_press)) ?
                          REPEAT_SHOT : SINGLE_SHOT;
        }
        return;
    }

    if (source == LAUNCH_SOURCE_NONE)
    {
        Launch_Reset_Shoot_Arm();
        return;
    }

    s2 = Launch_Filter_S2((uint8_t)rc_dev.info->s2.value);
    if ((s2 != RC_SW_UP) && (s2 != RC_SW_MID) && (s2 != RC_SW_DOWN))
    {
        Launch_Reset_Shoot_Arm();
        return;
    }
    if ((source == LAUNCH_SOURCE_RC_UP) && (s2 == RC_SW_DOWN))
    {
        Launch_Reset_Shoot_Arm();
        return;
    }

    if ((launch_shoot_switch_seen != 0u) &&
        (s2 != launch_shoot_previous_switch))
    {
        launch_remote_fric_armed = 1u;
    }
    launch_shoot_switch_seen = 1u;
    launch_shoot_previous_switch = s2;
    if ((source == LAUNCH_SOURCE_RC_MID) ||
        (launch_remote_fric_armed != 0u))
    {
        launch->state = L_UNLOCK;
    }

    /* 回中后才接受上拨触发 */
    if (s2 == RC_SW_MID)
    {
        launch_shoot_armed = 1u;
    }
    else if (s2 == RC_SW_DOWN)
    {
        launch_shoot_armed = 0u;
    }
    if ((launch->state == L_UNLOCK) && (launch_shoot_armed != 0u))
    {
        launch->feed_permit = 1u;
        launch->shoot_lock = 0u;
        launch->shoot_level = (s2 == RC_SW_UP) ? 1u : 0u;
        launch->mode = ((s1 == RC_SW_UP) && (s2 == RC_SW_UP)) ?
                      REPEAT_SHOT : SINGLE_SHOT;
    }
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
    board.tx_pkt->shoot_pkt.launch_state = launch->state;  /* 摩擦轮使能 */
    board.tx_pkt->shoot_pkt.shoot_mode = launch->mode;    /* 单发/连发 */
    board.tx_pkt->shoot_pkt.shoot_level = launch->shoot_level; /* 触发 */
    board.tx_pkt->shoot_pkt.feed_permit = launch->feed_permit; /* 供弹许可 */
}

/* 发射机构周期任务 */
static void Launch_Work(Launch_t *launch)
{
    Launch_Data_Update(launch);
    Launch_Cmd_Transmit(launch);
}
