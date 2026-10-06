/* launch.c - 发射机构控制 */

#include "launch.h"
#include "board_protocol.h"
#include "rc_sensor.h"
#include "chassis_input.h"
#include "board_comm_config.h"
#include "rc_protocol.h"

static void Launch_Init(Launch_t *launch);
static void Launch_Work(Launch_t *launch);
static void Launch_Offline_Update(Launch_t *launch);

static uint8_t launch_shoot_switch_seen;     /* S2 已完成首次采样 */
static uint8_t launch_shoot_previous_switch; /* 上拍 S2 原始值 */
static uint8_t launch_shoot_armed;           /* 发射已由拨杆动作解锁 */

static uint8_t launch_s2_filter_initialized;
static uint8_t launch_s2_raw_last;
static uint8_t launch_s2_filtered;
static uint8_t launch_s2_stable_ticks;

static uint8_t launch_single_seq;
static uint8_t launch_reset_seq;
static uint8_t launch_s1_seen;
static uint8_t launch_s1_previous;
static uint8_t launch_source = 2u;
static uint8_t launch_mouse_armed;
static uint32_t launch_mouse_press_seq;
static uint32_t launch_mouse_release_seq;

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
}

/* 转轮许可与供弹解锁分离 */
static void Launch_Data_Update(Launch_t *launch)
{
    rc_mouse_shot_t mouse;
    uint8_t s1;
    uint8_t s2;
    uint8_t keyboard;
    uint8_t previous_s2;

    Rc_GetMouseShotSnapshot(&mouse);
    launch->state = L_LOCK;
    launch->mode = SINGLE_SHOT;
    launch->shoot_level = 0u;
    if (rc_dev.work_state != DEV_ONLINE)
    {
        Launch_Reset_Shoot_Arm();
        Launch_Reset_S2_Filter();
        launch_s1_seen = 0u;
        launch_source = 2u;
        launch_mouse_armed = 0u;
        launch_mouse_press_seq = mouse.press_seq;
        launch_mouse_release_seq = mouse.release_seq;
        return;
    }

    s1 = (uint8_t)rc_dev.info->s1.value;
    if ((launch_s1_seen != 0u) &&
        (launch_s1_previous == RC_SW_DOWN) && (s1 == RC_SW_MID))
    {
        launch_reset_seq++;
    }
    launch_s1_seen = 1u;
    launch_s1_previous = s1;
    keyboard = Chassis_Input_IsKeyboardMode();
    if (launch_source != keyboard)
    {
        Launch_Reset_Shoot_Arm();
        Launch_Reset_S2_Filter();
        launch_mouse_press_seq = mouse.press_seq;
        launch_mouse_release_seq = mouse.release_seq;
        launch_mouse_armed = (mouse.pressed == 0u) ? 1u : 0u;
        launch_source = keyboard;
    }

    if (keyboard != 0u)
    {
        launch->state = L_UNLOCK;
        if ((mouse.pressed == 0u) ||
            (mouse.release_seq != launch_mouse_release_seq))
        {
            launch_mouse_armed = 1u;
        }
        if ((mouse.press_seq != launch_mouse_press_seq) &&
            (launch_mouse_armed != 0u))
        {
            launch_single_seq++;
        }
        launch_mouse_press_seq = mouse.press_seq;
        launch_mouse_release_seq = mouse.release_seq;
        if ((mouse.pressed != 0u) && (launch_mouse_armed != 0u))
        {
            launch->shoot_level = 1u;
            launch->mode = (rc_dev.info->mouse_btn_l.status == long_press) ?
                           REPEAT_SHOT : SINGLE_SHOT;
        }
        return;
    }

    launch_mouse_press_seq = mouse.press_seq;
    launch_mouse_release_seq = mouse.release_seq;
#if BOARD_LIFT_ENABLE
    if ((board.tx_pkt->gimbal_target_pkt.is_hole != 0u) ||
        (board.rx_meg->state_meg.is_down != 2u))
    {
        Launch_Reset_Shoot_Arm();
        Launch_Reset_S2_Filter();
        return;
    }
#endif
    s2 = Launch_Filter_S2((uint8_t)rc_dev.info->s2.value);
    if (((s1 != RC_SW_UP) && (s1 != RC_SW_MID)) ||
        ((s1 == RC_SW_UP) && (s2 == RC_SW_DOWN)))
    {
        Launch_Reset_Shoot_Arm();
        return;
    }

    launch->state = L_UNLOCK;
    previous_s2 = launch_shoot_previous_switch;
    if (s2 == RC_SW_MID)
    {
        launch_shoot_armed = 1u;
    }
    if ((launch_shoot_switch_seen != 0u) &&
        (launch_shoot_armed != 0u) && (s2 == RC_SW_UP))
    {
        launch->mode = (s1 == RC_SW_UP) ? REPEAT_SHOT : SINGLE_SHOT;
        launch->shoot_level = 1u;
        if ((previous_s2 != RC_SW_UP) && (launch->mode == SINGLE_SHOT))
        {
            launch_single_seq++;
        }
    }
    launch_shoot_switch_seen = 1u;
    launch_shoot_previous_switch = s2;
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
    uint32_t irq_state = __get_PRIMASK();
    /* 防止发送任务读到半更新字段 */
    __disable_irq();
    board.tx_pkt->shoot_pkt.launch_state = (uint8_t)launch->state;
    board.tx_pkt->shoot_pkt.shoot_mode = (uint8_t)launch->mode;
    board.tx_pkt->shoot_pkt.shoot_level = launch->shoot_level;
    board.tx_pkt->shoot_pkt.single_seq = launch_single_seq;
    board.tx_pkt->shoot_pkt.reset_seq = launch_reset_seq;
    __set_PRIMASK(irq_state);
}

/* 发射机构周期任务 */
static void Launch_Work(Launch_t *launch)
{
    Launch_Data_Update(launch);
    Launch_Cmd_Transmit(launch);
}
