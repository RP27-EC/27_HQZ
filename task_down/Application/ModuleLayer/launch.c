/* launch.c - 发射机构控制 */

#include "launch.h"
#include "board_protocol.h"
#include "rc_sensor.h"
#include "chassis_input.h"
#include "board_comm_config.h"

static void Launch_Init(Launch_t *launch);
static void Launch_Work(Launch_t *launch);
static void Launch_Offline_Update(Launch_t *launch);

typedef enum
{
    LAUNCH_RC_WAIT_MID = 0, // 等待稳定中位，值0
    LAUNCH_RC_READY = 1     // 中位确认待发，值1
} launch_rc_arm_e;

static launch_rc_arm_e launch_rc_arm;
static uint8_t launch_interlock_active; // 上拍机构互锁，0/1

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
    launch_rc_arm = LAUNCH_RC_WAIT_MID;
}

// NOTE: 重连首档也须连续消抖
static void Launch_Reset_S2_Filter(void)
{
    launch_s2_filter_initialized = 0u;
    launch_s2_raw_last = 0u;
    launch_s2_filtered = 0u;
    launch_s2_stable_ticks = 0u;
}

// NOTE: 连续稳定后承认新档位
static uint8_t Launch_Filter_S2(uint8_t raw)
{
    if (launch_s2_filter_initialized == 0u)
    {
        launch_s2_filter_initialized = 1u;
        launch_s2_raw_last = raw;
        launch_s2_stable_ticks = 0u;
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
    launch_interlock_active = 0u;
    launch_keyboard_active = 0u;
    launch_keyboard_fric_on = 0u;
    launch_keyboard_last_g = 0u;
    launch_keyboard_mouse_ready = 0u;
    launch_keyboard_last_mouse = 0u;
}

/* 根据键鼠或遥控拨杆更新发射状态与模式 */
static void Launch_Data_Update(Launch_t *launch)
{
    uint8_t s1; /* S1 档位 */
    uint8_t s2; /* S2 档位 */
    uint8_t s2_raw;
    uint8_t middle_stable;
    uint8_t g_now;
    uint8_t mouse_down;

    launch->state = L_LOCK;
    launch->mode = SINGLE_SHOT;
    launch->shoot_level = 0u;
    if (rc_dev.work_state != DEV_ONLINE)
    {
        Launch_Reset_Shoot_Arm();
        Launch_Reset_S2_Filter();
        launch_interlock_active = 0u;
        launch_keyboard_active = 0u;
        launch_keyboard_fric_on = 0u;
        launch_keyboard_last_g = 0u;
        launch_keyboard_mouse_ready = 0u;
        launch_keyboard_last_mouse = 0u;
        return;
    }

    if (Chassis_Input_IsKeyboardMode() != 0u)
    {
        Launch_Reset_Shoot_Arm();
        Launch_Reset_S2_Filter();
        launch_interlock_active = 0u;
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
        if (mouse_down == 0u) launch_keyboard_mouse_ready = 1u;
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
        if ((mouse_down != 0u) && (launch_keyboard_mouse_ready != 0u) &&
            (launch_keyboard_fric_on != 0u))
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

    launch_keyboard_active = 0u;
    launch_keyboard_fric_on = 0u;
    launch_keyboard_mouse_ready = 0u;
    launch_keyboard_last_g = 0u;
    launch_keyboard_last_mouse = 0u;

    s1 = (uint8_t)rc_dev.info->s1.value;
    s2_raw = (uint8_t)rc_dev.info->s2.value;
    if ((s2_raw != RC_SW_UP) && (s2_raw != RC_SW_MID) && (s2_raw != RC_SW_DOWN))
    {
        Launch_Reset_Shoot_Arm();
        Launch_Reset_S2_Filter();
        return;
    }
    s2 = Launch_Filter_S2(s2_raw);
    middle_stable = ((s2 == RC_SW_MID) && (s2_raw == RC_SW_MID) &&
        (launch_s2_stable_ticks >= BOARD_LAUNCH_S2_DEBOUNCE_TICKS)) ? 1u : 0u;

    if (((s1 != RC_SW_UP) && (s1 != RC_SW_MID)) ||
        ((s1 == RC_SW_UP) && (s2 == RC_SW_DOWN)))
    {
        Launch_Reset_Shoot_Arm();
        return;
    }

#if BOARD_LIFT_ENABLE
    if ((board.tx_pkt->gimbal_target_pkt.is_hole != 0u) ||
        (board.rx_meg->state_meg.is_down != 2u))
    {
        launch_interlock_active = 1u;
        if (middle_stable != 0u) launch_rc_arm = LAUNCH_RC_READY;
        else if (s2_raw != RC_SW_MID) Launch_Reset_Shoot_Arm();
        return;
    }
    if (launch_interlock_active != 0u)
    {
        // NOTE: 互锁内上拨不排队
        Launch_Reset_Shoot_Arm();
        launch_interlock_active = 0u;
    }
#endif

    if (middle_stable != 0u)
    {
        launch_rc_arm = LAUNCH_RC_READY;
    }
    else if (s2 == RC_SW_DOWN)
    {
        Launch_Reset_Shoot_Arm();
    }
    if ((launch_rc_arm == LAUNCH_RC_READY) && (s2 == RC_SW_UP))
    {
        launch->state = L_UNLOCK; /* S2 上拨发射 */
        launch->mode = (s1 == RC_SW_UP) ? REPEAT_SHOT : SINGLE_SHOT;
        launch->shoot_level = 1u;
    }
    else if ((launch_rc_arm == LAUNCH_RC_READY) && (s2 == RC_SW_MID))
    {
        launch->state = L_UNLOCK; /* S2 中位待发 */
        launch->mode = SINGLE_SHOT;
        launch->shoot_level = 0u;
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
    board.tx_pkt->shoot_pkt.launch_state = launch->state;  /* 发射许可 */
    board.tx_pkt->shoot_pkt.shoot_mode = launch->mode;    /* 单发/连发 */
    board.tx_pkt->shoot_pkt.shoot_level = launch->shoot_level; /* 触发 */
}

/* 发射机构周期任务 */
static void Launch_Work(Launch_t *launch)
{
    Launch_Data_Update(launch);
    Launch_Cmd_Transmit(launch);
}
