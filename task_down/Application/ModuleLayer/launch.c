/* launch.c - 发射机构控制 */

#include "launch.h"
#include "board_protocol.h"
#include "rc_sensor.h"
#include "chassis_input.h"

static void Launch_Init(Launch_t *launch);
static void Launch_Work(Launch_t *launch);
static void Launch_Offline_Update(Launch_t *launch);

static uint8_t launch_shoot_switch_seen;     /* S2 已完成首次采样 */
static uint8_t launch_shoot_previous_switch; /* 上拍 S2 原始值 */
static uint8_t launch_shoot_armed;           /* 发射已由拨杆动作解锁 */

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

/* 绑定周期任务与心跳接口 */
static void Launch_Init(Launch_t *launch)
{
    launch->work = Launch_Work;                 /* 绑定周期任务 */
    launch->heart_beat = Launch_Offline_Update; /* 绑定心跳任务 */
    Launch_Reset_Shoot_Arm();
}

/* 根据键鼠或遥控拨杆更新发射状态与模式 */
static void Launch_Data_Update(Launch_t *launch)
{
    uint8_t s1; /* S1 档位 */
    uint8_t s2; /* S2 档位 */

    if (Chassis_Input_IsKeyboardMode() != 0u)
    {
        Launch_Reset_Shoot_Arm();
        launch->state = L_UNLOCK; /* 键鼠直接解锁 */

        /* 键鼠：左键按下发射，长按切连发 */
        if ((rc_dev.info->mouse_btn_l.value & 0x01u) != 0u)
        {
            launch->mode = (rc_dev.info->mouse_btn_l.status == long_press) ?
                           REPEAT_SHOT : SINGLE_SHOT;
            launch->shoot_level = 1u; /* 触发发射 */
        }
        else
        {
            launch->mode = SINGLE_SHOT; /* 松开回单发 */
            launch->shoot_level = 0u;   /* 取消触发 */
        }

        return;
    }

    /* 遥控掉线立即锁定 */
    if (rc_dev.work_state != DEV_ONLINE)
    {
    Launch_Reset_Shoot_Arm(); /* 掉线清除解锁 */
    launch->state = L_LOCK;
    launch->mode = SINGLE_SHOT;
    launch->shoot_level = 0u;
    return;
    }

    s1 = (uint8_t)rc_dev.info->s1.value; /* 读取 S1 */
    s2 = (uint8_t)rc_dev.info->s2.value; /* 读取 S2 */

    /* 小陀螺档位禁止发射，避免机构互锁 */
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
    /* 安全解锁：上电后 S2 必须再动作一次 */
    if ((launch_shoot_switch_seen != 0u) &&
        (s2 != launch_shoot_previous_switch))
    {
        launch_shoot_armed = 1u; /* 拨杆动作后解锁 */
    }
    launch_shoot_switch_seen = 1u;
    launch_shoot_previous_switch = s2;

    if ((launch_shoot_armed != 0u) && (s2 == RC_SW_UP))
    {
        launch->state = L_UNLOCK; /* S2 上拨发射 */
        launch->mode = (s1 == RC_SW_UP) ? REPEAT_SHOT : SINGLE_SHOT;
        launch->shoot_level = 1u;
    }
    else if ((launch_shoot_armed != 0u) && (s2 == RC_SW_MID))
    {
        launch->state = L_UNLOCK; /* S2 中位待发 */
        launch->mode = SINGLE_SHOT;
        launch->shoot_level = 0u;
    }
    else
    {
        launch->state = L_LOCK; /* 未解锁 */
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
