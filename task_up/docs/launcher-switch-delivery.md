# 摩擦轮开关与上升期间发射：完整代码交付

- G 在键鼠模式按下切换一次；S1 中位默认开启；关闭不供弹。
- 正常上升即可发射；底部、下降及升降故障仍互锁，解除后重新触发即可恢复。
- D1 b5 bit0 为转轮使能、bit5 为供弹许可；上下板须同步更新。
- 未编译、未测试、未烧录；只做源码回读和静态 diff 审查。

## 下板状态与初始化配置（launch.c）

```c
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
```

## 下板 Launch_Reset_Shoot_Arm

```c
static void Launch_Reset_Shoot_Arm(void)
{
    launch_shoot_switch_seen = 0u;     /* 重新等待首次采样 */
    launch_shoot_previous_switch = 0u;
    launch_remote_fric_armed = 0u;
    launch_shoot_armed = 0u;           /* 清除解锁 */
}
```

## 下板 Launch_Init

```c
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
```

## 下板 Launch_Data_Update

```c
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
```

## 下板 Launch_Cmd_Transmit

```c
static void Launch_Cmd_Transmit(Launch_t *launch)
{
    board.tx_pkt->shoot_pkt.launch_state = launch->state;  /* 摩擦轮使能 */
    board.tx_pkt->shoot_pkt.shoot_mode = launch->mode;    /* 单发/连发 */
    board.tx_pkt->shoot_pkt.shoot_level = launch->shoot_level; /* 触发 */
    board.tx_pkt->shoot_pkt.feed_permit = launch->feed_permit; /* 供弹许可 */
}
```

## 下板对象及枚举（launch.h）

```c
/* launch.h - 发射机构控制 */

#ifndef __LAUNCH_H
#define __LAUNCH_H

#include "stdint.h"

/* 发射许可状态 */
typedef enum{
	L_LOCK = 0, /* 摩擦轮关闭，0 */
  L_UNLOCK,   /* 摩擦轮开启，1 */
}Launch_State_e;


/* 发射模式 */
typedef enum{
	SINGLE_SHOT, /* 单发模式，0 */
	REPEAT_SHOT, /* 连发模式，1 */
}Launch_Mode_e;


/* 发射机构子设备在线状态 */
typedef struct{
	uint8_t r_fric_heart; /* 右轮在线，0/1 */
  uint8_t l_fric_heart; /* 左轮在线，0/1 */
	uint8_t dial_heart;   /* 拨盘在线，0/1 */
}Launch_Heart_t;


/* 发射机构对象 */
typedef struct Launch_Struct_t{
	Launch_State_e state; /* 摩擦轮使能，0/1 */
  Launch_Mode_e mode;   /* 单发/连发，0/1 */
	Launch_Heart_t heart; /* 三电机在线，各0/1 */

	uint8_t shoot_level;  /* 供弹触发，0/1 */
	uint8_t shoot_lock;   /* 供弹禁止，0/1 */
	uint8_t feed_permit;  /* 供弹许可，0/1 */

	void (*init)(struct Launch_Struct_t* launch);       /* 初始化对象，无量纲 */
	void (*work)(struct Launch_Struct_t* launch);       /* 控制更新，周期1 ms */
	void (*heart_beat)(struct Launch_Struct_t* launch); /* 在线采样，值0/1 */
}Launch_t;

extern  Launch_t  launch;
#endif
```

## task_down/Application/ProtocolLayer/board_protocol.h

```c
typedef struct{
	uint8_t launch_state; /* 摩擦轮使能，0/1 */
  uint8_t shoot_mode;   /* 单发/连发，0/1 */
	uint8_t shoot_level;  /* 供弹触发，0/1 */
  uint8_t feed_permit;  /* 供弹许可，0/1 */
}Board_Shoot_Pkt_t;
```

## task_up/Application/ProtocolLayer/communicate.h

```c
typedef struct
{
    uint8_t launch_state; /* 摩擦轮使能，0/1 */
    uint8_t shoot_mode;   /* 单发/连发，0/1 */
    uint8_t shoot_level;  /* 供弹触发，0/1 */
    uint8_t is_hole;      /* 过洞请求，0/1 */
    uint8_t feed_permit;  /* 供弹许可，0/1 */
} Board_Shoot_Pkt_t;
```

## 下板 D1 完整打包函数

```c
void Board_Tx_Pkt_01(Board_t* board)
{
	board->status->gimbal_d1_tx_ok = 0u;
	board->status->gimbal_d2_tx_ok = 0u;
	memset(pkt_01, 0, 8); /* 清空缓存 */
	
	pkt_01[0] |= (board->tx_pkt->car_pkt.car_state & 0x03) << 0;   /* 车辆状态 */
	pkt_01[0] |= (board->tx_pkt->car_pkt.gimbal_mode & 0x01) << 2; /* 云台模式 */
	pkt_01[0] |= (board->tx_pkt->car_pkt.vision_mode & 0x07) << 3; /* 视觉模式 */
	pkt_01[0] |= (board->tx_pkt->car_pkt.game_start & 0x01) << 6;  /* 比赛开始 */
	pkt_01[0] |= (board->tx_pkt->car_pkt.my_color & 0x01) << 7;    /* 己方颜色 */
	
	uint16_t t1,t2; /* 速度压缩值 */
	
	t1 = float_to_uint(board->tx_pkt->car_pkt.v_x,-8000.f,8000.f,16); /* v_x 压缩值 */
	t2 = float_to_uint(board->tx_pkt->car_pkt.v_y,-8000.f,8000.f,16); /* v_y 压缩值 */
	
	pkt_01[1] = t1>>8; /* v_x 高字节 */
	pkt_01[2] = t1;    /* v_x 低字节 */
	pkt_01[3] = t2>>8; /* v_y 高字节 */
	pkt_01[4] = t2;    /* v_y 低字节 */

									 
	pkt_01[5] |= (board->tx_pkt->shoot_pkt.launch_state & 0x01) << 0; /* 摩擦轮使能 */
	pkt_01[5] |= (board->tx_pkt->shoot_pkt.shoot_mode & 0x01) << 1; /* 发射模式 */
	pkt_01[5] |= (board->tx_pkt->shoot_pkt.shoot_level & 0x01) << 2; /* 触发电平 */
	pkt_01[5] |= (board->tx_pkt->gimbal_target_pkt.is_hole & 0x01) << 3; /* 过洞标志 */
	pkt_01[5] |= (board->tx_pkt->car_pkt.r_turn_active & 0x01) << 4; /* R掉头，0/1 */
	pkt_01[5] |= (board->tx_pkt->shoot_pkt.feed_permit & 0x01) << 5; /* 供弹许可 */
	

	board->status->gimbal_d1_tx_ok =
		(CAN_SendData(&hfdcan2, ID_PKT_01, pkt_01) == HAL_OK) ? 1u : 0u;
	
	
}
```

## 上板 D1 完整解析函数

```c
static void Board_Rx_Pkt_01(uint8_t *rxbuf)
{
    Board_Rx_Info.state_pkt.car_state = rxbuf[0] & 0x03;          /* 车辆状态 */
    Board_Rx_Info.state_pkt.gimbal_mode = (rxbuf[0] >> 2) & 0x01; /* 云台模式 */
    Board_Rx_Info.state_pkt.vision_mode = (rxbuf[0] >> 3) & 0x07; /* 视觉模式 */
    Board_Rx_Info.state_pkt.game_start = (rxbuf[0] >> 6) & 0x01;  /* 比赛开始 */
    Board_Rx_Info.state_pkt.my_color = (rxbuf[0] >> 7) & 0x01;    /* 己方颜色 */

    Board_Rx_Shoot_Flags = rxbuf[5] & 0x3Fu;
    Board_Rx_Info.shoot_pkt.launch_state = rxbuf[5] & 0x01;       /* 摩擦轮使能 */
    Board_Rx_Info.shoot_pkt.shoot_mode = (rxbuf[5] >> 1) & 0x01;  /* 发射模式 */
    Board_Rx_Info.shoot_pkt.shoot_level = (rxbuf[5] >> 2) & 0x01; /* 触发电平 */
    Board_Rx_Info.shoot_pkt.is_hole = (rxbuf[5] >> 3) & 0x01;     /* 过洞标志 */
    Board_Rx_Info.state_pkt.r_turn_active = (rxbuf[5] >> 4) & 0x01; /* R掉头，0/1 */
    Board_Rx_Info.shoot_pkt.feed_permit = (rxbuf[5] >> 5) & 0x01; /* 供弹许可 */
}
```

## 上板新增运行变量

```c
static uint8_t launcher_feed_armed; /* 新供弹已解锁，0/1 */
```

## 上板 Launcher_Init

```c
void Launcher_Init(void)
{
    pid_ctrl_t *pid; /* 摩擦轮速度环临时指针 */

    memset(&launcher_heat, 0, sizeof(launcher_heat));
    memset(&launcher_heat_runtime, 0, sizeof(launcher_heat_runtime));
    launcher_heat_runtime.update_tick = HAL_GetTick();
    launcher_heat.blocked = 1u;

    launcher.state = LAUNCHER_SLEEP;
    launcher.state_tick = 0u;
    launcher.last_repeat_tick = 0u;
    launcher.jam_tick = 0u;
    launcher.fric_target_rpm = 0.0f;
    launcher.fric_l_speed_rpm = 0.0f;
    launcher.fric_r_speed_rpm = 0.0f;
    launcher.dial_angle = 0;
    launcher.dial_target_angle = 0;
    launcher.dial_zero_angle = 0;
    launcher.enabled = 0u;
    launcher.fric_ready = 0u;
    launcher.dial_online = 0u;
    launcher.last_shoot_level = 0u;
    launcher.fault = 0u;

    launcher_fric_ready_count = 0u;
    launcher_jam_count = 0u;
    launcher_fric_stop_count = 0u;
    launcher_dial_last_online = 0u;
    launcher_feed_armed = 0u;
    launcher_dial_stopped = 1u;
    launcher_dial_braking = 0u;
    launcher_dial_brake_tick = 0u;
    launcher_dial_stop_tick = 0u;
    launcher_dial_target_synced = 0u;
    launcher_dial_recovery_repeat = 0u;
#if LAUNCHER_DIAL_JAM_ENABLE
    launcher_dial_motion_direction = (int8_t)LAUNCHER_DIAL_DIRECTION;
#endif
    launcher_dial_target = 0;
    launcher_dial_feed_target = 0;

    for (uint8_t i = 0u; i < SHOOT_FRIC_NUM; i++)
    {
        pid = rm_motor[i].ctrl->speed_ctrl;
        pid->kp = LAUNCHER_FRIC_KP;
        pid->ki = LAUNCHER_FRIC_KI;
        pid->kd = LAUNCHER_FRIC_KD;
        pid->integral = 0.0f;
        pid->integral_max = LAUNCHER_FRIC_INTEGRAL_MAX;
        pid->out_max = LAUNCHER_FRIC_OUT_MAX;
        pid->deadband = 0.0f;
        pid->d_filter_alpha = 0.0f;
        pid->out = 0.0f;
    }

    launcher_dial_angle_pid.kp = LAUNCHER_DIAL_ANGLE_KP;
    launcher_dial_angle_pid.ki = LAUNCHER_DIAL_ANGLE_KI;
    launcher_dial_angle_pid.kd = LAUNCHER_DIAL_ANGLE_KD;
    launcher_dial_angle_pid.integral = 0.0f;
    launcher_dial_angle_pid.integral_max = LAUNCHER_DIAL_ANGLE_INTEGRAL_MAX;
    launcher_dial_angle_pid.out_max = (float)LAUNCHER_DIAL_MAX_SPEED_DPS;
    launcher_dial_angle_pid.deadband = LAUNCHER_DIAL_ANGLE_DEADBAND;
    launcher_dial_angle_pid.d_filter_alpha = 0.0f;
    launcher_dial_angle_pid.out = 0.0f;

    launcher_dial_speed_pid.kp = LAUNCHER_DIAL_SPEED_KP;
    launcher_dial_speed_pid.ki = LAUNCHER_DIAL_SPEED_KI;
    launcher_dial_speed_pid.kd = LAUNCHER_DIAL_SPEED_KD;
    launcher_dial_speed_pid.integral = 0.0f;
    launcher_dial_speed_pid.integral_max =
        LAUNCHER_DIAL_SPEED_INTEGRAL_MAX;
    launcher_dial_speed_pid.out_max = LAUNCHER_DIAL_SPEED_OUT_MAX;
    launcher_dial_speed_pid.deadband = 0.0f;
    launcher_dial_speed_pid.d_filter_alpha = 0.0f;
    launcher_dial_speed_pid.out = 0.0f;

    launcher_dial_repeat_pid.kp = LAUNCHER_DIAL_REPEAT_KP;
    launcher_dial_repeat_pid.ki = LAUNCHER_DIAL_REPEAT_KI;
    launcher_dial_repeat_pid.kd = LAUNCHER_DIAL_REPEAT_KD;
    launcher_dial_repeat_pid.integral = 0.0f;
    launcher_dial_repeat_pid.integral_max =
        LAUNCHER_DIAL_REPEAT_INTEGRAL_MAX;
    launcher_dial_repeat_pid.out_max = LAUNCHER_DIAL_REPEAT_OUT_MAX;
    launcher_dial_repeat_pid.deadband = 0.0f;
    launcher_dial_repeat_pid.d_filter_alpha = 0.0f;
    launcher_dial_repeat_pid.out = 0.0f;

    launcher_dial_brake_pid.kp = LAUNCHER_DIAL_BRAKE_KP;
    launcher_dial_brake_pid.ki = LAUNCHER_DIAL_BRAKE_KI;
    launcher_dial_brake_pid.kd = LAUNCHER_DIAL_BRAKE_KD;
    launcher_dial_brake_pid.integral = 0.0f;
    launcher_dial_brake_pid.integral_max =
        LAUNCHER_DIAL_BRAKE_INTEGRAL_MAX;
    launcher_dial_brake_pid.out_max = LAUNCHER_DIAL_BRAKE_OUT_MAX;
    launcher_dial_brake_pid.deadband = 0.0f;
    launcher_dial_brake_pid.d_filter_alpha = 0.0f;
    launcher_dial_brake_pid.out = 0.0f;
}
```

## 上板 Launcher_LiftAllowsShoot

```c
static uint8_t Launcher_LiftAllowsShoot(void)
{
    return ((lift.control_is_hole == 0u) &&
            ((lift.state == LIFT_WAIT) ||
             (lift.state == LIFT_READY_UP) ||
             (lift.state == LIFT_MOVING_UP) ||
             (lift.state == LIFT_HOMING_UP))) ? 1u : 0u;
}
```

## 上板 Launcher_Work

```c
void Launcher_Work(void)
{
    uint32_t now = HAL_GetTick(); /* 本次调度时刻 */
    uint8_t shoot_flags = Board_Rx_Shoot_Flags;
    uint8_t feed_permit = (shoot_flags >> 5) & 0x01u;
    uint8_t fric_on;              /* 摩擦轮总使能 */
    uint8_t dial_on;              /* 拨盘参与控制 */
    uint8_t shoot_level;          /* 发射触发电平 */
    uint8_t shoot_mode;           /* 0 = 单发，1 = 连发 */
    uint8_t shoot_active;         /* 发射保持状态 */
    uint8_t dial_ready;           /* 拨盘可参与控制 */
    uint8_t single_rising;        /* 单发触发升沿 */
    int32_t current_angle;        /* 拨盘当前角度 */

    current_angle = Launcher_DialAngle(); /* 当前拨盘角 */
    launcher.dial_angle = current_angle;
    launcher.dial_online = Launcher_DialOnline(); /* 拨盘在线 */
    Launcher_HeatUpdate(now);

    if ((launcher_dial_last_online == 0u) && (launcher.dial_online != 0u))
    {
        launcher.dial_zero_angle = current_angle; /* 记录上电零点 */
        launcher_dial_target = current_angle;
        launcher_dial_feed_target = current_angle;
        launcher_dial_target_synced = 0u;
    }
    launcher_dial_last_online = launcher.dial_online;

#if LAUNCHER_DIAL_ENABLE
    dial_ready = launcher.dial_online;
#else
    dial_ready = 1u;
#endif

    fric_on = ((Board_HeartBeat.status == DEV_ONLINE) &&
               ((shoot_flags & 0x01u) != 0u) &&
               ((shoot_flags & 0x08u) == 0u) &&
               (Launcher_LiftAllowsShoot() != 0u) &&
               (Launcher_FricOnline(SHOOT_FRIC_L) != 0u) &&
               (Launcher_FricOnline(SHOOT_FRIC_R) != 0u)) ? 1u : 0u;
    dial_on = ((fric_on != 0u) && (dial_ready != 0u)) ? 1u : 0u;



    /* 发射总开关关闭：降速后进入休眠 */
    if (fric_on == 0u)
    {
        launcher_feed_armed = 0u;
        launcher.enabled = 0u;
        launcher.last_shoot_level = (shoot_flags >> 2) & 0x01u;
        Launcher_UpdateFrictionReady(0u);

        if (launcher.state == LAUNCHER_SLEEP)
        {
            launcher.fric_target_rpm = 0.0f;
            launcher_fric_stop_count = 0u;
            Launcher_FricControl(0u);
            Launcher_DialSafeStop(now);
            return;
        }

        launcher.state = LAUNCHER_STOPPING;
        launcher.fric_target_rpm = Launcher_Ramp(
            launcher.fric_target_rpm,
            0.0f,
            LAUNCHER_FRIC_STOP_RAMP_RPM_PER_MS);
        Launcher_FricControl(1u);
        Launcher_DialSafeStop(now);

        if ((fabsf(launcher.fric_l_speed_rpm) <=
             LAUNCHER_FRIC_STOP_SPEED_RPM) &&
            (fabsf(launcher.fric_r_speed_rpm) <=
             LAUNCHER_FRIC_STOP_SPEED_RPM))
        {
            if (launcher_fric_stop_count < LAUNCHER_FRIC_STOP_CONFIRM_MS)
            {
                launcher_fric_stop_count++;
            }
        }
        else
        {
            launcher_fric_stop_count = 0u;
        }

        if (launcher_fric_stop_count >= LAUNCHER_FRIC_STOP_CONFIRM_MS)
        {
            launcher.fric_target_rpm = 0.0f;
            launcher.state = LAUNCHER_SLEEP;
            launcher_fric_stop_count = 0u;
            Launcher_FricControl(0u);
        }

        return;
    }

    launcher.enabled = 1u;
    launcher_fric_stop_count = 0u;

    /* 从停机态接管时重新采样拨盘零点 */
    if ((launcher.state == LAUNCHER_SLEEP) ||
        (launcher.state == LAUNCHER_STOPPING))
    {
        launcher_dial_target = current_angle;
        launcher_dial_feed_target = current_angle;
        launcher_dial_target_synced = 0u;
        launcher.state = LAUNCHER_READY;
        launcher.state_tick = now;
        launcher.jam_tick = 0u;
        launcher_jam_count = 0u;
    }


    launcher.fric_target_rpm = LAUNCHER_FRIC_TARGET_RPM; /* 目标转速 */
    Launcher_UpdateFrictionReady(1u);

    shoot_level = (shoot_flags >> 2) & 0x01u;
    shoot_mode = (shoot_flags >> 1) & 0x01u;
#if !LAUNCHER_DIAL_ENABLE
    shoot_level = 0u;
    shoot_mode = 0u;
#endif

    /* 恢复时不沿用旧触发 */
    if ((feed_permit == 0u) || (dial_ready == 0u))
    {
        launcher_feed_armed = 0u;
    }
    else if (shoot_level == 0u)
    {
        launcher_feed_armed = 1u;
    }

    single_rising = ((shoot_level != 0u) && /* 单发升沿 */
                     (launcher.last_shoot_level == 0u) &&
                     (shoot_mode == 0u)) ? 1u : 0u;
    shoot_active = (shoot_level != 0u) ? 1u : 0u; /* 发射保持 */

    /* 已预占的单发可完成本发 */
    if ((launcher_heat.ready == 0u) ||
        ((launcher.state == LAUNCHER_SINGLE) ?
         (launcher_heat.heat > launcher_heat.heat_limit) :
         ((launcher_heat.blocked != 0u) ||
          ((shoot_mode != 0u) && (launcher_heat.target_rate <= 0.0f)))))
    {
        shoot_active = 0u;
        single_rising = 0u;
        if (shoot_level != 0u)
        {
            launcher_feed_armed = 0u;
        }
    }

    /* 达速仅约束新供弹 */
    if ((shoot_active != 0u) &&
        (launcher.state != LAUNCHER_SINGLE) &&
        (launcher.state != LAUNCHER_REPEAT) &&
        (launcher.state != LAUNCHER_REVERSE) &&
        (launcher.state != LAUNCHER_RELOAD) &&
        (launcher.fric_ready == 0u))
    {
        launcher_feed_armed = 0u;
    }

    if (launcher_feed_armed == 0u)
    {
        shoot_active = 0u;
        single_rising = 0u;
    }

    /* 拒绝的点击不延后补射 */
    launcher.last_shoot_level = shoot_level;

    /* 拨盘离线不影响摩擦轮持续运行。 */
    if (dial_on == 0u)
    {
        launcher.state = LAUNCHER_READY;
        launcher.state_tick = now;
        launcher.last_shoot_level = shoot_level;
        launcher_dial_braking = 0u;
        Launcher_FricControl(1u);
        Launcher_DialSafeStop(now);
        return;
    }

    if (launcher_feed_armed == 0u)
    {
        Launcher_FricControl(1u);
        Launcher_DialControl(0u);
        return;
    }

    if ((shoot_active != 0u) && (launcher_dial_stopped != 0u))
    {
        if (Launcher_DialRun() != HAL_OK)
        {
            launcher_feed_armed = 0u;
            Launcher_FricControl(1u);
            return;
        }
        launcher_dial_stopped = 0u;
    }

    switch (launcher.state)
    {
    case LAUNCHER_SPINUP:
        if (launcher.fric_ready != 0u)
        {
#if LAUNCHER_DIAL_AUTO_RESET_ENABLE
            launcher.state = LAUNCHER_INIT;
#else
            launcher_dial_target = current_angle;
            launcher_dial_feed_target = current_angle;
            launcher_dial_target_synced = 1u;
            Launcher_DialClearPid();
            launcher.state = LAUNCHER_READY;
#endif
            launcher.state_tick = now;
            launcher.jam_tick = 0u;
        }
        break;

    case LAUNCHER_INIT:
        if ((fabsf(LAUNCHER_DIAL_RESET_ANGLE -
                   (float)Launcher_DialEncoder()) <=
             LAUNCHER_DIAL_STOP_ERROR) ||
            ((now - launcher.state_tick) >=
             LAUNCHER_DIAL_RESET_TIMEOUT_MS))
        {
            launcher_dial_target = current_angle;
            launcher_dial_feed_target = current_angle;
            launcher_dial_target_synced = 1u;
            Launcher_DialClearPid();
            launcher.state = LAUNCHER_READY;
            launcher.state_tick = now;
            launcher.jam_tick = 0u;
        }
        break;

    case LAUNCHER_READY:
    case LAUNCHER_SINGLE:
    case LAUNCHER_REPEAT:
    case LAUNCHER_REVERSE:
    case LAUNCHER_RELOAD:
        Launcher_DialUpdate(
            single_rising,
            ((shoot_mode != 0u) && (shoot_active != 0u)) ? 1u : 0u);
        break;

    case LAUNCHER_FAULT:
    case LAUNCHER_SLEEP:
    case LAUNCHER_STOPPING:
    default:
        break;
    }

    if (launcher.state == LAUNCHER_FAULT)
    {
        launcher.fric_target_rpm = 0.0f;
        Launcher_FricControl(0u);
        Launcher_DialSafeStop(now);
        return;
    }

    Launcher_FricControl(1u);
    Launcher_DialControl(shoot_active);
}
```

