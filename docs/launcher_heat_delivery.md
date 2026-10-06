# 发射机构热量限制：完整修改单元

2026-10-06。代码取自本次落地源码，保留完整函数、类型和配置块，在原工程中使用。未编译、未烧录，功能验证由人工完成。

## 配置块

源码：[task_up/Application/ConfigLayer/launcher_config.h](G:/STM32_ALL/RM_code/Train_code_plus/task_up/Application/ConfigLayer/launcher_config.h)

```c
/* 17 mm热量预算与限频 */
#define LAUNCHER_HEAT_PER_SHOT             10.0f // 每发增热，热量单位
#define LAUNCHER_HEAT_WARN                200.0f // 降速起点，热量单位
#define LAUNCHER_HEAT_SATURATE             50.0f // 平衡区起点，热量单位
#define LAUNCHER_HEAT_MARGIN               20.0f // 安全余量，热量单位，至少10
#define LAUNCHER_HEAT_STOP LAUNCHER_HEAT_MARGIN // 连发停发余量，热量单位
#define LAUNCHER_HEAT_RESUME               30.0f // 恢复余量，热量单位
#define LAUNCHER_HEAT_MAX_RATE             15.0f // 最高射频，发/s
#define LAUNCHER_HEAT_D3_TIMEOUT_MS        100u // 热量链路超时，ms
#define LAUNCHER_HEAT_TRAINING_ENABLE        0u // 固定参数训练，0/1
#define LAUNCHER_HEAT_TRAINING_LIMIT       0.0f // 训练上限，热量单位
#define LAUNCHER_HEAT_TRAINING_COOLING     0.0f // 训练冷却，热量单位/s
```

## 配置块

源码：[task_down/Application/ConfigLayer/board_comm_config.h](G:/STM32_ALL/RM_code/Train_code_plus/task_down/Application/ConfigLayer/board_comm_config.h)

```c
#define BOARD_COMM_D3_PERIOD_MS         10u // 热量报文间隔，ms
#define BOARD_COMM_D3_ENABLE             1u // 热量发送使能，0/1
#define BOARD_COMM_D4_PERIOD_MS         10u // 血量报文间隔，ms
#define BOARD_COMM_D4_ENABLE             0u // 血量发送使能，0/1
#define BOARD_HEAT_LIMIT_TIMEOUT_MS   1500u // 裁判参数超时，ms
#define BOARD_HEAT_VALUE_TIMEOUT_MS    300u // 裁判热量超时，ms
```

## launcher_heat_source_e

源码：[task_up/Application/ModuleLayer/launcher.h](G:/STM32_ALL/RM_code/Train_code_plus/task_up/Application/ModuleLayer/launcher.h)

```c
typedef enum
{
    LAUNCHER_HEAT_NONE = 0, // 无有效初始参数，禁发
    LAUNCHER_HEAT_REFEREE, // 裁判有效，值1
    LAUNCHER_HEAT_ESTIMATE, // 断链本地估算，值2
    LAUNCHER_HEAT_TRAINING, // 固定参数训练，值3
} launcher_heat_source_e;
```

## launcher_fric_observation_t

源码：[task_up/Application/ModuleLayer/launcher.h](G:/STM32_ALL/RM_code/Train_code_plus/task_up/Application/ModuleLayer/launcher.h)

```c
typedef struct
{
    float current_a; // 反馈电流，A
    float speed_rpm; // 反馈转速，rpm
    uint32_t feedback_tick; // 反馈接收时刻，ms
    uint32_t feedback_seq; // 反馈序号，uint32循环
    uint32_t feedback_age_ms; // 反馈距今时间，ms
    uint8_t online; // 电机在线，0/1
} launcher_fric_observation_t;
```

## launcher_heat_t

源码：[task_up/Application/ModuleLayer/launcher.h](G:/STM32_ALL/RM_code/Train_code_plus/task_up/Application/ModuleLayer/launcher.h)

```c
typedef struct
{
    float heat; // 裁判校准后的估计，热量单位
    float referee_heat; // 最新裁判值，热量单位
    float heat_limit; // 使用中上限，热量单位
    float cooling_rate; // 使用中冷却，热量单位/s
    float remaining; // 剩余预算，热量单位
    float target_rate; // 连发射频，发/s
    launcher_heat_source_e source; // 参数来源，0~3
    uint8_t ready; // 初始热量已建立，0/1
    uint8_t blocked; // 热停发锁定，0/1
    launcher_fric_observation_t fric_l; // 左轮观测，单位见类型
    launcher_fric_observation_t fric_r; // 右轮观测，单位见类型
} launcher_heat_t;
```

## launcher_heat_runtime_t

源码：[task_up/Application/ModuleLayer/launcher.c](G:/STM32_ALL/RM_code/Train_code_plus/task_up/Application/ModuleLayer/launcher.c)

```c
typedef struct
{
    uint32_t update_tick; // 上次估算时刻，ms
    uint32_t encoder_prev; // 上次累计编码器，count
    int64_t travel; // 本段正向位移，count
    int64_t high_water; // 已计热位移上界，count
    uint8_t encoder_valid; // 角度基准有效，0/1
    uint8_t repeat_tracking; // 连发及尾段计热，0/1
    uint8_t seq_seen; // 裁判源序号有效，0/1
    uint8_t heat_seq; // 上次源序号，0~255
} launcher_heat_runtime_t;
```

## Board_Heat_Pkt_t

源码：[task_up/Application/ProtocolLayer/communicate.h](G:/STM32_ALL/RM_code/Train_code_plus/task_up/Application/ProtocolLayer/communicate.h)

```c
typedef struct
{
    uint16_t heat_limit; /* 热量上限，热量单位 */
    uint16_t barrel_heat; /* 第一枪管热量，热量单位 */
    uint16_t cooling_rate; /* 冷却速率，热量单位/s */
    uint8_t heat_seq; /* 源热量序号，0~255循环 */
    uint8_t flags; /* bit0参数有效，bit1热量有效 */
    uint8_t seen; /* D3已接收，0/1 */
    uint32_t rx_tick; /* D3接收时刻，ms */
} Board_Heat_Pkt_t;
```

## Board_Rx_Info_t

源码：[task_up/Application/ProtocolLayer/communicate.h](G:/STM32_ALL/RM_code/Train_code_plus/task_up/Application/ProtocolLayer/communicate.h)

```c
typedef struct
{
    Board_State_Pkt_t state_pkt; /* 整车位域，范围见类型 */
    Board_Gimbal_Target_Pkt_t gimbal_target_pkt; /* 云台目标，rad/deg */
    Board_Shoot_Pkt_t shoot_pkt; /* 发射控制，各位0/1 */
    Board_Remote_Cmd_Pkt_t remote_cmd_pkt; /* 键鼠命令，单位见类型 */
    Board_Heat_Pkt_t heat_pkt; /* 热量快照，见成员单位 */
} Board_Rx_Info_t;
```

## rm_rx_t

源码：[task_up/Application/HardwareLayer/RM_motor.h](G:/STM32_ALL/RM_code/Train_code_plus/task_up/Application/HardwareLayer/RM_motor.h)

```c
typedef struct rm_rx_struct_t
{
    float torque; // 电机反馈力矩，N·m
    float torque_current; // 反馈扭矩电流，A
    int16_t torque_current_raw; // 反馈电流原始值，int16
    int16_t encoder_speed; // 编码器反馈转速，rpm
    float speed; // 换算角速度，rad/s
    uint16_t encoder; // 单圈编码器，0~8191
    int32_t encoder_sum; // 累计编码器，count
    uint16_t encoder_last; // 上帧编码器，0~8191
    float motor_angle_sum; // 累计电机角，rad
    float motor_angle; // 单圈电机角，rad
    float motor_angle_last; // 上帧电机角，rad
    int8_t temperature; // 电机反馈温度，℃
    uint32_t feedback_tick; // 反馈接收时刻，ms
    uint32_t feedback_seq; // 反馈序号，uint32循环
}rm_rx_t;
```

## judge_heat_snapshot_t

源码：[task_down/Application/DeviceLayer/judge.h](G:/STM32_ALL/RM_code/Train_code_plus/task_down/Application/DeviceLayer/judge.h)

```c
typedef struct
{
    uint32_t limit_tick; // 参数接收时刻，ms
    uint32_t heat_tick; // 热量接收时刻，ms
    uint16_t heat_limit; // 枪管热量上限，热量单位
    uint16_t barrel_heat; // 第一枪管热量，热量单位
    uint16_t cooling_rate; // 冷却速率，热量单位/s
    uint8_t heat_seq; // 热量源序号，0~255循环
    uint8_t limit_seen; // 参数已接收，0/1
    uint8_t heat_seen; // 热量已接收，0/1
} judge_heat_snapshot_t;
```

## Board_Judge_Shoot_Pkt_t

源码：[task_down/Application/ProtocolLayer/board_protocol.h](G:/STM32_ALL/RM_code/Train_code_plus/task_down/Application/ProtocolLayer/board_protocol.h)

```c
typedef struct{
  float shoot_speed;          /* 实测弹速，m/s */
  float shoot_freq;           /* 实测射频，发/s */
  uint16_t allowance_max;     /* 允许发弹量，发 */
}Board_Judge_Shoot_Pkt_t;
```

## 热量全局对象

源码：[task_up/Application/ModuleLayer/launcher.c](G:/STM32_ALL/RM_code/Train_code_plus/task_up/Application/ModuleLayer/launcher.c)

```c
launcher_heat_t launcher_heat;
static launcher_heat_runtime_t launcher_heat_runtime;
```

## 裁判热量缓存

源码：[task_down/Application/DeviceLayer/judge.c](G:/STM32_ALL/RM_code/Train_code_plus/task_down/Application/DeviceLayer/judge.c)

```c
static volatile judge_heat_snapshot_t judge_heat_data;
```

## 热量观测声明

源码：[task_up/Application/ModuleLayer/launcher.h](G:/STM32_ALL/RM_code/Train_code_plus/task_up/Application/ModuleLayer/launcher.h)

```c
extern launcher_heat_t launcher_heat;
```

## 热量快照声明

源码：[task_up/Application/ProtocolLayer/communicate.h](G:/STM32_ALL/RM_code/Train_code_plus/task_up/Application/ProtocolLayer/communicate.h)

```c
void Board_GetHeatSnapshot(Board_Heat_Pkt_t *snapshot);
```

## 裁判热量接口声明

源码：[task_down/Application/DeviceLayer/judge.h](G:/STM32_ALL/RM_code/Train_code_plus/task_down/Application/DeviceLayer/judge.h)

```c
uint8_t Judge_GetHeatSnapshot(judge_heat_snapshot_t *snapshot);
```

## Launcher_HeatConfigValid

源码：[task_up/Application/ModuleLayer/launcher.c](G:/STM32_ALL/RM_code/Train_code_plus/task_up/Application/ModuleLayer/launcher.c)

```c
static uint8_t Launcher_HeatConfigValid(void)
{
    return ((LAUNCHER_HEAT_PER_SHOT > 0.0f) &&
            (LAUNCHER_DIAL_ONE_SHOT_ANGLE > 0.0f) &&
            (LAUNCHER_HEAT_MAX_RATE > 0.0f) &&
            (LAUNCHER_HEAT_MARGIN >= LAUNCHER_HEAT_PER_SHOT) &&
            (LAUNCHER_HEAT_STOP >= LAUNCHER_HEAT_MARGIN) &&
            (LAUNCHER_HEAT_RESUME > LAUNCHER_HEAT_STOP) &&
            (LAUNCHER_HEAT_SATURATE >= LAUNCHER_HEAT_RESUME) &&
            (LAUNCHER_HEAT_WARN > LAUNCHER_HEAT_SATURATE)) ? 1u : 0u;
}
```

## Launcher_HeatRefreshRate

源码：[task_up/Application/ModuleLayer/launcher.c](G:/STM32_ALL/RM_code/Train_code_plus/task_up/Application/ModuleLayer/launcher.c)

```c
static void Launcher_HeatRefreshRate(void)
{
    float balanced_rate;
    float max_rate = LAUNCHER_HEAT_MAX_RATE;
    float mechanical_rate;
    launcher_heat.remaining = launcher_heat.heat_limit - launcher_heat.heat;
    launcher_heat.target_rate = 0.0f;
    if ((Launcher_HeatConfigValid() == 0u) || (launcher_heat.ready == 0u))
    {
        launcher_heat.blocked = 1u;
        return;
    }
    if (launcher_heat.remaining < LAUNCHER_HEAT_STOP)
    {
        launcher_heat.blocked = 1u;
    }
    else if (launcher_heat.remaining >= LAUNCHER_HEAT_RESUME)
    {
        launcher_heat.blocked = 0u;
    }
    if (launcher_heat.blocked != 0u)
    {
        return;
    }
    mechanical_rate = (float)LAUNCHER_DIAL_REPEAT_SPEED_DPS /
                      (LAUNCHER_DIAL_ONE_SHOT_ANGLE * 360.0f / 65536.0f);
    if (max_rate > mechanical_rate)
    {
        max_rate = mechanical_rate;
    }
    balanced_rate = constrain(launcher_heat.cooling_rate / LAUNCHER_HEAT_PER_SHOT,
                             0.0f, max_rate);
    if (launcher_heat.remaining >= LAUNCHER_HEAT_WARN)
    {
        launcher_heat.target_rate = max_rate;
    }
    else if (launcher_heat.remaining >= LAUNCHER_HEAT_SATURATE)
    {
        launcher_heat.target_rate = balanced_rate + (max_rate - balanced_rate) *
            (launcher_heat.remaining - LAUNCHER_HEAT_SATURATE) /
            (LAUNCHER_HEAT_WARN - LAUNCHER_HEAT_SATURATE);
    }
    else
    {
        launcher_heat.target_rate = balanced_rate;
    }
}
```

## Launcher_HeatObserveFriction

源码：[task_up/Application/ModuleLayer/launcher.c](G:/STM32_ALL/RM_code/Train_code_plus/task_up/Application/ModuleLayer/launcher.c)

```c
static void Launcher_HeatObserveFriction(uint8_t index,
                                        launcher_fric_observation_t *observation)
{
    uint32_t irq_state = __get_PRIMASK();
    rm_rx_t *feedback = rm_motor[index].rx_info;
    __disable_irq();
    observation->current_a = feedback->torque_current;
    observation->speed_rpm = (float)feedback->encoder_speed;
    observation->feedback_tick = feedback->feedback_tick;
    observation->feedback_seq = feedback->feedback_seq;
    observation->online = Launcher_FricOnline(index);
    __set_PRIMASK(irq_state);
    observation->feedback_age_ms = HAL_GetTick() - observation->feedback_tick;
}
```

## Launcher_HeatUpdate

源码：[task_up/Application/ModuleLayer/launcher.c](G:/STM32_ALL/RM_code/Train_code_plus/task_up/Application/ModuleLayer/launcher.c)

```c
static void Launcher_HeatUpdate(uint32_t now)
{
#if !LAUNCHER_HEAT_TRAINING_ENABLE
    Board_Heat_Pkt_t snapshot;
    uint8_t live;
#endif
    uint32_t elapsed = now - launcher_heat_runtime.update_tick;
    uint32_t encoder = (uint32_t)dail_motor.KT_motor_info.rx_info.encoder_sum;
    uint32_t raw_delta;
    int64_t delta;
    launcher_heat_runtime.update_tick = now;
    launcher_heat.heat = fmaxf(0.0f, launcher_heat.heat -
                              launcher_heat.cooling_rate * ((float)elapsed * 0.001f));

    if (launcher.dial_online == 0u)
    {
        launcher_heat_runtime.encoder_valid = 0u;
        launcher_heat_runtime.repeat_tracking = 0u;
    }
    else if (launcher_heat_runtime.encoder_valid == 0u)
    {
        launcher_heat_runtime.encoder_prev = encoder;
        launcher_heat_runtime.encoder_valid = 1u;
    }
    else
    {
        /* 模减避免累计编码器跨界 */
        raw_delta = encoder - launcher_heat_runtime.encoder_prev;
        delta = (raw_delta <= 0x7FFFFFFFu) ? (int64_t)raw_delta :
                (int64_t)raw_delta - 4294967296LL;
        launcher_heat_runtime.encoder_prev = encoder;
        if (LAUNCHER_DIAL_ANGLE_SIGN < 0.0f)
        {
            delta = -delta;
        }
        if (LAUNCHER_DIAL_DIRECTION < 0.0f)
        {
            delta = -delta;
        }
        if (launcher_heat_runtime.repeat_tracking != 0u)
        {
            launcher_heat_runtime.travel += delta;
            if (launcher_heat_runtime.travel > launcher_heat_runtime.high_water)
            {
                launcher_heat.heat += LAUNCHER_HEAT_PER_SHOT *
                    (float)(launcher_heat_runtime.travel - launcher_heat_runtime.high_water) /
                    LAUNCHER_DIAL_ONE_SHOT_ANGLE;
                launcher_heat_runtime.high_water = launcher_heat_runtime.travel;
            }
            if ((launcher.state != LAUNCHER_REPEAT) && (launcher_dial_stopped != 0u) &&
                (fabsf((float)dail_motor.KT_motor_info.rx_info.speed) <=
                 (float)LAUNCHER_DIAL_BRAKE_STOP_SPEED_DPS))
            {
                launcher_heat_runtime.repeat_tracking = 0u;
            }
        }
    }

#if LAUNCHER_HEAT_TRAINING_ENABLE
    launcher_heat.source = LAUNCHER_HEAT_TRAINING;
    launcher_heat.heat_limit = LAUNCHER_HEAT_TRAINING_LIMIT;
    launcher_heat.cooling_rate = LAUNCHER_HEAT_TRAINING_COOLING;
    launcher_heat.ready = ((LAUNCHER_HEAT_TRAINING_LIMIT > LAUNCHER_HEAT_RESUME) &&
                          (LAUNCHER_HEAT_TRAINING_COOLING > 0.0f)) ? 1u : 0u;
    launcher_heat_runtime.seq_seen = 0u;
#else
    Board_GetHeatSnapshot(&snapshot);
    live = ((snapshot.seen != 0u) &&
            ((uint32_t)(HAL_GetTick() - snapshot.rx_tick) <
             LAUNCHER_HEAT_D3_TIMEOUT_MS)) ? 1u : 0u;
    if ((live != 0u) && ((snapshot.flags & 0x01u) != 0u) &&
        (snapshot.heat_limit != 0u))
    {
        launcher_heat.heat_limit = (float)snapshot.heat_limit;
        launcher_heat.cooling_rate = (float)snapshot.cooling_rate;
    }
    if ((live != 0u) && ((snapshot.flags & 0x02u) != 0u))
    {
        if ((launcher_heat.ready == 0u) || (launcher_heat_runtime.seq_seen == 0u) ||
            (launcher_heat_runtime.heat_seq != snapshot.heat_seq))
        {
            launcher_heat.referee_heat = (float)snapshot.barrel_heat;
            /* 新源序号覆盖旧估算 */
            launcher_heat.heat = launcher_heat.referee_heat;
            launcher_heat_runtime.heat_seq = snapshot.heat_seq;
            launcher_heat_runtime.seq_seen = 1u;
        }
        if (((snapshot.flags & 0x01u) != 0u) && (snapshot.heat_limit != 0u))
        {
            launcher_heat.ready = 1u;
        }
    }
    else
    {
        launcher_heat_runtime.seq_seen = 0u;
    }
    launcher_heat.source = (launcher_heat.ready == 0u) ? LAUNCHER_HEAT_NONE :
        (((live != 0u) && (snapshot.flags == 0x03u)) ?
         LAUNCHER_HEAT_REFEREE : LAUNCHER_HEAT_ESTIMATE);
#endif
    Launcher_HeatObserveFriction(SHOOT_FRIC_L, &launcher_heat.fric_l);
    Launcher_HeatObserveFriction(SHOOT_FRIC_R, &launcher_heat.fric_r);
    Launcher_HeatRefreshRate();
}
```

## Launcher_HeatReserveSingle

源码：[task_up/Application/ModuleLayer/launcher.c](G:/STM32_ALL/RM_code/Train_code_plus/task_up/Application/ModuleLayer/launcher.c)

```c
static uint8_t Launcher_HeatReserveSingle(void)
{
    if ((launcher_heat.ready == 0u) || (launcher_heat.blocked != 0u) ||
        (launcher_heat.remaining < LAUNCHER_HEAT_STOP) ||
        ((launcher_heat.heat + LAUNCHER_HEAT_PER_SHOT) >
         (launcher_heat.heat_limit - LAUNCHER_HEAT_MARGIN)))
    {
        return 0u;
    }
    /* 预占保留至下次裁判校准 */
    launcher_heat_runtime.repeat_tracking = 0u;
    launcher_heat.heat += LAUNCHER_HEAT_PER_SHOT;
    Launcher_HeatRefreshRate();
    return 1u;
}
```

## Launcher_HeatStartRepeat

源码：[task_up/Application/ModuleLayer/launcher.c](G:/STM32_ALL/RM_code/Train_code_plus/task_up/Application/ModuleLayer/launcher.c)

```c
static void Launcher_HeatStartRepeat(void)
{
    if (launcher_heat_runtime.repeat_tracking == 0u)
    {
        launcher_heat_runtime.travel = 0;
        launcher_heat_runtime.high_water = 0;
        launcher_heat_runtime.repeat_tracking = 1u;
    }
}
```

## Launcher_DialSpeedControl

源码：[task_up/Application/ModuleLayer/launcher.c](G:/STM32_ALL/RM_code/Train_code_plus/task_up/Application/ModuleLayer/launcher.c)

```c
static void Launcher_DialSpeedControl(void)
{
    int16_t current_output; /* 连发速度环输出 */

    if (Launcher_DialOnline() == 0u)
    {
        Launcher_DialClearPid();
        Launcher_DialApplyTorque(0);
        return;
    }

    launcher_dial_repeat_pid.target = /* 连发目标速度 */
        LAUNCHER_DIAL_DIRECTION * launcher_heat.target_rate *
        (LAUNCHER_DIAL_ONE_SHOT_ANGLE * 360.0f / 65536.0f);
    launcher_dial_repeat_pid.measure =
        LAUNCHER_DIAL_SPEED_SIGN *
        (float)dail_motor.KT_motor_info.rx_info.speed;
    launcher_dial_repeat_pid.err =
        launcher_dial_repeat_pid.target - launcher_dial_repeat_pid.measure;
    single_pid_ctrl(&launcher_dial_repeat_pid);

    current_output = (int16_t)constrain( /* 连发电流限幅 */
        LAUNCHER_DIAL_OUTPUT_SIGN * launcher_dial_repeat_pid.out,
        -LAUNCHER_DIAL_CURRENT_LIMIT,
        LAUNCHER_DIAL_CURRENT_LIMIT);
    Launcher_DialApplyTorque(current_output);
}
```

## Launcher_DialUpdate

源码：[task_up/Application/ModuleLayer/launcher.c](G:/STM32_ALL/RM_code/Train_code_plus/task_up/Application/ModuleLayer/launcher.c)

```c
static void Launcher_DialUpdate(uint8_t single_rising, uint8_t continuous)
{
    uint32_t now = HAL_GetTick(); /* 本次调度时刻 */
    int64_t current_angle = (int64_t)Launcher_DialAngle(); /* 当前绝对角度 */

    continuous = ((continuous != 0u) && (launcher_heat.ready != 0u) &&
                  (launcher_heat.blocked == 0u) &&
                  (launcher_heat.target_rate > 0.0f)) ? 1u : 0u;

    /* 首次对齐反馈，避免上电跳变 */
    if (launcher_dial_target_synced == 0u)
    {
        launcher_dial_target = current_angle;
        launcher_dial_feed_target = current_angle;
        launcher_dial_target_synced = 1u;
        launcher.state = (continuous != 0u) ? LAUNCHER_REPEAT : LAUNCHER_READY;
        if (continuous != 0u)
        {
            Launcher_HeatStartRepeat();
        }
        launcher.state_tick = now;
        Launcher_DialClearPid();
    }

    switch (launcher.state)
    {
    /* 待发：单发升沿或连发请求触发供弹 */
    case LAUNCHER_READY:
        if ((single_rising != 0u) && (Launcher_HeatReserveSingle() != 0u))
        {
            launcher_dial_target +=
                (int64_t)(LAUNCHER_DIAL_DIRECTION *
                          LAUNCHER_DIAL_ONE_SHOT_ANGLE);
            launcher_dial_feed_target = launcher_dial_target;
            launcher.state = LAUNCHER_SINGLE;
            launcher.state_tick = now;
            launcher.jam_tick = 0u;
            Launcher_DialClearPid();
        }
#if LAUNCHER_REPEAT_ENABLE
        else if (continuous != 0u)
        {
            launcher_dial_target = current_angle;
            launcher_dial_feed_target = current_angle;
            launcher.state = LAUNCHER_REPEAT;
            Launcher_HeatStartRepeat();
            launcher.state_tick = now;
            launcher.jam_tick = 0u;
            Launcher_DialClearPid();
        }
#endif
        break;

    /* 单发：堵转优先，其次到达或超时 */
    case LAUNCHER_SINGLE:
#if LAUNCHER_DIAL_JAM_ENABLE
        if (Launcher_DialBlockCheck(
                (Launcher_AbsInt64(launcher_dial_target - current_angle) >
                 (int64_t)LAUNCHER_DIAL_STOP_ERROR) ? 1u : 0u) != 0u)
        {
            Launcher_DialEnterStuckRecovery(0u);
        }
        else
#endif
        if ((Launcher_DialAtTarget(launcher_dial_target) != 0u) || /* 到位 */
                 ((now - launcher.state_tick) >=
                  LAUNCHER_DIAL_SINGLE_TIMEOUT_MS))
        {
            launcher.state = LAUNCHER_READY; /* 单发完成 */
            launcher.state_tick = now;
            launcher.jam_tick = 0u;
            launcher_jam_count = 0u;
            Launcher_DialClearPid();
        }
        break;

    /* 连发：持续速度环，松触发后回待发 */
    case LAUNCHER_REPEAT:
        if (continuous == 0u)
        {
            launcher.state = LAUNCHER_READY; /* 松开连发 */
            launcher_dial_target_synced = 0u;
            Launcher_DialClearPid();
            break;
        }
#if LAUNCHER_REPEAT_ENABLE && LAUNCHER_DIAL_JAM_ENABLE
        if (Launcher_DialBlockCheck(1u) != 0u)
        {
            Launcher_DialEnterStuckRecovery(1u);
        }
#endif
        break;

    /* 退让完成后回原供弹目标 */
    case LAUNCHER_REVERSE:
        if ((Launcher_DialAtTarget(launcher_dial_target) != 0u) ||
            ((now - launcher.state_tick) >=
             LAUNCHER_DIAL_REVERSE_TIMEOUT_MS))
        {
            launcher_dial_target = launcher_dial_feed_target; /* 回到原供弹目标 */
            launcher.state = LAUNCHER_RELOAD;
            launcher.state_tick = now;
            Launcher_DialClearPid();
        }
        break;

    /* 回目标完成，按恢复类型回到待发或连发 */
    case LAUNCHER_RELOAD:
        if ((Launcher_DialAtTarget(launcher_dial_target) != 0u) ||
            ((now - launcher.state_tick) >=
             LAUNCHER_DIAL_RELOAD_TIMEOUT_MS))
        {
            launcher.jam_tick = 0u;
            if ((launcher_dial_recovery_repeat != 0u) && (continuous != 0u))
            {
                launcher.state = LAUNCHER_REPEAT;
                Launcher_HeatStartRepeat();
            }
            else
            {
                launcher.state = LAUNCHER_READY;
                launcher_jam_count = 0u;
            }
            launcher.state_tick = now;
            Launcher_DialClearPid();
        }
        break;

    /* 非法状态统一回待发 */
    default:
        launcher.state = LAUNCHER_READY;
        launcher_dial_target_synced = 0u;
        break;
    }

    launcher.dial_target_angle = (int32_t)launcher_dial_target;
}
```

## Launcher_Init

源码：[task_up/Application/ModuleLayer/launcher.c](G:/STM32_ALL/RM_code/Train_code_plus/task_up/Application/ModuleLayer/launcher.c)

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

## Launcher_Work

源码：[task_up/Application/ModuleLayer/launcher.c](G:/STM32_ALL/RM_code/Train_code_plus/task_up/Application/ModuleLayer/launcher.c)

```c
void Launcher_Work(void)
{
    uint32_t now = HAL_GetTick(); /* 本次调度时刻 */
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
               (Board_Rx_Info.shoot_pkt.launch_state != 0u) &&
               (Launcher_FricOnline(SHOOT_FRIC_L) != 0u) &&
               (Launcher_FricOnline(SHOOT_FRIC_R) != 0u)) ? 1u : 0u;
    dial_on = ((fric_on != 0u) && (dial_ready != 0u)) ? 1u : 0u;



    /* 发射总开关关闭：降速后进入休眠 */
    if (fric_on == 0u)
    {
        launcher.enabled = 0u;
        launcher.last_shoot_level = Board_Rx_Info.shoot_pkt.shoot_level;
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

    shoot_level = Board_Rx_Info.shoot_pkt.shoot_level;
    shoot_mode = Board_Rx_Info.shoot_pkt.shoot_mode;
#if !LAUNCHER_DIAL_ENABLE
    shoot_level = 0u;
    shoot_mode = 0u;
#endif

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

    if ((shoot_active != 0u) && (launcher_dial_stopped != 0u))
    {
        if (Launcher_DialRun() != HAL_OK)
        {
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

## Board_Rx_03

源码：[task_up/Application/ProtocolLayer/communicate.c](G:/STM32_ALL/RM_code/Train_code_plus/task_up/Application/ProtocolLayer/communicate.c)

```c
void Board_Rx_03(uint8_t *rxbuf)
{
    Board_Rx_Info.heat_pkt.heat_limit = (uint16_t)(((uint16_t)rxbuf[0] << 8) | rxbuf[1]);
    Board_Rx_Info.heat_pkt.barrel_heat = (uint16_t)(((uint16_t)rxbuf[2] << 8) | rxbuf[3]);
    Board_Rx_Info.heat_pkt.cooling_rate = (uint16_t)(((uint16_t)rxbuf[4] << 8) | rxbuf[5]);
    Board_Rx_Info.heat_pkt.heat_seq = rxbuf[6];
    Board_Rx_Info.heat_pkt.flags = rxbuf[7] & 0x03u;
    Board_Rx_Info.heat_pkt.rx_tick = HAL_GetTick();
    Board_Rx_Info.heat_pkt.seen = 1u;
    Board_HeartBeat.offline_cnt_3 = 0;
}
```

## Board_GetHeatSnapshot

源码：[task_up/Application/ProtocolLayer/communicate.c](G:/STM32_ALL/RM_code/Train_code_plus/task_up/Application/ProtocolLayer/communicate.c)

```c
void Board_GetHeatSnapshot(Board_Heat_Pkt_t *snapshot)
{
    uint32_t irq_state;
    if (snapshot == NULL)
    {
        return;
    }
    /* 防止接收中断撕裂快照 */
    irq_state = __get_PRIMASK();
    __disable_irq();
    *snapshot = Board_Rx_Info.heat_pkt;
    __set_PRIMASK(irq_state);
}
```

## rm_motor_update

源码：[task_up/Application/HardwareLayer/RM_motor.c](G:/STM32_ALL/RM_code/Train_code_plus/task_up/Application/HardwareLayer/RM_motor.c)

```c
static void rm_motor_update(rm_motor_t *rm_motor, uint8_t *rxBuf)
{
    rm_rx_t *motor_info = rm_motor->rx_info;
    
    motor_info->encoder = CAN_01_GetMotorAngle(rxBuf);
		Encoder_Sum_Cal(rm_motor);
		Encoder_to_Motor_Angle(rm_motor);
		motor_info->encoder_speed = CAN_23_GetMotorSpeed(rxBuf);
		motor_info->speed = RPM_to_Rads(rm_motor);
		motor_info->torque_current_raw = CAN_45_GetMotorCurrent(rxBuf);
		Raw_Current_to_Torque(rm_motor);
    motor_info->temperature = CAN_6_GetMotorTemperature(rxBuf);
    motor_info->feedback_tick = HAL_GetTick();
    motor_info->feedback_seq++;
    rm_motor->state->offline_cnt = 0;
}
```

## Judge_Init

源码：[task_down/Application/DeviceLayer/judge.c](G:/STM32_ALL/RM_code/Train_code_plus/task_down/Application/DeviceLayer/judge.c)

```c
void Judge_Init(Judge_t* judge)
{
    judge_heat_data.limit_tick = 0u;
    judge_heat_data.heat_tick = 0u;
    judge_heat_data.heat_limit = 0u;
    judge_heat_data.barrel_heat = 0u;
    judge_heat_data.cooling_rate = 0u;
    judge_heat_data.heat_seq = 0u;
    judge_heat_data.limit_seen = 0u;
    judge_heat_data.heat_seen = 0u;
	judge_power_data.limit_seen = 0u;
	judge_power_data.buffer_seen = 0u;
	judge_power_data.limit_tick = 0u;
	judge_power_data.buffer_tick = 0u;
	judge_power_data.buffer_seq = 0u;
	judge_power_data.limit_w = 0u;
	judge_power_data.buffer_j = 0u;
	judge->status->offline_cnt = judge->status->offline_cnt_max;
	judge->status->status = DEV_OFFLINE;
	
	judge->rx = Judge_Data_Update;
	
	judge->heartbeat = Judge_Heart_Beat;
}
```

## Judge_GetHeatSnapshot

源码：[task_down/Application/DeviceLayer/judge.c](G:/STM32_ALL/RM_code/Train_code_plus/task_down/Application/DeviceLayer/judge.c)

```c
uint8_t Judge_GetHeatSnapshot(judge_heat_snapshot_t *snapshot)
{
    uint32_t irq_state;
    if (snapshot == NULL)
    {
        return 0u;
    }
    /* 防止接收中断撕裂快照 */
    irq_state = __get_PRIMASK();
    __disable_irq();
    *snapshot = judge_heat_data;
    __set_PRIMASK(irq_state);
    return 1u;
}
```

## Judge_Data_Update

源码：[task_down/Application/DeviceLayer/judge.c](G:/STM32_ALL/RM_code/Train_code_plus/task_down/Application/DeviceLayer/judge.c)

```c
void Judge_Data_Update(uint16_t id, uint8_t *rxBuf)
{
    switch (id)
    {
    case ID_game_status:
      memcpy(&judge.info->game_status, rxBuf, LEN_game_status);
				
		  judge.pkt->game_progress = judge.info->game_status.game_progress;

      if(judge.pkt->game_progress == 4)
      {
				board.tx_pkt->car_pkt.game_start = 1;
			}				
		  else{
			  board.tx_pkt->car_pkt.game_start = 0;
			}
			
		  judge.status->offline_cnt = 0;
      judge.status->status = DEV_ONLINE;    
		  break;

    case ID_game_result:
      memcpy(&judge.info->game_result, rxBuf, LEN_game_result);
		  judge.status->offline_cnt = 0;
      judge.status->status = DEV_ONLINE;    
      break;

    case ID_game_robot_HP:
      memcpy(&judge.info->game_robot_HP, rxBuf, LEN_game_robot_HP);
	
		  judge.pkt->blood[J_HERO] = judge.info->game_robot_HP.ally_1_robot_HP;
		  judge.pkt->blood[J_ENGINEER] = judge.info->game_robot_HP.ally_2_robot_HP;
		  judge.pkt->blood[J_INFANTRY_3] = judge.info->game_robot_HP.ally_3_robot_HP;
		  judge.pkt->blood[J_INFANTRY_4] = judge.info->game_robot_HP.ally_4_robot_HP;
		  judge.pkt->blood[J_SENTRY] = judge.info->game_robot_HP.ally_7_robot_HP;
		  judge.pkt->blood[J_OUTPOST] = judge.info->game_robot_HP.ally_outpost_HP;
		  judge.pkt->blood[J_BASE] = judge.info->game_robot_HP.ally_base_HP;   
		 
		  for(uint8_t i = 0; i< J_ROBOT_CNT;i++)
		  {
			  board.tx_pkt->blood_pkt.blood[i] = judge.pkt->blood[i];
			}
		
		  judge.status->offline_cnt = 0;
      judge.status->status = DEV_ONLINE;    
      break;

    case ID_event_data:
      memcpy(&judge.info->event_data, rxBuf, LEN_event_data);
		  judge.status->offline_cnt = 0;
      judge.status->status = DEV_ONLINE;     
      break;

    case ID_referee_warning:
      memcpy(&judge.info->referee_warning, rxBuf, LEN_referee_warning);
		  judge.status->offline_cnt = 0;
      judge.status->status = DEV_ONLINE;    
      break;

    case ID_dart_info:
      memcpy(&judge.info->dart_info, rxBuf, LEN_dart_info);
		  judge.status->offline_cnt = 0;
      judge.status->status = DEV_ONLINE;    
      break;

    case ID_robot_status:
      memcpy(&judge.info->robot_status, rxBuf, LEN_robot_status);
      judge_heat_data.heat_limit = judge.info->robot_status.shooter_barrel_heat_limit;
      judge_heat_data.cooling_rate = judge.info->robot_status.shooter_barrel_cooling_value;
      judge_heat_data.limit_tick = HAL_GetTick();
      judge_heat_data.limit_seen = 1u;
		
		  judge.pkt->robot_id = judge.info->robot_status.robot_id;
		  judge.pkt->shooter_barrel_heat_limit = judge.info->robot_status.shooter_barrel_heat_limit; 
      judge.pkt->chassis_power_limit = judge.info->robot_status.chassis_power_limit;
		  judge_power_data.limit_w = judge.pkt->chassis_power_limit;
		  judge_power_data.limit_tick = HAL_GetTick();
		  judge_power_data.limit_seen = 1u;
		
		  if(judge.pkt->robot_id < 10)
		  {
				board.tx_pkt->car_pkt.my_color = 0;
			}
		  else{
			 board.tx_pkt->car_pkt.my_color = 1;
		  }
		
		  judge.status->offline_cnt = 0;
      judge.status->status = DEV_ONLINE;    
      break;

    case ID_power_heat_data:
      memcpy(&judge.info->power_heat_data, rxBuf, LEN_power_heat_data);
      judge_heat_data.barrel_heat = judge.info->power_heat_data.shooter_17mm_1_barrel_heat;
      judge_heat_data.heat_tick = HAL_GetTick();
      judge_heat_data.heat_seq++;
      judge_heat_data.heat_seen = 1u;
		
		  judge.pkt->buffer_energy = judge.info->power_heat_data.buffer_energy;
		  judge_power_data.buffer_j = judge.pkt->buffer_energy;
		  judge_power_data.buffer_tick = HAL_GetTick();
		  judge_power_data.buffer_seq++;
		  judge_power_data.buffer_seen = 1u;
		  judge.pkt->shooter_17mm_1_barrel_heat = judge.info->power_heat_data.shooter_17mm_1_barrel_heat;
		   
		  judge.status->offline_cnt = 0;
      judge.status->status = DEV_ONLINE;    
      break;

    case ID_robot_pos:
      memcpy(&judge.info->robot_pos, rxBuf, LEN_robot_pos);
		  judge.status->offline_cnt = 0;
      judge.status->status = DEV_ONLINE;    
      break;

    case ID_buff:
      memcpy(&judge.info->buff, rxBuf, LEN_buff);
		  judge.status->offline_cnt = 0;
      judge.status->status = DEV_ONLINE;    
      break;

    case ID_hurt_data:
      memcpy(&judge.info->hurt_data, rxBuf, LEN_hurt_data);
		  judge.status->offline_cnt = 0;
      judge.status->status = DEV_ONLINE;    
      break;

    case ID_shoot_data:
      memcpy(&judge.info->shoot_data, rxBuf, LEN_shoot_data);
		
		  judge.pkt->initial_speed = judge.info->shoot_data.initial_speed;
		  judge.pkt->launching_frequency = judge.info->shoot_data.launching_frequency;
		    
		  board.tx_pkt->judge_shoot_pkt.shoot_speed = judge.pkt->initial_speed;
		  board.tx_pkt->judge_shoot_pkt.shoot_freq = judge.pkt->launching_frequency;
		
		  Shooting_Cmd_Excute_Tick_Calculating(1);
		  Speed_Statistic();
		
		  if(shoot_statistics.shoot_mode == 1)
		  {
			  shoot_statistics.shooting_flag = 1;
				Shooting_Cmd_Excute_Tick_Calculating(0);
				
			}
		
		  judge.status->offline_cnt = 0;
      judge.status->status = DEV_ONLINE;    
      break;

    case ID_projectile_allowance:
      memcpy(&judge.info->projectile_allowance, rxBuf, LEN_projectile_allowance);
		
		  judge.pkt->projectile_allowance_17mm = judge.info->projectile_allowance.projectile_allowance_17mm;  
		
		  board.tx_pkt->judge_shoot_pkt.allowance_max = judge.pkt->projectile_allowance_17mm;
		
		  judge.status->offline_cnt = 0;
      judge.status->status = DEV_ONLINE;    
      break;

    case ID_rfid_status:
      memcpy(&judge.info->rfid_status, rxBuf, LEN_rfid_status);
		  judge.status->offline_cnt = 0;
      judge.status->status = DEV_ONLINE;    
      break;

    case ID_robot_interaction_data:
      memcpy(&judge.info->robot_interaction_data, rxBuf, LEN_robot_interaction_data);
			if (judge.info->robot_interaction_data.data_cmd_id == 0x211)
			{
				memcpy(&judge.info->radar_information_status, &judge.info->robot_interaction_data.user_data, sizeof(radar_information_status_t));
			}
		  judge.status->offline_cnt = 0;
      judge.status->status = DEV_ONLINE;    
      break;

    case ID_map_command:
      memcpy(&judge.info->map_command, rxBuf, LEN_map_command);
		  judge.status->offline_cnt = 0;
      judge.status->status = DEV_ONLINE;    
      break;

    case ID_map_robot_data:
      memcpy(&judge.info->map_robot_data, rxBuf, LEN_map_robot_data);
		  judge.status->offline_cnt = 0;
      judge.status->status = DEV_ONLINE;    
      break;

    case ID_map_data:
      memcpy(&judge.info->map_data, rxBuf, LEN_map_data);
		  judge.status->offline_cnt = 0;
      judge.status->status = DEV_ONLINE;    
      break;

    case ID_custom_info:
      memcpy(&judge.info->custom_info, rxBuf, LEN_custom_info);
		  judge.status->offline_cnt = 0;
      judge.status->status = DEV_ONLINE;    
      break;

    case ID_set_video_channel:
      memcpy(&judge.info->set_video_channel, rxBuf, LEN_set_video_channel);
		  judge.status->offline_cnt = 0;
      judge.status->status = DEV_ONLINE;    
      break;

    case ID_query_video_channel:
      memcpy(&judge.info->query_video_channel, rxBuf, LEN_query_video_channel);
		  judge.status->offline_cnt = 0;
      judge.status->status = DEV_ONLINE;    
      break;
    
    default:
      break;
    }
}
```

## Board_Tx_Pkt_03

源码：[task_down/Application/ProtocolLayer/board_protocol.c](G:/STM32_ALL/RM_code/Train_code_plus/task_down/Application/ProtocolLayer/board_protocol.c)

```c
void Board_Tx_Pkt_03(Board_t* board)
{
    judge_heat_snapshot_t snapshot;
    uint32_t now;
    uint32_t gap;
    uint8_t flags = 0u;
    board->status->heat_d3_tx_ok = 0u;
    if (Judge_GetHeatSnapshot(&snapshot) == 0u)
    {
        return;
    }
    now = HAL_GetTick();
    if ((snapshot.limit_seen != 0u) && (snapshot.heat_limit != 0u) &&
        ((uint32_t)(now - snapshot.limit_tick) < BOARD_HEAT_LIMIT_TIMEOUT_MS))
    {
        flags |= 0x01u;
    }
    if ((snapshot.heat_seen != 0u) &&
        ((uint32_t)(now - snapshot.heat_tick) < BOARD_HEAT_VALUE_TIMEOUT_MS))
    {
        flags |= 0x02u;
    }
    pkt_03[0] = (uint8_t)(snapshot.heat_limit >> 8);
    pkt_03[1] = (uint8_t)snapshot.heat_limit;
    pkt_03[2] = (uint8_t)(snapshot.barrel_heat >> 8);
    pkt_03[3] = (uint8_t)snapshot.barrel_heat;
    pkt_03[4] = (uint8_t)(snapshot.cooling_rate >> 8);
    pkt_03[5] = (uint8_t)snapshot.cooling_rate;
    pkt_03[6] = snapshot.heat_seq;
    pkt_03[7] = flags;
    if (CAN_SendData(&hfdcan2, ID_PKT_03, pkt_03) != HAL_OK)
    {
        board->status->heat_d3_tx_fail_count++;
        return;
    }
    now = HAL_GetTick();
    if (board->status->heat_d3_tx_ok_count != 0u)
    {
        gap = now - board->status->heat_d3_tx_tick;
        board->status->heat_d3_tx_gap_ms = gap;
        if (gap > board->status->heat_d3_tx_max_gap_ms)
        {
            board->status->heat_d3_tx_max_gap_ms = gap;
        }
    }
    board->status->heat_d3_tx_tick = now;
    board->status->heat_d3_tx_ok_count++;
    board->status->heat_d3_tx_ok = 1u;
}
```

## StartConnectTask

源码：[task_down/Application/TaskLayer/connect_task.c](G:/STM32_ALL/RM_code/Train_code_plus/task_down/Application/TaskLayer/connect_task.c)

```c
void StartConnectTask(void const *argument)
{
#if BOARD_COMM_TX_ENABLE && BOARD_COMM_D3_ENABLE
    uint32_t d3_tick = HAL_GetTick();
#endif
#if BOARD_COMM_TX_ENABLE && BOARD_COMM_D4_ENABLE
    uint32_t d4_tick = HAL_GetTick();
#endif
#if BOARD_COMM_TX_ENABLE
    uint32_t control_slots = 2u;
#if BOARD_COMM_D5_ENABLE
    control_slots++;
#endif
#endif

    (void)argument;

    for (;;)
    {
#if BOARD_COMM_TX_ENABLE
#if BOARD_COMM_D3_ENABLE
        if ((uint32_t)(HAL_GetTick() - d3_tick) >= BOARD_COMM_D3_PERIOD_MS)
        {
            board.tx_03(&board);
            if (board.status->heat_d3_tx_ok != 0u)
            {
                d3_tick = board.status->heat_d3_tx_tick;
            }
        }
#endif

        /* 整组容量不足时留待下周期 */
        board.status->gimbal_d1_tx_ok = 0u;
        board.status->gimbal_d2_tx_ok = 0u;
        if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) >= control_slots)
        {
            board.tx_01(&board);
            board.tx_02(&board);
#if BOARD_COMM_D5_ENABLE
            board.tx_05(&board);
#endif
        }
        else
        {
            board.status->control_tx_defer_count++;
        }
#if BOARD_COMM_D4_ENABLE
        if (((uint32_t)(HAL_GetTick() - d4_tick) >= BOARD_COMM_D4_PERIOD_MS) &&
            (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) > 0u))
        {
            d4_tick = HAL_GetTick();
            board.tx_04(&board);
        }
#endif
#endif

        osDelay(BOARD_COMM_D1D2_PERIOD_MS);
    }
}
```

## 关键边界

- D3 协议已替换，须同步更新上下板。
- 无有效初始裁判快照时默认禁止供弹；无裁判训练必须显式配置。
- 单发预算不主动退款，新有效裁判热量会覆盖预算；20 热量单位余量及停止余弹须人工实测。
- 摩擦轮电流仅用于标定观测，未用于出弹识别。

## 执行记录

- 源码：上板预算、限频、D3 接收与摩擦轮反馈时间；下板裁判热量快照、D3 打包与独立调度。
- 文档：README 热量行为、D3 布局、配置、Watch 与边界说明。
- 按用户规范跳过测试脚本、Mock、自动测试、自动编译与烧录。
- 状态：未编译，仅静态审查。
- 挂起：等待人工编译烧录与实机验证。

## CAN2 发送调度更新

2026-10-06：C1/C2 降为各 5 ms，D3 到期优先且失败重试。完整修改单元及人工观察项见 [CAN2 发送拥堵修复交付](can2_tx_delivery.md)。

## 裁判直接同步方案

新有效裁判热量允许上调或下调本地估计，重复源序号不覆盖；断链继续本地计热冷却，恢复首份有效快照重新校准。同步时不重置编码器计热基准。

安全余量 `LAUNCHER_HEAT_MARGIN=20`，连发低于余量停发、恢复到 30 解除；单发获准时预占 10，预占后须仍保留 20，即默认余量至少 30。`remaining` 为原始余量，不重复扣减安全余量。

人工观察：新的 `heat_seq` 到来时 `heat` 对齐 `referee_heat`；两次反馈之间仍计热冷却。同一序号的 D3 转发不得把本地增热清除。只需更新上板固件，本次没有改变 D3 协议。
