#include "launcher.h"

#include <math.h>
#include <string.h>

#include "communicate.h"
#include "launcher_config.h"
#include "main.h"
#include "motor.h"
#include "pid.h"
#include "rp_math.h"

launcher_t launcher; /* 发射机构对外状态 */
launcher_heat_t launcher_heat;

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

static launcher_heat_runtime_t launcher_heat_runtime;

static uint16_t launcher_fric_ready_count; /* 摩擦轮达速确认计数 */
uint8_t launcher_jam_count; /* 堵转次数，调试可观测 */
static uint16_t launcher_fric_stop_count; /* 摩擦轮停转确认计数 */
static uint8_t launcher_dial_last_online; /* 拨盘上次在线状态 */
static uint8_t launcher_dial_stopped; /* 1 = 拨盘已停机 */
static uint32_t launcher_dial_stop_tick; /* 上次停机命令时刻 */
static uint8_t launcher_dial_target_synced; /* 目标是否已对齐反馈 */
static uint8_t launcher_dial_recovery_repeat; /* 恢复后是否回连发 */
#if LAUNCHER_DIAL_JAM_ENABLE
static int8_t launcher_dial_motion_direction; /* 拨盘运动方向 */
#endif
static int64_t launcher_dial_target; /* 拨盘绝对目标角度，count */
static int64_t launcher_dial_feed_target; /* 退让前供弹目标，count */

static pid_ctrl_t launcher_dial_angle_pid; /* 拨盘位置环 */
static pid_ctrl_t launcher_dial_speed_pid; /* 拨盘单发速度环 */
pid_ctrl_t launcher_dial_repeat_pid; /* 拨盘连发速度环 */
static pid_ctrl_t launcher_dial_brake_pid; /* 拨盘释放制动环 */
static uint8_t launcher_dial_braking; /* 1 = 正在主动制动 */
static uint32_t launcher_dial_brake_tick; /* 制动开始时刻 */

/* 按步长将当前值斜坡到目标值 */
static float Launcher_Ramp(float current, float target, float step)
{
    float diff = target - current; /* 剩余变化量 */

    if (diff > step)
    {
        return current + step;
    }

    if (diff < -step)
    {
        return current - step;
    }

    return target;
}

/* 左/右摩擦轮是否在线 */
static uint8_t Launcher_FricOnline(uint8_t index)
{
    return ((rm_motor[index].state != NULL) &&
            (rm_motor[index].state->status == DEV_ONLINE)) ? 1u : 0u;
}

/* 拨盘是否在线 */
static uint8_t Launcher_DialOnline(void)
{
    return (dail_motor.KT_motor_info.state_info.work_state == M_ONLINE) ? 1u : 0u;
}

static uint8_t Launcher_HeatConfigValid(void)
{
    return ((LAUNCHER_HEAT_PER_SHOT > 0.0f) &&
            (LAUNCHER_DIAL_ONE_SHOT_ANGLE > 0.0f) &&
            (LAUNCHER_HEAT_MAX_RATE > 0.0f) &&
            (LAUNCHER_HEAT_STOP >= LAUNCHER_HEAT_PER_SHOT) &&
            (LAUNCHER_HEAT_RESUME > LAUNCHER_HEAT_STOP) &&
            (LAUNCHER_HEAT_SATURATE >= LAUNCHER_HEAT_RESUME) &&
            (LAUNCHER_HEAT_WARN > LAUNCHER_HEAT_SATURATE)) ? 1u : 0u;
}

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
            launcher_heat.heat = fmaxf(launcher_heat.heat, launcher_heat.referee_heat);
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

static uint8_t Launcher_HeatReserveSingle(void)
{
    if ((launcher_heat.ready == 0u) || (launcher_heat.blocked != 0u) ||
        (launcher_heat.remaining < LAUNCHER_HEAT_STOP) ||
        ((launcher_heat.heat + LAUNCHER_HEAT_PER_SHOT) > launcher_heat.heat_limit))
    {
        return 0u;
    }
    /* 单发预占不因取消而退还 */
    launcher_heat_runtime.repeat_tracking = 0u;
    launcher_heat.heat += LAUNCHER_HEAT_PER_SHOT;
    Launcher_HeatRefreshRate();
    return 1u;
}

static void Launcher_HeatStartRepeat(void)
{
    if (launcher_heat_runtime.repeat_tracking == 0u)
    {
        launcher_heat_runtime.travel = 0;
        launcher_heat_runtime.high_water = 0;
        launcher_heat_runtime.repeat_tracking = 1u;
    }
}

/* 拨盘角度按配置方向取符号 */
static int32_t Launcher_DialAngle(void)
{
    int32_t raw = dail_motor.KT_motor_info.rx_info.encoder_sum; /* 累计编码器值 */

    return (LAUNCHER_DIAL_ANGLE_SIGN < 0.0f) ? -raw : raw;
}

/* 拨盘单圈编码器角度 */
static int32_t Launcher_DialEncoder(void)
{
    return (int32_t)dail_motor.KT_motor_info.rx_info.encoder;
}

static int64_t Launcher_AbsInt64(int64_t value)
{
    return (value < 0) ? -value : value;
}

/* 判断拨盘是否到达目标容差 */
static uint8_t Launcher_DialAtTarget(int64_t target)
{
    int64_t error = target - (int64_t)Launcher_DialAngle(); /* 目标残差 */

    return (Launcher_AbsInt64(error) <=
            (int64_t)LAUNCHER_DIAL_STOP_ERROR) ? 1u : 0u;
}

/* 以力矩模式下发拨盘电流 */
static void Launcher_DialApplyTorque(int16_t current)
{
    if ((dail_motor.W_iqControl != NULL) && (dail_motor.tx_W_cmd != NULL))
    {
        dail_motor.W_iqControl(&dail_motor, current); /* 写入电流环 */
        (void)dail_motor.tx_W_cmd(&dail_motor, TORQUE_CLOSE_LOOP_ID);
    }
}

/* 清空拨盘 PID 历史状态 */
static void Launcher_DialClearPid(void)
{
    launcher_dial_angle_pid.integral = 0.0f;
    launcher_dial_angle_pid.last_err = 0.0f;
    launcher_dial_angle_pid.last_dout = 0.0f;
    launcher_dial_angle_pid.dout = 0.0f;
    launcher_dial_angle_pid.out = 0.0f;

    launcher_dial_speed_pid.integral = 0.0f;
    launcher_dial_speed_pid.last_err = 0.0f;
    launcher_dial_speed_pid.last_dout = 0.0f;
    launcher_dial_speed_pid.dout = 0.0f;
    launcher_dial_speed_pid.out = 0.0f;

    launcher_dial_repeat_pid.integral = 0.0f;
    launcher_dial_repeat_pid.last_err = 0.0f;
    launcher_dial_repeat_pid.last_dout = 0.0f;
    launcher_dial_repeat_pid.dout = 0.0f;
    launcher_dial_repeat_pid.out = 0.0f;

    launcher_dial_brake_pid.integral = 0.0f;
    launcher_dial_brake_pid.last_err = 0.0f;
    launcher_dial_brake_pid.last_dout = 0.0f;
    launcher_dial_brake_pid.dout = 0.0f;
    launcher_dial_brake_pid.out = 0.0f;
}

/* 安全停机，保留重试间隔 */
static HAL_StatusTypeDef Launcher_DialStop(void)
{
    Launcher_DialClearPid();
    if (dail_motor.tx_W_cmd == NULL)
    {
        return HAL_ERROR;
    }

    return dail_motor.tx_W_cmd(&dail_motor, MOTOR_STOP_ID);
}

/* 拨盘进入闭环运行 */
static HAL_StatusTypeDef Launcher_DialRun(void)
{
    if (dail_motor.tx_W_cmd == NULL)
    {
        return HAL_ERROR;
    }

    return dail_motor.tx_W_cmd(&dail_motor, MOTOR_RUN_ID);
}

/* 位置环生成速度目标，再由速度环出力矩 */
static void Launcher_DialPositionControl(int64_t target)
{
    float speed_target;    /* 位置环输出的速度目标 */
    int16_t current_output;/* 速度环输出的力矩电流 */

    if (Launcher_DialOnline() == 0u)
    {
        Launcher_DialClearPid();
        Launcher_DialApplyTorque(0);
        return;
    }

    launcher_dial_angle_pid.target = (float)target; /* 目标角 */
    launcher_dial_angle_pid.measure = (float)Launcher_DialAngle(); /* 反馈角 */
    launcher_dial_angle_pid.err =
        launcher_dial_angle_pid.target - launcher_dial_angle_pid.measure;
    single_pid_ctrl(&launcher_dial_angle_pid);

    speed_target = constrain(launcher_dial_angle_pid.out, /* 限制速度目标 */
                             -(float)LAUNCHER_DIAL_MAX_SPEED_DPS,
                             (float)LAUNCHER_DIAL_MAX_SPEED_DPS);

    launcher_dial_speed_pid.target = speed_target; /* 速度目标 */
    launcher_dial_speed_pid.measure =
        LAUNCHER_DIAL_SPEED_SIGN *
        (float)dail_motor.KT_motor_info.rx_info.speed; /* 反馈速度 */
    launcher_dial_speed_pid.err =
        launcher_dial_speed_pid.target - launcher_dial_speed_pid.measure;
    single_pid_ctrl(&launcher_dial_speed_pid);

    current_output = (int16_t)constrain( /* 电流输出限幅 */
        LAUNCHER_DIAL_OUTPUT_SIGN * launcher_dial_speed_pid.out,
        -LAUNCHER_DIAL_CURRENT_LIMIT,
        LAUNCHER_DIAL_CURRENT_LIMIT);
    Launcher_DialApplyTorque(current_output);
}

/* 连发独立速度环 */
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

/* 释放输入后用阻尼力矩刹车，速度降下来再断使能。 */
static void Launcher_DialBrakeControl(void)
{
    int16_t current_output;

    if (Launcher_DialOnline() == 0u)
    {
        Launcher_DialClearPid();
        Launcher_DialApplyTorque(0);
        return;
    }

    launcher_dial_brake_pid.target = 0.0f;
    launcher_dial_brake_pid.measure =
        LAUNCHER_DIAL_SPEED_SIGN *
        (float)dail_motor.KT_motor_info.rx_info.speed;
    launcher_dial_brake_pid.err =
        launcher_dial_brake_pid.target -
        launcher_dial_brake_pid.measure;
    single_pid_ctrl(&launcher_dial_brake_pid);

    current_output = (int16_t)constrain(
        LAUNCHER_DIAL_OUTPUT_SIGN * launcher_dial_brake_pid.out,
        -LAUNCHER_DIAL_CURRENT_LIMIT,
        LAUNCHER_DIAL_CURRENT_LIMIT);
    Launcher_DialApplyTorque(current_output);
}

/* 低速高电流连续确认后判定堵转 */
#if LAUNCHER_DIAL_JAM_ENABLE
static uint8_t Launcher_DialBlockCheck(uint8_t moving)
{
    uint8_t blocked; /* 本拍是否疑似堵转 */
    float speed = fabsf((float)dail_motor.KT_motor_info.rx_info.speed); /* 反馈转速 */
    float current = fabsf((float)dail_motor.KT_motor_info.rx_info.current); /* 反馈电流 */

    blocked = (moving != 0u) && /* 旋转中且低速高流 */
              (speed <= (float)LAUNCHER_DIAL_JAM_SPEED_DPS) &&
              (current >= (float)LAUNCHER_DIAL_JAM_CURRENT_RAW);

    if (blocked != 0u)
    {
        if (launcher.jam_tick < LAUNCHER_DIAL_JAM_CONFIRM_TICKS)
        {
            launcher.jam_tick++;
        }
    }
    else
    {
        launcher.jam_tick = 0u;
    }

    return (launcher.jam_tick >=
            LAUNCHER_DIAL_JAM_CONFIRM_TICKS) ? 1u : 0u;
}

/* 堵转后反向退让，再回到原供弹目标 */
static void Launcher_DialEnterStuckRecovery(uint8_t continuous)
{
    int64_t current_angle = (int64_t)Launcher_DialAngle(); /* 当前绝对角度 */
    int64_t error = launcher_dial_target - current_angle; /* 目标残差 */

    launcher_dial_recovery_repeat = continuous;
    launcher_dial_feed_target =
        (continuous != 0u) ? current_angle : launcher_dial_target;
    launcher_dial_motion_direction =
        (continuous != 0u) ? (int8_t)LAUNCHER_DIAL_DIRECTION :
                             ((error < 0) ? -1 : 1);
    launcher_dial_target = current_angle -
                           (int64_t)launcher_dial_motion_direction *
                           (int64_t)LAUNCHER_DIAL_REVERSE_ANGLE;

    launcher.state = LAUNCHER_REVERSE;
    launcher.state_tick = HAL_GetTick();
    launcher.jam_tick = 0u;
    launcher_jam_count++;
    Launcher_DialClearPid();
}
#endif

/* 拨盘状态机：单发升沿、连发、退让与复位 */
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

/* 非发射状态下周期停机，避免丢帧导致失控 */
static void Launcher_DialSafeStop(uint32_t now)
{
    if (((launcher_dial_stopped == 0u) ||
         ((now - launcher_dial_stop_tick) >=
          LAUNCHER_DIAL_SAFE_STOP_RETRY_MS)) &&
        (Launcher_DialStop() == HAL_OK))
    {
        launcher_dial_stopped = 1u;
        launcher_dial_stop_tick = now;
    }

    launcher.jam_tick = 0u;
    launcher_dial_target_synced = 0u;
}

/* 根据发射请求选择拨盘控制模式 */
static void Launcher_DialControl(uint8_t shoot_active)
{
    uint32_t now = HAL_GetTick();

    /* 重新发射时取消制动。 */
    if (shoot_active != 0u)
    {
        launcher_dial_braking = 0u;
    }
    else
    {
        if ((launcher.state != LAUNCHER_SLEEP) &&
            (launcher.state != LAUNCHER_STOPPING) &&
            (launcher.state != LAUNCHER_FAULT))
        {
            launcher.state = LAUNCHER_READY;
            launcher.state_tick = now;
            launcher.jam_tick = 0u;
            launcher_dial_target_synced = 0u;
        }

        if (launcher_dial_stopped == 0u)
        {
            if (launcher_dial_braking == 0u)
            {
                launcher_dial_braking = 1u;
                launcher_dial_brake_tick = now;
                Launcher_DialClearPid();
            }

            Launcher_DialBrakeControl();
            if ((fabsf((float)dail_motor.KT_motor_info.rx_info.speed) <=
                 (float)LAUNCHER_DIAL_BRAKE_STOP_SPEED_DPS) ||
                ((now - launcher_dial_brake_tick) >=
                 LAUNCHER_DIAL_BRAKE_TIMEOUT_MS))
            {
                launcher_dial_braking = 0u;
                Launcher_DialSafeStop(now);
            }
            return;
        }

        launcher_dial_braking = 0u;
        Launcher_DialSafeStop(now);
        return;
    }
#if LAUNCHER_DIAL_ENABLE
#if !LAUNCHER_DIAL_READY_HOLD_ENABLE
    if (launcher.state == LAUNCHER_READY)
    {
        Launcher_DialStop();
        return;
    }
#endif

    if (launcher.state == LAUNCHER_REPEAT)
    {
        Launcher_DialSpeedControl();
    }
    else
    {
        Launcher_DialPositionControl(launcher_dial_target);
    }
#else
    if (dail_motor.tx_W_cmd != NULL)
    {
        (void)dail_motor.tx_W_cmd(&dail_motor, MOTOR_CLOSE_ID);
    }
#endif
}

/* 双摩擦轮速度环，离线或失能时卸力 */
static void Launcher_FricControl(uint8_t enable)
{
    for (uint8_t i = 0u; i < SHOOT_FRIC_NUM; i++)
    {
        pid_ctrl_t *pid = rm_motor[i].ctrl->speed_ctrl; /* 当前速度环 */
        float direction = (i == SHOOT_FRIC_L) ?
                          LAUNCHER_FRIC_L_DIRECTION :
                          LAUNCHER_FRIC_R_DIRECTION;

        if ((enable != 0u) && (Launcher_FricOnline(i) != 0u))
        {
            pid->target = direction * launcher.fric_target_rpm; /* 目标转速 */
            pid->measure = (float)rm_motor[i].rx_info->encoder_speed; /* 反馈转速 */
            pid->err = pid->target - pid->measure;

            if (launcher.state == LAUNCHER_STOPPING)
            {
                pid->integral = 0.0f;
            }

            single_pid_ctrl(pid);
            rm_motor[i].tx_info->torque = /* 力矩限幅 */
                constrain(pid->out,
                          -LAUNCHER_FRIC_OUT_MAX,
                          LAUNCHER_FRIC_OUT_MAX);
        }
        else
        {
            pid->integral = 0.0f;
            pid->last_err = 0.0f;
            pid->out = 0.0f;
            rm_motor[i].tx_info->torque = 0.0f;
        }
    }

    RM_Group.group_set_torque(&RM_Group);
}

/* 双轮连续达速后才置就绪 */
static void Launcher_UpdateFrictionReady(uint8_t enabled)
{
    float l_speed = fabsf((float)rm_motor[SHOOT_FRIC_L].rx_info->encoder_speed); /* 左轮转速 */
    float r_speed = fabsf((float)rm_motor[SHOOT_FRIC_R].rx_info->encoder_speed); /* 右轮转速 */

    launcher.fric_l_speed_rpm = l_speed; /* 左轮反馈 */
    launcher.fric_r_speed_rpm = r_speed; /* 右轮反馈 */

    if ((enabled != 0u) &&
        (Launcher_FricOnline(SHOOT_FRIC_L) != 0u) &&
        (Launcher_FricOnline(SHOOT_FRIC_R) != 0u) &&
        (launcher.fric_target_rpm >= (LAUNCHER_FRIC_TARGET_RPM * 0.5f)) &&
        (fabsf(l_speed - launcher.fric_target_rpm) <=
         LAUNCHER_FRIC_READY_TOL_RPM) &&
        (fabsf(r_speed - launcher.fric_target_rpm) <=
         LAUNCHER_FRIC_READY_TOL_RPM))
    {
        if (launcher_fric_ready_count < LAUNCHER_FRIC_READY_TIME_MS)
        {
            launcher_fric_ready_count++;
        }
    }
    else
    {
        launcher_fric_ready_count = 0u;
    }

    launcher.fric_ready =
        (launcher_fric_ready_count >= LAUNCHER_FRIC_READY_TIME_MS) ? 1u : 0u;
}

/* 初始化发射机构状态与三套 PID */
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

/* 发射机构周期任务 */
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
