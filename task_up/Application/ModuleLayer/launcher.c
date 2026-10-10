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
launcher_dial_t launcher_dial;
launcher_speed_t launcher_speed;

typedef struct
{
    uint32_t repeat_tick; // 连发最近反馈时刻，ms
    uint32_t rearm_tick; // 反馈隔离开始时刻，ms
    uint32_t learning_tick; // 学习隔离起点，ms
    uint32_t repeat_ready_tick; // 连发目标达速时刻，ms
    uint16_t last_seq; // 最近消费序号，uint16循环
    uint8_t seq_seen; // 序号基准有效，0/1
    uint8_t last_mode; // 上次单发或连发，0/1
    uint8_t last_request; // 上次摩擦轮开关，0/1
    uint8_t last_vehicle; // 上次整车使能，0/1
    uint8_t last_source; // 上次反馈源有效，0/1
    uint8_t reserved; // 单发已受理未启动，0/1
    uint8_t learning_blocked; // 关联不明暂停学习，0/1
    uint8_t repeat_ready_seen; // 连发达速基准有效，0/1
    uint8_t press_single_started; // 本次按住已启动单发，0/1
    launcher_speed_reason_e fault; // 反馈阻止原因，0~4
} launcher_speed_runtime_t;

static launcher_speed_runtime_t launcher_speed_runtime;
static uint8_t launcher_keyboard_press_ready; // 本次键鼠按住许可，0/1

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
static uint8_t launcher_dial_stopped; /* 1 = 拨盘已停机 */
static uint32_t launcher_dial_stop_tick; /* 上次停机命令时刻 */
static uint8_t launcher_dial_target_synced; /* 目标是否已对齐反馈 */
static uint8_t launcher_dial_recovery_repeat; /* 恢复后是否回连发 */
static uint8_t launcher_dial_last_is_hole; // 上次过洞请求，0/1
static uint8_t launcher_dial_hole_release_pending; // 退出延时有效，0/1
static uint32_t launcher_dial_hole_release_tick; // 退出请求时刻，ms
static uint32_t launcher_dial_hold_tx_tick; // 上次保持发送时刻，ms
static uint32_t launcher_dial_hold_active_tick; // 上次保持纠偏时刻，ms
#if LAUNCHER_DIAL_JAM_ENABLE
static int8_t launcher_dial_motion_direction; /* 拨盘运动方向 */
#endif
static int64_t launcher_dial_target; /* 拨盘绝对目标角度，count */
static int64_t launcher_dial_feed_target; /* 退让前供弹目标，count */

static pid_ctrl_t launcher_dial_angle_pid; /* 拨盘位置环 */
static pid_ctrl_t launcher_dial_speed_pid; /* 拨盘单发速度环 */
pid_ctrl_t launcher_dial_repeat_pid; /* 拨盘连发速度环 */
static pid_ctrl_t launcher_dial_brake_pid; /* 拨盘释放制动环 */
static pid_ctrl_t launcher_dial_hold_angle_pid;
static pid_ctrl_t launcher_dial_hold_speed_pid;

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

static void Launcher_SpeedClearSamples(void)
{
    launcher_speed.sample_count = 0u;
    launcher_speed.sample_sum_mps = 0.0f;
}

static void Launcher_SpeedLearn(float speed, uint8_t mode)
{
    float error;
    float step;
    float next;
    float *target = (mode != 0u) ? &launcher_speed.repeat_target_rpm :
        &launcher_speed.single_target_rpm;

    launcher_speed.sample_sum_mps += speed;
    launcher_speed.sample_count++;
    if (launcher_speed.sample_count < LAUNCHER_SPEED_SAMPLE_COUNT)
    {
        return;
    }
    launcher_speed.average_mps = launcher_speed.sample_sum_mps /
        (float)LAUNCHER_SPEED_SAMPLE_COUNT;
    error = LAUNCHER_SPEED_TARGET_MPS - launcher_speed.average_mps;
    if (fabsf(error) > LAUNCHER_SPEED_DEADBAND_MPS)
    {
        step = constrain(LAUNCHER_SPEED_GAIN * error,
            -LAUNCHER_SPEED_STEP_MAX_RPM, LAUNCHER_SPEED_STEP_MAX_RPM);
        next = *target + step;
        launcher_speed.at_limit = ((next <= LAUNCHER_SPEED_MIN_RPM) ||
            (next >= LAUNCHER_SPEED_MAX_RPM)) ? 1u : 0u;
        next = constrain(next, LAUNCHER_SPEED_MIN_RPM, LAUNCHER_SPEED_MAX_RPM);
        if (next != *target)
        {
            *target = next;
            launcher_fric_ready_count = 0u;
            launcher.fric_ready = 0u;
            launcher_speed_runtime.repeat_ready_seen = 0u;
        }
    }
    else
    {
        launcher_speed.at_limit = 0u;
    }
    Launcher_SpeedClearSamples();
}

static void Launcher_SpeedFail(launcher_speed_reason_e reason)
{
    launcher_keyboard_press_ready = 0u;
    if ((reason == LAUNCHER_SPEED_TIMEOUT) &&
        (launcher_speed_runtime.fault != reason))
    {
        launcher_speed.timeout_count++;
    }
    if ((reason == LAUNCHER_SPEED_ASSOCIATION) &&
        (launcher_speed_runtime.fault != reason))
    {
        launcher_speed.association_count++;
    }
    launcher_speed_runtime.fault = reason;
    launcher_speed.waiting = 0u;
    launcher_speed_runtime.reserved = 0u;
    launcher_speed_runtime.press_single_started = 0u;
    launcher_speed_runtime.repeat_ready_seen = 0u;
    launcher_dial.trigger_ready = 0u;
    Launcher_SpeedClearSamples();
}

static void Launcher_SpeedDropLearning(launcher_speed_reason_e reason, uint32_t now)
{
    if (reason == LAUNCHER_SPEED_TIMEOUT)
    {
        launcher_speed.timeout_count++;
    }
    else if (reason == LAUNCHER_SPEED_ASSOCIATION)
    {
        launcher_speed.association_count++;
    }
    launcher_speed.waiting = 0u;
    launcher_speed_runtime.learning_blocked = 1u;
    launcher_speed_runtime.learning_tick = now;
    Launcher_SpeedClearSamples();
}

static void Launcher_SpeedResetGroup(void)
{
    launcher_speed.waiting = 0u;
    launcher_speed_runtime.reserved = 0u;
    launcher_speed_runtime.learning_blocked = 0u;
    launcher_speed_runtime.repeat_ready_seen = 0u;
    launcher_speed_runtime.press_single_started = 0u;
    launcher_speed_runtime.fault = LAUNCHER_SPEED_OK;
    launcher_speed.single_target_rpm = LAUNCHER_FRIC_TARGET_RPM;
    launcher_speed.repeat_target_rpm = LAUNCHER_FRIC_TARGET_RPM;
    launcher_speed.at_limit = 0u;
    launcher_dial.trigger_ready = 0u;
    Launcher_SpeedClearSamples();
}

/* 弹速外环只按新事件更新 */
static void Launcher_SpeedUpdate(uint32_t now, uint8_t vehicle_on,
                                 uint8_t request, uint8_t mode)
{
    Board_Speed_Pkt_t snapshot;
    uint32_t age;
    uint32_t elapsed;
    uint16_t delta;
    uint8_t fresh;
    uint8_t matching;
    uint8_t new_event = 0u;
    uint8_t reset_group;
    uint8_t continue_press;
    uint16_t event_delta = 0u;
    float speed;
    Board_GetSpeedSnapshot(&snapshot);
    now = HAL_GetTick();
    reset_group = ((mode != launcher_speed_runtime.last_mode) ||
                   ((request != 0u) && (launcher_speed_runtime.last_request == 0u)) ||
                   ((vehicle_on != 0u) && (launcher_speed_runtime.last_vehicle == 0u))) ? 1u : 0u;
    if ((request != 0u) && (launcher_speed_runtime.last_request == 0u))
    {
        launcher_speed.guard_latched = 0u;
    }
    if (reset_group != 0u)
    {
        continue_press = ((mode != 0u) && (launcher_speed_runtime.last_mode == 0u) &&
            (request != 0u) && (launcher_speed_runtime.last_request != 0u) &&
            (vehicle_on != 0u) && (launcher_speed_runtime.last_vehicle != 0u) &&
            (launcher_speed_runtime.last_source != 0u) &&
            (launcher_speed_runtime.fault == LAUNCHER_SPEED_OK) &&
            (launcher_speed.guard_latched == 0u) &&
            ((launcher_dial.trigger_ready != 0u) ||
             (launcher_speed_runtime.press_single_started != 0u) ||
             (launcher_keyboard_press_ready != 0u))) ? 1u : 0u;
        Launcher_SpeedResetGroup();
        launcher_dial.trigger_ready = continue_press;
        launcher_speed_runtime.rearm_tick = (continue_press != 0u) ?
            now - LAUNCHER_SPEED_REARM_MS : now;
        launcher_fric_ready_count = 0u;
        launcher.fric_ready = 0u;
    }
    launcher_speed_runtime.last_mode = mode;
    launcher_speed_runtime.last_request = request;
    launcher_speed_runtime.last_vehicle = vehicle_on;

    age = (uint32_t)(now - snapshot.event_rx_tick);
    if (age > (UINT32_MAX - snapshot.event_age_ms))
    {
        age = UINT32_MAX;
    }
    else
    {
        age += snapshot.event_age_ms;
    }
    launcher_speed.feedback_age_ms = age;
    fresh = ((snapshot.seen != 0u) && (snapshot.speed_cms != 0u) &&
             (age <= LAUNCHER_SPEED_WAIT_MS)) ? 1u : 0u;
    matching = ((snapshot.bullet_type == LAUNCHER_SPEED_BULLET_TYPE) &&
                (snapshot.shooter_number == LAUNCHER_SPEED_SHOOTER_NUMBER)) ? 1u : 0u;
    launcher_speed.source_ready = ((vehicle_on != 0u) &&
        (snapshot.seen != 0u) &&
        ((uint32_t)(now - snapshot.rx_tick) < LAUNCHER_SPEED_LINK_MS) &&
        (snapshot.age_ms != UINT16_MAX)) ? 1u : 0u;

    if (launcher_speed.source_ready == 0u)
    {
        Launcher_SpeedFail(((snapshot.seen == 0u) ||
            ((uint32_t)(now - snapshot.rx_tick) >= LAUNCHER_SPEED_LINK_MS) ||
            (vehicle_on == 0u)) ? LAUNCHER_SPEED_LINK : LAUNCHER_SPEED_SOURCE);
    }
    else
    {
        if ((launcher_speed_runtime.learning_blocked != 0u) &&
            (launcher_speed.waiting == 0u) &&
            (launcher_speed_runtime.reserved == 0u) &&
            ((uint32_t)(now - launcher_speed_runtime.learning_tick) >= LAUNCHER_SPEED_WAIT_MS))
        {
            launcher_speed_runtime.learning_blocked = 0u;
        }
        if (launcher_speed_runtime.seq_seen == 0u)
        {
            launcher_speed_runtime.last_seq = snapshot.event_seq;
            launcher_speed_runtime.seq_seen = 1u;
            new_event = 1u;
        }
        else if (snapshot.event_seq != launcher_speed_runtime.last_seq)
        {
            event_delta = (uint16_t)(snapshot.event_seq - launcher_speed_runtime.last_seq);
            new_event = 1u;
            launcher_speed_runtime.last_seq = snapshot.event_seq;
        }
        launcher_speed.event_seq = snapshot.event_seq;
        if (launcher_speed_runtime.last_source == 0u)
        {
            launcher_speed_runtime.rearm_tick = now;
            launcher_dial.trigger_ready = 0u;
            if ((launcher_speed_runtime.fault == LAUNCHER_SPEED_LINK) ||
                (launcher_speed_runtime.fault == LAUNCHER_SPEED_SOURCE))
            {
                launcher_speed_runtime.fault = LAUNCHER_SPEED_OK;
            }
        }

        if ((mode != 0u) && (launcher_dial.state == LAUNCHER_REPEAT) &&
            (launcher.fric_ready != 0u) &&
            (launcher_speed_runtime.repeat_ready_seen == 0u))
        {
            launcher_speed_runtime.repeat_ready_tick = now;
            launcher_speed_runtime.repeat_ready_seen = 1u;
        }
        if ((new_event != 0u) && (matching != 0u) && (fresh != 0u))
        {
            speed = (float)snapshot.speed_cms * 0.01f;
            launcher_speed.latest_mps = speed;
            launcher_speed_runtime.repeat_tick = now;
            if (speed >= LAUNCHER_SPEED_GUARD_MPS)
            {
                launcher_keyboard_press_ready = 0u;
                launcher_speed.guard_count++;
                if (speed >= LAUNCHER_SPEED_LIMIT_MPS)
                {
                    launcher_speed.limit_count++;
                }
                if (launcher_speed.guard_latched == 0u)
                {
                    launcher_speed.target_rpm =
                        ((mode != 0u) ? launcher_speed.repeat_target_rpm :
                         launcher_speed.single_target_rpm) * LAUNCHER_SPEED_GUARD_SCALE;
                    launcher_speed.guard_latched = 1u;
                }
                launcher_speed.waiting = 0u;
                launcher_speed_runtime.reserved = 0u;
                launcher_speed_runtime.press_single_started = 0u;
                launcher_dial.trigger_ready = 0u;
                Launcher_SpeedClearSamples();
            }
            else if ((mode != 0u) && (launcher_dial.state == LAUNCHER_REPEAT))
            {
                elapsed = (uint32_t)(now - launcher_speed_runtime.repeat_ready_tick);
                if ((event_delta != 1u) ||
                    (launcher_speed_runtime.repeat_ready_seen == 0u) ||
                    (age > elapsed) ||
                    ((uint32_t)(snapshot.event_rx_tick - launcher_speed_runtime.repeat_ready_tick) > elapsed))
                {
                    if (event_delta != 1u)
                    {
                        launcher_speed.association_count++;
                    }
                    Launcher_SpeedClearSamples();
                }
                else
                {
                    Launcher_SpeedLearn(speed, 1u);
                }
            }
            else if (launcher_speed_runtime.reserved != 0u)
            {
                Launcher_SpeedDropLearning(LAUNCHER_SPEED_ASSOCIATION, now);
                launcher_speed.pending_seq = snapshot.event_seq;
            }
            else if ((launcher_speed.waiting != 0u) && (mode == 0u))
            {
                delta = (uint16_t)(snapshot.event_seq - launcher_speed.pending_seq);
                elapsed = (uint32_t)(now - launcher_speed.wait_tick);
                if ((delta != 1u) || (elapsed > LAUNCHER_SPEED_WAIT_MS) ||
                    (age > elapsed) ||
                    ((uint32_t)(snapshot.event_rx_tick - launcher_speed.wait_tick) >
                     LAUNCHER_SPEED_WAIT_MS))
                {
                    Launcher_SpeedDropLearning(LAUNCHER_SPEED_ASSOCIATION, now);
                }
                else if (launcher_speed_runtime.learning_blocked != 0u)
                {
                    launcher_speed.waiting = 0u;
                    Launcher_SpeedClearSamples();
                }
                else
                {
                    launcher_speed.waiting = 0u;
                    Launcher_SpeedLearn(speed, 0u);
                }
            }
            else if ((launcher_speed_runtime.fault == LAUNCHER_SPEED_TIMEOUT) ||
                     (launcher_speed_runtime.fault == LAUNCHER_SPEED_ASSOCIATION))
            {
                /* 恢复样本不参与学习 */
                launcher_speed_runtime.fault = LAUNCHER_SPEED_OK;
                launcher_dial.trigger_ready = 0u;
                launcher_speed_runtime.rearm_tick = now;
                Launcher_SpeedClearSamples();
            }
        }
        else if ((new_event != 0u) && (mode != 0u) &&
                 (launcher_dial.state == LAUNCHER_REPEAT))
        {
            launcher_speed.association_count++;
            Launcher_SpeedClearSamples();
        }
        else if ((new_event != 0u) && (launcher_speed.waiting != 0u))
        {
            Launcher_SpeedDropLearning(LAUNCHER_SPEED_ASSOCIATION, now);
        }
        if ((launcher_speed.waiting != 0u) &&
            ((uint32_t)(now - launcher_speed.wait_tick) >= LAUNCHER_SPEED_WAIT_MS))
        {
            Launcher_SpeedDropLearning(LAUNCHER_SPEED_TIMEOUT, now);
        }
        if ((launcher_dial.state == LAUNCHER_REPEAT) && (request != 0u) &&
            ((uint32_t)(now - launcher_speed_runtime.repeat_tick) >= LAUNCHER_SPEED_WAIT_MS))
        {
            launcher_speed.timeout_count++;
            launcher_speed_runtime.repeat_tick = now;
            launcher_speed_runtime.repeat_ready_seen = 0u;
            Launcher_SpeedClearSamples();
        }
    }
    launcher_speed_runtime.last_source = launcher_speed.source_ready;
    if ((request == 0u) || (vehicle_on == 0u))
    {
        launcher_speed.waiting = 0u;
        launcher_speed_runtime.reserved = 0u;
        Launcher_SpeedClearSamples();
    }
    if (launcher_speed.guard_latched == 0u)
    {
        launcher_speed.target_rpm = (mode != 0u) ?
            launcher_speed.repeat_target_rpm : launcher_speed.single_target_rpm;
    }
    launcher_speed.block_reason = (launcher_speed.guard_latched != 0u) ?
        LAUNCHER_SPEED_HIGH : launcher_speed_runtime.fault;
    if ((launcher_speed.block_reason == LAUNCHER_SPEED_OK) &&
        ((uint32_t)(now - launcher_speed_runtime.rearm_tick) < LAUNCHER_SPEED_REARM_MS))
    {
        launcher_speed.block_reason = LAUNCHER_SPEED_REARM;
    }
    launcher_speed.feed_ready = ((launcher_speed.source_ready != 0u) &&
        (launcher_speed.block_reason == LAUNCHER_SPEED_OK)) ? 1u : 0u;
}

static uint8_t Launcher_SpeedReserve(void)
{
    Board_Speed_Pkt_t snapshot;
    uint32_t now;
    Board_GetSpeedSnapshot(&snapshot);
    now = HAL_GetTick();
    if ((snapshot.event_seq != launcher_speed_runtime.last_seq) ||
        (snapshot.age_ms == UINT16_MAX) || (snapshot.seen == 0u) ||
        ((uint32_t)(now - snapshot.rx_tick) >= LAUNCHER_SPEED_LINK_MS))
    {
        return 0u;
    }
    /* 重叠反馈只隔离学习 */
    if (launcher_speed.waiting != 0u)
    {
        Launcher_SpeedDropLearning(LAUNCHER_SPEED_ASSOCIATION, now);
    }
    if (launcher_speed_runtime.learning_blocked != 0u)
    {
        launcher_speed_runtime.learning_tick = now;
    }
    launcher_speed.pending_seq = snapshot.event_seq;
    launcher_speed_runtime.reserved = 1u;
    return 1u;
}

static void Launcher_SpeedStart(uint32_t now)
{
    now = HAL_GetTick();
    launcher_speed.wait_tick = now;
    launcher_speed.waiting = 1u;
    launcher_speed_runtime.reserved = 0u;
    launcher_speed_runtime.press_single_started = 1u;
    if (launcher_speed_runtime.learning_blocked != 0u)
    {
        launcher_speed_runtime.learning_tick = now;
    }
    launcher_fric_ready_count = 0u;
    launcher.fric_ready = 0u;
    launcher_dial.trigger_ready = 0u;
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
            (LAUNCHER_HEAT_MARGIN >= LAUNCHER_HEAT_PER_SHOT) &&
            (LAUNCHER_HEAT_STOP >= LAUNCHER_HEAT_MARGIN) &&
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
            if (((launcher_dial.state == LAUNCHER_READY) ||
                 (launcher_dial.state == LAUNCHER_SLEEP)) &&
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
    launcher_dial.output_current_raw = current;
    launcher_dial.torque_tx_status = (uint8_t)HAL_ERROR;
    if ((dail_motor.W_iqControl != NULL) && (dail_motor.tx_W_cmd != NULL))
    {
        dail_motor.W_iqControl(&dail_motor, current);
        launcher_dial.torque_tx_status = (uint8_t)
            dail_motor.tx_W_cmd(&dail_motor, TORQUE_CLOSE_LOOP_ID);
    }
}

/* 清空拨盘 PID 历史状态 */
static void Launcher_DialClearPid(void)
{
    integral_to_zero(&launcher_dial_angle_pid);
    integral_to_zero(&launcher_dial_speed_pid);
    integral_to_zero(&launcher_dial_repeat_pid);
    integral_to_zero(&launcher_dial_brake_pid);
    integral_to_zero(&launcher_dial_hold_angle_pid);
    integral_to_zero(&launcher_dial_hold_speed_pid);
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
    launcher_dial_angle_pid.err = (float)(target - (int64_t)Launcher_DialAngle());
    single_pid_ctrl(&launcher_dial_angle_pid);

    speed_target = constrain(launcher_dial_angle_pid.out, /* 限制速度目标 */
                             -(float)LAUNCHER_DIAL_MAX_SPEED_DPS,
                             (float)LAUNCHER_DIAL_MAX_SPEED_DPS);
    launcher_dial.speed_target_dps = speed_target;

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
    launcher_dial.speed_target_dps = launcher_dial_repeat_pid.target;
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

/* 制动后接管停止位置 */
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
    launcher_dial.speed_target_dps = 0.0f;
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

    launcher_dial.state = LAUNCHER_REVERSE;
    launcher_dial.state_tick = HAL_GetTick();
    launcher.jam_tick = 0u;
    launcher_jam_count++;
    Launcher_DialClearPid();
}
#endif

static void Launcher_DialReject(launcher_dial_reject_e reason)
{
    if ((reason != LAUNCHER_DIAL_REJECT_FRIC) &&
        (reason != LAUNCHER_DIAL_REJECT_SPEED))
    {
        launcher_keyboard_press_ready = 0u;
    }
    launcher_speed_runtime.press_single_started = 0u;
    launcher_dial.reject_reason = reason;
    launcher_dial.rejected_count++;
    launcher_dial.trigger_ready = 0u;
}

static void Launcher_DialEnterHold(uint32_t now, uint8_t capture)
{
    if (capture != 0u)
    {
        launcher_dial_target = (int64_t)Launcher_DialAngle();
        launcher_dial_feed_target = launcher_dial_target;
    }
    launcher_dial.state = LAUNCHER_READY;
    launcher_dial.state_tick = now;
    launcher_dial.settling = 0u;
    launcher.jam_tick = 0u;
    Launcher_DialClearPid();
    launcher_dial_hold_tx_tick = now - LAUNCHER_DIAL_HOLD_TX_INTERVAL_MS;
    launcher_dial_hold_active_tick = now;
}

static void Launcher_DialEnterBrake(uint32_t now)
{
    launcher_keyboard_press_ready = 0u;
    launcher_speed_runtime.press_single_started = 0u;
    launcher_dial.trigger_ready = 0u;
    launcher_dial.state = LAUNCHER_STOPPING;
    launcher_dial.state_tick = now;
    launcher_dial.settling = 0u;
    launcher.jam_tick = 0u;
    Launcher_DialClearPid();
}

/* 死区内保留速度阻尼 */
static void Launcher_DialHoldControl(void)
{
    int16_t current_output;
    uint32_t now = HAL_GetTick();
    uint32_t tx_interval;

    launcher_dial_hold_angle_pid.target = (float)launcher_dial_target;
    launcher_dial_hold_angle_pid.measure = (float)Launcher_DialAngle();
    launcher_dial_hold_angle_pid.err =
        (float)(launcher_dial_target - (int64_t)Launcher_DialAngle());
    single_pid_ctrl(&launcher_dial_hold_angle_pid);
    launcher_dial.speed_target_dps = launcher_dial_hold_angle_pid.out;

    launcher_dial_hold_speed_pid.target = launcher_dial.speed_target_dps;
    launcher_dial_hold_speed_pid.measure = LAUNCHER_DIAL_SPEED_SIGN *
        (float)dail_motor.KT_motor_info.rx_info.speed;
    launcher_dial_hold_speed_pid.err =
        launcher_dial_hold_speed_pid.target - launcher_dial_hold_speed_pid.measure;
    single_pid_ctrl(&launcher_dial_hold_speed_pid);
    current_output = (int16_t)constrain(
        LAUNCHER_DIAL_OUTPUT_SIGN * launcher_dial_hold_speed_pid.out,
        -LAUNCHER_DIAL_CURRENT_LIMIT, LAUNCHER_DIAL_CURRENT_LIMIT);
    if ((fabsf(launcher_dial_hold_angle_pid.err) > 0.0f) ||
        (fabsf(launcher_dial_hold_speed_pid.measure) >
         (float)LAUNCHER_DIAL_SETTLE_SPEED_DPS))
    {
        launcher_dial_hold_active_tick = now;
    }

    // NOTE: 纠偏保留快速阻尼
    tx_interval = ((now - launcher_dial_hold_active_tick) <
                   LAUNCHER_DIAL_HOLD_IDLE_CONFIRM_MS) ?
                  LAUNCHER_DIAL_HOLD_ACTIVE_TX_MS :
                  LAUNCHER_DIAL_HOLD_TX_INTERVAL_MS;
    if ((now - launcher_dial_hold_tx_tick) >= tx_interval)
    {
        Launcher_DialApplyTorque(current_output);
        launcher_dial_hold_tx_tick = now;
    }
}

static uint8_t Launcher_DialHoleHoldAllowed(uint32_t now)
{
    if (Board_Rx_Info.shoot_pkt.is_hole != 0u)
    {
        launcher_dial_last_is_hole = 1u;
        launcher_dial_hole_release_pending = 0u;
        return 0u;
    }

    if (launcher_dial_last_is_hole != 0u)
    {
        launcher_dial_last_is_hole = 0u;
        launcher_dial_hole_release_pending = 1u;
        launcher_dial_hole_release_tick = now;
    }

    // NOTE: 恢复不依赖升降到位
    if (launcher_dial_hole_release_pending != 0u)
    {
        if ((now - launcher_dial_hole_release_tick) <
            LAUNCHER_DIAL_HOLE_RELEASE_DELAY_MS)
        {
            return 0u;
        }
        launcher_dial_hole_release_pending = 0u;
    }

    return 1u;
}

/* 失能后禁止追赶旧目标 */
static void Launcher_DialSafeStop(uint32_t now)
{
    HAL_StatusTypeDef status;

    launcher_dial.speed_target_dps = 0.0f;
    launcher_dial.output_current_raw = 0;
    if ((launcher_dial_stopped == 0u) ||
        ((now - launcher_dial_stop_tick) >= LAUNCHER_DIAL_SAFE_STOP_RETRY_MS))
    {
        status = Launcher_DialStop();
        launcher_dial.stop_tx_status = (uint8_t)status;
        if (status == HAL_OK)
        {
            launcher_dial_stopped = 1u;
            launcher_dial_stop_tick = now;
        }
        else
        {
            Launcher_DialApplyTorque(0);
        }
    }
    launcher.jam_tick = 0u;
    launcher_dial_target_synced = 0u;
    launcher_dial.settling = 0u;
}

static void Launcher_DialUpdate(uint32_t now, uint8_t single_rising,
                                uint8_t continuous)
{
    HAL_StatusTypeDef status;
    float speed = fabsf((float)dail_motor.KT_motor_info.rx_info.speed);
    launcher_dial_reject_e reason;

    if ((launcher_dial.pending_single != 0u) &&
        ((launcher_speed.source_ready == 0u) ||
         (launcher_speed.guard_latched != 0u) ||
         (launcher_speed_runtime.fault != LAUNCHER_SPEED_OK) ||
         (launcher_speed_runtime.reserved == 0u)))
    {
        Launcher_DialReject(LAUNCHER_DIAL_REJECT_SPEED);
        launcher_dial.pending_single = 0u;
        launcher_speed_runtime.reserved = 0u;
    }

    if (launcher_dial.hold_allowed == 0u)
    {
        launcher_speed_runtime.reserved = 0u;
        launcher_speed_runtime.press_single_started = 0u;
        launcher_dial.trigger_ready = 0u;
        reason = ((Board_HeartBeat.status != DEV_ONLINE) ||
                  (launcher.dial_online == 0u)) ?
                 LAUNCHER_DIAL_REJECT_OFFLINE : LAUNCHER_DIAL_REJECT_INTERLOCK;
        if (launcher_dial.pending_single != 0u)
        {
            Launcher_DialReject(reason);
            launcher_dial.pending_single = 0u;
        }
        if (single_rising != 0u)
        {
            Launcher_DialReject(reason);
        }
        launcher_dial.state = (launcher.fault != 0u) ?
                              LAUNCHER_FAULT : LAUNCHER_SLEEP;
        Launcher_DialSafeStop(now);
        return;
    }

    if (launcher_dial_target_synced == 0u)
    {
        launcher.dial_zero_angle = Launcher_DialAngle();
        Launcher_DialEnterHold(now, 1u);
        launcher_dial_target_synced = 1u;
    }

    if (launcher_dial.feed_allowed == 0u)
    {
        launcher_speed_runtime.reserved = 0u;
        if (launcher_dial.pending_single != 0u)
        {
            Launcher_DialReject(LAUNCHER_DIAL_REJECT_INTERLOCK);
            launcher_dial.pending_single = 0u;
            launcher_dial.trigger_ready = 0u;
        }
        if ((launcher_dial.state == LAUNCHER_SINGLE) ||
            (launcher_dial.state == LAUNCHER_REPEAT) ||
            (launcher_dial.state == LAUNCHER_REVERSE) ||
            (launcher_dial.state == LAUNCHER_RELOAD))
        {
            launcher_dial.reject_reason = LAUNCHER_DIAL_REJECT_INTERLOCK;
            Launcher_DialEnterBrake(now);
        }
    }
    else if ((launcher_heat.ready == 0u) ||
             (launcher_heat.heat > launcher_heat.heat_limit))
    {
        launcher_speed_runtime.reserved = 0u;
        if (launcher_dial.pending_single != 0u)
        {
            Launcher_DialReject(LAUNCHER_DIAL_REJECT_HEAT);
            launcher_dial.pending_single = 0u;
            launcher_dial.trigger_ready = 0u;
        }
        if ((launcher_dial.state == LAUNCHER_SINGLE) ||
            (launcher_dial.state == LAUNCHER_REVERSE) ||
            (launcher_dial.state == LAUNCHER_RELOAD))
        {
            launcher_dial.reject_reason = LAUNCHER_DIAL_REJECT_HEAT;
            Launcher_DialEnterBrake(now);
        }
    }

    if (single_rising != 0u)
    {
        if (launcher_dial.feed_allowed == 0u)
        {
            Launcher_DialReject(LAUNCHER_DIAL_REJECT_INTERLOCK);
        }
        else if ((launcher_dial.state != LAUNCHER_READY) ||
                 (launcher_dial.pending_single != 0u))
        {
            Launcher_DialReject(LAUNCHER_DIAL_REJECT_BUSY);
        }
        else if (launcher_dial.trigger_ready == 0u)
        {
            Launcher_DialReject(LAUNCHER_DIAL_REJECT_INTERLOCK);
        }
        else if (launcher_speed.feed_ready == 0u)
        {
            Launcher_DialReject(LAUNCHER_DIAL_REJECT_SPEED);
        }
        else if (launcher.fric_ready == 0u)
        {
            Launcher_DialReject(LAUNCHER_DIAL_REJECT_FRIC);
        }
        else if (Launcher_SpeedReserve() == 0u)
        {
            Launcher_DialReject(LAUNCHER_DIAL_REJECT_SPEED);
        }
        else if (Launcher_HeatReserveSingle() == 0u)
        {
            launcher_speed_runtime.reserved = 0u;
            Launcher_DialReject(LAUNCHER_DIAL_REJECT_HEAT);
        }
        else
        {
            launcher_dial.pending_single = 1u;
            launcher_dial.pending_tick = now;
            launcher_dial.accepted_count++;
            launcher_dial.reject_reason = LAUNCHER_DIAL_REJECT_NONE;
        }
    }

    if ((launcher_dial.pending_single != 0u) &&
        ((now - launcher_dial.pending_tick) >= LAUNCHER_DIAL_START_TIMEOUT_MS))
    {
        Launcher_DialReject(LAUNCHER_DIAL_REJECT_START);
        launcher_dial.start_timeout_count++;
        launcher_dial.pending_single = 0u;
        launcher_dial.trigger_ready = 0u;
        launcher_speed_runtime.reserved = 0u;
    }

    if (launcher_dial_stopped != 0u)
    {
        status = Launcher_DialRun();
        launcher_dial.run_tx_status = (uint8_t)status;
        if (status != HAL_OK)
        {
            return;
        }
        launcher_dial_stopped = 0u;
    }

    if (launcher_dial.pending_single != 0u)
    {
        launcher_dial.pending_single = 0u;
        launcher_dial_target += (int64_t)(LAUNCHER_DIAL_DIRECTION *
                                         LAUNCHER_DIAL_ONE_SHOT_ANGLE);
        launcher_dial_feed_target = launcher_dial_target;
        launcher_dial.state = LAUNCHER_SINGLE;
        launcher_dial.state_tick = now;
        launcher_dial.settling = 0u;
        launcher.jam_tick = 0u;
        Launcher_DialClearPid();
        Launcher_SpeedStart(now);
    }

    continuous = ((continuous != 0u) &&
                  ((launcher_dial.trigger_ready != 0u) ||
                   (launcher_keyboard_press_ready != 0u))) ? 1u : 0u;
    switch (launcher_dial.state)
    {
    case LAUNCHER_READY:
#if LAUNCHER_REPEAT_ENABLE
        if ((continuous != 0u) && (launcher.fric_ready != 0u))
        {
            launcher_dial.state = LAUNCHER_REPEAT;
            launcher_dial.state_tick = now;
            launcher.jam_tick = 0u;
            launcher_speed_runtime.repeat_tick = now;
            launcher_speed_runtime.repeat_ready_tick = now;
            launcher_speed_runtime.repeat_ready_seen = 1u;
            Launcher_SpeedClearSamples();
            Launcher_HeatStartRepeat();
            Launcher_DialClearPid();
        }
#endif
        break;

    case LAUNCHER_SINGLE:
#if LAUNCHER_REPEAT_ENABLE
        // NOTE: 等待达速不占单发计时
        if ((continuous != 0u) && (launcher_keyboard_press_ready != 0u) &&
            (launcher.fric_ready == 0u))
        {
            Launcher_DialEnterHold(now, 1u);
            break;
        }
        if ((continuous != 0u) && (launcher.fric_ready != 0u))
        {
            launcher_dial.state = LAUNCHER_REPEAT;
            launcher_dial.state_tick = now;
            launcher_dial.settling = 0u;
            launcher.jam_tick = 0u;
            launcher_speed_runtime.repeat_tick = now;
            launcher_speed_runtime.repeat_ready_tick = now;
            launcher_speed_runtime.repeat_ready_seen = 1u;
            Launcher_SpeedClearSamples();
            Launcher_HeatStartRepeat();
            Launcher_DialClearPid();
            break;
        }
#endif
        if ((Launcher_DialAtTarget(launcher_dial_target) != 0u) &&
            (speed <= (float)LAUNCHER_DIAL_SETTLE_SPEED_DPS))
        {
            if (launcher_dial.settling == 0u)
            {
                launcher_dial.settling = 1u;
                launcher_dial.settle_tick = now;
            }
            if ((now - launcher_dial.settle_tick) >= LAUNCHER_DIAL_SETTLE_TIME_MS)
            {
                launcher_dial.completed_count++;
                Launcher_DialEnterHold(now, 0u);
                break;
            }
        }
        else
        {
            launcher_dial.settling = 0u;
        }
        if ((now - launcher_dial.state_tick) >= LAUNCHER_DIAL_SINGLE_TIMEOUT_MS)
        {
            launcher_dial.timeout_count++;
            launcher_dial.reject_reason = LAUNCHER_DIAL_REJECT_TIMEOUT;
            Launcher_DialEnterBrake(now);
            break;
        }
#if LAUNCHER_DIAL_JAM_ENABLE
        if (Launcher_DialBlockCheck(
                (Launcher_DialAtTarget(launcher_dial_target) == 0u) ? 1u : 0u) != 0u)
        {
            Launcher_DialEnterStuckRecovery(0u);
        }
#endif
        break;

    case LAUNCHER_REPEAT:
        if (continuous == 0u)
        {
            if ((launcher_heat.ready == 0u) ||
                (launcher_heat.blocked != 0u) ||
                (launcher_heat.target_rate <= 0.0f))
            {
                launcher_dial.reject_reason = LAUNCHER_DIAL_REJECT_HEAT;
            }
            Launcher_DialEnterBrake(now);
            break;
        }
#if LAUNCHER_DIAL_JAM_ENABLE
        if (Launcher_DialBlockCheck(1u) != 0u)
        {
            Launcher_DialEnterStuckRecovery(1u);
        }
#endif
        break;

    case LAUNCHER_REVERSE:
        if ((Launcher_DialAtTarget(launcher_dial_target) != 0u) ||
            ((now - launcher_dial.state_tick) >= LAUNCHER_DIAL_REVERSE_TIMEOUT_MS))
        {
            launcher_dial_target = launcher_dial_feed_target;
            launcher_dial.state = LAUNCHER_RELOAD;
            launcher_dial.state_tick = now;
            Launcher_DialClearPid();
        }
        break;

    case LAUNCHER_RELOAD:
        if ((Launcher_DialAtTarget(launcher_dial_target) != 0u) ||
            ((now - launcher_dial.state_tick) >= LAUNCHER_DIAL_RELOAD_TIMEOUT_MS))
        {
            if ((launcher_dial_recovery_repeat != 0u) && (continuous != 0u))
            {
                launcher_dial.state = LAUNCHER_REPEAT;
                launcher_dial.state_tick = now;
                Launcher_HeatStartRepeat();
                Launcher_DialClearPid();
            }
            else
            {
                Launcher_DialEnterHold(now, 0u);
            }
        }
        break;

    case LAUNCHER_STOPPING:
        if ((speed <= (float)LAUNCHER_DIAL_BRAKE_STOP_SPEED_DPS) ||
            ((now - launcher_dial.state_tick) >= LAUNCHER_DIAL_BRAKE_TIMEOUT_MS))
        {
            Launcher_DialEnterHold(now, 1u);
        }
        break;

    default:
        Launcher_DialEnterBrake(now);
        break;
    }

    switch (launcher_dial.state)
    {
    case LAUNCHER_READY:
        Launcher_DialHoldControl();
        break;
    case LAUNCHER_REPEAT:
        Launcher_DialSpeedControl();
        break;
    case LAUNCHER_STOPPING:
        Launcher_DialBrakeControl();
        break;
    default:
        Launcher_DialPositionControl(launcher_dial_target);
        break;
    }
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
            float recovery_output = 0.0f;
            pid->target = direction * launcher.fric_target_rpm; /* 目标转速 */
            pid->measure = (float)rm_motor[i].rx_info->encoder_speed; /* 反馈转速 */
            pid->err = pid->target - pid->measure;

            if (launcher.state == LAUNCHER_STOPPING)
            {
                pid->integral = 0.0f;
            }

            single_pid_ctrl(pid);
            /* 连发掉速补偿保持总限幅 */
            if ((launcher_dial.state == LAUNCHER_REPEAT) &&
                (Board_Rx_Info.shoot_pkt.shoot_mode != 0u) &&
                (Board_Rx_Info.shoot_pkt.shoot_level != 0u) &&
                (launcher_speed.guard_latched == 0u) &&
                (launcher.state != LAUNCHER_STOPPING))
            {
                recovery_output = direction * constrain(
                    LAUNCHER_FRIC_REPEAT_BOOST_KP *
                    (direction * pid->err - LAUNCHER_FRIC_REPEAT_BOOST_DEAD_RPM),
                    0.0f, LAUNCHER_FRIC_REPEAT_BOOST_MAX);
            }
            rm_motor[i].tx_info->torque = /* 力矩限幅 */
                constrain(pid->out + recovery_output,
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
        if (i == SHOOT_FRIC_L)
        {
            launcher.fric_l_output_raw = rm_motor[i].tx_info->torque;
        }
        else
        {
            launcher.fric_r_output_raw = rm_motor[i].tx_info->torque;
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
        (launcher.fric_target_rpm == launcher_speed.target_rpm) &&
        (((float)rm_motor[SHOOT_FRIC_L].rx_info->encoder_speed *
          LAUNCHER_FRIC_L_DIRECTION) > 0.0f) &&
        (((float)rm_motor[SHOOT_FRIC_R].rx_info->encoder_speed *
          LAUNCHER_FRIC_R_DIRECTION) > 0.0f) &&
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

/* 初始化发射状态与控制环 */
void Launcher_Init(void)
{
    pid_ctrl_t *pid; /* 摩擦轮速度环临时指针 */

    memset(&launcher_heat, 0, sizeof(launcher_heat));
    memset(&launcher_heat_runtime, 0, sizeof(launcher_heat_runtime));
    memset(&launcher_dial, 0, sizeof(launcher_dial));
    memset(&launcher_speed, 0, sizeof(launcher_speed));
    memset(&launcher_speed_runtime, 0, sizeof(launcher_speed_runtime));
    launcher_keyboard_press_ready = 0u;
    launcher_speed.single_target_rpm = LAUNCHER_FRIC_TARGET_RPM;
    launcher_speed.repeat_target_rpm = LAUNCHER_FRIC_TARGET_RPM;
    launcher_speed.target_rpm = LAUNCHER_FRIC_TARGET_RPM;
    launcher_speed.block_reason = LAUNCHER_SPEED_LINK;
    launcher_dial.state = LAUNCHER_SLEEP;
    launcher_dial.run_tx_status = (uint8_t)HAL_ERROR;
    launcher_dial.torque_tx_status = (uint8_t)HAL_ERROR;
    launcher_dial.stop_tx_status = (uint8_t)HAL_ERROR;
    launcher_heat_runtime.update_tick = HAL_GetTick();
    launcher_heat.blocked = 1u;

    launcher.state = LAUNCHER_SLEEP;
    launcher.state_tick = 0u;
    launcher.last_repeat_tick = 0u;
    launcher.jam_tick = 0u;
    launcher.fric_target_rpm = 0.0f;
    launcher.fric_l_speed_rpm = 0.0f;
    launcher.fric_r_speed_rpm = 0.0f;
    launcher.fric_l_output_raw = 0.0f;
    launcher.fric_r_output_raw = 0.0f;
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
    launcher_dial_stopped = 1u;
    launcher_dial_stop_tick = 0u;
    launcher_dial_target_synced = 0u;
    launcher_dial_recovery_repeat = 0u;
    launcher_dial_last_is_hole = 0u;
    launcher_dial_hole_release_pending = 0u;
    launcher_dial_hole_release_tick = 0u;
    launcher_dial_hold_tx_tick = 0u;
    launcher_dial_hold_active_tick = 0u;
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

    launcher_dial_hold_angle_pid = launcher_dial_angle_pid;
    launcher_dial_hold_angle_pid.kp = LAUNCHER_DIAL_HOLD_ANGLE_KP;
    launcher_dial_hold_angle_pid.ki = 0.0f;
    launcher_dial_hold_angle_pid.kd = 0.0f;
    launcher_dial_hold_angle_pid.integral_max = 0.0f;
    launcher_dial_hold_angle_pid.deadband = LAUNCHER_DIAL_HOLD_DEADBAND;
    launcher_dial_hold_speed_pid = launcher_dial_speed_pid;
    launcher_dial_hold_speed_pid.kp = LAUNCHER_DIAL_HOLD_SPEED_KP;
    launcher_dial_hold_speed_pid.ki = LAUNCHER_DIAL_HOLD_SPEED_KI;
    launcher_dial_hold_speed_pid.kd = LAUNCHER_DIAL_HOLD_SPEED_KD;
}

static void Launcher_KeyboardPressUpdate(uint8_t fric_on)
{
    if ((Board_Rx_Info.remote_cmd_pkt.valid == 0u) ||
        (Board_Rx_Info.remote_cmd_pkt.ctrl_source != 1u) ||
        (Board_HeartBeat.offline_cnt_5 >= Board_HeartBeat.offline_cnt_max) ||
        (fric_on == 0u) || (launcher_dial.feed_allowed == 0u) ||
        (launcher_speed.source_ready == 0u) ||
        (launcher_speed_runtime.fault != LAUNCHER_SPEED_OK) ||
        (launcher_speed.guard_latched != 0u) ||
        (launcher_heat.ready == 0u) || (launcher_heat.blocked != 0u) ||
        (launcher_heat.target_rate <= 0.0f))
    {
        launcher_keyboard_press_ready = 0u;
    }
    else if ((Board_Rx_Info.shoot_pkt.shoot_level == 0u) &&
             ((Board_Rx_Info.remote_cmd_pkt.button_bits & 0x01u) == 0u))
    {
        launcher_keyboard_press_ready = 1u;
    }
}

/* 发射机构周期任务 */
void Launcher_Work(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t fric_on;
    uint8_t shoot_level = Board_Rx_Info.shoot_pkt.shoot_level;
    uint8_t shoot_mode = Board_Rx_Info.shoot_pkt.shoot_mode;
    uint8_t single_rising;
    uint8_t continuous;
    uint8_t vehicle_on;
    uint8_t hole_hold_allowed;
    uint8_t last_feed_allowed = launcher_dial.feed_allowed;
    int32_t current_angle = Launcher_DialAngle();

    launcher.dial_angle = current_angle;
    launcher.dial_online = Launcher_DialOnline();
    Launcher_HeatUpdate(now);
    hole_hold_allowed = Launcher_DialHoleHoldAllowed(now);

    vehicle_on = ((Board_HeartBeat.status == DEV_ONLINE) &&
                  (Board_Rx_Info.state_pkt.car_state != 0u)) ? 1u : 0u;
    if (launcher.state == LAUNCHER_FAULT)
    {
        launcher.fault = 1u;
    }
    fric_on = ((vehicle_on != 0u) && (launcher.fault == 0u) &&
               (Board_Rx_Info.shoot_pkt.launch_state != 0u) &&
               (Launcher_FricOnline(SHOOT_FRIC_L) != 0u) &&
               (Launcher_FricOnline(SHOOT_FRIC_R) != 0u)) ? 1u : 0u;

    Launcher_SpeedUpdate(now, vehicle_on,
                         Board_Rx_Info.shoot_pkt.launch_state, shoot_mode);

#if LAUNCHER_DIAL_ENABLE
    launcher_dial.hold_allowed = ((vehicle_on != 0u) &&
        (launcher.dial_online != 0u) &&
        (dail_motor.KT_motor_info.rx_info.encoder_sum_ready != 0u) &&
        (hole_hold_allowed != 0u) &&
        (launcher.fault == 0u)) ? 1u : 0u;
#else
    launcher_dial.hold_allowed = 0u;
    shoot_level = 0u;
    shoot_mode = 0u;
#endif
    launcher_dial.feed_allowed = ((launcher_dial.hold_allowed != 0u) &&
        (fric_on != 0u) && (Board_Rx_Info.shoot_pkt.is_hole == 0u)) ? 1u : 0u;
    Launcher_KeyboardPressUpdate(fric_on);
    if ((launcher_dial.hold_allowed == 0u) ||
        ((last_feed_allowed != 0u) && (launcher_dial.feed_allowed == 0u)))
    {
        launcher_speed_runtime.press_single_started = 0u;
        launcher_dial.trigger_ready = 0u;
    }
    else if (shoot_level == 0u)
    {
        launcher_speed_runtime.press_single_started = 0u;
        launcher_dial.trigger_ready = 1u;
    }
    single_rising = ((shoot_level != 0u) &&
                     (launcher.last_shoot_level == 0u) &&
                     (shoot_mode == 0u)) ? 1u : 0u;
    launcher.last_shoot_level = shoot_level;
    continuous = ((launcher_dial.feed_allowed != 0u) &&
                  (((Board_Rx_Info.remote_cmd_pkt.ctrl_source == 1u) ?
                    launcher_keyboard_press_ready : launcher_dial.trigger_ready) != 0u) &&
                  (shoot_level != 0u) && (shoot_mode != 0u) &&
                  (launcher_speed.feed_ready != 0u) &&
                  (launcher_heat.ready != 0u) &&
                  (launcher_heat.blocked == 0u) &&
                  (launcher_heat.target_rate > 0.0f)) ? 1u : 0u;

    /* 摩擦轮停机不覆盖拨盘阶段 */
    if (launcher.fault != 0u)
    {
        launcher.enabled = 0u;
        launcher.state = LAUNCHER_FAULT;
        launcher.fric_target_rpm = 0.0f;
        launcher_fric_stop_count = 0u;
        Launcher_UpdateFrictionReady(0u);
        Launcher_FricControl(0u);
    }
    else if (fric_on == 0u)
    {
        launcher.enabled = 0u;
        Launcher_UpdateFrictionReady(0u);
        if (launcher.state == LAUNCHER_SLEEP)
        {
            launcher.fric_target_rpm = 0.0f;
            launcher_fric_stop_count = 0u;
            Launcher_FricControl(0u);
        }
        else
        {
            launcher.state = LAUNCHER_STOPPING;
            launcher.fric_target_rpm = Launcher_Ramp(
                launcher.fric_target_rpm, 0.0f,
                LAUNCHER_FRIC_STOP_RAMP_RPM_PER_MS);
            Launcher_FricControl(1u);
            if ((launcher.fric_l_speed_rpm <= LAUNCHER_FRIC_STOP_SPEED_RPM) &&
                (launcher.fric_r_speed_rpm <= LAUNCHER_FRIC_STOP_SPEED_RPM))
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
        }
    }
    else
    {
        launcher.enabled = 1u;
        launcher_fric_stop_count = 0u;
        launcher.state = LAUNCHER_READY;
        launcher.fric_target_rpm = Launcher_Ramp(
            launcher.fric_target_rpm, launcher_speed.target_rpm,
            LAUNCHER_FRIC_RAMP_RPM_PER_MS);
        Launcher_UpdateFrictionReady(1u);
        Launcher_FricControl(1u);
    }

    if ((launcher_dial.state == LAUNCHER_READY) && (launcher.fric_ready == 0u))
    {
        continuous = 0u;
    }
    Launcher_DialUpdate(now, single_rising, continuous);
    launcher_dial.target_angle = launcher_dial_target;
    launcher_dial.target_error = launcher_dial_target - (int64_t)current_angle;
    launcher.dial_target_angle = (int32_t)launcher_dial_target;
    if (fric_on != 0u)
    {
        launcher.state = ((launcher_dial.state == LAUNCHER_STOPPING) ||
                          (launcher_dial.state == LAUNCHER_SLEEP)) ?
                         LAUNCHER_READY : launcher_dial.state;
        launcher.state_tick = launcher_dial.state_tick;
    }
}
