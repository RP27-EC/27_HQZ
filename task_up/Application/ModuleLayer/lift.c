/* lift.c - 狗洞升降控制 */

#include "lift.h"

#include "communicate.h"
#include "gimbal.h"
#include "lift_config.h"
#include "main.h"
#include <math.h>

lift_t lift;
lift_tune_t lift_tune;
volatile lift_debug_t lift_debug;
static uint32_t lift_control_dt_ms; // 本次控制间隔，0~100ms

#define LIFT_FAULT_HOME_TIMEOUT   1u
#define LIFT_FAULT_MOVE_TIMEOUT   2u
#define LIFT_FAULT_OVERTRAVEL     3u
#define LIFT_FAULT_STALL_CURRENT  4u
#define LIFT_FAULT_STALL_PROGRESS 5u
#define LIFT_FAULT_HOLD_CURRENT   6u // 保持过载，值6
#define LIFT_FAULT_FEEDBACK       7u // 反馈异常，值7

static float lift_abs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float lift_encoder_units(void)
{
    return (float)lift.motor->rx_info->encoder_sum /
           LIFT_POSITION_COUNTS_PER_UNIT;
}

static float lift_rpm_to_rad_s(float rpm)
{
    return rpm * 0.1047197551f;
}

static float lift_clamp(float value, float min_value, float max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

static float lift_wrap_deg(float angle)
{
    while (angle >= 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

static uint32_t lift_delta_ms(uint32_t now, uint32_t last)
{
    uint32_t delta = now - last;
    return (delta > 100u) ? 100u : delta;
}

static void lift_pid_clear(pid_ctrl_t *pid)
{
    integral_to_zero(pid);
    pid->measure = 0.0f;
    pid->err = 0.0f;
    pid->pout = 0.0f;
    pid->iout = 0.0f;
    pid->dout = 0.0f;
    pid->last_dout = 0.0f;
    pid->last_err = 0.0f;
}

static void lift_enter_state(lift_state_e state, uint32_t now)
{
    float hold_i_raw = lift.speed_pid.iout;
    lift.state = state;
    if ((state != LIFT_FAULT) && (state != LIFT_PAUSED) &&
        (state != LIFT_STALL_STOP) && (state != LIFT_LOAD_HOLD)) lift.active_fault_code = 0u;
    lift.state_enter_tick = now;
    lift.homing_confirm_ms = 0u;
    lift.stable_confirm_ms = 0u;
    lift.overcurrent_confirm_ms = 0u;
    lift.stall_progress_tick = now;
    lift.stall_progress_count = lift.motor->rx_info->encoder_sum;
    lift.speed_target_rpm = 0.0f;
    lift.overtravel_ms = 0u;
    lift.hold_current_ms = 0u;
    lift_pid_clear(&lift.position_pid);
    lift_pid_clear(&lift.speed_pid);
    if ((state == LIFT_WAIT) || (state == LIFT_FAULT) ||
        (state == LIFT_PAUSED) || (state == LIFT_STALL_STOP))
    {
        lift.limit_recovery = 0u;
        lift.hold_overload = 0u;
    }
    if ((state == LIFT_READY_UP) || (state == LIFT_READY_DOWN) ||
        (state == LIFT_ALIGN_DOWN) || (state == LIFT_BOTTOM_RECOVER) ||
        (state == LIFT_LOAD_HOLD) || (state == LIFT_START_UP) || (state == LIFT_MOVING_UP))
    {
        lift.speed_pid.integral = lift_clamp(hold_i_raw, -LIFT_HOLD_I_MAX_RAW,
            LIFT_HOLD_I_MAX_RAW) / LIFT_HOLD_SPEED_KI;
    }

    if (state == LIFT_START_UP)
    {
        lift.start_count = lift.motor->rx_info->encoder_sum;
        lift.start_direction = ((float)lift.top_target >= lift_encoder_units()) ? 1.0f : -1.0f;
        lift.start_progress_counts = 0.0f;
        lift.start_confirm_ms = 0u;
    }

    if (state == LIFT_HOMING_UP)
    {
        lift.home_start_count = lift.motor->rx_info->encoder_sum;
        lift.home_window_count = lift.motor->rx_info->encoder_sum;
        lift.home_window_tick = now;
        lift.home_stationary_ok = 0u;
        lift.home_speed_ok = 0u;
        lift.home_current_ok = 0u;
    }
}

static void lift_output_speed(float target_rpm, float output_limit)
{
    float output;
    float integral_before;
    float raw_output;
    uint8_t terminal = ((lift.state == LIFT_READY_UP) || (lift.state == LIFT_READY_DOWN) ||
        (lift.state == LIFT_BOTTOM_RECOVER) || (lift.state == LIFT_LOAD_HOLD) ||
        (lift.state == LIFT_START_UP) || (lift.limit_recovery != 0u) ||
        (lift.state == LIFT_ALIGN_DOWN) || (lift.state == LIFT_RETRACT_DOWN) ||
        ((lift.state == LIFT_MOVING_UP) &&
         (lift_abs((float)lift.top_target - lift_encoder_units()) <= LIFT_BRAKE_COUNTS)) ||
        ((lift.state == LIFT_MOVING_DOWN) &&
         (lift_abs((float)lift.bottom_target - lift_encoder_units()) <= LIFT_BRAKE_COUNTS)));

    // NOTE: 末端积分承担静载
    lift.speed_pid.ki = terminal ? LIFT_HOLD_SPEED_KI : LIFT_SPEED_KI;
    lift.speed_pid.integral_max = terminal ?
        LIFT_HOLD_I_MAX_RAW / LIFT_HOLD_SPEED_KI : LIFT_SPEED_INTEGRAL_MAX;
    lift.speed_pid.out_max = fminf(lift_tune.speed_out_max_raw, lift_abs(output_limit));
    if (lift.limit_recovery != 0u)
    {
        float recovery_limit = (lift.state == LIFT_START_UP) ?
            LIFT_START_OUTPUT_MAX_RAW : LIFT_HOLD_OUTPUT_MAX_RAW;
        output_limit = fminf(lift_abs(output_limit), recovery_limit);
        lift.speed_pid.out_max = fminf(lift.speed_pid.out_max, output_limit);
        target_rpm = lift_clamp(target_rpm, -LIFT_HOLD_SPEED_MAX_RPM,
                               LIFT_HOLD_SPEED_MAX_RPM);
    }
    if (((lift.limit_recovery == 1u) || (lift.state == LIFT_BOTTOM_RECOVER)) &&
        (target_rpm * LIFT_UP_DIRECTION < 0.0f)) target_rpm = 0.0f;
    if ((lift.limit_recovery == 2u) &&
        (target_rpm * LIFT_UP_DIRECTION > 0.0f)) target_rpm = 0.0f;

    if ((lift.home_valid != 0u) && (lift.state != LIFT_HOMING_UP))
    {
        float position = lift_encoder_units();
        if (((position - (float)lift.top_zero) * LIFT_UP_DIRECTION >= 0.0f &&
             target_rpm * LIFT_UP_DIRECTION > 0.0f) ||
            ((position - (float)lift.bottom_target) * LIFT_UP_DIRECTION <= -LIFT_POS_TOL_COUNTS &&
             target_rpm * LIFT_UP_DIRECTION < 0.0f))
        {
            target_rpm = 0.0f;
        }
    }

    lift.speed_pid.target = lift_rpm_to_rad_s(target_rpm);
    lift.speed_pid.measure = lift_rpm_to_rad_s(
        (float)lift.motor->rx_info->encoder_speed *
        lift_tune.speed_direction_sign);
    lift.speed_pid.err = lift.speed_pid.target - lift.speed_pid.measure;
    integral_before = lift_clamp(lift.speed_pid.integral,
                                -lift.speed_pid.integral_max, lift.speed_pid.integral_max);
    lift.speed_pid.integral = integral_before;
    single_pid_ctrl(&lift.speed_pid);
    raw_output = lift.speed_pid.pout + lift.speed_pid.iout + lift.speed_pid.dout;
    if ((terminal != 0u) && (lift_abs(raw_output) >= lift.speed_pid.out_max) &&
        (raw_output * lift.speed_pid.err > 0.0f))
    {
        // NOTE: 当拍饱和不累加积分
        lift.speed_pid.integral = integral_before;
        lift.speed_pid.iout = lift.speed_pid.ki * integral_before;
        lift.speed_pid.out = lift_clamp(lift.speed_pid.pout + lift.speed_pid.iout +
            lift.speed_pid.dout, -lift.speed_pid.out_max, lift.speed_pid.out_max);
    }

    output = lift.speed_pid.out * LIFT_OUTPUT_DIRECTION;
    // NOTE: 越程只允许向内施力
    if ((((lift.limit_recovery == 1u) || (lift.state == LIFT_BOTTOM_RECOVER)) &&
         (output * LIFT_OUTPUT_DIRECTION * LIFT_UP_DIRECTION < 0.0f)) ||
        ((lift.limit_recovery == 2u) &&
         (output * LIFT_OUTPUT_DIRECTION * LIFT_UP_DIRECTION > 0.0f)))
    {
        output = 0.0f;
        integral_to_zero(&lift.speed_pid);
    }
    if ((lift.home_valid != 0u) && (lift.state != LIFT_HOMING_UP))
    {
        float position = lift_encoder_units();
        if (((position - (float)lift.top_zero) * LIFT_UP_DIRECTION >= 0.0f &&
             output * LIFT_OUTPUT_DIRECTION * LIFT_UP_DIRECTION > 0.0f) ||
            ((position - (float)lift.bottom_target) * LIFT_UP_DIRECTION <= -LIFT_POS_TOL_COUNTS &&
             output * LIFT_OUTPUT_DIRECTION * LIFT_UP_DIRECTION < 0.0f))
        {
            output = 0.0f;
            integral_to_zero(&lift.speed_pid);
        }
    }
    lift.motor->tx_info->torque =
        lift_clamp(output, -output_limit, output_limit);
}

static void lift_output_position_limited(int32_t target_count,
                                         float speed_limit,
                                         float output_limit)
{
    float speed_target;

    lift.position_pid.kp = (lift.state == LIFT_RETRACT_DOWN) ? LIFT_TERMINAL_KP :
        (((lift.state == LIFT_READY_DOWN) || (lift.state == LIFT_BOTTOM_RECOVER) ||
          ((lift.state == LIFT_LOAD_HOLD) && (target_count == lift.bottom_target))) ?
         LIFT_BOTTOM_HOLD_KP : lift_tune.pos_kp);
    lift.position_pid.target = (float)target_count;
    lift.position_pid.measure = lift_encoder_units();
    lift.position_pid.err = lift.position_pid.target - lift.position_pid.measure;
    single_pid_ctrl(&lift.position_pid);

    speed_target = lift_clamp(lift.position_pid.out,
                              -lift_abs(speed_limit),
                              lift_abs(speed_limit));
    if ((lift.state == LIFT_READY_UP) || (lift.state == LIFT_READY_DOWN) ||
        (lift.state == LIFT_ALIGN_DOWN) || (lift.state == LIFT_BOTTOM_RECOVER) ||
        (lift.state == LIFT_LOAD_HOLD))
    {
        float step = LIFT_HOLD_RATE_STEP_RPM_PER_MS * (float)lift_control_dt_ms;
        // NOTE: 保持纠偏避免速度阶跃
        lift.speed_target_rpm += lift_clamp(speed_target - lift.speed_target_rpm, -step, step);
        lift.speed_target_rpm = lift_clamp(lift.speed_target_rpm,
                                         -lift_abs(speed_limit), lift_abs(speed_limit));
        speed_target = lift.speed_target_rpm;
    }
    lift_output_speed(speed_target, lift_abs(output_limit));
}

static void lift_output_position(int32_t target_count)
{
    lift_output_position_limited(target_count,
                                 LIFT_HOLD_SPEED_MAX_RPM,
                                 LIFT_HOLD_OUTPUT_MAX_RAW);
}

static uint8_t lift_alignment_ok(void)
{
    float yaw_error;

    if ((Gimbal.gimbal_mode != G_MEC) || (Gimbal.init_info.init_flag == 0u))
    {
        return 0u;
    }

    // NOTE: 实际回正后才下降
    if (lift_abs(lift_wrap_deg(Gimbal.base_info.yaw_mec_angle)) >
        LIFT_FRONT_TOL_DEG)
    {
        return 0u;
    }

    yaw_error = lift_abs(lift_wrap_deg(Gimbal.base_info.yaw_mec_angle -
                                       Gimbal.pid_info.yaw_target));
    if (yaw_error > LIFT_ALIGN_TOL_DEG)
    {
        return 0u;
    }

    if (lift_abs(Gimbal.base_info.yaw_mec_speed) > LIFT_ALIGN_SPEED_RAD_S)
    {
        return 0u;
    }

    return 1u;
}

static int32_t lift_current_target(void)
{
    if (lift.state == LIFT_LOAD_HOLD) return lift.load_hold_target;
    return ((lift.state == LIFT_MOVING_DOWN) || (lift.state == LIFT_READY_DOWN) ||
            (lift.state == LIFT_BOTTOM_RECOVER)) ? lift.bottom_target : lift.top_target;
}

static void lift_update_recovery_direction(void)
{
    float travel = ((float)lift.top_zero - lift_encoder_units()) * LIFT_UP_DIRECTION;
    lift.limit_recovery = (travel > LIFT_TRAVEL_COUNTS) ? 1u :
                          ((travel < 0.0f) ? 2u : 0u);
}

static void lift_record_protection(uint8_t code, uint32_t now)
{
    lift.fault_code = code;
    lift.active_fault_code = code;
    lift.fault_state = lift.state;
    lift.fault_position = (int32_t)lift_encoder_units();
    lift.fault_target = lift_current_target();
    lift.fault_tick = now;
    lift.fault_output_raw = lift.motor->tx_info->torque;
    lift.fault_current_raw = lift.motor->rx_info->torque_current_raw;
    lift.fault_feedback_age_ms = now - lift.motor->rx_info->feedback_tick;
    lift.fault_count++;
}

static uint8_t lift_can_hold(uint32_t now)
{
    return ((lift.home_valid != 0u) && (lift.motor->state->status == DEV_ONLINE) &&
        (lift.motor->rx_info->feedback_seq != 0u) &&
        ((now - lift.motor->rx_info->feedback_tick) <= LIFT_FEEDBACK_TIMEOUT_MS) &&
        (Board_HeartBeat.status == DEV_ONLINE) &&
        (Board_HeartBeat.offline_cnt_1 < Board_HeartBeat.offline_cnt_max) &&
        (Board_Rx_Info.state_pkt.car_state != 0u)) ? 1u : 0u;
}

static void lift_load_hold(uint8_t code, uint32_t now)
{
    int32_t target = lift_current_target();
    float previous_rate = lift.speed_target_rpm;
    lift_record_protection(code, now);
    lift.load_hold_target = target;
    lift.recover_request = 0u;
    lift.recover_force_up = 0u;
    lift_enter_state(LIFT_LOAD_HOLD, now);
    lift_update_recovery_direction();
    // NOTE: 同向交接保留支撑速度
    if (previous_rate * ((float)target - lift_encoder_units()) > 0.0f)
        lift.speed_target_rpm = lift_clamp(previous_rate, -LIFT_HOLD_SPEED_MAX_RPM,
                                         LIFT_HOLD_SPEED_MAX_RPM);
    lift.pitch_zero_hold = 0u;
    lift_output_position(target);
}

static void lift_fault(uint8_t code)
{
    uint32_t now = HAL_GetTick();
    if ((code >= LIFT_FAULT_MOVE_TIMEOUT) && (code <= LIFT_FAULT_HOLD_CURRENT) &&
        (lift_can_hold(now) != 0u))
    {
        lift_load_hold(code, now);
        return;
    }
    lift_record_protection(code, now);
    lift.motor->tx_info->torque = 0.0f;
    lift.pitch_zero_hold = 0u;
    lift_enter_state(LIFT_FAULT, now);
}

static void lift_pause(uint8_t code, uint32_t now)
{
    if (lift_can_hold(now) != 0u)
    {
        lift_load_hold(code, now);
        return;
    }
    lift_record_protection(code, now);
    lift.motor->tx_info->torque = 0.0f;
    lift.pitch_zero_hold = 0u;
    lift_enter_state(LIFT_PAUSED, now);
}

static uint8_t lift_move_stalled(uint32_t now, uint32_t dt, uint8_t moving_up)
{
    float threshold = (moving_up != 0u) ? lift_tune.over_current_up_raw :
                      lift_tune.over_current_down_raw;
    uint32_t confirm = (uint32_t)((moving_up != 0u) ?
        lift_tune.up_over_current_confirm_ms : lift_tune.down_over_current_confirm_ms);
    uint8_t driven = (lift_abs(lift.motor->tx_info->torque) >= LIFT_STALL_OUTPUT_MIN_RAW);
    uint8_t low_speed = (lift_abs((float)lift.motor->rx_info->encoder_speed) <= LIFT_SPEED_TOL_RPM);
    uint8_t high_current = (lift_abs((float)lift.motor->rx_info->torque_current_raw) >= threshold);

    if (((now - lift.state_enter_tick) < LIFT_STALL_START_GRACE_MS) || (driven == 0u))
    {
        lift.overcurrent_confirm_ms = 0u;
        lift.stall_progress_tick = now;
        lift.stall_progress_count = lift.motor->rx_info->encoder_sum;
        return 0u;
    }
    if ((high_current != 0u) && (low_speed != 0u)) lift.overcurrent_confirm_ms += dt;
    else lift.overcurrent_confirm_ms = 0u;
    if ((now - lift.stall_progress_tick) >= confirm)
    {
        float direction = ((moving_up != 0u) ? 1.0f : -1.0f) * LIFT_UP_DIRECTION;
        float progress;
        if (lift.state == LIFT_START_UP) direction = lift.start_direction;
        progress = (float)((int64_t)lift.motor->rx_info->encoder_sum -
                          lift.stall_progress_count) * direction / LIFT_POSITION_COUNTS_PER_UNIT;
        lift.stall_progress_counts = progress;
        lift.stall_progress_tick = now;
        lift.stall_progress_count = lift.motor->rx_info->encoder_sum;
        if ((progress < lift_tune.stall_progress_counts) && (low_speed != 0u))
        {
            return (lift.overcurrent_confirm_ms >= confirm) ? 1u : 2u;
        }
        lift.overcurrent_confirm_ms = 0u;
    }
    return 0u;
}

static void lift_homing_up_update(uint32_t now, uint32_t dt)
{
    int32_t count = lift.motor->rx_info->encoder_sum;
    int32_t count_units = (int32_t)lift_encoder_units();
    float target_speed = LIFT_UP_DIRECTION * lift_tune.home_speed_rpm;

    if ((now - lift.state_enter_tick) >= LIFT_HOME_TIMEOUT_MS)
    {
        lift_fault(LIFT_FAULT_HOME_TIMEOUT);
        return;
    }

    lift.home_current_ok =
        (lift_abs((float)lift.motor->rx_info->torque_current_raw) >=
         lift_tune.home_current_raw) ? 1u : 0u;
    lift.home_speed_ok =
        (lift_abs((float)lift.motor->rx_info->encoder_speed) <=
         lift_tune.home_stall_speed_rpm) ? 1u : 0u;

    if ((now - lift.state_enter_tick) >=
        (uint32_t)lift_tune.home_speed_grace_ms)
    {
        if ((now - lift.home_window_tick) >=
            LIFT_HOME_STATIONARY_WINDOW_MS)
        {
            lift.home_stationary_ok =
                (lift_abs((float)(count - lift.home_window_count)) <=
                 LIFT_HOME_STATIONARY_COUNTS) ? 1u : 0u;
            lift.home_window_count = count;
            lift.home_window_tick = now;
        }
    }
    else
    {
        lift.home_stationary_ok = 0u;
        lift.home_window_count = count;
        lift.home_window_tick = now;
    }

    if (((now - lift.state_enter_tick) >=
         (uint32_t)lift_tune.home_speed_grace_ms) &&
        (lift.home_current_ok != 0u) &&
        ((lift.home_stationary_ok != 0u) ||
         (lift.home_speed_ok != 0u)))
    {
        lift.homing_confirm_ms += dt;
    }
    else
    {
        lift.homing_confirm_ms = 0u;
    }

    if (lift.homing_confirm_ms >= (uint32_t)lift_tune.home_confirm_ms)
    {
        lift.top_zero = count_units;
        lift.top_target = lift.top_zero -
                          (int32_t)(LIFT_UP_DIRECTION *
                                    lift_tune.retract_counts);
        lift.bottom_target = lift.top_zero -
                             (int32_t)(LIFT_UP_DIRECTION *
                                       LIFT_TRAVEL_COUNTS);
        lift.home_valid = 1u;
        lift_enter_state(LIFT_RETRACT_DOWN, now);
        lift.motor->tx_info->torque = 0.0f;
        return;
    }

    lift_output_speed(target_speed, lift_tune.home_output_limit_raw);
}

static void lift_retract_update(uint32_t now, uint32_t dt)
{
    float position_error = (float)lift.top_target - lift_encoder_units();
    float speed = (float)lift.motor->rx_info->encoder_speed;

    if ((now - lift.state_enter_tick) >= LIFT_MOVE_TIMEOUT_MS)
    {
        lift_fault(LIFT_FAULT_MOVE_TIMEOUT);
        return;
    }

    if ((lift_abs(position_error) <= LIFT_POS_TOL_COUNTS) &&
        (lift_abs(speed) <= LIFT_SPEED_TOL_RPM))
    {
        lift.stable_confirm_ms += dt;
    }
    else
    {
        lift.stable_confirm_ms = 0u;
    }

    lift_output_position_limited(lift.top_target,
                                 lift_tune.retract_speed_rpm,
                                 lift_tune.retract_output_limit_raw);

    if (lift.stable_confirm_ms >= LIFT_STABLE_CONFIRM_MS)
    {
        lift.home_valid = 1u;
        lift_enter_state(LIFT_READY_UP, now);
    }
}

static void lift_move_update(uint32_t now, uint32_t dt, int32_t target,
                            uint8_t moving_up)
{
    float error = (float)target - lift_encoder_units();
    float remaining = lift_abs(error);
    float speed = (float)lift.motor->rx_info->encoder_speed;
    float speed_cap;
    float desired;
    float step;
    uint8_t stall;

    if ((now - lift.state_enter_tick) >= LIFT_MOVE_TIMEOUT_MS)
    {
        lift_pause(LIFT_FAULT_MOVE_TIMEOUT, now);
        return;
    }
    if (remaining <= LIFT_POS_TOL_COUNTS)
    {
        if (lift_abs(speed) <= LIFT_SPEED_TOL_RPM) lift.stable_confirm_ms += dt;
        else lift.stable_confirm_ms = 0u;
        lift_output_position(target);
        if (lift.stable_confirm_ms >= LIFT_STABLE_CONFIRM_MS)
        {
            lift_enter_state(moving_up ? LIFT_READY_UP : LIFT_READY_DOWN, now);
        }
        return;
    }
    lift.stable_confirm_ms = 0u;
    stall = lift_move_stalled(now, dt, moving_up);
    if (stall != 0u)
    {
        lift_pause((stall == 1u) ? LIFT_FAULT_STALL_CURRENT : LIFT_FAULT_STALL_PROGRESS, now);
        return;
    }
    // NOTE: 末段按制动距离限速
    speed_cap = lift_tune.move_speed_rpm * sqrtf(fminf(remaining / LIFT_BRAKE_COUNTS, 1.0f));
    speed_cap = fminf(speed_cap, lift_tune.pos_out_max_rpm);
    desired = lift_clamp(LIFT_TERMINAL_KP * error, -speed_cap, speed_cap);
    step = lift_tune.move_speed_rpm * (float)dt / LIFT_ACCEL_TIME_MS;
    lift.speed_target_rpm += lift_clamp(desired - lift.speed_target_rpm, -step, step);
    lift.speed_target_rpm = lift_clamp(lift.speed_target_rpm, -speed_cap, speed_cap);
    lift_output_speed(lift.speed_target_rpm, lift_tune.speed_out_max_raw);
}

static void lift_begin_up(uint32_t now)
{
    if (lift.state == LIFT_START_UP) return;
    lift.control_is_hole = 0u;
    lift.pending_valid = 0u;
    lift.recover_request = 0u;
    lift.recover_force_up = 0u;
    lift.restart_required = 0u;
    lift_update_recovery_direction();
    if (lift_abs((float)lift.top_target - lift_encoder_units()) <= LIFT_POS_TOL_COUNTS)
        lift_enter_state(LIFT_MOVING_UP, now);
    else
        lift_enter_state(LIFT_START_UP, now);
}

static void lift_start_up_update(uint32_t now, uint32_t dt)
{
    float error = (float)lift.top_target - lift_encoder_units();
    float step = LIFT_HOLD_RATE_STEP_RPM_PER_MS * (float)dt;
    float desired;
    float speed_along = (float)lift.motor->rx_info->encoder_speed *
        lift_tune.speed_direction_sign * lift.start_direction;
    uint8_t stall;

    if ((now - lift.state_enter_tick) >= LIFT_MOVE_TIMEOUT_MS)
    {
        lift_pause(LIFT_FAULT_MOVE_TIMEOUT, now);
        return;
    }
    if (lift_abs(error) <= LIFT_POS_TOL_COUNTS)
    {
        lift_enter_state(LIFT_MOVING_UP, now);
        lift_move_update(now, dt, lift.top_target, 1u);
        return;
    }
    stall = lift_move_stalled(now, dt, 1u);
    if (stall != 0u)
    {
        lift_pause((stall == 1u) ? LIFT_FAULT_STALL_CURRENT : LIFT_FAULT_STALL_PROGRESS, now);
        return;
    }
    desired = lift_clamp(LIFT_TERMINAL_KP * error, -LIFT_START_SPEED_MAX_RPM,
                         LIFT_START_SPEED_MAX_RPM);
    lift.speed_target_rpm += lift_clamp(desired - lift.speed_target_rpm, -step, step);
    lift_output_speed(lift.speed_target_rpm, LIFT_START_OUTPUT_MAX_RAW);
    lift.start_progress_counts = (float)((int64_t)lift.motor->rx_info->encoder_sum -
        lift.start_count) * lift.start_direction / LIFT_POSITION_COUNTS_PER_UNIT;
    if ((lift.start_progress_counts >= LIFT_START_PROGRESS_COUNTS) &&
        (speed_along > LIFT_SPEED_TOL_RPM)) lift.start_confirm_ms += dt;
    else lift.start_confirm_ms = 0u;
    if ((lift.start_confirm_ms >= LIFT_START_CONFIRM_MS) && (lift.limit_recovery == 0u))
    {
        float accepted_rate = lift.speed_target_rpm;
        lift_enter_state(LIFT_MOVING_UP, now);
        lift.speed_target_rpm = accepted_rate;
    }
}

static uint8_t lift_check_limits(uint32_t now, uint32_t dt)
{
    float travel = ((float)lift.top_zero - lift_encoder_units()) * LIFT_UP_DIRECTION;
    uint8_t inward_recovery;

    // NOTE: 底部受压保持原目标
    if ((lift.state == LIFT_READY_DOWN) &&
        (travel > LIFT_TRAVEL_COUNTS + LIFT_POS_TOL_COUNTS))
    {
        lift_enter_state(LIFT_BOTTOM_RECOVER, now);
        lift.limit_recovery = 1u;
        lift.motor->tx_info->torque = 0.0f;
    }
    inward_recovery =
        (((lift.limit_recovery == 1u) && (travel > LIFT_TRAVEL_COUNTS)) ||
         ((lift.limit_recovery == 2u) && (travel < 0.0f))) &&
        ((lift.state == LIFT_MOVING_UP) || (lift.state == LIFT_MOVING_DOWN) ||
         (lift.state == LIFT_ALIGN_DOWN) || (lift.state == LIFT_BOTTOM_RECOVER) ||
         (lift.state == LIFT_START_UP));
    if ((travel >= 0.0f) && (travel <= LIFT_TRAVEL_COUNTS))
    {
        lift.limit_recovery = 0u;
    }
    if ((inward_recovery != 0u) &&
        ((now - lift.state_enter_tick) >= LIFT_MOVE_TIMEOUT_MS))
    {
        lift_pause(LIFT_FAULT_MOVE_TIMEOUT, now);
        return 1u;
    }
    if ((travel < -LIFT_OVERTRAVEL_COUNTS) ||
        (travel > LIFT_TRAVEL_COUNTS + LIFT_OVERTRAVEL_COUNTS))
    {
        lift.overtravel_ms = (inward_recovery != 0u) ? 0u : lift.overtravel_ms + dt;
        if (lift.overtravel_ms >= LIFT_OVERTRAVEL_CONFIRM_MS)
        {
            lift_fault(LIFT_FAULT_OVERTRAVEL);
            return 1u;
        }
    }
    else lift.overtravel_ms = 0u;
    if (((lift.state == LIFT_READY_UP) || (lift.state == LIFT_READY_DOWN) ||
         (lift.state == LIFT_BOTTOM_RECOVER)) &&
        (lift_abs((float)lift.motor->rx_info->torque_current_raw) >= LIFT_HOLD_CURRENT_RAW))
    {
        lift.hold_current_ms += dt;
        if (lift.hold_current_ms >= LIFT_HOLD_CONFIRM_MS)
        {
            lift.hold_current_ms = LIFT_HOLD_CONFIRM_MS;
            if ((lift.state == LIFT_READY_DOWN) || (lift.state == LIFT_BOTTOM_RECOVER))
            {
                // NOTE: 底部受载继续限流保持
                lift.hold_overload = 1u;
            }
            else
            {
                lift_pause(LIFT_FAULT_HOLD_CURRENT, now);
                return 1u;
            }
        }
    }
    else
    {
        lift.hold_current_ms = 0u;
        lift.hold_overload = 0u;
    }
    return 0u;
}

static void lift_debug_record(int32_t count)
{
    if (lift_debug.stats_valid == 0u)
    {
        lift_debug.min_count = count;
        lift_debug.max_count = count;
        lift_debug.stats_valid = 1u;
    }
    else
    {
        if (count < lift_debug.min_count) lift_debug.min_count = count;
        if (count > lift_debug.max_count) lift_debug.max_count = count;
    }
    lift_debug.travel_counts = lift_debug.max_count - lift_debug.min_count;
}

static void lift_debug_update(uint32_t now, uint32_t dt)
{
    int32_t count;
    float output;
    float over_current;
    float over_current_confirm;

    if (lift_debug.mode == 3u)
    {
        lift_debug.stats_valid = 0u;
        lift_debug.min_count = 0;
        lift_debug.max_count = 0;
        lift_debug.travel_counts = 0;
        lift_debug.overcurrent_ms = 0u;
        lift_debug.mode = 0u;
        lift.motor->tx_info->torque = 0.0f;
        return;
    }

    if ((lift.motor->state->status != DEV_ONLINE) ||
        (Board_HeartBeat.status != DEV_ONLINE))
    {
        lift.motor->tx_info->torque = 0.0f;
        lift_debug.mode = 0u;
        return;
    }

    output = lift_clamp(lift_debug.output_raw, 0.0f,
                        LIFT_DEBUG_OUTPUT_MAX_RAW);
    if (lift_debug.mode == 4u)
    {
        lift.motor->tx_info->torque = 0.0f;
        lift_debug_record(lift.motor->rx_info->encoder_sum);
        return;
    }
    if (lift_debug.mode == 1u)
    {
        lift.motor->tx_info->torque = -output * LIFT_OUTPUT_DIRECTION;
        over_current = lift_tune.over_current_down_raw;
        over_current_confirm = lift_tune.down_over_current_confirm_ms;
    }
    else if (lift_debug.mode == 2u)
    {
        lift.motor->tx_info->torque = output * LIFT_OUTPUT_DIRECTION;
        over_current = lift_tune.over_current_up_raw;
        over_current_confirm = lift_tune.up_over_current_confirm_ms;
    }
    else
    {
        lift.motor->tx_info->torque = 0.0f;
        lift_debug.mode = 0u;
        return;
    }

    count = lift.motor->rx_info->encoder_sum;
    lift_debug_record(count);

    if (lift_abs((float)lift.motor->rx_info->torque_current_raw) >= over_current)
    {
        lift_debug.overcurrent_ms += dt;
        if (lift_debug.overcurrent_ms >=
            (uint32_t)over_current_confirm)
        {
            lift_debug.mode = 0u;
            lift.motor->tx_info->torque = 0.0f;
        }
    }
    else
    {
        lift_debug.overcurrent_ms = 0u;
    }
}

void Lift_Init(void)
{
    lift.motor = &lift_motor;
    lift.state = LIFT_WAIT;
    lift.top_zero = 0;
    lift.top_target = 0;
    lift.bottom_target = 0;
    lift.home_valid = 0u;
    lift.cmd_seen = 0u;
    lift.last_is_hole = 0u;
    lift.pending_is_hole = 0u;
    lift.pending_valid = 0u;
    lift.control_is_hole = 0u;
    lift.pitch_zero_hold = 0u;
    lift.fault_code = 0u;
    lift.active_fault_code = 0u;
    lift.fault_state = LIFT_WAIT;
    lift.fault_position = 0;
    lift.fault_target = 0;
    lift.fault_tick = 0u;
    lift.fault_output_raw = 0.0f;
    lift.fault_current_raw = 0;
    lift.fault_feedback_age_ms = 0u;
    lift.load_hold_target = 0;
    lift.cmd_seq_seen = 0u;
    lift.last_cmd_seq = 0u;
    lift.recover_force_up = 0u;
    lift.restart_required = 0u;
    lift.start_count = 0;
    lift.start_direction = LIFT_UP_DIRECTION;
    lift.start_progress_counts = 0.0f;
    lift.start_confirm_ms = 0u;
    lift.stall_progress_counts = 0.0f;
    lift.fault_count = 0u;
    lift.recover_request = 0u;
    lift.hold_overload = 0u;
    lift.limit_recovery = 0u;
    lift.speed_target_rpm = 0.0f;
    lift.overtravel_ms = 0u;
    lift.hold_current_ms = 0u;
    lift.state_enter_tick = HAL_GetTick();
    lift.init_tick = lift.state_enter_tick;
    lift.home_window_tick = lift.state_enter_tick;
    lift.stall_progress_tick = lift.state_enter_tick;
    lift.stall_progress_count = 0;
    lift.last_tick = lift.state_enter_tick;
    lift_control_dt_ms = 0u;

    lift_debug.mode = 0u;
    lift_debug.stats_valid = 0u;
    lift_debug.output_raw = 500.0f;
    lift_debug.min_count = 0;
    lift_debug.max_count = 0;
    lift_debug.travel_counts = 0;
    lift_debug.overcurrent_ms = 0u;

    lift_tune.home_speed_rpm = LIFT_HOME_SPEED_RPM;
    lift_tune.home_output_limit_raw = LIFT_HOME_OUTPUT_LIMIT_RAW;
    lift_tune.home_current_raw = LIFT_HOME_CURRENT_RAW;
    lift_tune.home_confirm_ms = (float)LIFT_HOME_CONFIRM_MS;
    lift_tune.home_stall_speed_rpm = LIFT_HOME_STALL_SPEED_RPM;
    lift_tune.home_speed_grace_ms = (float)LIFT_HOME_SPEED_GRACE_MS;
    lift_tune.retract_counts = LIFT_RETRACT_COUNTS;
    lift_tune.retract_speed_rpm = LIFT_RETRACT_SPEED_RPM;
    lift_tune.retract_output_limit_raw = LIFT_RETRACT_OUTPUT_LIMIT_RAW;
    lift_tune.move_speed_rpm = LIFT_MOVE_SPEED_RPM;
    lift_tune.pos_kp = LIFT_POS_KP;
    lift_tune.pos_out_max_rpm = LIFT_POS_OUT_MAX_RPM;
    lift_tune.pos_min_speed_rpm = LIFT_POS_MIN_SPEED_RPM;
    lift_tune.speed_kp = LIFT_SPEED_KP;
    lift_tune.speed_direction_sign = LIFT_SPEED_DIRECTION;
    lift_tune.speed_out_max_raw = LIFT_SPEED_OUT_MAX_RAW;
    lift_tune.over_current_down_raw = LIFT_DOWN_OVER_CURRENT_RAW;
    lift_tune.over_current_up_raw = LIFT_UP_OVER_CURRENT_RAW;
    lift_tune.over_current_confirm_ms = (float)LIFT_OVER_CURRENT_CONFIRM_MS;
    lift_tune.down_over_current_confirm_ms =
        (float)LIFT_DOWN_OVER_CURRENT_CONFIRM_MS;
    lift_tune.up_over_current_confirm_ms =
        (float)LIFT_UP_OVER_CURRENT_CONFIRM_MS;
    lift_tune.stall_progress_counts = LIFT_STALL_PROGRESS_COUNTS;

    lift.position_pid.kp = LIFT_POS_KP;
    lift.position_pid.ki = LIFT_POS_KI;
    lift.position_pid.kd = LIFT_POS_KD;
    lift.position_pid.integral_max = LIFT_POS_INTEGRAL_MAX;
    lift.position_pid.out_max = LIFT_POS_OUT_MAX_RPM;
    lift.position_pid.deadband = 0.0f;
    lift.position_pid.d_filter_alpha = 0.0f;

    lift.speed_pid.kp = LIFT_SPEED_KP;
    lift.speed_pid.ki = LIFT_SPEED_KI;
    lift.speed_pid.kd = LIFT_SPEED_KD;
    lift.speed_pid.integral_max = LIFT_SPEED_INTEGRAL_MAX;
    lift.speed_pid.out_max = LIFT_SPEED_OUT_MAX_RAW;
    lift.speed_pid.deadband = 0.0f;
    lift.speed_pid.d_filter_alpha = 0.0f;

    lift_pid_clear(&lift.position_pid);
    lift_pid_clear(&lift.speed_pid);
}

void Lift_Work(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t dt = lift_delta_ms(now, lift.last_tick);
    uint8_t is_hole;
    uint8_t raw_is_hole;
    uint8_t cmd_seq;
    uint8_t cmd_valid;
    uint8_t new_operation = 0u;
    uint8_t board_enabled;
    uint32_t primask;

    lift.last_tick = now;
    lift_control_dt_ms = dt;

    if ((lift.motor == NULL) || (lift.motor->state == NULL))
    {
        return;
    }

    if (lift_debug.mode != 0u)
    {
        lift_debug_update(now, dt);
        return;
    }

    lift.position_pid.kp = lift_tune.pos_kp;
    lift.position_pid.out_max = lift_tune.pos_out_max_rpm;
    lift.speed_pid.kp = lift_tune.speed_kp;
    lift.speed_pid.out_max = lift_tune.speed_out_max_raw;

    primask = __get_PRIMASK();
    __disable_irq();
    raw_is_hole = (Board_Rx_Info.shoot_pkt.is_hole != 0u) ? 1u : 0u;
    cmd_seq = Board_Rx_Info.shoot_pkt.lift_cmd_seq;
    cmd_valid = Board_Rx_Info.shoot_pkt.lift_cmd_valid;
    board_enabled = ((Board_HeartBeat.status == DEV_ONLINE) &&
        (Board_HeartBeat.offline_cnt_1 < Board_HeartBeat.offline_cnt_max) &&
        (Board_Rx_Info.state_pkt.car_state != 0u)) ? 1u : 0u;
    __set_PRIMASK(primask);

    if ((board_enabled == 0u) || (cmd_valid == 0u))
    {
        lift.motor->tx_info->torque = 0.0f;
        if ((lift.home_valid != 0u) || (lift.cmd_seq_seen != 0u) ||
            (lift.state != LIFT_WAIT)) lift.restart_required = 1u;
        lift.cmd_seen = 0u;
        lift.cmd_seq_seen = 0u;
        lift.pending_valid = 0u;
        lift.control_is_hole = 0u;
        lift.recover_request = 0u;
        lift.recover_force_up = 0u;
        lift.pitch_zero_hold = 0u;
        lift_enter_state(LIFT_WAIT, now);
        return;
    }

    if (lift.cmd_seq_seen == 0u)
    {
        lift.cmd_seq_seen = 1u;
        lift.last_cmd_seq = cmd_seq;
    }
    else if (lift.last_cmd_seq != cmd_seq)
    {
        lift.last_cmd_seq = cmd_seq;
        new_operation = 1u;
    }
    if (lift.cmd_seen == 0u)
    {
        lift.cmd_seen = 1u;
        lift.last_is_hole = raw_is_hole;
        lift.pending_valid = 0u;
        lift.control_is_hole = 0u;
    }
    else if (raw_is_hole != lift.last_is_hole)
    {
        lift.last_is_hole = raw_is_hole;
        lift.pending_is_hole = raw_is_hole;
        lift.pending_valid = 1u;
    }

    if (lift.pending_valid != 0u)
    {
        lift.control_is_hole = lift.pending_is_hole;
        lift.pending_valid = 0u;
    }
    // NOTE: 保护恢复不依赖电平翻转
    if ((new_operation != 0u) && ((lift.state == LIFT_PAUSED) ||
        (lift.state == LIFT_FAULT) || (lift.state == LIFT_STALL_STOP) ||
        (lift.state == LIFT_LOAD_HOLD) || (lift.restart_required != 0u)))
    {
        lift.recover_request = 1u;
        lift.recover_force_up = 1u;
    }
    if ((lift.recover_force_up != 0u) || (lift.state == LIFT_START_UP))
    {
        lift.control_is_hole = 0u;
        lift.pending_valid = 0u;
    }

    if ((lift.motor->state->status != DEV_ONLINE) ||
        (lift.motor->rx_info->feedback_seq == 0u) ||
        ((now - lift.motor->rx_info->feedback_tick) > LIFT_FEEDBACK_TIMEOUT_MS))
    {
        if ((lift.state != LIFT_FAULT) || (lift.active_fault_code != LIFT_FAULT_FEEDBACK))
            lift_fault(LIFT_FAULT_FEEDBACK);
        if (new_operation != 0u)
        {
            lift.recover_request = 1u;
            lift.recover_force_up = 1u;
        }
        lift.motor->tx_info->torque = 0.0f;
        lift.pitch_zero_hold = 0u;
        return;
    }
    is_hole = lift.control_is_hole;
    if ((lift.home_valid != 0u) && (lift.state != LIFT_FAULT) &&
        (lift.state != LIFT_PAUSED) && (lift.state != LIFT_LOAD_HOLD) &&
        (lift.state != LIFT_WAIT) && (lift.state != LIFT_HOMING_UP) &&
        (lift_check_limits(now, dt) != 0u))
    {
        if (new_operation != 0u)
        {
            lift.recover_request = 1u;
            lift.recover_force_up = 1u;
        }
        return;
    }
    if (is_hole != 0u)
    {
        lift.pitch_zero_hold = 1u;
    }

    if (lift.state == LIFT_WAIT)
    {
        if ((lift.restart_required != 0u) && (lift.recover_request == 0u))
        {
            lift.motor->tx_info->torque = 0.0f;
            return;
        }
        if (lift.home_valid != 0u)
        {
            if ((is_hole != 0u) && (lift.recover_force_up == 0u))
                lift_enter_state(LIFT_ALIGN_DOWN, now);
            else lift_begin_up(now);
            is_hole = lift.control_is_hole;
        }
        else if ((lift.recover_request != 0u) || (is_hole != 0u) ||
                 ((now - lift.init_tick) >= LIFT_AUTO_HOME_DELAY_MS))
        {
            lift.recover_request = 0u;
            lift.recover_force_up = 0u;
            lift.restart_required = 0u;
            lift_enter_state(LIFT_HOMING_UP, now);
        }
        else
        {
            lift.motor->tx_info->torque = 0.0f;
            return;
        }
    }

    switch (lift.state)
    {
    case LIFT_HOMING_UP:
        lift_homing_up_update(now, dt);
        break;

    case LIFT_RETRACT_DOWN:
        lift_retract_update(now, dt);
        break;

    case LIFT_BOTTOM_RECOVER:
        if (is_hole == 0u)
        {
            lift_begin_up(now);
        }
        else if (lift.limit_recovery == 0u)
        {
            lift_enter_state(LIFT_READY_DOWN, now);
            lift_output_position(lift.bottom_target);
        }
        else if ((now - lift.state_enter_tick) >= LIFT_MOVE_TIMEOUT_MS)
        {
            lift_pause(LIFT_FAULT_MOVE_TIMEOUT, now);
        }
        else
        {
            uint8_t stall = lift_move_stalled(now, dt, 1u);
            if (stall != 0u)
            {
                lift_pause((stall == 1u) ? LIFT_FAULT_STALL_CURRENT :
                           LIFT_FAULT_STALL_PROGRESS, now);
                break;
            }
            lift_output_position_limited(lift.bottom_target, LIFT_HOLD_SPEED_MAX_RPM,
                                         LIFT_HOLD_OUTPUT_MAX_RAW);
            if ((lift_abs((float)lift.bottom_target - lift_encoder_units()) <=
                 LIFT_POS_TOL_COUNTS) &&
                (lift_abs((float)lift.motor->rx_info->encoder_speed) <= LIFT_SPEED_TOL_RPM))
                lift.stable_confirm_ms += dt;
            else lift.stable_confirm_ms = 0u;
            if (lift.stable_confirm_ms >= LIFT_STABLE_CONFIRM_MS)
                lift_enter_state(LIFT_READY_DOWN, now);
        }
        break;

    case LIFT_READY_DOWN:
        if (is_hole == 0u)
        {
            lift_begin_up(now);
        }
        else
        {
            lift_output_position(lift.bottom_target);
        }
        break;

    case LIFT_ALIGN_DOWN:
        if (is_hole == 0u)
        {
            lift_begin_up(now);
        }
        else if (lift_alignment_ok() != 0u)
        {
            lift_enter_state(LIFT_MOVING_DOWN, now);
        }
        else
        {
            lift_output_position(lift.top_target);
        }
        break;

    case LIFT_MOVING_DOWN:
        if (is_hole == 0u)
        {
            lift_begin_up(now);
        }
        else
        {
            lift_move_update(now, dt, lift.bottom_target, 0u);
        }
        break;

    case LIFT_MOVING_UP:
        if (is_hole != 0u)
        {
            lift_enter_state(LIFT_ALIGN_DOWN, now);
        }
        else
        {
            lift_move_update(now, dt, lift.top_target, 1u);
        }
        break;

    case LIFT_READY_UP:
        if (is_hole != 0u)
        {
            lift_enter_state(LIFT_ALIGN_DOWN, now);
        }
        else
        {
            lift_output_position(lift.top_target);
        }
        break;

    case LIFT_START_UP:
        lift_start_up_update(now, dt);
        break;

    case LIFT_LOAD_HOLD:
        if (lift.recover_request != 0u)
        {
            lift_begin_up(now);
            if (lift.state == LIFT_START_UP) lift_start_up_update(now, dt);
            else lift_move_update(now, dt, lift.top_target, 1u);
        }
        else
        {
            lift_update_recovery_direction();
            lift_output_position(lift.load_hold_target);
        }
        break;

    case LIFT_STALL_STOP:
    case LIFT_PAUSED:
    case LIFT_FAULT:
        lift.motor->tx_info->torque = 0.0f;
        if ((lift.recover_request == 0u) ||
            ((now - lift.state_enter_tick) < LIFT_PAUSE_COOLDOWN_MS)) break;
        lift.recover_request = 0u;
        if (lift.home_valid == 0u)
        {
            lift.recover_force_up = 0u;
            lift.control_is_hole = 0u;
            lift_enter_state(LIFT_HOMING_UP, now);
        }
        else
        {
            lift_begin_up(now);
            if (lift.state == LIFT_START_UP) lift_start_up_update(now, dt);
            else lift_move_update(now, dt, lift.top_target, 1u);
        }
        break;

    default:
        lift.motor->tx_info->torque = 0.0f;
        break;
    }

    if (((lift.state == LIFT_READY_UP) && (is_hole == 0u)) ||
        (lift.state == LIFT_MOVING_UP) || (lift.state == LIFT_BOTTOM_RECOVER) ||
        (lift.state == LIFT_LOAD_HOLD) || (lift.state == LIFT_START_UP) ||
        (lift.state == LIFT_FAULT) || (lift.state == LIFT_PAUSED) || (lift.state == LIFT_STALL_STOP))
    {
        lift.pitch_zero_hold = 0u;
    }
    else if ((lift.state == LIFT_ALIGN_DOWN) ||
             (lift.state == LIFT_MOVING_DOWN))
    {
        // NOTE: 重新下行需恢复锁零
        lift.pitch_zero_hold = 1u;
    }
}

uint8_t Lift_IsPitchZeroHoldActive(void)
{
    return lift.pitch_zero_hold;
}

uint8_t Lift_MotorOnline(void)
{
    if ((lift.motor == NULL) || (lift.motor->state == NULL))
    {
        return 0u;
    }

    return (lift.motor->state->status == DEV_ONLINE) ? 1u : 0u;
}

uint8_t Lift_Get_Report_State(void)
{
    switch (lift.state)
    {
    case LIFT_READY_DOWN:
        return 0u;

    case LIFT_READY_UP:
        return 2u;

    case LIFT_PAUSED:
    case LIFT_FAULT:
    case LIFT_LOAD_HOLD:
    case LIFT_START_UP:
        return 3u;

    case LIFT_STALL_STOP:
        return 0u;

    case LIFT_WAIT:
        return (lift.restart_required != 0u) ? 3u : 2u;

    case LIFT_HOMING_UP:
    case LIFT_RETRACT_DOWN:
    case LIFT_ALIGN_DOWN:
    case LIFT_MOVING_DOWN:
    case LIFT_MOVING_UP:
    case LIFT_BOTTOM_RECOVER:
    default:
        return 1u;
    }
}
