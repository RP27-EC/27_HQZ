/* lift.c - 狗洞升降控制 */

#include "lift.h"

#include "communicate.h"
#include "gimbal.h"
#include "lift_config.h"

lift_t lift;
lift_tune_t lift_tune;
volatile lift_debug_t lift_debug;

#define LIFT_FAULT_HOME_TIMEOUT   1u
#define LIFT_FAULT_MOVE_TIMEOUT   2u
#define LIFT_FAULT_OVERTRAVEL     3u
#define LIFT_FAULT_STALL_CURRENT  4u
#define LIFT_FAULT_STALL_PROGRESS 5u

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
    lift.state = state;
    lift.state_enter_tick = now;
    lift.homing_confirm_ms = 0u;
    lift.stable_confirm_ms = 0u;
    lift.overcurrent_confirm_ms = 0u;
    lift.stall_progress_tick = now;
    lift.stall_progress_count = lift.motor->rx_info->encoder_sum;
    lift_pid_clear(&lift.position_pid);
    lift_pid_clear(&lift.speed_pid);

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

    lift.speed_pid.target = lift_rpm_to_rad_s(target_rpm);
    lift.speed_pid.measure = lift_rpm_to_rad_s(
        (float)lift.motor->rx_info->encoder_speed *
        lift_tune.speed_direction_sign);
    lift.speed_pid.err = lift.speed_pid.target - lift.speed_pid.measure;
    single_pid_ctrl(&lift.speed_pid);

    output = lift.speed_pid.out * LIFT_OUTPUT_DIRECTION;
    lift.motor->tx_info->torque =
        lift_clamp(output, -output_limit, output_limit);
}

static void lift_output_position_limited(int32_t target_count,
                                         float speed_limit,
                                         float output_limit)
{
    float speed_target;
    float output;
    float limit = lift_abs(output_limit);

    lift.position_pid.target = (float)target_count;
    lift.position_pid.measure = lift_encoder_units();
    lift.position_pid.err = lift.position_pid.target - lift.position_pid.measure;
    single_pid_ctrl(&lift.position_pid);

    speed_target = lift_clamp(lift.position_pid.out,
                              -lift_abs(speed_limit),
                              lift_abs(speed_limit));
    if ((speed_target > 0.0f) &&
        (speed_target < lift_tune.pos_min_speed_rpm))
    {
        speed_target = lift_tune.pos_min_speed_rpm;
    }
    else if ((speed_target < 0.0f) &&
             (speed_target > -lift_tune.pos_min_speed_rpm))
    {
        speed_target = -lift_tune.pos_min_speed_rpm;
    }
    lift.speed_pid.target = lift_rpm_to_rad_s(speed_target);
    lift.speed_pid.measure = lift_rpm_to_rad_s(
        (float)lift.motor->rx_info->encoder_speed *
        lift_tune.speed_direction_sign);
    lift.speed_pid.err = lift.speed_pid.target - lift.speed_pid.measure;
    single_pid_ctrl(&lift.speed_pid);

    output = lift.speed_pid.out * LIFT_OUTPUT_DIRECTION;
    lift.motor->tx_info->torque = lift_clamp(output, -limit, limit);
}

static void lift_output_position(int32_t target_count)
{
    lift_output_position_limited(target_count,
                                 lift_tune.move_speed_rpm,
                                 lift_tune.speed_out_max_raw);
}

static uint8_t lift_alignment_ok(void)
{
    float yaw_error;
    float pitch_error;

    if ((Gimbal.gimbal_mode != G_MEC) || (Gimbal.init_info.init_flag == 0u))
    {
        return 0u;
    }

    yaw_error = lift_abs(lift_wrap_deg(Gimbal.base_info.yaw_mec_angle -
                                       Gimbal.pid_info.yaw_target));
    pitch_error = lift_abs(Gimbal.base_info.pitch_mec_angle -
                           Gimbal.pid_info.pitch_target);

    if ((yaw_error > LIFT_ALIGN_TOL_DEG) ||
        (pitch_error > LIFT_ALIGN_TOL_DEG))
    {
        return 0u;
    }

    if ((lift_abs(Gimbal.base_info.yaw_mec_speed) > LIFT_ALIGN_SPEED_RAD_S) ||
        (lift_abs(Gimbal.base_info.pitch_mec_speed) > LIFT_ALIGN_SPEED_RAD_S))
    {
        return 0u;
    }

    return 1u;
}

static void lift_fault(uint8_t code)
{
    lift.motor->tx_info->torque = 0.0f;
    lift.fault_code = code;
    lift_enter_state(LIFT_FAULT, HAL_GetTick());
}

static uint8_t lift_move_stalled(uint32_t now, uint32_t dt, uint8_t moving_up)
{
    uint8_t high_current;
    uint8_t low_speed;
    uint8_t no_progress;
    uint32_t confirm_ms;
    float over_current;

    over_current = (moving_up != 0u) ?
                   lift_tune.over_current_up_raw :
                   lift_tune.over_current_down_raw;
    confirm_ms = (moving_up != 0u) ?
                 (uint32_t)lift_tune.up_over_current_confirm_ms :
                 (uint32_t)lift_tune.down_over_current_confirm_ms;

    high_current = (lift_abs((float)lift.motor->rx_info->torque_current_raw) >=
                    over_current) ? 1u : 0u;
    low_speed = (lift_abs((float)lift.motor->rx_info->encoder_speed) <=
                 LIFT_SPEED_TOL_RPM) ? 1u : 0u;

    if ((high_current != 0u) && (low_speed != 0u))
    {
        lift.overcurrent_confirm_ms += dt;
    }
    else
    {
        lift.overcurrent_confirm_ms = 0u;
    }

    no_progress = ((now - lift.stall_progress_tick) >= confirm_ms) &&
                  (lift_abs((float)(lift.motor->rx_info->encoder_sum -
                           lift.stall_progress_count)) <
                   lift_tune.stall_progress_counts);

    if ((lift.overcurrent_confirm_ms >= confirm_ms) ||
        (no_progress != 0u))
    {
        return (lift.overcurrent_confirm_ms >= confirm_ms) ? 1u : 2u;
    }

    if ((now - lift.stall_progress_tick) >= confirm_ms)
    {
        lift.stall_progress_tick = now;
        lift.stall_progress_count = lift.motor->rx_info->encoder_sum;
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
        lift_enter_state(LIFT_READY_UP, now);
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
    float position_error;
    float speed;
    uint8_t stall;
    int32_t travel;
    position_error = (float)target - lift_encoder_units();
    speed = (float)lift.motor->rx_info->encoder_speed;

    travel = (int32_t)(((float)lift.top_zero - lift_encoder_units()) *
                       LIFT_UP_DIRECTION);
    if ((travel < -(int32_t)LIFT_OVERTRAVEL_COUNTS) ||
        (travel > (int32_t)(LIFT_TRAVEL_COUNTS + LIFT_OVERTRAVEL_COUNTS)))
    {
        lift_fault(LIFT_FAULT_OVERTRAVEL);
        return;
    }

    if ((now - lift.state_enter_tick) >= LIFT_MOVE_TIMEOUT_MS)
    {
        lift_fault(LIFT_FAULT_MOVE_TIMEOUT);
        return;
    }

    if (lift_abs(position_error) <= LIFT_POS_TOL_COUNTS)
    {
        if (lift_abs(speed) <= LIFT_SPEED_TOL_RPM)
        {
            lift.stable_confirm_ms += dt;
        }
        else
        {
            lift.stable_confirm_ms = 0u;
        }

        lift_output_position(target);
        if (lift.stable_confirm_ms >= LIFT_STABLE_CONFIRM_MS)
        {
            lift_enter_state((target == lift.bottom_target) ?
                             LIFT_READY_DOWN : LIFT_READY_UP, now);
        }
        return;
    }

    lift.stable_confirm_ms = 0u;
    stall = lift_move_stalled(now, dt, moving_up);
    if (stall != 0u)
    {
        if (moving_up == 0u)
        {
            lift.motor->tx_info->torque = 0.0f;
            lift_enter_state(LIFT_STALL_STOP, now);
            return;
        }
        lift_fault((stall == 1u) ?
                   LIFT_FAULT_STALL_CURRENT : LIFT_FAULT_STALL_PROGRESS);
        return;
    }

    lift_output_position(target);
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
    lift.fault_code = 0u;
    lift.state_enter_tick = HAL_GetTick();
    lift.init_tick = lift.state_enter_tick;
    lift.home_window_tick = lift.state_enter_tick;
    lift.stall_progress_tick = lift.state_enter_tick;
    lift.stall_progress_count = 0;
    lift.last_tick = lift.state_enter_tick;

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
    uint8_t cmd_changed;

    lift.last_tick = now;

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

    if ((lift.motor->state->status != DEV_ONLINE) ||
        (Board_HeartBeat.status != DEV_ONLINE) ||
        (Board_Rx_Info.state_pkt.car_state == 0u))
    {
        lift.motor->tx_info->torque = 0.0f;
        lift.cmd_seen = 0u;
        lift.pending_valid = 0u;
        lift.control_is_hole = 0u;
        lift_enter_state(LIFT_WAIT, now);
        return;
    }

    raw_is_hole = (Board_Rx_Info.shoot_pkt.is_hole != 0u) ? 1u : 0u;
    cmd_changed = 0u;
    if (lift.cmd_seen == 0u)
    {
        lift.cmd_seen = 1u;
        lift.last_is_hole = raw_is_hole;
        lift.pending_valid = 0u;
        lift.control_is_hole = 0u;
    }
    else if (raw_is_hole != lift.last_is_hole)
    {
        cmd_changed = 1u;
        lift.last_is_hole = raw_is_hole;
        lift.pending_is_hole = raw_is_hole;
        lift.pending_valid = 1u;
    }

    if (lift.pending_valid != 0u)
    {
        lift.control_is_hole = lift.pending_is_hole;
        lift.pending_valid = 0u;
    }
    is_hole = lift.control_is_hole;

    if (lift.state == LIFT_WAIT)
    {
        if (lift.home_valid != 0u)
        {
            lift_enter_state(is_hole ? LIFT_MOVING_DOWN : LIFT_MOVING_UP, now);
        }
        else if ((is_hole != 0u) ||
                 ((now - lift.init_tick) >= LIFT_AUTO_HOME_DELAY_MS))
        {
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

    case LIFT_READY_DOWN:
        if (is_hole == 0u)
        {
            lift_enter_state(LIFT_MOVING_UP, now);
        }
        else
        {
            lift_output_position(lift.bottom_target);
        }
        break;

    case LIFT_ALIGN_DOWN:
        if (is_hole == 0u)
        {
            lift_enter_state(LIFT_MOVING_UP, now);
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
            lift_enter_state(LIFT_MOVING_UP, now);
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
            lift.motor->tx_info->torque = 0.0f;
        }
        break;

    case LIFT_STALL_STOP:
        lift.motor->tx_info->torque = 0.0f;
        if (is_hole == 0u)
        {
            lift_enter_state(LIFT_MOVING_UP, now);
        }
        break;

    case LIFT_FAULT:
        lift.motor->tx_info->torque = 0.0f;
        if (cmd_changed == 0u)
        {
            break;
        }

        lift.fault_code = 0u;
        if (lift.home_valid == 0u)
        {
            lift_enter_state(LIFT_HOMING_UP, now);
        }
        else if (is_hole != 0u)
        {
            lift_enter_state(LIFT_ALIGN_DOWN, now);
        }
        else
        {
            lift_enter_state(LIFT_MOVING_UP, now);
        }
        break;

    default:
        lift.motor->tx_info->torque = 0.0f;
        break;
    }
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

    case LIFT_FAULT:
        return 3u;

    case LIFT_STALL_STOP:
        return 0u;

    case LIFT_WAIT:
        return 2u;

    case LIFT_HOMING_UP:
    case LIFT_RETRACT_DOWN:
    case LIFT_ALIGN_DOWN:
    case LIFT_MOVING_DOWN:
    case LIFT_MOVING_UP:
    default:
        return 1u;
    }
}
