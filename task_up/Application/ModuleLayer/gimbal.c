/* gimbal.c - 云台控制 */

#include "gimbal.h"
#include "lift.h"
#include "imu_sensor.h"
#include "rc_sensor.h"
#include "board_remote_config.h"
#include "rp_math.h"
#include <math.h> 

gimbal_t Gimbal;

gimbal_tune_t gimbal_tune;

static float gimbal_abs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float gimbal_clamp(float value, float min_value, float max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

static float gimbal_ramp(float current, float target, float step)
{
    if (step <= 0.0f) return target;
    if (target - current > step) return current + step;
    if (target - current < -step) return current - step;
    return target;
}

static float gimbal_wrap_deg(float angle);

static float gimbal_ramp_wrapped(float current, float target, float step)
{
    float error;

    if (step <= 0.0f) return gimbal_wrap_deg(target);

    error = gimbal_wrap_deg(target - current);
    if (error > step)
    {
        error = step;
    }
    else if (error < -step)
    {
        error = -step;
    }

    return gimbal_wrap_deg(current + error);
}

/* -180≤角度<180度 */
static float gimbal_wrap_deg(float angle)
{
    while (angle >= 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}


/* 遥控器通道转换为目标角速度 */
#if GIMBAL_LOCAL_RC_ENABLE
static float gimbal_axis_to_rate(int16_t axis, float max_rate)
{
    float value = (float)axis;

    if (gimbal_abs(value) <= GIMBAL_RC_AXIS_DEADBAND)
    {
        return 0.0f;
    }

    value -= (value > 0.0f) ? GIMBAL_RC_AXIS_DEADBAND : -GIMBAL_RC_AXIS_DEADBAND;
    value /= (GIMBAL_RC_AXIS_MAX - GIMBAL_RC_AXIS_DEADBAND);
    value = gimbal_clamp(value, -1.0f, 1.0f);

    return value * max_rate;
}
#endif

/* 统一遥控与键鼠速率指令 */
static void gimbal_manual_input_update(gimbal_t *gimbal)
{
    float yaw_rate = 0.0f;
    float pitch_rate = 0.0f;
    uint8_t source = GIMBAL_INPUT_RC;

    gimbal->feedforward.mouse_dx_counts = 0.0f;
    gimbal->feedforward.mouse_dy_counts = 0.0f;

#if GIMBAL_DOWN_RC_ENABLE
    if ((Board_HeartBeat.offline_cnt_5 < Board_HeartBeat.offline_cnt_max) &&
        (Board_Rx_Info.remote_cmd_pkt.valid != 0u) &&
        (Board_Rx_Info.state_pkt.car_state != 0u))
    {
        if ((Board_Rx_Info.remote_cmd_pkt.ctrl_source == 1u) &&
            (Board_Rx_Info.remote_cmd_pkt.cmd_type == 1u))
        {
            source = GIMBAL_INPUT_KEYBOARD;
            gimbal->feedforward.mouse_dx_counts =
                (float)Board_Rx_Info.remote_cmd_pkt.mouse_dx;
            gimbal->feedforward.mouse_dy_counts =
                (float)Board_Rx_Info.remote_cmd_pkt.mouse_dy;
        }
        else
        {
            yaw_rate = gimbal_clamp(Board_Rx_Info.remote_cmd_pkt.yaw_rate_deg_s,
                                    -gimbal_tune.yaw_manual_rate_max_deg_s,
                                    gimbal_tune.yaw_manual_rate_max_deg_s);
            pitch_rate = gimbal_clamp(Board_Rx_Info.remote_cmd_pkt.pitch_rate_deg_s,
                                      -gimbal_tune.pitch_manual_rate_max_deg_s,
                                      gimbal_tune.pitch_manual_rate_max_deg_s);
        }
    }
#elif GIMBAL_LOCAL_RC_ENABLE
    if (rc_dev.work_state == DEV_ONLINE)
    {
        if (Board_Rx_Info.state_pkt.car_state == 1u)
        {
            yaw_rate = gimbal_axis_to_rate(RC_RIGH_CH_LR_VALUE,
                                           gimbal_tune.yaw_manual_rate_max_deg_s);
            pitch_rate = gimbal_axis_to_rate(RC_RIGH_CH_UD_VALUE,
                                             gimbal_tune.pitch_manual_rate_max_deg_s);
        }
        else if (Board_Rx_Info.state_pkt.car_state == 2u)
        {
            yaw_rate = gimbal_clamp(rc_data.mouse_x * GIMBAL_MOUSE_YAW_RATE_GAIN,
                                    -gimbal_tune.yaw_manual_rate_max_deg_s,
                                    gimbal_tune.yaw_manual_rate_max_deg_s);
            pitch_rate = gimbal_clamp(rc_data.mouse_y * GIMBAL_MOUSE_PITCH_RATE_GAIN,
                                      -gimbal_tune.pitch_manual_rate_max_deg_s,
                                      gimbal_tune.pitch_manual_rate_max_deg_s);
        }
    }
#else
    (void)gimbal;
#endif

    /* 输入解码后统一方向 */
    yaw_rate *= gimbal_tune.manual_yaw_sign;
    pitch_rate *= gimbal_tune.manual_pitch_sign;

    if (source != gimbal->feedforward.manual_source)
    {
        gimbal->feedforward.manual_source = source;
        gimbal->feedforward.manual_source_changed = 1u;
    }

    gimbal->feedforward.yaw_rate_cmd_deg_s = yaw_rate;
    gimbal->feedforward.pitch_rate_cmd_deg_s = pitch_rate;
}

/* 清空PID积分与微分历史 */
static void gimbal_pid_clear(pid_ctrl_t *pid)
{
    integral_to_zero(pid);
    pid->target = 0.0f;
    pid->measure = 0.0f;
    pid->err = 0.0f;
    pid->pout = 0.0f;
    pid->iout = 0.0f;
    pid->dout = 0.0f;
    pid->last_dout = 0.0f;
}

/* 模式切换清除控制环历史 */
static void gimbal_clear_all_pid(gimbal_t *gimbal)
{
    gimbal_pid_clear(&gimbal->pid_info.yaw_gyro_outer);
    gimbal_pid_clear(&gimbal->pid_info.yaw_gyro_inner);
    gimbal_pid_clear(&gimbal->pid_info.yaw_mec_outer);
    gimbal_pid_clear(&gimbal->pid_info.yaw_mec_inner);
    gimbal_pid_clear(&gimbal->pid_info.yaw_turn_outer);
    gimbal_pid_clear(&gimbal->pid_info.yaw_turn_inner);
    gimbal_pid_clear(&gimbal->pid_info.yaw_init_outer);
    gimbal_pid_clear(&gimbal->pid_info.yaw_init_inner);
    gimbal_pid_clear(&gimbal->pid_info.pitch_init_outer);
    gimbal_pid_clear(&gimbal->pid_info.pitch_init_inner);
    gimbal_pid_clear(&gimbal->pid_info.pitch_gyro_outer);
    gimbal_pid_clear(&gimbal->pid_info.pitch_gyro_inner);
    gimbal_pid_clear(&gimbal->pid_info.yaw_hold);
    gimbal_pid_clear(&gimbal->pid_info.pitch_hold);
}

/* 初始化串级与保持环参数 */
static void gimbal_pid_init(gimbal_t *gimbal)
{
	pid_ctrl_t *pid;

    /* Yaw 陀螺仪串级 */
    pid = &gimbal->pid_info.yaw_gyro_outer;//  外环
    pid->kp = 10.0f; pid->ki = 0.05f; pid->kd = 0.0f;
    pid->integral_max = 200.0f; pid->out_max = 500.0f;

    pid = &gimbal->pid_info.yaw_gyro_inner;//  内环
    pid->kp = 0.06f; pid->ki = 0.0f; pid->kd = 0.0f;
    pid->integral_max = 0.0f; pid->out_max = 10.0f;

    /* Pitch 陀螺仪串级 */
    pid = &gimbal->pid_info.pitch_gyro_outer;//  外环
    pid->kp = 56.0f; pid->ki = 0.0f; pid->kd = 0.0f;
    pid->integral_max = 0.0f; pid->out_max = 100.0f;

    pid = &gimbal->pid_info.pitch_gyro_inner;//  内环
    pid->kp = 0.03f; pid->ki = 0.0f; pid->kd = 0.0f;
    pid->integral_max = 0.0f; pid->out_max = 10.0f;

    /* Yaw 机械编码器串级 */
    pid = &gimbal->pid_info.yaw_mec_outer;
    pid->kp = 1.0f; pid->ki = 0.0f; pid->kd = GIMBAL_MEC_OUTER_KD;
    pid->deadband = GIMBAL_MEC_ERR_DEADBAND_DEG;
    pid->d_filter_alpha = GIMBAL_MEC_OUTER_D_FILTER_ALPHA;
    pid->integral_max = 0.0f; pid->out_max = 500.0f;

    pid = &gimbal->pid_info.yaw_mec_inner;
    pid->kp = 5.5f; pid->ki = 0.0f; pid->kd = 0.2f;
    pid->integral_max = 0.0f; pid->out_max = 100.0f;

    /* 掉头参数独立于归中 */
    pid = &gimbal->pid_info.yaw_turn_outer;
    pid->kp = GIMBAL_TURN_YAW_OUTER_KP;
    pid->ki = GIMBAL_TURN_YAW_OUTER_KI;
    pid->kd = GIMBAL_TURN_YAW_OUTER_KD;
    pid->integral_max = GIMBAL_TURN_YAW_OUTER_INTEGRAL_MAX;
    pid->out_max = GIMBAL_TURN_YAW_OUTER_OUT_MAX;
    pid->deadband = 0.0f;
    pid->d_filter_alpha = GIMBAL_TURN_D_FILTER_ALPHA;

    pid = &gimbal->pid_info.yaw_turn_inner;
    pid->kp = GIMBAL_TURN_YAW_INNER_KP;
    pid->ki = GIMBAL_TURN_YAW_INNER_KI;
    pid->kd = GIMBAL_TURN_YAW_INNER_KD;
    pid->integral_max = GIMBAL_TURN_YAW_INNER_INTEGRAL_MAX;
    pid->out_max = GIMBAL_TURN_YAW_INNER_OUT_MAX;
    pid->deadband = 0.0f;
    pid->d_filter_alpha = GIMBAL_TURN_D_FILTER_ALPHA;

    /* Yaw 归中编码器串级 */
    pid = &gimbal->pid_info.yaw_init_outer;
    pid->kp = GIMBAL_INIT_YAW_OUTER_KP;
    pid->ki = GIMBAL_INIT_YAW_OUTER_KI;
    pid->kd = GIMBAL_INIT_YAW_OUTER_KD;
    pid->integral_max = GIMBAL_INIT_YAW_OUTER_INTEGRAL_MAX;
    pid->out_max = GIMBAL_INIT_YAW_OUTER_OUT_MAX;
    pid->deadband = 0.0f;
    pid->d_filter_alpha = GIMBAL_INIT_D_FILTER_ALPHA;

    pid = &gimbal->pid_info.yaw_init_inner;
    pid->kp = GIMBAL_INIT_YAW_INNER_KP;
    pid->ki = GIMBAL_INIT_YAW_INNER_KI;
    pid->kd = GIMBAL_INIT_YAW_INNER_KD;
    pid->integral_max = GIMBAL_INIT_YAW_INNER_INTEGRAL_MAX;
    pid->out_max = GIMBAL_INIT_YAW_INNER_OUT_MAX;
    pid->deadband = 0.0f;
    pid->d_filter_alpha = GIMBAL_INIT_D_FILTER_ALPHA;

    /* Pitch 归中编码器串级 */
    pid = &gimbal->pid_info.pitch_init_outer;
    pid->kp = GIMBAL_INIT_PITCH_OUTER_KP;
    pid->ki = GIMBAL_INIT_PITCH_OUTER_KI;
    pid->kd = GIMBAL_INIT_PITCH_OUTER_KD;
    pid->integral_max = GIMBAL_INIT_PITCH_OUTER_INTEGRAL_MAX;
    pid->out_max = GIMBAL_INIT_PITCH_OUTER_OUT_MAX;
    pid->deadband = 0.0f;
    pid->d_filter_alpha = GIMBAL_INIT_D_FILTER_ALPHA;

    pid = &gimbal->pid_info.pitch_init_inner;
    pid->kp = GIMBAL_INIT_PITCH_INNER_KP;
    pid->ki = GIMBAL_INIT_PITCH_INNER_KI;
    pid->kd = GIMBAL_INIT_PITCH_INNER_KD;
    pid->integral_max = GIMBAL_INIT_PITCH_INNER_INTEGRAL_MAX;
    pid->out_max = GIMBAL_INIT_PITCH_INNER_OUT_MAX;
    pid->deadband = 0.0f;
    pid->d_filter_alpha = GIMBAL_INIT_D_FILTER_ALPHA;

    /* 保持环输出角速度修正 */
    pid = &gimbal->pid_info.yaw_hold;
    pid->kp = GIMBAL_YAW_HOLD_KP; pid->ki = GIMBAL_YAW_HOLD_KI; pid->kd = 0.0f;
    pid->integral_max = GIMBAL_YAW_HOLD_INTEGRAL_MAX;
    pid->out_max = GIMBAL_YAW_HOLD_OUT_MAX;
    pid->deadband = GIMBAL_HOLD_ERR_DEADBAND_DEG;
    pid->d_filter_alpha = 0.0f;

    pid = &gimbal->pid_info.pitch_hold;
    pid->kp = GIMBAL_PITCH_HOLD_KP; pid->ki = GIMBAL_PITCH_HOLD_KI; pid->kd = 0.0f;
    pid->integral_max = GIMBAL_PITCH_HOLD_INTEGRAL_MAX;
    pid->out_max = GIMBAL_PITCH_HOLD_OUT_MAX;
    pid->deadband = GIMBAL_HOLD_ERR_DEADBAND_DEG;
    pid->d_filter_alpha = 0.0f;

    gimbal_clear_all_pid(gimbal);
}

/* 同步传感器与控制输入 */
static void gimbal_info_update(gimbal_t *gimbal)
{
	imu_data_t *imu = imu_dev.info;

    /* IMU 姿态与角速度 */
    gimbal->base_info.yaw_imu_angle = imu->base_info.yaw;
    gimbal->base_info.yaw_imu_speed = imu->base_info.rate_yaw;
    gimbal->base_info.pitch_imu_angle = imu->base_info.pitch;
    gimbal->base_info.pitch_imu_speed = imu->base_info.rate_pitch;

    /* 电机编码器角度换算 */
    gimbal->base_info.yaw_mec_angle = gimbal_wrap_deg( /* Yaw 机械角 */
        gimbal->yaw_motor->rx_info->motor_angle * GIMBAL_RAD_TO_DEG -
        GIMBAL_YAW_MIDDLE_DEG);
    gimbal->base_info.yaw_mec_speed = gimbal->yaw_motor->rx_info->speed;

    gimbal->base_info.pitch_mec_angle = gimbal_wrap_deg( /* Pitch 机械角 */
        gimbal->pitch_motor->rx_info->motor_angle * GIMBAL_RAD_TO_DEG -
        GIMBAL_PITCH_MIDDLE_DEG);
    gimbal->base_info.pitch_mec_speed = gimbal->pitch_motor->rx_info->speed;

    gimbal->pid_info.yaw_mec_target_raw =
        Board_Rx_Info.gimbal_target_pkt.yaw_mec_tar * GIMBAL_RAD_TO_DEG;
    gimbal->pid_info.pitch_mec_target_raw =
        Board_Rx_Info.gimbal_target_pkt.pitch_mec_tar * GIMBAL_RAD_TO_DEG;
    gimbal->pid_info.yaw_imu_target_raw =
        Board_Rx_Info.gimbal_target_pkt.yaw_imu_tar;
    gimbal->pid_info.pitch_imu_target_raw =
        Board_Rx_Info.gimbal_target_pkt.pitch_imu_tar;

    gimbal_manual_input_update(gimbal);
}

/* 板间在线且整车使能才闭环 */
static uint8_t gimbal_safety_allows_control(gimbal_t *gimbal)
{
    (void)gimbal;
    /* 板间在线且整车使能 */
    if (Board_HeartBeat.status != DEV_ONLINE) return 0;
    if (Board_Rx_Info.state_pkt.car_state == 0) return 0;

    return 1;
}

/* 安全与归中优先仲裁模式 */
static gimbal_mode_e gimbal_select_mode(gimbal_t *gimbal)
{
    // 安全条件失效时休眠
    if (!gimbal_safety_allows_control(gimbal))
    {
        gimbal->init_info.init_flag = 0;
        gimbal->init_info.init_time = 0;
        return G_SLEEP;
    }

    // 重新初始化
    if (gimbal->init_info.init_flag == 0)
    {
        return G_INIT;
    }

    // 板间模式0选择机械控制
    if (Board_Rx_Info.state_pkt.gimbal_mode == 0)
    {
        return G_MEC;
    }

    return G_RATE;// 速控模式
}

/* 读取当前模式两轴目标 */
static void gimbal_get_command_target(gimbal_t *gimbal, float *yaw_target, float *pitch_target)
{
    switch (gimbal->gimbal_mode)
    {
    case G_INIT:
        *yaw_target = gimbal_wrap_deg(gimbal->init_info.yaw_home_deg);
        *pitch_target = gimbal->init_info.pitch_home_deg;
        break;

    case G_MEC:
        *yaw_target = gimbal_wrap_deg(gimbal->pid_info.yaw_mec_target_raw);
        *pitch_target = gimbal->pid_info.pitch_mec_target_raw;
        break;

    case G_GYRO:
    case G_AUTO:
        *yaw_target = gimbal_wrap_deg(gimbal->pid_info.yaw_imu_target_raw);
        *pitch_target = gimbal->pid_info.pitch_imu_target_raw;
        break;

    case G_SLEEP:
    default:
        *yaw_target = gimbal->pid_info.yaw_target;
        *pitch_target = gimbal->pid_info.pitch_target;
        break;
    }
}

/* 机械限位约束惯性角目标 */
static float gimbal_limit_pitch_target(gimbal_t *gimbal, float pitch_target)
{
    if ((gimbal->gimbal_mode == G_GYRO) ||
        (gimbal->gimbal_mode == G_AUTO))
    {
        float delta = pitch_target - gimbal->base_info.pitch_imu_angle;
        float predicted_mec_angle = gimbal->base_info.pitch_mec_angle + delta;

        if (predicted_mec_angle > GIMBAL_PITCH_MAX_DEG)
        {
            delta -= predicted_mec_angle - GIMBAL_PITCH_MAX_DEG;
        }
        else if (predicted_mec_angle < GIMBAL_PITCH_MIN_DEG)
        {
            delta += GIMBAL_PITCH_MIN_DEG - predicted_mec_angle;
        }

        return gimbal->base_info.pitch_imu_angle + delta;
    }

    return gimbal_clamp(pitch_target, GIMBAL_PITCH_MIN_DEG, GIMBAL_PITCH_MAX_DEG);
}

/* 归中到位判定与超时退出 */
static void gimbal_update_init(gimbal_t *gimbal)
{
    float yaw_error = gimbal_abs(gimbal_wrap_deg(
        gimbal->base_info.yaw_mec_angle - gimbal->init_info.yaw_home_deg));
    float pitch_error = gimbal_abs(
        gimbal->base_info.pitch_mec_angle - gimbal->init_info.pitch_home_deg);
    uint8_t position_ok = (yaw_error <= gimbal->init_info.yaw_angle_tolerance) &&
                          (pitch_error <= gimbal->init_info.pitch_angle_tolerance);
    uint8_t speed_ok = (gimbal_abs(gimbal->base_info.yaw_mec_speed) <=
                        gimbal->init_info.yaw_speed_tolerance) &&
                       (gimbal_abs(gimbal->base_info.pitch_mec_speed) <=
                        gimbal->init_info.pitch_speed_tolerance);
    uint8_t target_ok = (gimbal_abs(gimbal_wrap_deg(
                           gimbal->pid_info.yaw_target - gimbal->init_info.yaw_home_deg)) <=
                        GIMBAL_TARGET_ARRIVE_EPS_DEG) &&
                       (gimbal_abs(gimbal->pid_info.pitch_target -
                                    gimbal->init_info.pitch_home_deg) <=
                        GIMBAL_TARGET_ARRIVE_EPS_DEG);

    if (position_ok && speed_ok && target_ok)
    {
        if (gimbal->init_info.stable_time < gimbal->init_info.stable_time_max)
        {
            gimbal->init_info.stable_time++;
        }
    }
    else
    {
        gimbal->init_info.stable_time = 0;
    }

    if (gimbal->init_info.stable_time >= gimbal->init_info.stable_time_max)
    {
        gimbal->init_info.init_flag = 1;
        gimbal->init_info.init_time = 0;
        gimbal->init_info.stable_time = 0;
        gimbal->init_info.mode_transition_active = 0;
        return;
    }

    if (gimbal->init_info.init_time >= gimbal->init_info.init_time_max)
    {
        gimbal->init_info.init_flag = 1;
        gimbal->init_info.init_time = 0;
        gimbal->init_info.stable_time = 0;
        gimbal->init_info.mode_transition_active = 0;
        return;
    }

    gimbal->init_info.init_time++;
}

/* 目标值更新与限位 */
static void gimbal_update_targets(gimbal_t *gimbal)
{
    float yaw_final;
    float pitch_final;

    if (gimbal->gimbal_mode == G_SLEEP)
    {
        gimbal->pid_info.yaw_target = 0.0f;
        gimbal->pid_info.pitch_target = 0.0f;
        return;
    }

    if (gimbal->gimbal_mode == G_RATE)
    {
        gimbal->init_info.mode_transition_active = 0;
        return;
    }

    gimbal_get_command_target(gimbal, &yaw_final, &pitch_final);
    pitch_final = gimbal_limit_pitch_target(gimbal, pitch_final);

    if (gimbal->gimbal_mode == G_INIT)
    {
        gimbal->pid_info.yaw_target = gimbal_ramp_wrapped(
            gimbal->pid_info.yaw_target, yaw_final, gimbal->init_info.yaw_ramp_step);
        gimbal->pid_info.pitch_target = gimbal_ramp(
            gimbal->pid_info.pitch_target, pitch_final, gimbal->init_info.pitch_ramp_step);
    }
    else if (gimbal->gimbal_mode == G_MEC)
    {
        /* 机械目标不等待双轴切换到位。 */
        gimbal->init_info.mode_transition_active = 0;
        gimbal->pid_info.yaw_target = gimbal_ramp_wrapped(
            gimbal->pid_info.yaw_target, yaw_final,
            (Board_Rx_Info.state_pkt.r_turn_active != 0u) ?
                GIMBAL_R_TURN_YAW_RAMP_DEG_PER_MS : GIMBAL_TURN_YAW_RAMP_DEG_PER_MS);
        gimbal->pid_info.pitch_target = pitch_final;
    }
    else if (gimbal->init_info.mode_transition_active)
    {
        gimbal->pid_info.yaw_target = gimbal_ramp_wrapped(
            gimbal->pid_info.yaw_target, yaw_final, gimbal->init_info.mode_yaw_ramp_step);
        gimbal->pid_info.pitch_target = gimbal_ramp(
            gimbal->pid_info.pitch_target, pitch_final, gimbal->init_info.mode_pitch_ramp_step);

        if (gimbal_abs(gimbal_wrap_deg(yaw_final - gimbal->pid_info.yaw_target)) <=
                GIMBAL_TARGET_ARRIVE_EPS_DEG &&
            gimbal_abs(pitch_final - gimbal->pid_info.pitch_target) <=
                GIMBAL_TARGET_ARRIVE_EPS_DEG)
        {
            gimbal->init_info.mode_transition_active = 0;
        }
    }
    else
    {
        gimbal->pid_info.yaw_target = yaw_final;
        gimbal->pid_info.pitch_target = pitch_final;
    }
}

/* 保持权重，0手控1保持 */
static float gimbal_hold_blend(float rate_mag)
{
    float enter = gimbal_tune.rate_hold_enter_deg_s;
    float exit = gimbal_tune.rate_hold_exit_deg_s;

    if (rate_mag <= enter)
    {
        return 1.0f;
    }
    if ((rate_mag >= exit) || (exit <= enter))
    {
        return 0.0f;
    }

    return (exit - rate_mag) / (exit - enter);
}

static void gimbal_yaw_release_reset(gimbal_t *gimbal)
{
    gimbal_yaw_release_t *release = &gimbal->feedforward.yaw_release;

    release->phase = GIMBAL_YAW_RELEASE_HOLD;
    release->brake_start_ms = 0u;
    release->stable_start_ms = 0u;
    release->stable_tracking = 0u;
}

static uint8_t gimbal_yaw_release_update(gimbal_t *gimbal, float yaw_mag)
{
    gimbal_yaw_release_t *release = &gimbal->feedforward.yaw_release;
    uint32_t now = HAL_GetTick();

    if (yaw_mag > gimbal_tune.rate_hold_enter_deg_s)
    {
        release->phase = GIMBAL_YAW_RELEASE_MANUAL;
        release->stable_tracking = 0u;
        gimbal->feedforward.yaw_hold_angle_deg = gimbal->base_info.yaw_imu_angle;
        integral_to_zero(&gimbal->pid_info.yaw_hold);
        return 0u;
    }

    if (release->phase == GIMBAL_YAW_RELEASE_MANUAL)
    {
        release->phase = GIMBAL_YAW_RELEASE_BRAKE;
        release->brake_start_ms = now;
        release->stable_tracking = 0u;
    }

    if (release->phase != GIMBAL_YAW_RELEASE_BRAKE)
    {
        return 0u;
    }

    // NOTE: 制动期间不拉回旧角
    gimbal->feedforward.yaw_hold_angle_deg = gimbal->base_info.yaw_imu_angle;
    integral_to_zero(&gimbal->pid_info.yaw_hold);

    if (gimbal_abs(gimbal->base_info.yaw_imu_speed) < GIMBAL_YAW_RELEASE_STOP_DEG_S)
    {
        if (release->stable_tracking == 0u)
        {
            release->stable_start_ms = now;
            release->stable_tracking = 1u;
        }
    }
    else
    {
        release->stable_tracking = 0u;
    }

    if (((release->stable_tracking != 0u) &&
         ((uint32_t)(now - release->stable_start_ms) >= GIMBAL_YAW_RELEASE_STABLE_MS)) ||
        ((uint32_t)(now - release->brake_start_ms) >= GIMBAL_YAW_RELEASE_TIMEOUT_MS))
    {
        release->phase = GIMBAL_YAW_RELEASE_HOLD;
        release->stable_tracking = 0u;
        return 0u;
    }

    return 1u;
}

// NOTE: 增量角输入保留原锁角
static void gimbal_update_rate_targets(gimbal_t *gimbal)
{
    float yaw_cmd = gimbal->feedforward.yaw_rate_cmd_deg_s;
    float pitch_cmd = gimbal->feedforward.pitch_rate_cmd_deg_s;
    float yaw_mag = gimbal_abs(yaw_cmd);
    float pitch_mag = gimbal_abs(pitch_cmd);
    float yaw_blend;
    float pitch_blend;
    uint8_t pitch_zero_hold = Lift_IsPitchZeroHoldActive();
    uint8_t pitch_zero_transition =
        (pitch_zero_hold != gimbal->feedforward.pitch_zero_hold_last);

    if (pitch_zero_transition != 0u)
    {
        gimbal->feedforward.pitch_hold_angle_deg = 0.0f;
        integral_to_zero(&gimbal->pid_info.pitch_hold);
    }

    /* 每周期同步调节增益 */
    gimbal->pid_info.yaw_hold.kp = gimbal_tune.yaw_hold_kp;
    gimbal->pid_info.yaw_hold.ki = gimbal_tune.yaw_hold_ki;
    gimbal->pid_info.yaw_hold.integral_max = gimbal_tune.yaw_hold_integral_max;
    gimbal->pid_info.yaw_hold.out_max = gimbal_tune.yaw_hold_out_max;
    gimbal->pid_info.pitch_hold.kp = gimbal_tune.pitch_hold_kp;
    gimbal->pid_info.pitch_hold.ki = gimbal_tune.pitch_hold_ki;
    gimbal->pid_info.pitch_hold.integral_max = gimbal_tune.pitch_hold_integral_max;
    gimbal->pid_info.pitch_hold.out_max = gimbal_tune.pitch_hold_out_max;

    if (gimbal->feedforward.manual_source_changed != 0u)
    {
        gimbal_yaw_release_reset(gimbal);
        gimbal->feedforward.yaw_hold_angle_deg = gimbal->base_info.yaw_imu_angle;
        if ((pitch_zero_hold == 0u) && (pitch_zero_transition == 0u))
        {
            gimbal->feedforward.pitch_hold_angle_deg = gimbal->base_info.pitch_mec_angle;
        }
        integral_to_zero(&gimbal->pid_info.yaw_hold);
        integral_to_zero(&gimbal->pid_info.pitch_hold);
        gimbal->feedforward.manual_source_changed = 0u;
    }

    if (gimbal->feedforward.manual_source == GIMBAL_INPUT_KEYBOARD)
    {
        float yaw_delta = gimbal->feedforward.mouse_dx_counts;
        float pitch_delta = (pitch_zero_hold != 0u) ? 0.0f :
                            gimbal->feedforward.mouse_dy_counts;

        if (gimbal_abs(yaw_delta) < gimbal_tune.mouse_deadband_count)
        {
            yaw_delta = 0.0f;
        }
        if (gimbal_abs(pitch_delta) < gimbal_tune.mouse_deadband_count)
        {
            pitch_delta = 0.0f;
        }

        yaw_delta *= gimbal_tune.mouse_yaw_sign;
        pitch_delta *= gimbal_tune.mouse_pitch_sign;

        if (yaw_delta != 0.0f)
        {
            gimbal->feedforward.yaw_hold_angle_deg = gimbal_wrap_deg(
                gimbal->feedforward.yaw_hold_angle_deg +
                yaw_delta * gimbal_tune.mouse_yaw_deg_per_count);
            integral_to_zero(&gimbal->pid_info.yaw_hold);
        }
        if (pitch_delta != 0.0f)
        {
            gimbal->feedforward.pitch_hold_angle_deg = gimbal_clamp(
                gimbal->feedforward.pitch_hold_angle_deg +
                pitch_delta * gimbal_tune.mouse_pitch_deg_per_count,
                GIMBAL_PITCH_MIN_DEG,
                GIMBAL_PITCH_MAX_DEG);
            integral_to_zero(&gimbal->pid_info.pitch_hold);
        }

        yaw_cmd = gimbal_clamp(
            yaw_delta * gimbal_tune.mouse_rate_ff_dps_per_count,
            -gimbal_tune.yaw_manual_rate_max_deg_s,
            gimbal_tune.yaw_manual_rate_max_deg_s);
        pitch_cmd = gimbal_clamp(
            pitch_delta * gimbal_tune.mouse_rate_ff_dps_per_count,
            -gimbal_tune.pitch_manual_rate_max_deg_s,
            gimbal_tune.pitch_manual_rate_max_deg_s);
        yaw_blend = 1.0f;
        pitch_blend = 1.0f;
    }
    else
    {
        yaw_blend = gimbal_hold_blend(yaw_mag);
        if (gimbal->gimbal_mode == G_RATE)
        {
            uint8_t yaw_braking = gimbal_yaw_release_update(gimbal, yaw_mag);

            if (yaw_mag <= gimbal_tune.rate_hold_enter_deg_s)
            {
                yaw_cmd = 0.0f;
            }
            if (yaw_braking != 0u)
            {
                yaw_blend = 0.0f;
            }
        }
        else if (yaw_mag > gimbal_tune.rate_hold_enter_deg_s)
        {
            gimbal->feedforward.yaw_hold_angle_deg = gimbal->base_info.yaw_imu_angle;
            integral_to_zero(&gimbal->pid_info.yaw_hold);
        }
        if ((pitch_zero_hold == 0u) &&
            (pitch_mag > gimbal_tune.rate_hold_enter_deg_s))
        {
            gimbal->feedforward.pitch_hold_angle_deg = gimbal->base_info.pitch_mec_angle;
            integral_to_zero(&gimbal->pid_info.pitch_hold);
        }

        pitch_blend = gimbal_hold_blend(pitch_mag);
    }

    if (pitch_zero_hold != 0u)
    {
        // NOTE: 升降全程优先锁零
        gimbal->feedforward.pitch_hold_angle_deg = 0.0f;
        gimbal->pid_info.pitch_target = 0.0f;
        pitch_cmd = 0.0f;
        pitch_blend = 1.0f;
    }
    gimbal->feedforward.pitch_zero_hold_last = pitch_zero_hold;

    /* 保持环输出角速度修正 */
    gimbal->pid_info.yaw_hold.err = gimbal_wrap_deg(
        gimbal->feedforward.yaw_hold_angle_deg - gimbal->base_info.yaw_imu_angle);
    single_pid_ctrl(&gimbal->pid_info.yaw_hold);

    /* Pitch保持采用机械角 */
    gimbal->pid_info.pitch_hold.err =
        gimbal->feedforward.pitch_hold_angle_deg - gimbal->base_info.pitch_mec_angle;
    single_pid_ctrl(&gimbal->pid_info.pitch_hold);

    gimbal->feedforward.yaw_rate_target_deg_s = gimbal_clamp(
        yaw_cmd + yaw_blend * gimbal->pid_info.yaw_hold.out,
        -gimbal_tune.yaw_manual_rate_max_deg_s,
        gimbal_tune.yaw_manual_rate_max_deg_s);
    gimbal->feedforward.pitch_rate_target_deg_s = gimbal_clamp(
        pitch_cmd + pitch_blend * gimbal->pid_info.pitch_hold.out,
        -gimbal_tune.pitch_manual_rate_max_deg_s,
        gimbal_tune.pitch_manual_rate_max_deg_s);
}

/* Pitch 轴重力力矩补偿 */
//重力补偿，需调试数值
static float gimbal_gravity_compensation(gimbal_t *gimbal)
{
    float angle_rad;

    if (gimbal_tune.gravity_enable == 0u)
    {
        (void)gimbal;
        return 0.0f;
    }

    angle_rad = (gimbal->base_info.pitch_mec_angle -
                 gimbal_tune.gravity_middle_deg) * GIMBAL_DEG_TO_RAD;

    return gimbal_tune.gravity_sign *
           (gimbal_tune.gravity_k_nm * cosf(angle_rad) +
            gimbal_tune.gravity_b_nm);
}

/* 归中独立重力补偿 */
static float gimbal_init_gravity_compensation(gimbal_t *gimbal)
{
    float angle_rad;

    if (gimbal->init_info.pitch_gravity_enable == 0u)
    {
        return 0.0f;
    }

    angle_rad = (gimbal->base_info.pitch_mec_angle -
                 gimbal->init_info.pitch_gravity_middle_deg) * GIMBAL_DEG_TO_RAD;

    return gimbal->init_info.pitch_gravity_sign *
           (gimbal->init_info.pitch_gravity_k_nm * cosf(angle_rad) +
            gimbal->init_info.pitch_gravity_b_nm);
}

/* 机械 Yaw 制动限速。 */
/* 剩余角约束制动速限 */
static float gimbal_mec_speed_limit(float error_deg,
                                     float max_rate_deg_s,
                                     float decel_rad_s2)
{
    float error_rad = gimbal_abs(error_deg) * GIMBAL_DEG_TO_RAD;
    float max_rate_rad_s = max_rate_deg_s * GIMBAL_DEG_TO_RAD;
    float brake_rate = sqrtf(2.0f * decel_rad_s2 * error_rad);

    if (brake_rate > max_rate_rad_s)
    {
        brake_rate = max_rate_rad_s;
    }

    return brake_rate * GIMBAL_RAD_TO_DEG;
}

/* 归中独立速度与制动限幅 */
static float gimbal_init_pid_calc(pid_ctrl_t *outer,
                                 pid_ctrl_t *inner,
                                 float target,
                                 float angle,
                                 float speed,
                                 float max_rate_deg_s,
                                 float decel_rad_s2,
                                 uint8_t speed_limit_enable,
                                 uint8_t wrap)
{
    float speed_limit;

    outer->target = target;
    outer->measure = angle;
    outer->err = target - angle;
    if (wrap != 0u)
    {
        outer->err = gimbal_wrap_deg(outer->err);
    }
    single_pid_ctrl(outer);

    /* 归中与掉头独立选择制动限速 */
    if (speed_limit_enable != 0u)
    {
        speed_limit = gimbal_mec_speed_limit(outer->err,
                                              max_rate_deg_s,
                                              decel_rad_s2);
        outer->out = gimbal_clamp(outer->out, -speed_limit, speed_limit);
    }

    inner->target = outer->out;
    inner->measure = speed;
    inner->err = inner->target - inner->measure;
    single_pid_ctrl(inner);

    return inner->out;
}

/* 机械定位使用独立PID状态 */
static pid_ctrl_t gimbal_mec_yaw_pos_pid;
static pid_ctrl_t gimbal_mec_yaw_rate_pid;
static float gimbal_mec_yaw_ff_blend;
static float gimbal_mec_yaw_chassis_rate_lpf;
static uint8_t gimbal_mec_yaw_hard_hold;

/* 积分乘周期微分除周期 */
static float gimbal_pid_calc(pid_ctrl_t *pid,
                             float setpoint,
                             float measure,
                             float control_period_s)
{
    float candidate_integral;
    float derivative;
    float raw_output;

    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->target = setpoint;
    pid->measure = measure;
    pid->err = setpoint - measure;
    derivative = (pid->err - pid->last_err) / control_period_s;

    /* 加剧饱和时拒绝候选积分 */
    candidate_integral = pid->integral + pid->err * control_period_s;
    if (candidate_integral > pid->integral_max)
    {
        candidate_integral = pid->integral_max;
    }
    else if (candidate_integral < -pid->integral_max)
    {
        candidate_integral = -pid->integral_max;
    }

    raw_output = pid->kp * pid->err
               + pid->ki * candidate_integral
               + pid->kd * derivative;

    if (((raw_output > pid->out_max) && (pid->err > 0.0f)) ||
        ((raw_output < -pid->out_max) && (pid->err < 0.0f)))
    {
        raw_output = pid->kp * pid->err
                   + pid->ki * pid->integral
                   + pid->kd * derivative;
    }
    else
    {
        pid->integral = candidate_integral;
    }
    pid->iout = pid->ki * pid->integral;
    pid->dout = pid->kd * derivative;

    pid->out = gimbal_clamp(raw_output, -pid->out_max, pid->out_max);
    pid->last_err = pid->err;

    return pid->out;
}

/* 重置PID历史保留参数 */
static void gimbal_mec_yaw_pid_reset(pid_ctrl_t *pid)
{
    pid->integral = 0.0f;
    pid->last_err = 0.0f;
    pid->target = 0.0f;
    pid->measure = 0.0f;
    pid->err = 0.0f;
    pid->pout = 0.0f;
    pid->iout = 0.0f;
    pid->dout = 0.0f;
    pid->last_dout = 0.0f;
    pid->out = 0.0f;
}


/* 计数位置环串级惯性速率环 */
static float gimbal_mec_yaw_calc(gimbal_t *gimbal)
{
    float err_count;
    float deadzone_count;
    float control_target_count;
    float measure_count;
    float ref_target_count;
    float speed_limit;
    float brake_limit;
    float near_rate_limit;
    float max_rate = (Board_Rx_Info.state_pkt.r_turn_active != 0u) ?
        GIMBAL_R_TURN_YAW_MAX_RATE_DEG_S : gimbal_tune.mec_yaw_max_rate_deg_s;
    float speed_command;
    float speed_target;
    float rate_command;
    float rate_measure;
    float rate_feedback;
    float chassis_rate;
    float relative_rate;
    float ff_target;
    float ff_step;
    float gyro_dir;
    float torque;
    static uint8_t gyro_was_online; /* IMU 恢复沿检测，用于防突跳 */

    /* IMU失联清环并卸力 */
    if (imu_dev.work_state.dev_state != DEV_ONLINE)
    {
        gimbal_mec_yaw_pid_reset(&gimbal_mec_yaw_pos_pid);
        gimbal_mec_yaw_pid_reset(&gimbal_mec_yaw_rate_pid);
        gimbal_mec_yaw_ff_blend = 0.0f;
        gimbal_mec_yaw_chassis_rate_lpf = 0.0f;
        gimbal_mec_yaw_hard_hold = 0u;
        gyro_was_online = 0u;
        return 0.0f;
    }

    /* 最短角误差换算为计数 */
    err_count = gimbal_wrap_deg(gimbal->pid_info.yaw_target -
                                gimbal->base_info.yaw_mec_angle) *
                GIMBAL_DEG_TO_COUNT;

    /* 软死区抑制小角误差。 */
    deadzone_count = gimbal_tune.mec_hold_deadzone_count;
    control_target_count = err_count;
    if (err_count > deadzone_count)       { control_target_count -= deadzone_count; }
    else if (err_count < -deadzone_count) { control_target_count += deadzone_count; }
    else                                  { control_target_count = 0.0f; }

    /* 剩余角度约束制动速度 */
    speed_limit = max_rate;
    brake_limit = gimbal_mec_speed_limit(
        gimbal_abs(control_target_count) * GIMBAL_COUNT_TO_DEG,
        max_rate,
        gimbal_tune.mec_yaw_decel_rad_s2);
    if (speed_limit > brake_limit)
    {
        speed_limit = brake_limit;
    }
    near_rate_limit = GIMBAL_MEC_YAW_NEAR_RATE_KP *
                      gimbal_abs(control_target_count) * GIMBAL_COUNT_TO_DEG;
    if (speed_limit > near_rate_limit)
    {
        speed_limit = near_rate_limit;
    }

    /* 目标与反馈使用同一计数基准 */
    measure_count = gimbal->base_info.yaw_mec_angle * GIMBAL_DEG_TO_COUNT;
    ref_target_count = measure_count + control_target_count;

    gimbal_mec_yaw_pos_pid.kp = gimbal_tune.mec_hold_kp_nm_per_deg;
    gimbal_mec_yaw_pos_pid.ki = gimbal_tune.mec_hold_ki_nm_per_deg;
    gimbal_mec_yaw_pos_pid.kd = gimbal_tune.mec_hold_kd_nm_per_deg;
    gimbal_mec_yaw_pos_pid.integral_max = gimbal_tune.mec_hold_ki_limit;
    gimbal_mec_yaw_pos_pid.out_max = speed_limit;

    speed_command = gimbal_pid_calc(&gimbal_mec_yaw_pos_pid,
                                    ref_target_count,
                                    measure_count,
                                    GIMBAL_CONTROL_PERIOD_S);

    /* 外环输出换算为deg/s */
    speed_target = speed_command * GIMBAL_COUNT_TO_DEG_S;

    /* 内环使用IMU角速度反馈 */
    gyro_dir = gimbal_tune.mec_yaw_gyro_direction;
    rate_measure = gyro_dir * gimbal->base_info.yaw_imu_speed;
    relative_rate = gimbal->base_info.yaw_mec_speed * GIMBAL_RAD_TO_DEG;
    chassis_rate = rate_measure - relative_rate;

    /* 底盘估速保留原始反馈 */

    gimbal_mec_yaw_chassis_rate_lpf += GIMBAL_MEC_YAW_FF_FILTER_ALPHA *
        (chassis_rate - gimbal_mec_yaw_chassis_rate_lpf);
    ff_target = gimbal_clamp(
        (gimbal_abs(gimbal_mec_yaw_chassis_rate_lpf) - GIMBAL_MEC_YAW_FF_OFF_DPS) /
        (GIMBAL_MEC_YAW_FF_FULL_DPS - GIMBAL_MEC_YAW_FF_OFF_DPS),
        0.0f, 1.0f) * GIMBAL_MEC_YAW_FF_MAX_GAIN;
    ff_step = (ff_target < gimbal_mec_yaw_ff_blend) ?
              GIMBAL_MEC_YAW_FF_FALL_STEP : GIMBAL_MEC_YAW_FF_BLEND_STEP;
    gimbal_mec_yaw_ff_blend = gimbal_ramp(
        gimbal_mec_yaw_ff_blend, ff_target, ff_step);
    rate_command = speed_target + gimbal_mec_yaw_ff_blend *
                   gimbal_mec_yaw_chassis_rate_lpf;

#if GIMBAL_MEC_YAW_HARD_HOLD_ENABLE
    if (gimbal_abs(gimbal_mec_yaw_chassis_rate_lpf) < GIMBAL_MEC_YAW_FF_OFF_DPS)
    {
        /* 静止边界留滞回。 */
        gimbal_mec_yaw_hard_hold = (gimbal_mec_yaw_hard_hold != 0u) ?
            (gimbal_abs(err_count) <
             GIMBAL_MEC_YAW_HOLD_EXIT_DEG * GIMBAL_DEG_TO_COUNT) :
            (gimbal_abs(err_count) <= deadzone_count);
    }
    else
    {
        gimbal_mec_yaw_hard_hold = 0u;
    }

    if (gimbal_mec_yaw_hard_hold != 0u)
    {
        gimbal_mec_yaw_pid_reset(&gimbal_mec_yaw_pos_pid);
        gimbal_mec_yaw_pid_reset(&gimbal_mec_yaw_rate_pid);
        gimbal_mec_yaw_ff_blend = 0.0f;
        gyro_was_online = 0u;
        return 0.0f;
    }
#else
    /* 关闭卸力时保留双环控制 */
    gimbal_mec_yaw_hard_hold = 0u;
#endif

    gimbal_mec_yaw_rate_pid.kp = gimbal_tune.mec_hold_rate_kp_nm_per_dps;
    gimbal_mec_yaw_rate_pid.ki = gimbal_tune.mec_hold_rate_ki_nm_per_dps;
    gimbal_mec_yaw_rate_pid.kd = gimbal_tune.mec_hold_rate_kd_nm_per_dps;
    gimbal_mec_yaw_rate_pid.integral_max = gimbal_tune.mec_hold_rate_ki_limit;
    gimbal_mec_yaw_rate_pid.out_max = gimbal_tune.mec_hold_rate_out_max_nm;

    /* 速率死区仅用于内环反馈 */
    rate_feedback = rate_measure;
    if (gimbal_abs(rate_feedback) < gimbal_tune.mec_hold_rate_deadband_dps)
    {
        rate_feedback = 0.0f;
    }

    /* 恢复沿对齐误差防微分突变 */
    if (gyro_was_online == 0u)
    {
        gimbal_mec_yaw_pid_reset(&gimbal_mec_yaw_pos_pid);
        gimbal_mec_yaw_pid_reset(&gimbal_mec_yaw_rate_pid);
        gimbal_mec_yaw_pos_pid.last_err = control_target_count;
        gimbal_mec_yaw_rate_pid.last_err = rate_command - rate_feedback;
        gyro_was_online = 1u;
    }

    torque = gimbal_pid_calc(&gimbal_mec_yaw_rate_pid,
                             rate_command,
                             rate_feedback,
                             GIMBAL_CONTROL_PERIOD_S);

    return gimbal_clamp(torque,
                        -gimbal_tune.mec_hold_rate_out_max_nm,
                        gimbal_tune.mec_hold_rate_out_max_nm);
}

/* 可选掉头串级控制 */
static float gimbal_mec_yaw_turn_calc(gimbal_t *gimbal)
{
    return gimbal_clamp(
        gimbal_init_pid_calc(&gimbal->pid_info.yaw_turn_outer,
                             &gimbal->pid_info.yaw_turn_inner,
                             gimbal->pid_info.yaw_target,
                             gimbal->base_info.yaw_mec_angle,
                             gimbal->base_info.yaw_mec_speed,
                             GIMBAL_TURN_YAW_MAX_RATE_DEG_S,
                             GIMBAL_TURN_YAW_DECEL_RAD_S2,
                             GIMBAL_TURN_SPEED_LIMIT_ENABLE,
                             1u),
        -GIMBAL_TURN_YAW_TORQUE_LIMIT_NM,
        GIMBAL_TURN_YAW_TORQUE_LIMIT_NM);
}

/* 模式闭环计算后统一限矩 */
static void gimbal_calc_output(gimbal_t *gimbal)
{
    float gravity = (gimbal->gimbal_mode == G_INIT) ?
                    gimbal_init_gravity_compensation(gimbal) :
                    gimbal_gravity_compensation(gimbal);
    gimbal->base_info.gravity_f = gravity;


    switch (gimbal->gimbal_mode)
    {
    case G_INIT: /* 上电归中 */
        gimbal_update_init(gimbal);
        gimbal->base_info.output_gimbal_p = gimbal_clamp(
            gimbal_init_pid_calc(&gimbal->pid_info.pitch_init_outer,
                                 &gimbal->pid_info.pitch_init_inner,
                                 gimbal->pid_info.pitch_target,
                                 gimbal->base_info.pitch_mec_angle,
                                 gimbal->base_info.pitch_mec_speed,
                                 gimbal->init_info.pitch_max_rate_deg_s,
                                 gimbal->init_info.pitch_decel_rad_s2,
                                 GIMBAL_INIT_SPEED_LIMIT_ENABLE,
                                 0u) + gravity,
            -gimbal->init_info.pitch_torque_limit_nm,
            gimbal->init_info.pitch_torque_limit_nm);

        gimbal->base_info.output_gimbal_y = gimbal_clamp(
            gimbal_init_pid_calc(&gimbal->pid_info.yaw_init_outer,
                                 &gimbal->pid_info.yaw_init_inner,
                                 gimbal->pid_info.yaw_target,
                                 gimbal->base_info.yaw_mec_angle,
                                 gimbal->base_info.yaw_mec_speed,
                                 gimbal->init_info.yaw_max_rate_deg_s,
                                 gimbal->init_info.yaw_decel_rad_s2,
                                 GIMBAL_INIT_SPEED_LIMIT_ENABLE,
                                 3u),
            -gimbal->init_info.yaw_torque_limit_nm,
            gimbal->init_info.yaw_torque_limit_nm);
        break;
    case G_MEC: /* 机械模式复用Pitch速控 */
    {
        float yaw_out;

        /* 机械与速控共用Pitch内环 */
        gimbal_update_rate_targets(gimbal);

        gimbal->base_info.output_gimbal_p =
            all_pid_calc(NULL,
                         &gimbal->pid_info.pitch_gyro_inner,
                         gimbal->feedforward.pitch_rate_target_deg_s,
                         0.0f,
                         gimbal->base_info.pitch_mec_speed,
                         0.0f,
                         GIMBAL_RAD_TO_DEG,
                         0) + gravity;

        /* 按开关选择机械定位律 */
#if GIMBAL_MEC_YAW_USE_TURN_PATH
        if (gimbal_abs(gimbal_wrap_deg(gimbal->pid_info.yaw_target -
                                       gimbal->base_info.yaw_mec_angle)) >
            GIMBAL_TURN_ENTER_ERR_DEG)
        {
            gimbal->base_info.output_gimbal_y = gimbal_mec_yaw_turn_calc(gimbal);
            break;
        }
#endif
        yaw_out = gimbal_mec_yaw_calc(gimbal);
        gimbal->base_info.output_gimbal_y = gimbal_ramp(
            gimbal->base_info.output_gimbal_y,
            yaw_out,
            gimbal_tune.mec_yaw_torque_step_nm);
        break;
    }
    case G_GYRO:
        // Pitch 陀螺仪环
        gimbal->base_info.output_gimbal_p =
            all_pid_calc(&gimbal->pid_info.pitch_gyro_outer,
                         &gimbal->pid_info.pitch_gyro_inner,
                         gimbal->pid_info.pitch_target,
                         gimbal->base_info.pitch_imu_angle,
                         gimbal->base_info.pitch_imu_speed,
                         gimbal->feedforward.pitch_rate_cmd_deg_s,
                         1.0f, 
                         0) + gravity;

        // Yaw 陀螺仪环
        gimbal->base_info.output_gimbal_y =
            all_pid_calc(&gimbal->pid_info.yaw_gyro_outer,
                         &gimbal->pid_info.yaw_gyro_inner,
                         gimbal->pid_info.yaw_target,
                         gimbal->base_info.yaw_imu_angle,
                         gimbal->base_info.yaw_imu_speed,
                         gimbal->feedforward.yaw_rate_cmd_deg_s,
                         1.0f, 
                         3);
        break;

    case G_AUTO:
        gimbal->base_info.output_gimbal_p =
            all_pid_calc(&gimbal->pid_info.pitch_gyro_outer,
                         &gimbal->pid_info.pitch_gyro_inner,
                         gimbal->pid_info.pitch_target,
                         gimbal->base_info.pitch_imu_angle,
                         gimbal->base_info.pitch_imu_speed,
                         0.0f,
                         1.0f,
                         0) + gravity;

        gimbal->base_info.output_gimbal_y =
            all_pid_calc(&gimbal->pid_info.yaw_gyro_outer,
                         &gimbal->pid_info.yaw_gyro_inner,
                         gimbal->pid_info.yaw_target,
                         gimbal->base_info.yaw_imu_angle,
                         gimbal->base_info.yaw_imu_speed,
                         0.0f,
                         1.0f,
                         3);
        break;

    case G_RATE:
        gimbal_update_rate_targets(gimbal);

        gimbal->base_info.output_gimbal_p =
            all_pid_calc(NULL,
                         &gimbal->pid_info.pitch_gyro_inner,
                         gimbal->feedforward.pitch_rate_target_deg_s,
                         0.0f,
                         gimbal->base_info.pitch_mec_speed,
                         0.0f,
                         GIMBAL_RAD_TO_DEG,
                         0) + gravity;

        gimbal->base_info.output_gimbal_y =
            all_pid_calc(NULL,
                         &gimbal->pid_info.yaw_gyro_inner,
                         gimbal->feedforward.yaw_rate_target_deg_s,
                         0.0f,
                         gimbal->base_info.yaw_imu_speed,
                         0.0f,
                         1.0f,
                         0);
        break;

    case G_SLEEP:// 休眠关闭两轴输出
    default:
        gimbal->base_info.output_gimbal_p = 0.0f;
        gimbal->base_info.output_gimbal_y = 0.0f;
        break;
    }


    /* 硬件输出力矩限幅 */
    if (gimbal->gimbal_mode != G_SLEEP)
    {
        gimbal->base_info.output_gimbal_p += /* 叠加重力 */
            gimbal->feedforward.pitch_torque_ff_nm;
        gimbal->base_info.output_gimbal_y += /* 叠加前馈 */
            gimbal->feedforward.yaw_torque_ff_nm;

        gimbal->base_info.output_gimbal_p = /* 力矩限幅 */
            gimbal_clamp(gimbal->base_info.output_gimbal_p,
                         -gimbal_tune.pitch_torque_limit_nm,
                         gimbal_tune.pitch_torque_limit_nm);
        gimbal->base_info.output_gimbal_y = /* 力矩限幅 */
            gimbal_clamp(gimbal->base_info.output_gimbal_y,
                         -gimbal_tune.yaw_torque_limit_nm,
                         gimbal_tune.yaw_torque_limit_nm);
    }
    else
    {
        gimbal->base_info.output_gimbal_p = 0.0f;
        gimbal->base_info.output_gimbal_y = 0.0f;
    }
}

/* 初始化设备与控制参数 */
void Gimbal_Init(gimbal_t *gimbal)
{
    if (gimbal == NULL) return;

    /* 运行参数支持在线调节 */
    gimbal_tune.gravity_enable = GIMBAL_GRAVITY_ENABLE;
    gimbal_tune.gravity_k_nm = GIMBAL_GRAVITY_K_NM;
    gimbal_tune.gravity_b_nm = GIMBAL_GRAVITY_B_NM;
    gimbal_tune.gravity_sign = GIMBAL_GRAVITY_SIGN;
    gimbal_tune.gravity_middle_deg = GIMBAL_GRAVITY_MIDDLE_DEG;
    gimbal_tune.pitch_torque_limit_nm = GIMBAL_TORQUE_LIMIT;
    gimbal_tune.yaw_torque_limit_nm = GIMBAL_TORQUE_LIMIT;
    gimbal_tune.yaw_hold_kp = GIMBAL_YAW_HOLD_KP;
    gimbal_tune.yaw_hold_ki = GIMBAL_YAW_HOLD_KI;
    gimbal_tune.yaw_hold_integral_max = GIMBAL_YAW_HOLD_INTEGRAL_MAX;
    gimbal_tune.yaw_hold_out_max = GIMBAL_YAW_HOLD_OUT_MAX;
    gimbal_tune.pitch_hold_kp = GIMBAL_PITCH_HOLD_KP;
    gimbal_tune.pitch_hold_ki = GIMBAL_PITCH_HOLD_KI;
    gimbal_tune.pitch_hold_integral_max = GIMBAL_PITCH_HOLD_INTEGRAL_MAX;
    gimbal_tune.pitch_hold_out_max = GIMBAL_PITCH_HOLD_OUT_MAX;
    gimbal_tune.rate_hold_enter_deg_s = GIMBAL_RATE_HOLD_ENTER_DEG_S;
    gimbal_tune.rate_hold_exit_deg_s = GIMBAL_RATE_HOLD_EXIT_DEG_S;
    gimbal_tune.pitch_manual_rate_max_deg_s = GIMBAL_MANUAL_PITCH_RATE_DEG_S;
    gimbal_tune.yaw_manual_rate_max_deg_s = GIMBAL_MANUAL_YAW_RATE_DEG_S;
    gimbal_tune.manual_pitch_sign = GIMBAL_MANUAL_PITCH_SIGN;
    gimbal_tune.manual_yaw_sign = GIMBAL_MANUAL_YAW_SIGN;
    gimbal_tune.mouse_yaw_deg_per_count = GIMBAL_MOUSE_YAW_DEG_PER_COUNT;
    gimbal_tune.mouse_pitch_deg_per_count = GIMBAL_MOUSE_PITCH_DEG_PER_COUNT;
    gimbal_tune.mouse_yaw_sign = GIMBAL_MOUSE_YAW_SIGN;
    gimbal_tune.mouse_pitch_sign = GIMBAL_MOUSE_PITCH_SIGN;
    gimbal_tune.mouse_rate_ff_dps_per_count = GIMBAL_MOUSE_RATE_FF_DPS_PER_COUNT;
    gimbal_tune.mouse_deadband_count = GIMBAL_MOUSE_DEADBAND_COUNT;
    /* 兼容字段不参与当前控制 */
    gimbal_tune.mec_yaw_max_rate_deg_s = GIMBAL_MEC_YAW_MAX_RATE_DEG_S;
    gimbal_tune.mec_yaw_decel_rad_s2 = GIMBAL_MEC_YAW_DECEL_RAD_S2;
    gimbal_tune.mec_yaw_deadband_deg = GIMBAL_MEC_ERR_DEADBAND_DEG;
    gimbal_tune.mec_yaw_torque_step_nm = GIMBAL_MEC_YAW_TORQUE_STEP_NM;
    gimbal_tune.mec_yaw_friction_ff_nm = GIMBAL_MEC_YAW_FRICTION_FF_NM;
    gimbal_tune.mec_hold_full_err_deg = GIMBAL_MEC_HOLD_FULL_ERR_DEG;
    gimbal_tune.mec_hold_enter_err_deg = GIMBAL_MEC_HOLD_ENTER_ERR_DEG;
    /* 机械定位双环参数 */
    gimbal_tune.mec_hold_kp_nm_per_deg = GIMBAL_MEC_HOLD_KP_NM_PER_DEG;
    gimbal_tune.mec_hold_ki_nm_per_deg = GIMBAL_MEC_HOLD_KI_NM_PER_DEG;
    gimbal_tune.mec_hold_kd_nm_per_deg = GIMBAL_MEC_HOLD_KD_NM_PER_DEG;
    gimbal_tune.mec_hold_ki_limit = GIMBAL_MEC_HOLD_KI_LIMIT;
    gimbal_tune.mec_hold_deadzone_count = GIMBAL_MEC_HOLD_DEADZONE_DEG *
                                          GIMBAL_DEG_TO_COUNT;
    gimbal_tune.mec_hold_rate_kp_nm_per_dps = GIMBAL_MEC_HOLD_RATE_KP_NM_PER_DPS;
    gimbal_tune.mec_hold_rate_ki_nm_per_dps = GIMBAL_MEC_HOLD_RATE_KI_NM_PER_DPS;
    gimbal_tune.mec_hold_rate_kd_nm_per_dps = GIMBAL_MEC_HOLD_RATE_KD_NM_PER_DPS;
    gimbal_tune.mec_hold_rate_ki_limit = GIMBAL_MEC_HOLD_RATE_KI_LIMIT;
    gimbal_tune.mec_hold_rate_out_max_nm = GIMBAL_MEC_HOLD_RATE_OUT_MAX_NM;
    gimbal_tune.mec_hold_rate_deadband_dps = GIMBAL_MEC_HOLD_RATE_DEADBAND_DPS;
    gimbal_tune.mec_yaw_gyro_direction = 1.0f; /* IMU与机械角方向符号 */

    /* 绑定电机驱动 */
    gimbal->pitch_motor = &dm_motor[PITCH];
    gimbal->yaw_motor = &dm_motor[YAW];
    gimbal->init = Gimbal_Init;
    gimbal->work = Gimbal_Work;

    gimbal->gimbal_mode = G_SLEEP;
    gimbal->last_gimbal_mode = G_SLEEP;

    /* 归中参数初始化 */
    gimbal->init_info.init_flag = 0;
    gimbal->init_info.init_time = 0;
    gimbal->init_info.init_time_max = GIMBAL_INIT_TIMEOUT_MS;
    gimbal->init_info.pitch_angle_tolerance = GIMBAL_INIT_PITCH_TOL_DEG;
    gimbal->init_info.yaw_angle_tolerance = GIMBAL_INIT_YAW_TOL_DEG;
    gimbal->init_info.pitch_ramp_step = GIMBAL_INIT_PITCH_RAMP_DEG_PER_MS;
    gimbal->init_info.yaw_ramp_step = GIMBAL_INIT_YAW_RAMP_DEG_PER_MS;
    gimbal->init_info.mode_transition_active = 0;
    gimbal->init_info.mode_pitch_ramp_step = 0.1f;
    gimbal->init_info.mode_yaw_ramp_step = 0.1f;
    gimbal->init_info.pitch_speed_tolerance = GIMBAL_INIT_PITCH_SPEED_TOL_RAD_S;
    gimbal->init_info.yaw_speed_tolerance = GIMBAL_INIT_YAW_SPEED_TOL_RAD_S;
    gimbal->init_info.stable_time = 0;
    gimbal->init_info.stable_time_max = GIMBAL_INIT_STABLE_MS;
    gimbal->init_info.yaw_home_deg = GIMBAL_INIT_YAW_HOME_DEG;
    gimbal->init_info.pitch_home_deg = GIMBAL_INIT_PITCH_HOME_DEG;
    gimbal->init_info.yaw_max_rate_deg_s = GIMBAL_INIT_YAW_MAX_RATE_DEG_S;
    gimbal->init_info.pitch_max_rate_deg_s = GIMBAL_INIT_PITCH_MAX_RATE_DEG_S;
    gimbal->init_info.yaw_decel_rad_s2 = GIMBAL_INIT_YAW_DECEL_RAD_S2;
    gimbal->init_info.pitch_decel_rad_s2 = GIMBAL_INIT_PITCH_DECEL_RAD_S2;
    gimbal->init_info.yaw_torque_limit_nm = GIMBAL_INIT_YAW_TORQUE_LIMIT_NM;
    gimbal->init_info.pitch_torque_limit_nm = GIMBAL_INIT_PITCH_TORQUE_LIMIT_NM;
    gimbal->init_info.pitch_gravity_enable = GIMBAL_INIT_PITCH_GRAVITY_ENABLE;
    gimbal->init_info.pitch_gravity_k_nm = GIMBAL_INIT_PITCH_GRAVITY_K_NM;
    gimbal->init_info.pitch_gravity_b_nm = GIMBAL_INIT_PITCH_GRAVITY_B_NM;
    gimbal->init_info.pitch_gravity_sign = GIMBAL_INIT_PITCH_GRAVITY_SIGN;
    gimbal->init_info.pitch_gravity_middle_deg = GIMBAL_INIT_PITCH_GRAVITY_MIDDLE_DEG;

    gimbal->feedforward.yaw_rate_cmd_deg_s = 0.0f;
    gimbal->feedforward.pitch_rate_cmd_deg_s = 0.0f;
    gimbal->feedforward.yaw_rate_target_deg_s = 0.0f;
    gimbal->feedforward.pitch_rate_target_deg_s = 0.0f;
    gimbal->feedforward.yaw_torque_ff_nm = 0.0f;
    gimbal->feedforward.pitch_torque_ff_nm = 0.0f;
    gimbal->feedforward.yaw_hold_angle_deg = 0.0f;
    gimbal->feedforward.pitch_hold_angle_deg = 0.0f;
    gimbal->feedforward.pitch_zero_hold_last = 0u;
    gimbal_yaw_release_reset(gimbal);
    gimbal->feedforward.manual_source = GIMBAL_INPUT_RC;
    gimbal->feedforward.manual_source_changed = 0u;
    gimbal->feedforward.mouse_dx_counts = 0.0f;
    gimbal->feedforward.mouse_dy_counts = 0.0f;

    gimbal_pid_init(gimbal);
    gimbal->base_info.output_gimbal_p = 0.0f;
    gimbal->base_info.output_gimbal_y = 0.0f;
    gimbal->base_info.yaw_mec_speed_lpf = 0.0f;
    gimbal_mec_yaw_pid_reset(&gimbal_mec_yaw_pos_pid);   /* 机械 Yaw 双环专用 */
    gimbal_mec_yaw_pid_reset(&gimbal_mec_yaw_rate_pid);
    gimbal_mec_yaw_ff_blend = 0.0f;
    gimbal_mec_yaw_chassis_rate_lpf = 0.0f;
    gimbal_mec_yaw_hard_hold = 0u;
}

/* 控制任务周期入口 */
void Gimbal_Work(gimbal_t *gimbal)
{
    gimbal_mode_e selected_mode;

    if (gimbal == NULL) return;

    //  采集更新
    gimbal_info_update(gimbal);
    selected_mode = gimbal_select_mode(gimbal);

    //  模式切换保护
    if (selected_mode != gimbal->gimbal_mode)
    {
        gimbal_yaw_release_reset(gimbal);
        gimbal_mec_yaw_ff_blend = 0.0f;
        gimbal_mec_yaw_chassis_rate_lpf = 0.0f;
        gimbal_mec_yaw_hard_hold = 0u;
        gimbal->gimbal_mode = selected_mode;
        gimbal_clear_all_pid(gimbal);
        gimbal->last_gimbal_mode = selected_mode;
        gimbal->init_info.mode_transition_active = 1;
        gimbal->init_info.stable_time = 0;

        // 切换目标对齐当前反馈
        if ((selected_mode == G_GYRO) || (selected_mode == G_AUTO))
        {
            gimbal->pid_info.yaw_target = gimbal->base_info.yaw_imu_angle;
            gimbal->pid_info.pitch_target = gimbal->base_info.pitch_imu_angle;
        }
        else
        {
            gimbal->pid_info.yaw_target = gimbal->base_info.yaw_mec_angle;
            gimbal->pid_info.pitch_target = gimbal->base_info.pitch_mec_angle;
        }

        gimbal->feedforward.yaw_hold_angle_deg = gimbal->base_info.yaw_imu_angle;
        gimbal->feedforward.pitch_hold_angle_deg =
            ((Lift_IsPitchZeroHoldActive() != 0u) ||
             (gimbal->feedforward.pitch_zero_hold_last != 0u)) ?
            0.0f : gimbal->base_info.pitch_mec_angle;

        // 切换帧卸力后斜坡接管
        gimbal->base_info.output_gimbal_p = 0.0f;
        gimbal->base_info.output_gimbal_y = 0.0f;
    }
    else
    {
        // 闭环运算
        gimbal_update_targets(gimbal);
        gimbal_calc_output(gimbal);
    }

    //  休眠下电置零
    if (gimbal->gimbal_mode == G_SLEEP)
    {
        gimbal->feedforward.pitch_zero_hold_last = 0u;
        gimbal->base_info.output_gimbal_p = 0.0f;
        gimbal->base_info.output_gimbal_y = 0.0f;
    }

    //  写入电机底层发送缓冲区
    gimbal->pitch_motor->tx_info->torque = gimbal->base_info.output_gimbal_p;
    gimbal->yaw_motor->tx_info->torque = gimbal->base_info.output_gimbal_y;
}



