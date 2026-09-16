#include "gimbal.h"

#include <stddef.h>
#include <string.h>
#include <math.h>

gimbal_t Gimbal;

static float clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static void pid_reset(gimbal_pid_t *pid)
{
    pid->integral = 0.0f;
    pid->last_measurement = 0.0f;
    pid->has_last_measurement = false;
}

static void pid_configure(gimbal_pid_t *pid,
                          float kp,
                          float ki,
                          float kd,
                          float integral_limit,
                          float output_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
    pid_reset(pid);
}

static float pid_update(gimbal_pid_t *pid, float target, float measurement, float dt_s)
{
    float error = target - measurement;
    float derivative = 0.0f;
    float output;

    if (pid->has_last_measurement)
    {
        derivative = -(measurement - pid->last_measurement) / dt_s;
    }

    pid->integral += error * dt_s;
    pid->integral = clamp_float(pid->integral, -pid->integral_limit, pid->integral_limit);

    output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
    output = clamp_float(output, -pid->output_limit, pid->output_limit);

    pid->last_measurement = measurement;
    pid->has_last_measurement = true;

    return output;
}

static void reset_controllers(gimbal_t *gimbal)
{
    pid_reset(&gimbal->yaw_angle_pid);
    pid_reset(&gimbal->yaw_rate_pid);
    pid_reset(&gimbal->pitch_angle_pid);
    pid_reset(&gimbal->pitch_rate_pid);
}

static void configure_mode_pids(gimbal_t *gimbal)
{
    if (gimbal->mode == G_GYRO)
    {
        pid_configure(&gimbal->yaw_angle_pid,
                      GIMBAL_YAW_GYRO_ANGLE_KP,
                      GIMBAL_YAW_GYRO_ANGLE_KI,
                      GIMBAL_YAW_GYRO_ANGLE_KD,
                      GIMBAL_YAW_RATE_LIMIT,
                      GIMBAL_YAW_RATE_LIMIT);
        pid_configure(&gimbal->yaw_rate_pid,
                      GIMBAL_YAW_GYRO_RATE_KP,
                      GIMBAL_YAW_GYRO_RATE_KI,
                      GIMBAL_YAW_GYRO_RATE_KD,
                      GIMBAL_TORQUE_LIMIT,
                      GIMBAL_TORQUE_LIMIT);
        pid_configure(&gimbal->pitch_angle_pid,
                      GIMBAL_PITCH_GYRO_ANGLE_KP,
                      GIMBAL_PITCH_GYRO_ANGLE_KI,
                      GIMBAL_PITCH_GYRO_ANGLE_KD,
                      GIMBAL_PITCH_RATE_LIMIT,
                      GIMBAL_PITCH_RATE_LIMIT);
        pid_configure(&gimbal->pitch_rate_pid,
                      GIMBAL_PITCH_GYRO_RATE_KP,
                      GIMBAL_PITCH_GYRO_RATE_KI,
                      GIMBAL_PITCH_GYRO_RATE_KD,
                      GIMBAL_TORQUE_LIMIT,
                      GIMBAL_TORQUE_LIMIT);
    }
    else
    {
        pid_configure(&gimbal->yaw_angle_pid,
                      GIMBAL_YAW_MEC_ANGLE_KP,
                      GIMBAL_YAW_MEC_ANGLE_KI,
                      GIMBAL_YAW_MEC_ANGLE_KD,
                      GIMBAL_YAW_RATE_LIMIT,
                      GIMBAL_YAW_RATE_LIMIT);
        pid_configure(&gimbal->yaw_rate_pid,
                      GIMBAL_YAW_MEC_RATE_KP,
                      GIMBAL_YAW_MEC_RATE_KI,
                      GIMBAL_YAW_MEC_RATE_KD,
                      GIMBAL_TORQUE_LIMIT,
                      GIMBAL_TORQUE_LIMIT);
        pid_configure(&gimbal->pitch_angle_pid,
                      GIMBAL_PITCH_MEC_ANGLE_KP,
                      GIMBAL_PITCH_MEC_ANGLE_KI,
                      GIMBAL_PITCH_MEC_ANGLE_KD,
                      GIMBAL_PITCH_RATE_LIMIT,
                      GIMBAL_PITCH_RATE_LIMIT);
        pid_configure(&gimbal->pitch_rate_pid,
                      GIMBAL_PITCH_MEC_RATE_KP,
                      GIMBAL_PITCH_MEC_RATE_KI,
                      GIMBAL_PITCH_MEC_RATE_KD,
                      GIMBAL_TORQUE_LIMIT,
                      GIMBAL_TORQUE_LIMIT);
    }
}

static bool values_are_finite(const gimbal_t *gimbal, const gimbal_input_t *inputs)
{
    if (!isfinite(inputs->yaw_mec_angle) ||
        !isfinite(inputs->yaw_mec_speed) ||
        !isfinite(inputs->pitch_mec_angle) ||
        !isfinite(inputs->pitch_mec_speed) ||
        !isfinite(inputs->yaw_target) ||
        !isfinite(inputs->pitch_target))
    {
        return false;
    }

    if ((gimbal->mode == G_GYRO) &&
        (!isfinite(inputs->yaw_imu_angle) ||
         !isfinite(inputs->yaw_imu_speed) ||
         !isfinite(inputs->pitch_imu_angle) ||
         !isfinite(inputs->pitch_imu_speed)))
    {
        return false;
    }

    return true;
}

static void capture_targets(gimbal_t *gimbal, const gimbal_input_t *inputs)
{
    switch (gimbal->mode)
    {
    case G_MEC:
    case G_INIT:
        gimbal->yaw_target = inputs->yaw_mec_angle;
        gimbal->pitch_target = inputs->pitch_mec_angle;
        break;

    case G_GYRO:
        gimbal->yaw_target = inputs->yaw_imu_angle;
        gimbal->pitch_target = inputs->pitch_imu_angle;
        break;

    case G_SLEEP:
    default:
        gimbal->yaw_target = 0.0f;
        gimbal->pitch_target = 0.0f;
        break;
    }
}

static float relative_target(float target, float measurement)
{
    return measurement + Gimbal_WrapPi(target - measurement);
}

static float protect_pitch_target(float target)
{
    return clamp_float(target, GIMBAL_PITCH_MIN_RAD, GIMBAL_PITCH_MAX_RAD);
}

static void update_targets(gimbal_t *gimbal, const gimbal_input_t *inputs)
{
    float raw_yaw_target = gimbal->debug_targets_enabled ? gimbal->debug_yaw_target : inputs->yaw_target;
    float raw_pitch_target = gimbal->debug_targets_enabled ? gimbal->debug_pitch_target : inputs->pitch_target;

    switch (gimbal->mode)
    {
    case G_MEC:
        gimbal->yaw_target = relative_target(raw_yaw_target, inputs->yaw_mec_angle);
        gimbal->pitch_target = protect_pitch_target(raw_pitch_target);
        break;

    case G_GYRO:
        gimbal->yaw_target = relative_target(raw_yaw_target, inputs->yaw_imu_angle);
        {
            float delta = raw_pitch_target - inputs->pitch_imu_angle;
            float predicted_mechanical_angle = inputs->pitch_mec_angle + delta;

            if (predicted_mechanical_angle > GIMBAL_PITCH_MAX_RAD)
            {
                delta -= predicted_mechanical_angle - GIMBAL_PITCH_MAX_RAD;
            }
            else if (predicted_mechanical_angle < GIMBAL_PITCH_MIN_RAD)
            {
                delta += GIMBAL_PITCH_MIN_RAD - predicted_mechanical_angle;
            }

            gimbal->pitch_target = inputs->pitch_imu_angle + delta;
        }
        break;

    case G_INIT:
    case G_SLEEP:
    default:
        break;
    }
}

static bool safety_allows_control(const gimbal_t *gimbal, const gimbal_input_t *inputs)
{
    if (inputs->emergency_stop)
    {
        return false;
    }

    if (!inputs->yaw_motor_online || !inputs->pitch_motor_online)
    {
        return false;
    }

    if (!gimbal->debug_targets_enabled && !inputs->rc_online)
    {
        return false;
    }

    if ((gimbal->mode == G_GYRO) && (!inputs->imu_online || !inputs->imu_calibrated))
    {
        return false;
    }

    return values_are_finite(gimbal, inputs);
}

static void enter_sleep(gimbal_t *gimbal)
{
	if (gimbal->mode != G_SLEEP)
	{
		gimbal->safety_trip_count++;
	}

	gimbal->mode = G_SLEEP;
	gimbal->last_mode = G_SLEEP;
	gimbal->yaw_target = 0.0f;
	gimbal->pitch_target = 0.0f;
	gimbal->output.yaw_torque = 0.0f;
	gimbal->output.pitch_torque = 0.0f;
	reset_controllers(gimbal);
}

void Gimbal_Init(gimbal_t *gimbal)
{
    memset(gimbal, 0, sizeof(*gimbal));

    gimbal->mode = G_SLEEP;
    gimbal->last_mode = G_SLEEP;

    configure_mode_pids(gimbal);
}

void Gimbal_SetMode(gimbal_t *gimbal, gimbal_mode_e mode)
{
    if ((mode != G_SLEEP) && (mode != G_INIT) && (mode != G_MEC) && (mode != G_GYRO))
    {
        return;
    }

    gimbal->mode = mode;
}

void Gimbal_SetDebugTargets(gimbal_t *gimbal, bool enabled, float yaw_target, float pitch_target)
{
    gimbal->debug_targets_enabled = enabled;
    gimbal->debug_yaw_target = yaw_target;
    gimbal->debug_pitch_target = pitch_target;
}

float Gimbal_WrapPi(float angle)
{
    angle = fmodf(angle + GIMBAL_PI, GIMBAL_TWO_PI);
    if (angle < 0.0f)
    {
        angle += GIMBAL_TWO_PI;
    }

    return angle - GIMBAL_PI;
}

void Gimbal_Update(gimbal_t *gimbal, const gimbal_input_t *inputs, float dt_s, gimbal_output_t *output)
{
    float yaw_rate_target;
    float pitch_rate_target;
    float pitch_gravity_compensation;

    if ((gimbal == NULL) || (inputs == NULL) || (output == NULL))
    {
        return;
    }

    gimbal->output.yaw_torque = 0.0f;
    gimbal->output.pitch_torque = 0.0f;

    if ((dt_s < 0.0001f) || (dt_s > 0.02f) || !safety_allows_control(gimbal, inputs))
    {
        enter_sleep(gimbal);
        *output = gimbal->output;
        return;
    }

    if (gimbal->mode != gimbal->last_mode)
    {
        configure_mode_pids(gimbal);
        capture_targets(gimbal, inputs);
        gimbal->last_mode = gimbal->mode;
        gimbal->transition_ticks++;
        *output = gimbal->output;
        return;
    }

    if ((gimbal->mode == G_SLEEP) || (gimbal->mode == G_INIT))
    {
        reset_controllers(gimbal);
        *output = gimbal->output;
        return;
    }

    update_targets(gimbal, inputs);

    if (gimbal->mode == G_MEC)
    {

        yaw_rate_target = pid_update(&gimbal->yaw_angle_pid,
                                     gimbal->yaw_target,
                                     inputs->yaw_mec_angle,
                                     dt_s);
        pitch_rate_target = pid_update(&gimbal->pitch_angle_pid,
                                       gimbal->pitch_target,
                                       inputs->pitch_mec_angle,
                                       dt_s);

        gimbal->output.yaw_torque = pid_update(&gimbal->yaw_rate_pid,
                                               yaw_rate_target,
                                               inputs->yaw_mec_speed,
                                               dt_s);
        gimbal->output.pitch_torque = pid_update(&gimbal->pitch_rate_pid,
                                                 pitch_rate_target,
                                                 inputs->pitch_mec_speed,
                                                 dt_s);
    }
    else
    {

        yaw_rate_target = pid_update(&gimbal->yaw_angle_pid,
                                     gimbal->yaw_target,
                                     inputs->yaw_imu_angle,
                                     dt_s);
        pitch_rate_target = pid_update(&gimbal->pitch_angle_pid,
                                       gimbal->pitch_target,
                                       inputs->pitch_imu_angle,
                                       dt_s);

        gimbal->output.yaw_torque = pid_update(&gimbal->yaw_rate_pid,
                                               yaw_rate_target,
                                               inputs->yaw_imu_speed,
                                               dt_s);
        gimbal->output.pitch_torque = pid_update(&gimbal->pitch_rate_pid,
                                                 pitch_rate_target,
                                                 inputs->pitch_imu_speed,
                                                 dt_s);
    }

    pitch_gravity_compensation = GIMBAL_GRAVITY_COMP_NM *
                                 cosf(inputs->pitch_mec_angle - GIMBAL_GRAVITY_PHASE_RAD);

    gimbal->output.yaw_torque = clamp_float(gimbal->output.yaw_torque,
                                            -GIMBAL_TORQUE_LIMIT,
                                            GIMBAL_TORQUE_LIMIT);
    gimbal->output.pitch_torque = clamp_float(gimbal->output.pitch_torque + pitch_gravity_compensation,
                                              -GIMBAL_TORQUE_LIMIT,
                                              GIMBAL_TORQUE_LIMIT);

    *output = gimbal->output;
}
