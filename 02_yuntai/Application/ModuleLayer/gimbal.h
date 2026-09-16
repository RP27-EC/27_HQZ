#ifndef __GIMBAL_H
#define __GIMBAL_H

#include <stdbool.h>
#include <stdint.h>

#include "gimbal_config.h"

typedef enum
{
    G_SLEEP = 0,
    G_INIT,
    G_MEC,
    G_GYRO,
} gimbal_mode_e;

typedef struct
{
    bool yaw_motor_online;
    bool pitch_motor_online;
    bool imu_online;
    bool imu_calibrated;
    bool rc_online;
    bool emergency_stop;

    float yaw_mec_angle;
    float yaw_mec_speed;
    float pitch_mec_angle;
    float pitch_mec_speed;

    float yaw_imu_angle;
    float yaw_imu_speed;
    float pitch_imu_angle;
    float pitch_imu_speed;

    float yaw_target;
    float pitch_target;
} gimbal_input_t;

typedef struct
{
    float yaw_torque;
    float pitch_torque;
} gimbal_output_t;

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float integral_limit;
    float output_limit;
    float last_measurement;
    bool has_last_measurement;
} gimbal_pid_t;

typedef struct
{
    gimbal_mode_e mode;
    gimbal_mode_e last_mode;

    float yaw_target;
    float pitch_target;
    gimbal_output_t output;

    gimbal_pid_t yaw_angle_pid;
    gimbal_pid_t yaw_rate_pid;
    gimbal_pid_t pitch_angle_pid;
    gimbal_pid_t pitch_rate_pid;

    bool debug_targets_enabled;
    float debug_yaw_target;
    float debug_pitch_target;

    uint32_t transition_ticks;
    uint32_t safety_trip_count;
} gimbal_t;

extern gimbal_t Gimbal;

void Gimbal_Init(gimbal_t *gimbal);
void Gimbal_SetMode(gimbal_t *gimbal, gimbal_mode_e mode);
void Gimbal_SetDebugTargets(gimbal_t *gimbal, bool enabled, float yaw_target, float pitch_target);
void Gimbal_Update(gimbal_t *gimbal, const gimbal_input_t *inputs, float dt_s, gimbal_output_t *output);
float Gimbal_WrapPi(float angle);

#endif
