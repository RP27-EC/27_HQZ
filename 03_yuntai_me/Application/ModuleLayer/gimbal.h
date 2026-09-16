#ifndef __GIMBAL_H
#define __GIMBAL_H

#include <stdint.h>

#include "PID.h"
#include "communicate.h"
#include "motor.h"
#include "rp_device_config.h"

#define GIMBAL_PI                  3.14159265358979323846f
#define GIMBAL_TWO_PI              6.28318530717958647692f
#define GIMBAL_DEG_TO_RAD          (GIMBAL_PI / 180.0f)
#define GIMBAL_RAD_TO_DEG          (180.0f / GIMBAL_PI)

/* These zero offsets and limits must be verified on the real gimbal. */
#define GIMBAL_YAW_MIDDLE_DEG      0.0f
#define GIMBAL_PITCH_MIDDLE_DEG    0.0f
#define GIMBAL_PITCH_MIN_DEG       (-7.5f)
#define GIMBAL_PITCH_MAX_DEG       30.0f
#define GIMBAL_TORQUE_LIMIT        3.0f
#define GIMBAL_GRAVITY_K_NM        4.2072f
#define GIMBAL_GRAVITY_B_NM        (-2.496f)
#define GIMBAL_GRAVITY_MIDDLE_RAD  (2.678706762f - GIMBAL_PITCH_MIDDLE_DEG * GIMBAL_DEG_TO_RAD)
#define GIMBAL_TARGET_ARRIVE_EPS_DEG 0.1f

typedef enum
{
    G_SLEEP = 0,
    G_INIT,
    G_MEC,
    G_GYRO,
} gimbal_mode_e;

typedef struct
{
    uint8_t init_flag;
    uint16_t init_time;
    uint16_t init_time_max;
    float pitch_angle_tolerance;
    float yaw_angle_tolerance;
    float pitch_ramp_step;    
    float yaw_ramp_step;      
    uint8_t mode_transition_active;
    float mode_pitch_ramp_step;
    float mode_yaw_ramp_step;
    float pitch_speed_tolerance;
    float yaw_speed_tolerance;
    uint16_t stable_time;
    uint16_t stable_time_max;
} gimbal_init_info_t;

typedef struct
{
    float yaw_mec_target_raw;
    float pitch_mec_target_raw;
    float yaw_imu_target_raw;
    float pitch_imu_target_raw;

    float yaw_target;
    float pitch_target;

    float yaw_speed_ff;                   
    float pitch_speed_ff;                 

    pid_ctrl_t yaw_gyro_outer;
    pid_ctrl_t yaw_gyro_inner;
    pid_ctrl_t yaw_mec_outer;
    pid_ctrl_t yaw_mec_inner;
    pid_ctrl_t pitch_gyro_outer;
    pid_ctrl_t pitch_gyro_inner;
    pid_ctrl_t pitch_mec_outer;
    pid_ctrl_t pitch_mec_inner;
} gimbal_pid_info_t;

typedef struct
{
    float yaw_imu_angle;
    float yaw_imu_speed;
    float pitch_imu_angle;
    float pitch_imu_speed;

    float yaw_mec_angle;
    float yaw_mec_speed;
    float pitch_mec_angle;
    float pitch_mec_speed;

    float output_gimbal_y;
    float output_gimbal_p;
    float gravity_f;
} gimbal_base_info_t;

typedef struct gimbal_class_t
{
    Motor_DM_t *pitch_motor;
    Motor_DM_t *yaw_motor;

    gimbal_base_info_t base_info;
    gimbal_pid_info_t pid_info;
    gimbal_init_info_t init_info;

    gimbal_mode_e gimbal_mode;
    gimbal_mode_e last_gimbal_mode;

    void (*init)(struct gimbal_class_t *gimbal);
    void (*work)(struct gimbal_class_t *gimbal);
} gimbal_t;

extern gimbal_t Gimbal;

void Gimbal_Init(gimbal_t *gimbal);
void Gimbal_Work(gimbal_t *gimbal);

#endif
