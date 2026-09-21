/* gimbal.h - 云台控制 */

#ifndef __GIMBAL_H
#define __GIMBAL_H

#include <stdint.h>

#include "PID.h"
#include "communicate.h"
#include "motor.h"
#include "rp_device_config.h"

/*
 * 云台控制层对外接口。
 *
 * 单位：
 *   1. IMU 角度：deg
 *   2. IMU 角速度：deg/s
 *   3. 电机机械角：内部使用 deg
 *   4. 电机机械角速度：rad/s
 *   5. 电机输出力矩：N*m
 */

//一些换算的常量
#define GIMBAL_PI                  3.14159265358979323846f
#define GIMBAL_TWO_PI              6.28318530717958647692f
#define GIMBAL_DEG_TO_RAD          (GIMBAL_PI / 180.0f)
#define GIMBAL_RAD_TO_DEG          (180.0f / GIMBAL_PI)

/* Yaw 机械中值 */
#define GIMBAL_YAW_MIDDLE_DEG      (-22.224138f)
/* Pitch 机械中值 */
#define GIMBAL_PITCH_MIDDLE_DEG    148.573157f

/* Pitch 机械下限 */
#define GIMBAL_PITCH_MIN_DEG       (-7.5f)
/* Pitch 机械上限 */
#define GIMBAL_PITCH_MAX_DEG       30.0f
/* 最终输出力矩限幅 */
#define GIMBAL_TORQUE_LIMIT        3.0f
/* 重力补偿开关：0 关闭，1 开启 */
#define GIMBAL_GRAVITY_ENABLE      1
/* 余弦重力补偿幅值*/
#define GIMBAL_GRAVITY_K_NM        1.1f
/* 重力补偿固定偏置 */
#define GIMBAL_GRAVITY_B_NM        0.0f
/* 重力补偿方向：当前正方向输出能抬升 Pitch 时用 1.0f，方向相反时用 -1.0f */
#define GIMBAL_GRAVITY_SIGN        1.0f
/* 重力补偿相位角 */
#define GIMBAL_GRAVITY_MIDDLE_DEG  0.0f
/* 斜坡目标判定误差阈值 */
#define GIMBAL_TARGET_ARRIVE_EPS_DEG 0.1f

/* 速控目标角速度的最大变化率 */
#define GIMBAL_RATE_CMD_RAMP_DEG_S_PER_MS 6.0f

/* 遥控器摇杆满量程值 */
#define GIMBAL_RC_AXIS_MAX                 660.0f
/* 遥控器摇杆中位死区 */
#define GIMBAL_RC_AXIS_DEADBAND            20.0f
/* 遥控器满杆时 Yaw 最大目标角速度*/
#define GIMBAL_MANUAL_YAW_RATE_DEG_S       300.0f
/* 操作手 Yaw 方向符号 */
#define GIMBAL_MANUAL_YAW_SIGN             (-1.0f)
/* 遥控器满杆时 Pitch 最大目标角速度*/
#define GIMBAL_MANUAL_PITCH_RATE_DEG_S     40.0f
/* 操作手 Pitch 方向符号*/
#define GIMBAL_MANUAL_PITCH_SIGN           (1.0f)
/* 鼠标 X 转换为 Yaw 的增益 */
#define GIMBAL_MOUSE_YAW_RATE_GAIN         1.0f
/* 鼠标 Y 输入为 Pitch 的增益 */
#define GIMBAL_MOUSE_PITCH_RATE_GAIN       1.0f
/* 速控模式回转弱角度 */
#define GIMBAL_RATE_HOLD_KP                0.5f
/* 判断操作手输入已回中的角速度阈值 */
#define GIMBAL_RATE_HOLD_DEADBAND_DEG_S    2.0f

/* 云台运行模式 */
typedef enum
{
    G_SLEEP = 0,    /* 休眠 */
    G_INIT,         /* 初始化 */
    G_MEC,          /* 机械模式 */
    G_GYRO,         /* 陀螺模式 */
    G_RATE,         /* 速控模式 */
    G_AUTO,         /* 自瞄模式 */
} gimbal_mode_e;

/* 初始化归中参数 */
typedef struct
{
    uint8_t init_flag;                 /* 0：未完成初始化，1：初始化完成 */
    uint16_t init_time;                /* 初始化已计时时间 */
    uint16_t init_time_max;            /* 初始化超时时间 */
    float pitch_angle_tolerance;       /* Pitch 到位误差阈值 */
    float yaw_angle_tolerance;         /* Yaw 到位误差阈值 */
    float pitch_ramp_step;             /* 归中时 Pitch 目标变化步长 */
    float yaw_ramp_step;               /* 归中时 Yaw 目标变化步长 */
    uint8_t mode_transition_active;    /* 模式切换目标斜坡是否进行 */
    float mode_pitch_ramp_step;        /* 模式切换时 Pitch 步长 */
    float mode_yaw_ramp_step;          /* 模式切换时 Yaw 步长 */
    float pitch_speed_tolerance;       /* Pitch 到位速度阈值 */
    float yaw_speed_tolerance;         /* Yaw 到位速度阈值 */
    uint16_t stable_time;              /* 到位条件的连续时间 */
    uint16_t stable_time_max;          /* 稳定所需的连续时间 */
} gimbal_init_info_t;

/* 云台前馈量。 */
typedef struct
{
    float yaw_rate_cmd_deg_s;       /* 操作手原始 Yaw 角速度指令 */
    float pitch_rate_cmd_deg_s;     /* 操作手原始 Pitch 角速度指令 */
    float yaw_rate_target_deg_s;    /* 速控分支使用的 Yaw 目标角速度 */
    float pitch_rate_target_deg_s;  /* 速控分支使用的 Pitch 目标角速度 */
    float yaw_rate_cmd_last_deg_s;  /* 上一周期 Yaw 指令 */
    float pitch_rate_cmd_last_deg_s;/* 上一周期 Pitch 指令 */
    float yaw_torque_ff_nm;         /* Yaw 最终力矩前馈 */
    float pitch_torque_ff_nm;       /* Pitch 最终力矩前馈 */
    float yaw_hold_angle_deg;       /* 速控回中后保持的 Yaw 角度 */
    float pitch_hold_angle_deg;     /* 速控回中后保持的 Pitch 角度 */
} gimbal_feedforward_t;

/* Runtime tuning values. Edit these in Keil Watch without reflashing. */
typedef struct
{
    volatile uint8_t gravity_enable;
    volatile float gravity_k_nm;
    volatile float gravity_b_nm;
    volatile float gravity_sign;
    volatile float gravity_middle_deg;
    volatile float pitch_torque_limit_nm;
    volatile float yaw_torque_limit_nm;
    volatile float pitch_rate_hold_kp;
    volatile float yaw_rate_hold_kp;
    volatile float pitch_manual_rate_max_deg_s;
    volatile float yaw_manual_rate_max_deg_s;
    volatile float rate_hold_deadband_deg_s;
    volatile float manual_pitch_sign;
    volatile float manual_yaw_sign;
} gimbal_tune_t;

/* 目标角与 8 路串级 PID */
typedef struct
{
    float yaw_mec_target_raw;      /* 下板 Yaw 机械目标角 */
    float pitch_mec_target_raw;    /* 下板 Pitch 机械目标角 */
    float yaw_imu_target_raw;      /* 下板 Yaw IMU 目标角 */
    float pitch_imu_target_raw;    /* 下板 Pitch IMU 目标角  */

    float yaw_target;              /* Yaw 当前限幅和斜坡后的控制目标 */
    float pitch_target;            /* Pitch 当前限幅和斜坡后的控制目标 */


    pid_ctrl_t yaw_gyro_outer;     /* Yaw 陀螺角度外环 */
    pid_ctrl_t yaw_gyro_inner;     /* Yaw IMU 角速度内环 */
    pid_ctrl_t yaw_mec_outer;      /* Yaw 机械角度外环 */
    pid_ctrl_t yaw_mec_inner;      /* Yaw 电机速度内环 */
    pid_ctrl_t pitch_gyro_outer;   /* Pitch 陀螺角度外环 */
    pid_ctrl_t pitch_gyro_inner;   /* Pitch IMU 角速度内环 */
    pid_ctrl_t pitch_mec_outer;    /* Pitch 机械角度外环 */
    pid_ctrl_t pitch_mec_inner;    /* Pitch 电机速度内环 */
} gimbal_pid_info_t;

/* 云台反馈与输出信息 */
typedef struct
{
    float yaw_imu_angle;           /* IMU Yaw 角度 */
    float yaw_imu_speed;           /* IMU Yaw 角速度*/
    float pitch_imu_angle;         /* IMU Pitch 角度 */
    float pitch_imu_speed;         /* IMU Pitch 角速度 */

    float yaw_mec_angle;           /* Yaw 电机机械角 */
    float yaw_mec_speed;           /* Yaw 电机机械角速度 */
    float pitch_mec_angle;         /* Pitch 电机机械角 */
    float pitch_mec_speed;         /* Pitch 电机机械角速度*/

    float output_gimbal_y;         /* Yaw 最终输出力矩 */
    float output_gimbal_p;         /* Pitch 最终输出力矩 */
    float gravity_f;               /* 当前 Pitch 重力补偿力矩 */
} gimbal_base_info_t;

/* 云台对象，集中保存设备、状态、参数和控制接口 */
typedef struct gimbal_class_t
{
    dm_motor_t *pitch_motor;        /* Pitch DM4310 对象 */
    dm_motor_t *yaw_motor;          /* Yaw DM4310 对象 */

    gimbal_base_info_t base_info;   /* 反馈与输出信息 */
    gimbal_pid_info_t pid_info;     /* 目标与 PID 参数 */
    gimbal_init_info_t init_info;   /* 初始化与模式参数 */
    gimbal_feedforward_t feedforward;/* 角速度前馈与力矩前馈 */

    gimbal_mode_e gimbal_mode;      /* 当前云台模式 */
    gimbal_mode_e last_gimbal_mode; /* 上一次云台模式 */

    void (*init)(struct gimbal_class_t *gimbal); /* 初始化接口 */
    void (*work)(struct gimbal_class_t *gimbal); /* 周期工作接口 */
} gimbal_t;

extern gimbal_t Gimbal;
extern gimbal_tune_t gimbal_tune;

void Gimbal_Init(gimbal_t *gimbal);
void Gimbal_Work(gimbal_t *gimbal);

#endif

