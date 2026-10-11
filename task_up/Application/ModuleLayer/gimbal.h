/* gimbal.h - 云台控制 */

#ifndef __GIMBAL_H
#define __GIMBAL_H

#include <stdint.h>

#include "PID.h"
#include "communicate.h"
#include "motor.h"
#include "rp_device_config.h"
#include "gimbal_init_config.h"
#include "gimbal_turn_config.h"
#include "gimbal_rate_config.h"

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

/* 角度换算常量 */
#define GIMBAL_PI                  3.14159265358979323846f // 圆周率，rad
#define GIMBAL_TWO_PI              6.28318530717958647692f // 一圈角度，rad
#define GIMBAL_DEG_TO_RAD          (GIMBAL_PI / 180.0f) // 角度换算，rad/deg
#define GIMBAL_RAD_TO_DEG          (180.0f / GIMBAL_PI) // 角度换算，deg/rad

/* 中点与机械行程配置 */
#define GIMBAL_YAW_MIDDLE_DEG      (-22.224138f) // Yaw编码器零偏，deg
#define GIMBAL_PITCH_MIDDLE_DEG    148.573157f // Pitch编码器零偏，deg
#define GIMBAL_PITCH_MIN_DEG       (-7.5f) // Pitch机械下限，deg
#define GIMBAL_PITCH_MAX_DEG       30.0f // Pitch机械上限，deg

/* 机械 Yaw 运动与滤波配置 */
#define GIMBAL_MEC_OUTER_KD              0.1f // 机械外环微分增益，按周期
#define GIMBAL_MEC_OUTER_D_FILTER_ALPHA  0.85f // 微分历史权重，0~1
#define GIMBAL_MEC_ERR_DEADBAND_DEG      0.2f // 机械角误差死区，deg
#define GIMBAL_MEC_YAW_MAX_RATE_DEG_S    300.0f // 机械Yaw速限，deg/s
#define GIMBAL_MEC_YAW_DECEL_RAD_S2      25.0f // 机械Yaw减速度，rad/s²
#define GIMBAL_MEC_YAW_TORQUE_STEP_NM    0.08f // 兼容力矩步长，N·m/周期

#define GIMBAL_MEC_YAW_FF_OFF_DPS        25.0f // 底盘前馈起始速率，deg/s
#define GIMBAL_MEC_YAW_FF_FULL_DPS       35.0f // 底盘前馈满幅速率，deg/s
#define GIMBAL_MEC_YAW_FF_BLEND_STEP     0.02f // 前馈上升步长，0~1/周期
#define GIMBAL_MEC_YAW_FF_FALL_STEP      0.10f // 前馈下降步长，0~1/周期
#define GIMBAL_MEC_YAW_FF_FILTER_ALPHA   0.05f // 底盘估速滤波权重，0~1
#define GIMBAL_MEC_YAW_FF_MAX_GAIN       0.90f // 底盘前馈最大权重，0~1
#define GIMBAL_MEC_YAW_NEAR_RATE_KP      10.0f // 近目标速限斜率，1/s
/* 保留未接入的摩擦补偿参数 */
#define GIMBAL_MEC_YAW_FRICTION_FF_NM    0.3f // 停用摩擦前馈，N·m
/* 保留未接入的机械速度滤波参数 */
#define GIMBAL_MEC_YAW_SPEED_LPF_ALPHA   0.1f // 兼容速度滤波权重，0~1

/* 机械 Yaw：编码器位置外环，IMU 角速度内环。 */

/* 机械定位PID按实际周期积分 */
#define GIMBAL_CONTROL_PERIOD_S    0.001f // 机械定位控制周期，s

/* 机械位置环输入为编码器计数 */

/* Yaw 角度与电机编码器计数换算。 */
#define GIMBAL_COUNT_PER_DEG       182.04f // 机械角计数标度，count/deg
#define GIMBAL_DEG_TO_COUNT        (GIMBAL_COUNT_PER_DEG) // 机械角换算，count/deg
#define GIMBAL_COUNT_TO_DEG        (1.0f / GIMBAL_COUNT_PER_DEG) // 机械角换算，deg/count
/* 外环指令换算为目标角速度 */
#define GIMBAL_COUNT_TO_DEG_S      (1.0f) // 外环标度，deg/s/指令

/* 机械 Yaw 位置环与速度环参数。 */

/* 位置环增益与积分限幅。 */
#define GIMBAL_MEC_HOLD_KP_NM_PER_DEG    1.5f // 位置比例增益，指令/count
#define GIMBAL_MEC_HOLD_KI_NM_PER_DEG    0.0f // 位置积分增益，指令/(count·s)
#define GIMBAL_MEC_HOLD_KD_NM_PER_DEG    0.0f // 位置微分增益，指令·s/count
#define GIMBAL_MEC_HOLD_KI_LIMIT         150.0f // 位置积分限幅，count·s

/* 速度环增益、限幅和死区。 */
#define GIMBAL_MEC_HOLD_RATE_KP_NM_PER_DPS  0.1f // 速率比例增益，N·m/(deg/s)
#define GIMBAL_MEC_HOLD_RATE_KI_NM_PER_DPS  0.0f // 速率积分增益，N·m/deg
#define GIMBAL_MEC_HOLD_RATE_KD_NM_PER_DPS  0.0f // 速率微分增益，N·m/(deg/s²)
#define GIMBAL_MEC_HOLD_RATE_KI_LIMIT       0.0f // 速率积分限幅，deg
#define GIMBAL_MEC_HOLD_RATE_OUT_MAX_NM     6.0f // 速率力矩限幅，N·m
#define GIMBAL_MEC_HOLD_RATE_DEADBAND_DPS   10.0f // 内环反馈死区，deg/s

/* 位置保持的死区与退出阈值，单位 deg。 */
#define GIMBAL_MEC_HOLD_DEADZONE_DEG     0.3f // 位置软死区，deg
#define GIMBAL_MEC_YAW_HOLD_EXIT_DEG     0.5f // 静止卸力退出门限，deg

/* 静止卸力开关；默认关闭，避免死区内丢失保持力。 */
#define GIMBAL_MEC_YAW_HARD_HOLD_ENABLE  0 // 静止卸力开关，0/1

/* 旧机械 Yaw 参数，保留结构兼容。 */
#define GIMBAL_MEC_YAW_HOLD_KD_NM_PER_RAD_S 12.0f // 停用兼容阻尼，N·m/(rad/s)
#define GIMBAL_MEC_HOLD_FULL_ERR_DEG     0.3f // 兼容全保持门限，deg
#define GIMBAL_MEC_HOLD_ENTER_ERR_DEG    1.0f // 兼容保持进入门限，deg
#define GIMBAL_MEC_HOLD_TORQUE_LIMIT_NM  3.0f // 兼容保持力矩限幅，N·m
/* 最终输出力矩限幅 */
#define GIMBAL_TORQUE_LIMIT        6.0f // 电机力矩上限，N·m
/* 重力补偿开关：0 关闭，1 开启 */
#define GIMBAL_GRAVITY_ENABLE      1 // 重力补偿开关，0/1
/* 余弦重力补偿幅值*/
#define GIMBAL_GRAVITY_K_NM        1.1f // 重力补偿幅值，N·m
/* 重力补偿固定偏置 */
#define GIMBAL_GRAVITY_B_NM        0.0f // 重力补偿偏置，N·m
/* 重力补偿方向：当前正方向输出能抬升 Pitch 时用 1.0f，方向相反时用 -1.0f */
#define GIMBAL_GRAVITY_SIGN        1.0f // 重力补偿方向，仅±1
/* 重力补偿相位角 */
#define GIMBAL_GRAVITY_MIDDLE_DEG  0.0f // 重力补偿相位，deg
/* 斜坡目标判定误差阈值 */
#define GIMBAL_TARGET_ARRIVE_EPS_DEG 0.1f // 目标斜坡到位容差，deg

/* 速控目标角速度的最大变化率 */
#define GIMBAL_RATE_CMD_RAMP_DEG_S_PER_MS 6.0f // 角速度斜坡，deg/s/ms

/* 遥控器摇杆满量程值 */
#define GIMBAL_RC_AXIS_MAX                 660.0f // 摇杆满量程，原始量
/* 遥控器摇杆中位死区 */
#define GIMBAL_RC_AXIS_DEADBAND            20.0f // 摇杆死区，原始量
/* 遥控器满杆时 Yaw 最大目标角速度*/
#define GIMBAL_MANUAL_YAW_RATE_DEG_S       200.0f // 满杆Yaw速率，deg/s
/* 操作手 Yaw 方向符号 */
#define GIMBAL_MANUAL_YAW_SIGN             (-1.0f) // Yaw手控方向，仅±1
/* 遥控器满杆时 Pitch 最大目标角速度*/
#define GIMBAL_MANUAL_PITCH_RATE_DEG_S     150.0f // 满杆Pitch速率，deg/s
/* 操作手 Pitch 方向符号*/
#define GIMBAL_MANUAL_PITCH_SIGN           (1.0f) // Pitch手控方向，仅±1
/* 鼠标 X 转换为 Yaw 的增益 */
#define GIMBAL_MOUSE_YAW_RATE_GAIN         1.0f // 兼容Yaw鼠标速率倍率，≥0
/* 鼠标 Y 输入为 Pitch 的增益 */
#define GIMBAL_MOUSE_PITCH_RATE_GAIN       1.0f // 兼容Pitch鼠标速率倍率，≥0

/* 键鼠输入：鼠标计数 -> 目标角度增量 / 角速度前馈 */
#define GIMBAL_MOUSE_YAW_DEG_PER_COUNT     0.04f // Yaw鼠标增益，deg/count
#define GIMBAL_MOUSE_PITCH_DEG_PER_COUNT   0.04f // Pitch鼠标增益，deg/count
#define GIMBAL_MOUSE_YAW_SIGN              (-1.0f) // Yaw鼠标方向，仅±1
#define GIMBAL_MOUSE_PITCH_SIGN            (-1.0f) // Pitch鼠标方向，仅±1
#define GIMBAL_MOUSE_RATE_FF_DPS_PER_COUNT 0.0f // 鼠标前馈，deg/s/count
#define GIMBAL_MOUSE_DEADBAND_COUNT         1.0f // 鼠标死区，count
#define GIMBAL_INPUT_RC                    0u // 遥控器输入源，值0
#define GIMBAL_INPUT_KEYBOARD              1u // 键鼠输入源，值1

/* ========== 速控松杆位置保持：Yaw / Pitch 各自独立的 PI ========== */
/* 操作手角速度低于该值，保持环完全接管，松杆后锁在当前位置 */
#define GIMBAL_RATE_HOLD_ENTER_DEG_S       2.0f // 完全保持速限，deg/s
/* 操作手角速度高于该值，完全交还操作手，保持环不参与 */
#define GIMBAL_RATE_HOLD_EXIT_DEG_S        8.0f // 完全手控速限，deg/s

/* Yaw 保持环：输入角度误差(deg)，输出目标角速度(deg/s) */
#define GIMBAL_YAW_HOLD_KP                 15.0f // Yaw保持比例增益，1/s
#define GIMBAL_YAW_HOLD_KI                 0.003f // Yaw保持积分增益，1/s/周期
#define GIMBAL_YAW_HOLD_INTEGRAL_MAX       5000.0f // Yaw积分限幅，deg·周期
#define GIMBAL_YAW_HOLD_OUT_MAX            300.0f // Yaw保持速限，deg/s

/* Pitch 保持环：输入角度误差(deg)，输出目标角速度(deg/s) */
#define GIMBAL_PITCH_HOLD_KP               50.0f // Pitch保持比例增益，1/s
#define GIMBAL_PITCH_HOLD_KI               0.0f // Pitch积分增益，1/s/周期
#define GIMBAL_PITCH_HOLD_INTEGRAL_MAX     5000.0f // Pitch积分限幅，deg·周期
#define GIMBAL_PITCH_HOLD_OUT_MAX          1000.0f // Pitch保持速限，deg/s

/* 保持环误差死区，抑制 IMU 噪声引起的静态抖动 */
#define GIMBAL_HOLD_ERR_DEADBAND_DEG       0.2f // 保持角误差死区，deg


/* 云台运行模式 */
typedef enum
{
    G_SLEEP = 0, // 休眠卸力，模式0
    G_INIT, // 归中初始化，模式1
    G_MEC, // 机械定位，模式2
    G_GYRO, // 惯性角控制，模式3
    G_RATE, // 角速度控制，模式4
    G_AUTO, // 自瞄角控制，模式5
} gimbal_mode_e;

/* 初始化归中参数 */
typedef struct
{
    uint8_t init_flag; // 初始化完成，0/1
    uint16_t init_time; // 归中累计周期数，1ms/周期
    uint16_t init_time_max; // 归中超时周期数，1ms/周期
    float pitch_angle_tolerance; // Pitch到位容差，deg
    float yaw_angle_tolerance; // Yaw到位容差，deg
    float pitch_ramp_step; // Pitch归中步长，deg/周期
    float yaw_ramp_step; // Yaw归中步长，deg/周期
    uint8_t mode_transition_active; // 目标切换斜坡生效，0/1
    float mode_pitch_ramp_step; // Pitch切换步长，deg/周期
    float mode_yaw_ramp_step; // Yaw切换步长，deg/周期
    float pitch_speed_tolerance; // Pitch到位速限，rad/s
    float yaw_speed_tolerance; // Yaw到位速限，rad/s
    uint16_t stable_time; // 连续到位周期数，1ms/周期
    uint16_t stable_time_max; // 到位确认周期数，1ms/周期
    float yaw_home_deg; // Yaw归中目标，deg
    float pitch_home_deg; // Pitch归中目标，deg
    float yaw_max_rate_deg_s; // Yaw归中速限，deg/s
    float pitch_max_rate_deg_s; // Pitch归中速限，deg/s
    float yaw_decel_rad_s2; // Yaw制动减速度，rad/s²
    float pitch_decel_rad_s2; // Pitch制动减速度，rad/s²
    float yaw_torque_limit_nm; // Yaw力矩限幅，N·m
    float pitch_torque_limit_nm; // Pitch力矩限幅，N·m
    uint8_t pitch_gravity_enable; // 归中重力补偿开关，0/1
    float pitch_gravity_k_nm; // 归中重力补偿幅值，N·m
    float pitch_gravity_b_nm; // 归中重力补偿偏置，N·m
    float pitch_gravity_sign; // 归中重力补偿方向，仅±1
    float pitch_gravity_middle_deg; // 归中重力补偿相位，deg
} gimbal_init_info_t;

typedef enum
{
    GIMBAL_YAW_RELEASE_HOLD = 0, // 锁角保持，状态0
    GIMBAL_YAW_RELEASE_MANUAL = 1, // 主动转向，状态1
    GIMBAL_YAW_RELEASE_BRAKE = 2 // 松手制动，状态2
} gimbal_yaw_release_phase_e;

typedef struct
{
    gimbal_yaw_release_phase_e phase; // 松手阶段，0~2
    uint32_t brake_start_ms; // 制动起始时刻，ms
    uint32_t stable_start_ms; // 停稳起始时刻，ms
    uint8_t stable_tracking; // 连续停稳计时，0/1
} gimbal_yaw_release_t;

/* 云台前馈量。 */
typedef struct
{
    float yaw_rate_cmd_deg_s;       // Yaw输入角速度，deg/s
    float pitch_rate_cmd_deg_s;     // Pitch输入角速度，deg/s
    uint8_t manual_source;          // 输入源，0遥控/1键鼠
    uint8_t manual_source_changed;  // 输入源变更，0/1
    float mouse_dx_counts;          // 鼠标横向增量，count
    float mouse_dy_counts;          // 鼠标纵向增量，count
    float yaw_rate_target_deg_s;    // Yaw目标角速度，deg/s
    float pitch_rate_target_deg_s;  // Pitch目标角速度，deg/s
    float yaw_rate_cmd_last_deg_s;  // 上次Yaw指令，deg/s
    float pitch_rate_cmd_last_deg_s;// 上次Pitch指令，deg/s
    float yaw_torque_ff_nm;         // Yaw力矩前馈，N·m
    float pitch_torque_ff_nm;       // Pitch力矩前馈，N·m
    float yaw_hold_angle_deg;       // Yaw保持朝向，deg
    float pitch_hold_angle_deg;     // Pitch保持机械角，deg
    uint8_t pitch_zero_hold_last;   // 上次Pitch锁零，0/1
    gimbal_yaw_release_t yaw_release; // Yaw松手阶段，0~2
} gimbal_feedforward_t;

/* 运行参数支持在线调节 */
typedef struct
{
    volatile uint8_t gravity_enable; // 重力补偿开关，0/1
    volatile float gravity_k_nm; // 重力补偿幅值，N·m
    volatile float gravity_b_nm; // 重力补偿偏置，N·m
    volatile float gravity_sign; // 重力补偿方向，仅±1
    volatile float gravity_middle_deg; // 重力补偿相位，deg
    volatile float pitch_torque_limit_nm; // Pitch力矩限幅，N·m
    volatile float yaw_torque_limit_nm; // Yaw力矩限幅，N·m
    /* 速控松杆保持环：两轴独立 PI，Keil Watch 在线可改 */
    volatile float yaw_hold_kp; // Yaw保持比例增益，1/s
    volatile float yaw_hold_ki; // Yaw保持积分增益，1/s/周期
    volatile float yaw_hold_integral_max; // Yaw积分限幅，deg·周期
    volatile float yaw_hold_out_max; // Yaw保持速限，deg/s
    volatile float pitch_hold_kp; // Pitch保持比例增益，1/s
    volatile float pitch_hold_ki; // Pitch积分增益，1/s/周期
    volatile float pitch_hold_integral_max; // Pitch积分限幅，deg·周期
    volatile float pitch_hold_out_max; // Pitch保持速限，deg/s
    /* 保持环与操作手指令的交接区间 */
    volatile float rate_hold_enter_deg_s; // 完全保持速限，deg/s
    volatile float rate_hold_exit_deg_s; // 完全手控速限，deg/s
    volatile float pitch_manual_rate_max_deg_s; // Pitch手控速限，deg/s
    volatile float yaw_manual_rate_max_deg_s; // Yaw手控速限，deg/s
    volatile float manual_pitch_sign; // Pitch手控方向，仅±1
    volatile float manual_yaw_sign; // Yaw手控方向，仅±1
    volatile float mouse_yaw_deg_per_count; // Yaw鼠标增益，deg/count
    volatile float mouse_pitch_deg_per_count; // Pitch鼠标增益，deg/count
    volatile float mouse_yaw_sign; // Yaw鼠标方向，仅±1
    volatile float mouse_pitch_sign; // Pitch鼠标方向，仅±1
    volatile float mouse_rate_ff_dps_per_count; // 鼠标前馈，deg/s/count
    volatile float mouse_deadband_count; // 鼠标死区，count
    /* 机械模式 Yaw：只作用于 G_MEC 的 yaw 轴，Keil Watch 在线可改 */
    volatile float mec_yaw_hold_kp_nm_per_deg; // 停用兼容增益，N·m/deg
    volatile float mec_yaw_hold_kd_nm_per_rad_s; // 停用兼容阻尼，N·m/(rad/s)
    volatile float mec_yaw_max_rate_deg_s; // 机械Yaw速限，deg/s
    volatile float mec_yaw_decel_rad_s2; // 机械Yaw减速度，rad/s²
    volatile float mec_yaw_deadband_deg; // 兼容角度死区，deg
    volatile float mec_yaw_torque_step_nm; // 兼容力矩步长，N·m/周期
    volatile float mec_yaw_friction_ff_nm; // 停用摩擦补偿，N·m
    volatile float mec_hold_full_err_deg; // 兼容全保持门限，deg
    volatile float mec_hold_enter_err_deg; // 兼容保持进入门限，deg
    /* 机械定位参数按实际运算量纲 */
    volatile float mec_hold_kp_nm_per_deg; // 位置比例增益，指令/count
    volatile float mec_hold_ki_nm_per_deg; // 位置积分增益，指令/(count·s)
    volatile float mec_hold_kd_nm_per_deg; // 位置微分增益，指令·s/count
    volatile float mec_hold_ki_limit; // 位置积分限幅，count·s
    volatile float mec_hold_deadzone_count; // 位置软死区，count
    volatile float mec_hold_rate_kp_nm_per_dps; // 速率比例增益，N·m/(deg/s)
    volatile float mec_hold_rate_ki_nm_per_dps; // 速率积分增益，N·m/deg
    volatile float mec_hold_rate_kd_nm_per_dps; // 速率微分增益，N·m/(deg/s²)
    volatile float mec_hold_rate_ki_limit; // 速率积分限幅，deg
    volatile float mec_hold_rate_out_max_nm; // 速率输出限幅，N·m
    volatile float mec_hold_rate_deadband_dps; // 内环反馈死区，deg/s
    volatile float mec_yaw_gyro_direction; // IMU反馈方向，仅±1
} gimbal_tune_t;

/* 目标角与 8 路串级 PID */
typedef struct
{
    float yaw_mec_target_raw; // 板间Yaw机械目标，deg
    float pitch_mec_target_raw; // 板间Pitch机械目标，deg
    float yaw_imu_target_raw; // 板间Yaw姿态目标，deg
    float pitch_imu_target_raw; // 板间Pitch姿态目标，deg

    float yaw_target; // 当前Yaw目标，deg
    float pitch_target; // 当前Pitch目标，deg


    pid_ctrl_t yaw_gyro_outer; // Yaw姿态外环，输入deg
    pid_ctrl_t yaw_gyro_inner; // Yaw姿态内环，输入deg/s
    pid_ctrl_t yaw_mec_outer; // Yaw机械外环，输入deg
    pid_ctrl_t yaw_mec_inner; // Yaw机械内环，输入rad/s
    pid_ctrl_t yaw_turn_outer; // Yaw掉头外环，输入deg
    pid_ctrl_t yaw_turn_inner; // Yaw掉头内环，输入deg/s
    pid_ctrl_t yaw_init_outer; // Yaw归中外环，输入deg
    pid_ctrl_t yaw_init_inner; // Yaw归中内环，输入deg/s
    pid_ctrl_t pitch_init_outer; // Pitch归中外环，输入deg
    pid_ctrl_t pitch_init_inner; // Pitch归中内环，输入deg/s
    pid_ctrl_t pitch_gyro_outer; // Pitch姿态外环，输入deg
    pid_ctrl_t pitch_gyro_inner; // Pitch姿态内环，输入deg/s

    pid_ctrl_t yaw_hold; // Yaw保持环，输入deg
    pid_ctrl_t pitch_hold; // Pitch保持环，输入deg
} gimbal_pid_info_t;

/* 云台反馈与输出信息 */
typedef struct
{
    float yaw_imu_angle; // Yaw惯性角，deg
    float yaw_imu_speed; // Yaw惯性角速度，deg/s
    float pitch_imu_angle; // Pitch惯性角，deg
    float pitch_imu_speed; // Pitch惯性角速度，deg/s

    float yaw_mec_angle; // Yaw机械角，deg
    float yaw_mec_speed; // Yaw机械角速度，rad/s
    float yaw_mec_speed_lpf; // 机械速度滤波保留量，rad/s
    float pitch_mec_angle; // Pitch机械角，deg
    float pitch_mec_speed; // Pitch机械角速度，rad/s

    float output_gimbal_y; // Yaw输出力矩，N·m
    float output_gimbal_p; // Pitch输出力矩，N·m
    float gravity_f; // Pitch重力补偿，N·m
    /* 兼容误差观测字段 */
    float mec_yaw_err_deg; // 兼容机械误差字段，deg
} gimbal_base_info_t;

/* 云台对象，集中保存设备、状态、参数和控制接口 */
typedef struct gimbal_class_t
{
    dm_motor_t *pitch_motor; // Pitch电机对象指针
    dm_motor_t *yaw_motor; // Yaw电机对象指针

    gimbal_base_info_t base_info; // 反馈输出，单位见类型
    gimbal_pid_info_t pid_info; // 目标与控制环，单位见类型
    gimbal_init_info_t init_info; // 归中配置，单位见类型
    gimbal_feedforward_t feedforward; // 前馈保持量，单位见类型

    gimbal_mode_e gimbal_mode; // 当前控制模式，0~5
    gimbal_mode_e last_gimbal_mode; // 上周期控制模式，0~5

    void (*init)(struct gimbal_class_t *gimbal); /* 初始化接口 */
    void (*work)(struct gimbal_class_t *gimbal); /* 周期工作接口 */
} gimbal_t;

extern gimbal_t Gimbal;
extern gimbal_tune_t gimbal_tune;

void Gimbal_Init(gimbal_t *gimbal);
void Gimbal_Work(gimbal_t *gimbal);

#endif

