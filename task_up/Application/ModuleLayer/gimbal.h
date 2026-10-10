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
#define GIMBAL_PI                  3.14159265358979323846f
#define GIMBAL_TWO_PI              6.28318530717958647692f
#define GIMBAL_DEG_TO_RAD          (GIMBAL_PI / 180.0f)
#define GIMBAL_RAD_TO_DEG          (180.0f / GIMBAL_PI)

/* 中点与机械行程配置 */
#define GIMBAL_YAW_MIDDLE_DEG      (-22.224138f)
#define GIMBAL_PITCH_MIDDLE_DEG    148.573157f
#define GIMBAL_PITCH_MIN_DEG       (-7.5f)
#define GIMBAL_PITCH_MAX_DEG       30.0f

/* 机械 Yaw 运动与滤波配置 */
#define GIMBAL_MEC_OUTER_KD              0.1f
#define GIMBAL_MEC_OUTER_D_FILTER_ALPHA  0.85f
#define GIMBAL_MEC_ERR_DEADBAND_DEG      0.2f
#define GIMBAL_MEC_YAW_MAX_RATE_DEG_S    300.0f
#define GIMBAL_MEC_YAW_DECEL_RAD_S2      25.0f
#define GIMBAL_MEC_YAW_TORQUE_STEP_NM    0.08f

#define GIMBAL_MEC_YAW_FF_OFF_DPS        25.0f
#define GIMBAL_MEC_YAW_FF_FULL_DPS       35.0f
#define GIMBAL_MEC_YAW_FF_BLEND_STEP     0.02f
#define GIMBAL_MEC_YAW_FF_FALL_STEP      0.10f
#define GIMBAL_MEC_YAW_FF_FILTER_ALPHA   0.05f
#define GIMBAL_MEC_YAW_FF_MAX_GAIN       0.90f
#define GIMBAL_MEC_YAW_NEAR_RATE_KP      10.0f
/* 静摩擦补偿只在误差死区外生效。 */
#define GIMBAL_MEC_YAW_FRICTION_FF_NM    0.3f
/* 速度反馈低通滤波系数，0~1。 */
#define GIMBAL_MEC_YAW_SPEED_LPF_ALPHA   0.1f

/* 机械 Yaw：编码器位置外环，IMU 角速度内环。 */

/* 控制周期，秒。Gimbal_Work 在 control_task 里由 osDelayUntil(...,1u) 驱动，
 * 即 1 kHz。积分按时间累加时要用它，见 gimbal_mec_yaw_calc。 */
#define GIMBAL_CONTROL_PERIOD_S    0.001f

/* 位置环角度输入为 deg；速度环反馈与目标均为 deg/s。 */

/* Yaw 角度与电机编码器计数换算。 */
#define GIMBAL_COUNT_PER_DEG       182.04f
#define GIMBAL_DEG_TO_COUNT        (GIMBAL_COUNT_PER_DEG)
#define GIMBAL_COUNT_TO_DEG        (1.0f / GIMBAL_COUNT_PER_DEG)
/* 位置环输出换算为角速度，deg/s/count。 */
#define GIMBAL_COUNT_TO_DEG_S      (1.0f)

/* 机械 Yaw 位置环与速度环参数。 */

/* 位置环增益与积分限幅。 */
#define GIMBAL_MEC_HOLD_KP_NM_PER_DEG    1.5f
#define GIMBAL_MEC_HOLD_KI_NM_PER_DEG    0.0f
#define GIMBAL_MEC_HOLD_KD_NM_PER_DEG    0.0f
#define GIMBAL_MEC_HOLD_KI_LIMIT         150.0f

/* 速度环增益、限幅和死区。 */
#define GIMBAL_MEC_HOLD_RATE_KP_NM_PER_DPS  0.1f
#define GIMBAL_MEC_HOLD_RATE_KI_NM_PER_DPS  0.0f
#define GIMBAL_MEC_HOLD_RATE_KD_NM_PER_DPS  0.0f
#define GIMBAL_MEC_HOLD_RATE_KI_LIMIT       0.0f
#define GIMBAL_MEC_HOLD_RATE_OUT_MAX_NM     6.0f
#define GIMBAL_MEC_HOLD_RATE_DEADBAND_DPS   10.0f

/* 位置保持的死区与退出阈值，单位 deg。 */
#define GIMBAL_MEC_HOLD_DEADZONE_DEG     0.3f
#define GIMBAL_MEC_YAW_HOLD_EXIT_DEG     0.5f

/* 静止卸力开关；默认关闭，避免死区内丢失保持力。 */
#define GIMBAL_MEC_YAW_HARD_HOLD_ENABLE  0

/* 旧机械 Yaw 参数，保留结构兼容。 */
#define GIMBAL_MEC_YAW_HOLD_KD_NM_PER_RAD_S 12.0f
#define GIMBAL_MEC_HOLD_FULL_ERR_DEG     0.3f
#define GIMBAL_MEC_HOLD_ENTER_ERR_DEG    1.0f
#define GIMBAL_MEC_HOLD_TORQUE_LIMIT_NM  3.0f
/* 最终输出力矩限幅 */
#define GIMBAL_TORQUE_LIMIT        6.0f
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
#define GIMBAL_MANUAL_YAW_RATE_DEG_S       150.0f
/* 操作手 Yaw 方向符号 */
#define GIMBAL_MANUAL_YAW_SIGN             (-1.0f)
/* 遥控器满杆时 Pitch 最大目标角速度*/
#define GIMBAL_MANUAL_PITCH_RATE_DEG_S     150.0f
/* 操作手 Pitch 方向符号*/
#define GIMBAL_MANUAL_PITCH_SIGN           (1.0f)
/* 鼠标 X 转换为 Yaw 的增益 */
#define GIMBAL_MOUSE_YAW_RATE_GAIN         1.0f
/* 鼠标 Y 输入为 Pitch 的增益 */
#define GIMBAL_MOUSE_PITCH_RATE_GAIN       1.0f

/* 键鼠输入：鼠标计数 -> 目标角度增量 / 角速度前馈 */
#define GIMBAL_MOUSE_YAW_DEG_PER_COUNT     0.04f
#define GIMBAL_MOUSE_PITCH_DEG_PER_COUNT   0.04f
#define GIMBAL_MOUSE_YAW_SIGN              (-1.0f)
#define GIMBAL_MOUSE_PITCH_SIGN            (-1.0f)
#define GIMBAL_MOUSE_RATE_FF_DPS_PER_COUNT 0.0f
#define GIMBAL_MOUSE_DEADBAND_COUNT         1.0f
#define GIMBAL_INPUT_RC                    0u
#define GIMBAL_INPUT_KEYBOARD              1u

/* ========== 速控松杆位置保持：Yaw / Pitch 各自独立的 PI ========== */
/* 操作手角速度低于该值，保持环完全接管，松杆后锁在当前位置 */
#define GIMBAL_RATE_HOLD_ENTER_DEG_S       2.0f
/* 操作手角速度高于该值，完全交还操作手，保持环不参与 */
#define GIMBAL_RATE_HOLD_EXIT_DEG_S        8.0f

/* Yaw 保持环：输入角度误差(deg)，输出目标角速度(deg/s) */
#define GIMBAL_YAW_HOLD_KP                 15.0f
#define GIMBAL_YAW_HOLD_KI                 0.003f
#define GIMBAL_YAW_HOLD_INTEGRAL_MAX       5000.0f
#define GIMBAL_YAW_HOLD_OUT_MAX            300.0f

/* Pitch 保持环：输入角度误差(deg)，输出目标角速度(deg/s) */
#define GIMBAL_PITCH_HOLD_KP               50.0f
#define GIMBAL_PITCH_HOLD_KI               0.0f
#define GIMBAL_PITCH_HOLD_INTEGRAL_MAX     5000.0f
#define GIMBAL_PITCH_HOLD_OUT_MAX          1000.0f

/* 保持环误差死区，抑制 IMU 噪声引起的静态抖动 */
#define GIMBAL_HOLD_ERR_DEADBAND_DEG       0.2f


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
    float yaw_home_deg;                /* Yaw 归中机械目标 */
    float pitch_home_deg;              /* Pitch 归中机械目标 */
    float yaw_max_rate_deg_s;          /* Yaw 归中最大目标速度 */
    float pitch_max_rate_deg_s;        /* Pitch 归中最大目标速度 */
    float yaw_decel_rad_s2;            /* Yaw 归中制动假设 */
    float pitch_decel_rad_s2;          /* Pitch 归中制动假设 */
    float yaw_torque_limit_nm;         /* Yaw 归中输出限幅 */
    float pitch_torque_limit_nm;       /* Pitch 归中输出限幅 */
    uint8_t pitch_gravity_enable;      /* Pitch 归中重力补偿开关 */
    float pitch_gravity_k_nm;          /* Pitch 归中重力补偿幅值 */
    float pitch_gravity_b_nm;          /* Pitch 归中重力补偿偏置 */
    float pitch_gravity_sign;          /* Pitch 归中重力补偿方向 */
    float pitch_gravity_middle_deg;    /* Pitch 归中重力补偿相位 */
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
    /* 速控松杆保持环：两轴独立 PI，Keil Watch 在线可改 */
    volatile float yaw_hold_kp;
    volatile float yaw_hold_ki;
    volatile float yaw_hold_integral_max;
    volatile float yaw_hold_out_max;
    volatile float pitch_hold_kp;
    volatile float pitch_hold_ki;
    volatile float pitch_hold_integral_max;
    volatile float pitch_hold_out_max;
    /* 保持环与操作手指令的交接区间 */
    volatile float rate_hold_enter_deg_s;
    volatile float rate_hold_exit_deg_s;
    volatile float pitch_manual_rate_max_deg_s;
    volatile float yaw_manual_rate_max_deg_s;
    volatile float manual_pitch_sign;
    volatile float manual_yaw_sign;
    volatile float mouse_yaw_deg_per_count;
    volatile float mouse_pitch_deg_per_count;
    volatile float mouse_yaw_sign;
    volatile float mouse_pitch_sign;
    volatile float mouse_rate_ff_dps_per_count;
    volatile float mouse_deadband_count;
    /* 机械模式 Yaw：只作用于 G_MEC 的 yaw 轴，Keil Watch 在线可改 */
    volatile float mec_yaw_hold_kp_nm_per_deg;   /* 已并入 mec_hold_kp，保留兼容 */
    volatile float mec_yaw_hold_kd_nm_per_rad_s; /* 已废弃（改 IMU 内环） */
    volatile float mec_yaw_max_rate_deg_s;
    volatile float mec_yaw_decel_rad_s2;
    volatile float mec_yaw_deadband_deg;
    volatile float mec_yaw_torque_step_nm;
    volatile float mec_yaw_friction_ff_nm;       /* 已废弃（改位置环积分） */
    volatile float mec_hold_full_err_deg;
    volatile float mec_hold_enter_err_deg;
    /* 机械 Yaw 双环（照搬参考工程 cloud_control_yaw_mechanical）：
     * 位置外环用编码器相对角出目标角速度，速度内环用 IMU 角速度出力矩。 */
    volatile float mec_hold_kp_nm_per_deg;       /* 位置外环 kp（码域，参考原值） */
    volatile float mec_hold_ki_nm_per_deg;       /* 位置外环 ki */
    volatile float mec_hold_kd_nm_per_deg;       /* 位置外环 kd（秒） */
    volatile float mec_hold_ki_limit;            /* 位置外环积分累加限幅（码·秒） */
    volatile float mec_hold_deadzone_count;      /* 连续软死区（编码器计数） */
    volatile float mec_hold_rate_kp_nm_per_dps;  /* 速度内环 kp（IMU，deg/s） */
    volatile float mec_hold_rate_ki_nm_per_dps;  /* 速度内环 ki */
    volatile float mec_hold_rate_kd_nm_per_dps;  /* 速度内环 kd */
    volatile float mec_hold_rate_ki_limit;       /* 速度内环积分限幅 */
    volatile float mec_hold_rate_out_max_nm;     /* 速度内环输出限幅 */
    volatile float mec_hold_rate_deadband_dps;   /* 速度内环速率死区（deg/s） */
    volatile float mec_yaw_gyro_direction;       /* IMU 与编码器同向 +1，反向 -1 */
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
    pid_ctrl_t yaw_turn_outer;     /* Yaw 掉头角度外环（归中式串级，参数独立） */
    pid_ctrl_t yaw_turn_inner;     /* Yaw 掉头速度内环（归中式串级，参数独立） */
    pid_ctrl_t yaw_init_outer;     /* Yaw 归中角度外环 */
    pid_ctrl_t yaw_init_inner;     /* Yaw 归中速度内环 */
    pid_ctrl_t pitch_init_outer;   /* Pitch 归中角度外环 */
    pid_ctrl_t pitch_init_inner;   /* Pitch 归中速度内环 */
    pid_ctrl_t pitch_gyro_outer;   /* Pitch 陀螺角度外环 */
    pid_ctrl_t pitch_gyro_inner;   /* Pitch IMU 角速度内环 */

    pid_ctrl_t yaw_hold;           /* Yaw 速控松杆保持环（角度 -> 角速度） */
    pid_ctrl_t pitch_hold;         /* Pitch 速控松杆保持环（角度 -> 角速度） */
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
    float yaw_mec_speed_lpf;       /* Yaw 机械角速度低通值，仅用于阻尼项 */
    float pitch_mec_angle;         /* Pitch 电机机械角 */
    float pitch_mec_speed;         /* Pitch 电机机械角速度*/

    float output_gimbal_y;         /* Yaw 最终输出力矩 */
    float output_gimbal_p;         /* Pitch 最终输出力矩 */
    float gravity_f;               /* 当前 Pitch 重力补偿力矩 */
    /* 机械模式 Yaw 定位误差，deg（目标 - 实际，已过死区）。
     * 调试用：直接看它就能读到超调量，它符号翻转的时刻就是过冲峰值所在。 */
    float mec_yaw_err_deg;
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

