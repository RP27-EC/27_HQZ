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
/* 机械模式 Yaw：位置外环微分与制动整形。
 * 注意 OUTER_KD 已不再使用：阻尼改由 IMU 速度内环提供（见下面的双环说明），
 * 位置环再叠加微分只会在编码器量化噪声上做微分。 */
#define GIMBAL_MEC_OUTER_KD              0.1f
#define GIMBAL_MEC_OUTER_D_FILTER_ALPHA  0.85f
/* 中心死区：改用连续软死区，见 GIMBAL_MEC_HOLD_DEADZONE_DEG */
#define GIMBAL_MEC_ERR_DEADBAND_DEG      0.2f
/* 位置外环输出（目标角速度）上限，deg/s —— 参考原值直取。
 * 参考 position_output_limit = 300（码），而 mechanical_yaw_command_deg_s_per_raw
 * = 1.0（1 码 = 1 deg/s），所以它实际就是 300 deg/s 的目标速度上限。
 * 这里取同一个值，语义与参考完全一致。
 * 实际巡航速度不由此值决定，而是由下面的 sqrt 制动曲线决定
 * （80 deg/s @1°，9° 以上才会碰到这个上限）。 */
#define GIMBAL_MEC_YAW_MAX_RATE_DEG_S    300.0f
/* 制动假设角减速度，rad/s^2。
 * 反推自参考的 mechanical_yaw_brake_speed_at_1deg = 80（码@1°）：
 * 80 deg/s 对应 80*pi/180 = 1.396 rad/s，sqrt(2*a*(1° in rad)) = 1.396
 *   -> a = 80^2 * pi / 180 / 2 = 16.0
 * 这样 sqrt 曲线在 9° 以内就咬住上限，等效巡航速度 ≈ 80 deg/s，与参考一致。 */
#define GIMBAL_MEC_YAW_DECEL_RAD_S2      25.0f
/* 输出力矩斜率限幅，单位 N·m/ms，照搬底盘跟随的 Ramp(last_wz, wz, WZ_STEP)。
 * 满量程 6 N·m 约需 6/STEP 毫秒，0 表示不限幅。 */
#define GIMBAL_MEC_YAW_TORQUE_STEP_NM    0.08f

#define GIMBAL_MEC_YAW_FF_OFF_DPS        25.0f
#define GIMBAL_MEC_YAW_FF_FULL_DPS       35.0f
#define GIMBAL_MEC_YAW_FF_BLEND_STEP     0.02f
#define GIMBAL_MEC_YAW_FF_FALL_STEP      0.10f
#define GIMBAL_MEC_YAW_FF_FILTER_ALPHA   0.05f
#define GIMBAL_MEC_YAW_FF_MAX_GAIN       0.90f
#define GIMBAL_MEC_YAW_NEAR_RATE_KP      6.0f
/* 线束/静摩擦前馈：按误差方向叠加的恒定力矩，用来破静摩擦，
 * 避免"卡住 → 误差累积 → 猛冲"。死区内不叠加，否则会在中心来回翻转。 */
#define GIMBAL_MEC_YAW_FRICTION_FF_NM    0.3f
/* 阻尼所用转速的一阶低通系数，截止约 16 Hz。
 * 转速不过滤就对它做微分，会在编码器量化噪声上自激，发出嗡嗡声。 */
#define GIMBAL_MEC_YAW_SPEED_LPF_ALPHA   0.1f

/*
 * 机械模式 Pitch 不单独建环：直接复用速控的 pitch 通路
 * （操作手角速度 + pitch_hold 保持环 + pitch_gyro_inner 速度环）。
 * 机械模式与速控在 pitch 上是同一段代码，只是 Yaw 仍走机械环。
 */

/*
 * 机械模式 Yaw：双环结构，两个环的反馈来自【两个不同传感器】。
 * 这是照搬参考工程 infantry_up/user/cloud_terrace.c 的
 * cloud_control_yaw_mechanical()，参数按它的量纲折算。
 *
 *   位置外环：编码器累计机械角（相对底盘），输出「目标角速度 deg/s」
 *   速度内环：IMU 陀螺仪 yaw_imu_speed（deg/s），输出力矩
 *   制动曲线：外环输出上限 = MAX_RATE * sqrt(剩余角度/1deg)，接近目标自动减速
 *
 * 为什么内环必须用 IMU（这是参考能稳、原来不能稳的根本原因）：
 *   原来内环的 measure = yaw_mec_speed，单位 rad/s；而外环输出
 *   = kp * 角度误差(deg)，单位 deg/s。两者差 57.3 倍，轴一动 err 就是几十，
 *   kp=1.5 时立即顶到 out_max，内环退化成「速度超过约 38 deg/s 就满力刹车」
 *   的开关。开关的增益没有任何意义 —— 这就是「kd 从 0.5 调到 1000 没反应」
 *   以及四轮改动都无效的根本原因。
 *   IMU 的 yaw_imu_speed 是 deg/s，与外环输出同量纲，内环落在线性区，
 *   参数才真正起作用，也才能给出比例阻尼。
 *
 * 参考工程的对应参数（infantry_up/user/application_config.c:90-111）：
 *   yaw_angle_kp=10, ki=0, kd=0, yaw_rate_target_limit=600 deg/s
 *   yaw_rate_kp=10, ki=0, kd=0, yaw_torque_limit=2047 码
 *   mechanical_yaw_deadzone_deg=0.4
 *   mechanical_yaw_brake_speed_at_1deg=80 码，0 附近为 sqrt 制动曲线
 *   position_kp=0.3, ki=1.0, kd=0.005, output_limit=300 码
 *   mechanical_yaw_rate_kp=10, torque_limit=1800 码
 */

/* 控制周期，秒。Gimbal_Work 在 control_task 里由 osDelayUntil(...,1u) 驱动，
 * 即 1 kHz。积分按时间累加时要用它，见 gimbal_mec_yaw_calc。 */
#define GIMBAL_CONTROL_PERIOD_S    0.001f

/*
 * 机械 Yaw 双环参数 —— 全部采用参考工程 infantry_up 的【原值】，不做折算。
 *
 * 做法：位置环整条链写在【编码器计数域】，与参考一致
 *   （参考 MOTOR4310_ECD_PER_ROUND = 65535/圈，位置环用的就是累计计数）。
 *   本工程 base_info.yaw_mec_angle 是度（归中/掉头/lift 共用，不能动），
 *   所以在 gimbal_mec_yaw_calc 里用下面两个常数做域换算，
 *   而【参数本身保持参考原值】。这与参考自己"制动曲线用 deg、位置环用 count"
 *   是同一个手法，也和本工程 pitch 通路用 GIMBAL_RAD_TO_DEG 当 inner_scale 一致。
 *
 * 参考出处：infantry_up/user/application_config.c:90-111
 *           infantry_up/bottom_driven/peripheral_config.c:10-36
 *
 * ┌─────────────────────────────┬─────────┬────────┬──────────────────────────┐
 * │ 参考参数                     │ 参考原值 │ 本工程  │ 说明                     │
 * ├─────────────────────────────┼─────────┼────────┼──────────────────────────┤
 * │ position_kp                  │ 0.3     │ 0.3    │ 原值直取（位置环在码域） │
 * │ position_integral_limit      │ 150     │ 150    │ 原值直取                 │
 * │ mechanical_yaw_deadzone_deg  │ 0.4     │ 0.4    │ 原值直取（换算成码见下） │
 * │ mechanical_yaw_rate_kp       │ 10      │ 0.05   │ 注① 单位换表示，非重设计 │
 * │ mechanical_yaw_torque_limit  │ 1800    │ 6.0    │ 注② 最终限幅就是 6       │
 * │ position_output_limit        │ 300     │ 300    │ 原值直取（注③）         │
 * │ position_ki                  │ 1.0     │ 0.25   │ 注④ 周期 4ms→1ms         │
 * │ brake_speed_at_1deg          │ 80      │ 16.0   │ 注⑤ 反推曲线系数         │
 * └─────────────────────────────┴─────────┴────────┴──────────────────────────┘
 *
 * 注① 内环 kp：参考输出是"转矩码"，本工程是 N·m。但反馈增益是【相对量】，
 *      两家电机同为 DM4310、额定转矩同量级，域换算系数相同，可直接搬：
 *        10 码/(deg/s) ÷ (2048 码 / 10 N·m) = 0.0488 N·m/(deg/s)
 *      这不是重新设计的参数，是同一增益换一个单位表示。
 * 注② 参考 1800/2047 ≈ 88% 量程 = 8.79 N·m。本工程最终限幅
 *      GIMBAL_TORQUE_LIMIT 就是 6 N·m，填 6 与参考"内环不额外限制"意图一致。
 * 注③ 参考 position_output_limit = 300 码，而 command_scale = 1.0（1 码 = 1 deg/s），
 *      所以就是 300 deg/s。这里取同一个值。实际巡航速度由注⑤的制动曲线决定。
 * 注④ 参考控制周期 4 ms，本工程 1 ms。积分项 Ki 的时间常数与周期成反比，
 *      故 position_ki × 0.001/0.004 = 0.25。比例项与周期无关。
 * 注⑤ 参考 brake_speed_at_1deg = 80，即 1° 时允许 80 deg/s。本工程用
 *      gimbal_mec_speed_limit() 的 sqrt(2·a·θ) 形式，反推
 *        a = 80² × π/180 / 2 = 16.0 rad/s²
 *      代入后 1° 处正好 80 deg/s，与参考曲线重合。
 */

/* 位置域换算常数：1 度 = 65535/360 = 182.04 个编码器计数。
 * 参考的 MOTOR4310_ECD_PER_ROUND 就是这个 65535/圈。 */
#define GIMBAL_COUNT_PER_DEG       182.04f
#define GIMBAL_DEG_TO_COUNT        (GIMBAL_COUNT_PER_DEG)
#define GIMBAL_COUNT_TO_DEG        (1.0f / GIMBAL_COUNT_PER_DEG)
/* 命令比例：参考 mechanical_yaw_command_deg_s_per_raw = 1.0，
 * 即位置环每 1 码输出对应 1 deg/s 目标角速度。 */
#define GIMBAL_COUNT_TO_DEG_S      (1.0f)

/* 机械 Yaw 双环的【实车整定值】。
 * 下面每个宏后面都标注了它是"参考原值/折算值"还是"现场整定值"。
 * 现场整定值来自 Keil Watch 实测可用的一组（2026-xx 实测），
 * 改这几个宏等于改上电默认值。 */

/* 位置外环 kp。参考原值 0.3（码域）；本工程实测整定为 0.5。 */
#define GIMBAL_MEC_HOLD_KP_NM_PER_DEG    1.5f
/* 位置外环积分。参考折算值 0.25（1.0 × 1ms/4ms）；本工程实测整定为 0.5。 */
#define GIMBAL_MEC_HOLD_KI_NM_PER_DEG    0.0f
/* 位置外环微分。参考 position_kd = 0.005（单位 s，配 4 ms 周期），
 * 按周期折算到 1 ms：0.005 / 0.001 = 5.0。
 * 【现场整定】接近目标时冲过 -> 加大；高频发抖 -> 减小或置 0。 */
#define GIMBAL_MEC_HOLD_KD_NM_PER_DEG    0.0f
/* 积分累加限幅 —— 参考原值直取（单位「码·秒」）。 */
#define GIMBAL_MEC_HOLD_KI_LIMIT         150.0f

/* 速度内环 kp。参考折算值 0.0488（10 码/(deg/s) ÷ 204.8）；
 * 本工程实测整定为 0.1（比参考折算值高，因为本工程响应对应更快）。 */
#define GIMBAL_MEC_HOLD_RATE_KP_NM_PER_DPS  0.1f
/* 速度内环积分 —— 参考 mechanical_yaw_rate_ki = 0，原值直取。 */
#define GIMBAL_MEC_HOLD_RATE_KI_NM_PER_DPS  0.0f
/* 速度内环微分 —— 参考 mechanical_yaw_rate_kd = 0，原值直取。 */
#define GIMBAL_MEC_HOLD_RATE_KD_NM_PER_DPS  0.0f
/* 速度内环积分限幅 —— 参考 mechanical_yaw_rate_integral_limit = 0，原值直取。 */
#define GIMBAL_MEC_HOLD_RATE_KI_LIMIT       0.0f
/* 速度内环输出限幅（参考 1800/2047 ≈ 88% 量程；本工程最终限幅就是 6 N·m） */
#define GIMBAL_MEC_HOLD_RATE_OUT_MAX_NM     6.0f
/* NOTE: 保留 Watch 布局。 */
#define GIMBAL_MEC_HOLD_RATE_DEADBAND_DPS   10.0f

/* 静止时的零力矩角度范围，deg。
 * 【本工程实测】0.3 / 0.5 是可用值，已改回。
 * 注意不要把 DEADZONE 放大：它会同时放大 hard_hold 的生效范围，
 * 而 hard_hold 是完全卸力（return 0.0f），生效范围一大，云台就长期无力、
 * 被线束推着漂，漂出 HOLD_EXIT 又变满力 —— 0 到 ±6 N·m 的阶跃反而更震。 */
#define GIMBAL_MEC_HOLD_DEADZONE_DEG     0.3f
#define GIMBAL_MEC_YAW_HOLD_EXIT_DEG     0.5f

/* 静止卸力（hard_hold）开关：1 = 启用，0 = 关闭。
 *
 * 关掉的理由：这个机制在误差进入死区时直接 return 0.0f，完全不输出力矩。
 * 它有副作用 —— 云台在死区内是"松"的，会被线束/走线推着漂，
 * 漂出 HOLD_EXIT_DEG 后控制器又重新全力接管，形成一次阶跃，
 * 表现为静止时的间歇性抽动。而且它与"静止时也要有力顶住"的目标相反。
 *
 * 关掉之后，静止附近由【位置环 + 速率死区】共同保证安静：
 *   位置环：误差在软死区内时位置项自然为 0，不会主动出力
 *   速率死区：陀螺在静止时的微小噪声被当作 0，内环不出反向力矩
 * 这样既有安静的静止，又保留了必要的保持力（误差一到死区外就出力）。
 * 置 1 可恢复旧行为对照。 */
#define GIMBAL_MEC_YAW_HARD_HOLD_ENABLE  0

/* 以下两项不再用于机械 Yaw（改成 IMU 内环后不需要了），保留供对照：
 *   GIMBAL_MEC_YAW_HOLD_KD_NM_PER_RAD_S  编码器速度阻尼，已被 IMU 内环替代
 *   GIMBAL_MEC_HOLD_FULL/ENTER_ERR_DEG   误差混合权重，死区已改连续软死区 */
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
#define GIMBAL_MANUAL_YAW_RATE_DEG_S       300.0f
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
#define GIMBAL_YAW_HOLD_KP                 10.0f
#define GIMBAL_YAW_HOLD_KI                 0.003f
#define GIMBAL_YAW_HOLD_INTEGRAL_MAX       5000.0f
#define GIMBAL_YAW_HOLD_OUT_MAX            150.0f

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

/* 云台前馈量。 */
typedef struct
{
    float yaw_rate_cmd_deg_s;       /* 操作手原始 Yaw 角速度指令 */
    float pitch_rate_cmd_deg_s;     /* 操作手原始 Pitch 角速度指令 */
    uint8_t manual_source;          /* 当前操作输入源 */
    uint8_t manual_source_changed;  /* 输入源切换标志 */
    float mouse_dx_counts;          /* 鼠标 X 原始增量 */
    float mouse_dy_counts;          /* 鼠标 Y 原始增量 */
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

