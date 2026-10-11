/* chassis_config.h - 底盘参数配置 */

#ifndef __CHASSIS_CONFIG_H
#define __CHASSIS_CONFIG_H

/* 输入来源开关 */
#define CHASSIS_RC_INPUT_ENABLE         1u // 遥控输入使能，0/1
#define CHASSIS_KEYBOARD_INPUT_ENABLE   1u // 键鼠输入使能，0/1
#define CHASSIS_OWNS_RC_YAW             1u // 底盘接管遥控Yaw，0/1

/* 键鼠输入参数。 */
#define CHASSIS_KEY_SPEED_BOOST         1.5f // Shift速度倍率，>0
#define CHASSIS_KEY_SPEED_SLOW          0.5f // Ctrl速度倍率，>0
#define CHASSIS_KEY_VX_SIGN             -1.0f // 键鼠纵向方向，仅±1
#define CHASSIS_KEY_VY_SIGN             -1.0f // 键鼠横向方向，仅±1
#define CHASSIS_KEY_WZ_SIGN              1.0f // 键鼠旋转方向，仅±1

/* 控制功能开关 */
#define CHASSIS_GIMBAL_FOLLOW_ENABLE    1u // 云台跟随使能，0/1
#define CHASSIS_SPIN_ENABLE              1u // 小陀螺使能，0/1
/* 功率限制参数见 power_limit_config.h，开关为 CHASSIS_POWER_LIMIT_ENABLE */

#define CHASSIS_CONTROL_PERIOD_MS       1u // 底盘控制周期，ms

/* 小陀螺参数。遥控 S2 下拨选择，键鼠按 C 选择，ch0 控制旋转速度。 */
#define CHASSIS_SPIN_MAX_WZ              25.0f // 自转轮速分量限幅，rad/s
#define CHASSIS_SPIN_BASE_WZ             25.0f // 自转基础轮速分量，rad/s
#define CHASSIS_SPIN_TRIM_WZ             0.0f // 满杆自转修正量，rad/s
#define CHASSIS_SPIN_STEP                0.1f // 自转斜坡，rad/s/周期
#define CHASSIS_SPIN_DIRECTION           1.0f // 自转方向，仅±1
#define CHASSIS_SPIN_RC_DEADBAND         30.0f // 自转摇杆死区，原始量
#define CHASSIS_SPIN_TRANSLATION_ENABLE   1u // 自转平移使能，0/1
#define CHASSIS_SPIN_TRANSLATION_FRAME_GIMBAL 1u // 按云台系旋转平移，0/1
#define CHASSIS_SPIN_TRANSLATION_YAW_SIGN 1.0f // 平移旋转角方向，仅±1
#define CHASSIS_SPIN_TRANSLATION_SIGN     1.0f // 平移整体方向，仅±1
#define CHASSIS_SPIN_GIMBAL_TIMEOUT_MS    50u // 自转云台反馈超时，ms
#define CHASSIS_SPIN_TORQUE_LIMIT_NM     4.0f // 自转电机力矩上限，N·m

/* 机械档鼠标控制底盘转向 */
#define CHASSIS_KEY_MECH_MOUSE_WZ_GAIN    10.0f // 鼠标转向，rad/s/count
#define CHASSIS_KEY_MECH_MOUSE_WZ_MAX     15.0f // 鼠标转向轮速限幅，rad/s
#define CHASSIS_KEY_MECH_MOUSE_WZ_SIGN    1.0f // 鼠标转向方向，仅±1

/* 到位门限与跟随死区匹配，避免交接后补转。 */
#define CHASSIS_KEY_UTURN_TOL_DEG         0.5f // 掉头到位容差，deg
#define CHASSIS_KEY_UTURN_STABLE_MS       100u // 掉头连续到位确认，ms
#define CHASSIS_KEY_UTURN_HANDOFF_PERIODS 2u // 交接成功发送周期数，≥1
#define CHASSIS_KEY_UTURN_TIMEOUT_MS      2500u // 掉头动作超时，ms
#define CHASSIS_KEY_UTURN_FEEDBACK_TIMEOUT_MS 50u // 掉头反馈超时，ms

/* 底盘跟随云台参数。遥控 S2 上/中拨选择，键鼠按 Z 选择。 */
#define CHASSIS_FOLLOW_TRANSLATION_ENABLE 1u   // 平移坐标旋转，0/1
#define CHASSIS_FOLLOW_CENTER_RAD         0.0f // 固定跟随中心，rad
#define CHASSIS_FOLLOW_YAW_REFERENCE_ENABLE 1u // 使用前后基准，0/1
#define CHASSIS_FOLLOW_YAW_ANGLE_SIGN     1.0f // 角度方向，±1
#define CHASSIS_FOLLOW_TRANSLATION_SIGN   1.0f // 平移方向，±1
#define CHASSIS_FOLLOW_WZ_SIGN            -1.4f // 跟随输出方向与倍率，无量纲
#define CHASSIS_FOLLOW_KP                 20.0f // 跟随轮速增益，1/s
#define CHASSIS_FOLLOW_MAX_WZ             40.0f // 跟随轮速分量限幅，rad/s
#define CHASSIS_FOLLOW_WZ_STEP            0.4f // 跟随轮速斜坡，rad/s/周期
#define CHASSIS_FOLLOW_RATE_FF            -1.0f // 操纵角速度前馈倍率，无量纲
#define CHASSIS_FOLLOW_RATE_PER_DEG_S     0.032f // 角速前馈标度，rad/deg
#define CHASSIS_FOLLOW_FRICTION_FF        1.0f // 摩擦补偿轮速分量，rad/s
#define CHASSIS_FOLLOW_DEADBAND_DEG       1.0f  // 停止纠偏角误差，°
#define CHASSIS_FOLLOW_RESUME_DEG         2.0f  // 恢复门限，>停止角，°
#define CHASSIS_FOLLOW_TURN_LOCK_DEG      150.0f // 锁定转向误差，°
#define CHASSIS_FOLLOW_TURN_UNLOCK_DEG    20.0f // 释放转向误差，°
#define CHASSIS_FOLLOW_TORQUE_LIMIT_NM    4.0f  // 跟随力矩上限，N·m
#define CHASSIS_FOLLOW_YAW_JUMP_LIMIT_DEG 30.0f // 单拍角跳变上限，°
#define CHASSIS_FOLLOW_TIMEOUT_MS         50u  // 云台反馈超时，ms
#define CHASSIS_FOLLOW_BLEND_TIME_MS      10u  // 接管融合时间，ms
#define CHASSIS_FOLLOW_BLEND_STEP         ((float)CHASSIS_CONTROL_PERIOD_MS / (float)CHASSIS_FOLLOW_BLEND_TIME_MS) // 融合比例/周期，0~1

/* 仅停稳受扰和小陀螺退出启用。 */
#define CHASSIS_FOLLOW_RECOVERY_ARRIVE_DEG 0.5f // 特殊回正到位误差，°
#define CHASSIS_FOLLOW_RECOVERY_FF_WZ     0.0f // 回正轮速前馈，rad/s，0关闭
#define CHASSIS_FOLLOW_RECOVERY_TRIGGER_DEG 6.0f // 受扰触发误差，°
#define CHASSIS_FOLLOW_RECOVERY_FULL_DEG  7.0f  // 前馈全幅误差，>停止角，°
#define CHASSIS_FOLLOW_RECOVERY_STABLE_MS 200u  // 停稳确认时间，ms
#define CHASSIS_FOLLOW_RECOVERY_TIMEOUT_MS 4000u // 辅助最长持续时间，ms
#define CHASSIS_FOLLOW_RECOVERY_INPUT_EPS 0.001f // 无输入轮速分量门限，rad/s
#define CHASSIS_FOLLOW_RECOVERY_RATE_EPS  0.1f  // 无输入角速度阈值，°/s
#define CHASSIS_FOLLOW_RECOVERY_SPIN_EPS  0.001f // 自转交接轮速门限，rad/s

/* 当前轮速与限幅参数 */
#define CHASSIS_CTRL_MAX_SPEED          80.0f // 轮速分量总幅值上限，rad/s
/* 逆解直接叠加轮速分量 */
#define CHASSIS_MAX_VX                  15.0f // 纵向轮速分量上限，rad/s
#define CHASSIS_MAX_VY                  15.0f // 横向轮速分量上限，rad/s
#define CHASSIS_MAX_WZ                  17.0f // 旋转轮速分量上限，rad/s
#define CHASSIS_RC_DEADBAND             30.0f // 摇杆中位死区，原始量
#define CHASSIS_RC_AXIS_MAX             660.0f // 摇杆单侧满量程，原始量

#define CHASSIS_SPEED_KP                0.8f // 速度比例增益，N·m/(rad/s)
#define CHASSIS_SPEED_KI                0.0f // 速度积分增益，按周期累加
#define CHASSIS_SPEED_KD                0.0f // 速度微分增益，按周期差分
#define CHASSIS_TEST_TORQUE_LIMIT_NM    2.0f // 直控电机力矩上限，N·m

/* 目标与反馈均低速时卸力 */
#define CHASSIS_ZERO_TARGET_BAND        0.5f // 目标零速区，rad/s
#define CHASSIS_STOP_SPEED_BAND         1.0f // 反馈零速区，rad/s



#endif
