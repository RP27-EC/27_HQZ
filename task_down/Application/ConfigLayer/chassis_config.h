/* chassis_config.h - 底盘参数配置 */

#ifndef __CHASSIS_CONFIG_H
#define __CHASSIS_CONFIG_H

/* 第一阶段底盘调试开关。 */
#define CHASSIS_BRINGUP_ENABLE          1u
#define CHASSIS_RC_INPUT_ENABLE         1u
#define CHASSIS_KEYBOARD_INPUT_ENABLE   1u
#define CHASSIS_OWNS_RC_YAW             1u

/* 键鼠输入参数。 */
#define CHASSIS_KEY_SPEED_BOOST         1.5f
#define CHASSIS_KEY_SPEED_SLOW          0.5f
#define CHASSIS_KEY_VX_SIGN             -1.0f
#define CHASSIS_KEY_VY_SIGN             -1.0f
#define CHASSIS_KEY_WZ_SIGN              1.0f

/* 后续阶段保留，第一阶段默认关闭。 */
#define CHASSIS_PLANNER_ENABLE          0u
#define CHASSIS_FEEDFORWARD_ENABLE      0u
#define CHASSIS_GIMBAL_FOLLOW_ENABLE    1u
#define CHASSIS_SPIN_ENABLE              1u
/* 功率限制参数见 power_limit_config.h，开关为 CHASSIS_POWER_LIMIT_ENABLE */

#define CHASSIS_CONTROL_PERIOD_MS       1u

/* 小陀螺参数。遥控 S2 下拨选择，键鼠按 C 选择，ch0 控制旋转速度。 */
#define CHASSIS_SPIN_MAX_WZ              25.0f
#define CHASSIS_SPIN_BASE_WZ             25.0f
#define CHASSIS_SPIN_TRIM_WZ             0.0f
#define CHASSIS_SPIN_STEP                0.1f
#define CHASSIS_SPIN_DIRECTION           1.0f
#define CHASSIS_SPIN_RC_DEADBAND         30.0f
#define CHASSIS_SPIN_TRANSLATION_ENABLE   1u
#define CHASSIS_SPIN_TRANSLATION_FRAME_GIMBAL 1u
#define CHASSIS_SPIN_TRANSLATION_YAW_SIGN 1.0f
#define CHASSIS_SPIN_TRANSLATION_SIGN     1.0f
#define CHASSIS_SPIN_GIMBAL_TIMEOUT_MS    50u
#define CHASSIS_SPIN_TORQUE_LIMIT_NM     4.0f

/* 键鼠机械档：鼠标 X 直接转底盘，等价遥控 S1 下位时 ch0 的转向作用。
 * 增益单位 rad/s per 鼠标计数，初值对齐 D5 鼠标手感（5 deg/s per count）。
 * 上车要标定：GAIN 决定转速手感，SIGN 决定左右方向。 */
#define CHASSIS_KEY_MECH_MOUSE_WZ_GAIN    10.0f
#define CHASSIS_KEY_MECH_MOUSE_WZ_MAX     15.0f
#define CHASSIS_KEY_MECH_MOUSE_WZ_SIGN    1.0f

/* 到位门限与跟随死区匹配，避免交接后补转。 */
#define CHASSIS_KEY_UTURN_TOL_DEG         0.5f
#define CHASSIS_KEY_UTURN_STABLE_MS       100u
#define CHASSIS_KEY_UTURN_HANDOFF_PERIODS 2u
#define CHASSIS_KEY_UTURN_TIMEOUT_MS      2500u
#define CHASSIS_KEY_UTURN_FEEDBACK_TIMEOUT_MS 50u /* 上板反馈超时，ms */

/* 底盘跟随云台参数。遥控 S2 上/中拨选择，键鼠按 Z 选择。 */
#define CHASSIS_FOLLOW_TRANSLATION_ENABLE 1u   // 平移坐标旋转，0/1
#define CHASSIS_FOLLOW_CENTER_RAD         0.0f // 固定跟随中心，rad
#define CHASSIS_FOLLOW_YAW_REFERENCE_ENABLE 1u // 使用前后基准，0/1
#define CHASSIS_FOLLOW_YAW_ANGLE_SIGN     1.0f // 角度方向，±1
#define CHASSIS_FOLLOW_TRANSLATION_SIGN   1.0f // 平移方向，±1
#define CHASSIS_FOLLOW_WZ_SIGN            -1.0f // 旋转输出方向，±1
#define CHASSIS_FOLLOW_KP                 20.0f // 转速控制量/rad
#define CHASSIS_FOLLOW_MAX_WZ             40.0f // 转速控制量上限，>0
#define CHASSIS_FOLLOW_WZ_STEP            0.4f  // 转速控制量/周期，>0
#define CHASSIS_FOLLOW_RATE_FF            -3.3f // 指令前馈倍率
#define CHASSIS_FOLLOW_RATE_PER_DEG_S     0.032f // 转速控制量/(°/s)
#define CHASSIS_FOLLOW_FRICTION_FF        0.0f  // 摩擦补偿转速控制量
#define CHASSIS_FOLLOW_DEADBAND_DEG       5.0f  // 停止纠偏角误差，°
#define CHASSIS_FOLLOW_RESUME_DEG         6.0f  // 恢复门限，>停止角，°
#define CHASSIS_FOLLOW_TURN_LOCK_DEG      150.0f // 锁定转向误差，°
#define CHASSIS_FOLLOW_TURN_UNLOCK_DEG    20.0f // 释放转向误差，°
#define CHASSIS_FOLLOW_TORQUE_LIMIT_NM    4.0f  // 跟随力矩上限，N·m
#define CHASSIS_FOLLOW_YAW_JUMP_LIMIT_DEG 30.0f // 单拍角跳变上限，°
#define CHASSIS_FOLLOW_TIMEOUT_MS         50u  // 云台反馈超时，ms
#define CHASSIS_FOLLOW_BLEND_TIME_MS      10u  // 接管融合时间，ms
#define CHASSIS_FOLLOW_BLEND_STEP         ((float)CHASSIS_CONTROL_PERIOD_MS / (float)CHASSIS_FOLLOW_BLEND_TIME_MS) // 融合比例/周期，0~1

/* 仅停稳受扰和小陀螺退出启用。 */
#define CHASSIS_FOLLOW_RECOVERY_FF_WZ     10.0f  // 辅助前馈转速控制量，≥0
#define CHASSIS_FOLLOW_RECOVERY_TRIGGER_DEG 6.0f // 受扰触发误差，°
#define CHASSIS_FOLLOW_RECOVERY_FULL_DEG  7.0f  // 前馈全幅误差，>停止角，°
#define CHASSIS_FOLLOW_RECOVERY_STABLE_MS 200u  // 停稳确认时间，ms
#define CHASSIS_FOLLOW_RECOVERY_TIMEOUT_MS 4000u // 辅助最长持续时间，ms
#define CHASSIS_FOLLOW_RECOVERY_INPUT_EPS 0.001f // 无输入转速控制量阈值
#define CHASSIS_FOLLOW_RECOVERY_RATE_EPS  0.1f  // 无输入角速度阈值，°/s
#define CHASSIS_FOLLOW_RECOVERY_SPIN_EPS  0.001f // 旋转交接控制量阈值

/* 原底盘代码参数。 */
#define CHASSIS_CTRL_MAX_SPEED          80.0f
/* 架空调试阶段限速，稳定后再逐步恢复到 50/50/40。 */
//直接映射到遥控器
//50 50 40有点太快了，改小一点
#define CHASSIS_MAX_VX                  25.0f
#define CHASSIS_MAX_VY                  25.0f
#define CHASSIS_MAX_WZ                  20.0f
#define CHASSIS_TURN_CYCLE_SPEED        45.0f
#define CHASSIS_RC_DEADBAND             30.0f
#define CHASSIS_RC_AXIS_MAX             660.0f

#define CHASSIS_SPEED_KP                0.8f
#define CHASSIS_SPEED_KI                0.0f
#define CHASSIS_SPEED_KD                0.0f
#define CHASSIS_TEST_TORQUE_LIMIT_NM    2.0f
#define CHASSIS_FIXED_CURRENT_LIMIT_A   20.0f
#define CHASSIS_FIXED_TORQUE_LIMIT_NM   5.4f

/* 零速区域：目标和反馈都接近 0 时停止输出，避免速度环来回抽搐。 */
#define CHASSIS_ZERO_TARGET_BAND        0.5f
#define CHASSIS_STOP_SPEED_BAND         1.0f

/* 原底盘几何和重量参数。 */
#define CHASSIS_LENGTH_M                0.39994f
#define CHASSIS_WIDTH_M                 0.39990f
#define CHASSIS_DIAGONAL_LENGTH_M       0.56558f
#define CHASSIS_MASS_KG                 11.85f
#define CHASSIS_WHEEL_RADIUS_M          0.154f
#define CHASSIS_GRAVITY_MPS2            9.81f

#endif

