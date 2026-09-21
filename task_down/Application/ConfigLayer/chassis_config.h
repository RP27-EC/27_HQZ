#ifndef __CHASSIS_CONFIG_H
#define __CHASSIS_CONFIG_H

/* 第一阶段底盘调试开关。 */
#define CHASSIS_BRINGUP_ENABLE          1u
#define CHASSIS_RC_INPUT_ENABLE         1u
#define CHASSIS_KEYBOARD_INPUT_ENABLE   0u
#define CHASSIS_OWNS_RC_YAW             1u

/* 后续阶段保留，第一阶段默认关闭。 */
#define CHASSIS_PLANNER_ENABLE          0u
#define CHASSIS_FEEDFORWARD_ENABLE      0u
#define CHASSIS_GIMBAL_FOLLOW_ENABLE    1u
#define CHASSIS_SPIN_ENABLE              1u
#define CHASSIS_POWER_LIMIT_ENABLE      0u

#define CHASSIS_CONTROL_PERIOD_MS       1u

/* 小陀螺参数。S1 上拨使能，S2 下拨选择小陀螺模式，ch0 控制旋转速度。 */
#define CHASSIS_SPIN_MAX_WZ              20.0f
#define CHASSIS_SPIN_STEP                0.1f
#define CHASSIS_SPIN_DIRECTION           1.0f
#define CHASSIS_SPIN_RC_DEADBAND         30.0f
#define CHASSIS_SPIN_TORQUE_LIMIT_NM     3.0f

/* 底盘跟随云台参数。S1 上拨使能，S2 上拨选择跟随模式。 */
#define CHASSIS_FOLLOW_TRANSLATION_ENABLE 1u
#define CHASSIS_FOLLOW_CENTER_RAD         0.0f
#define CHASSIS_FOLLOW_YAW_ANGLE_SIGN     1.0f
#define CHASSIS_FOLLOW_TRANSLATION_SIGN   1.0f
#define CHASSIS_FOLLOW_WZ_SIGN            -1.0f
#define CHASSIS_FOLLOW_KP                 20.0f
#define CHASSIS_FOLLOW_MAX_WZ             20.0f
#define CHASSIS_FOLLOW_WZ_STEP            0.2f
#define CHASSIS_FOLLOW_FRICTION_FF        7.0f
#define CHASSIS_FOLLOW_DEADBAND_DEG       7.0f
#define CHASSIS_FOLLOW_TURN_LOCK_DEG      150.0f
#define CHASSIS_FOLLOW_TURN_UNLOCK_DEG    20.0f
#define CHASSIS_FOLLOW_TORQUE_LIMIT_NM    3.0f
#define CHASSIS_FOLLOW_YAW_JUMP_LIMIT_DEG 30.0f
#define CHASSIS_FOLLOW_TIMEOUT_MS         50u
#define CHASSIS_FOLLOW_BLEND_TIME_MS      200u
#define CHASSIS_FOLLOW_BLEND_STEP         ((float)CHASSIS_CONTROL_PERIOD_MS / (float)CHASSIS_FOLLOW_BLEND_TIME_MS)

/* 原底盘代码参数。 */
#define CHASSIS_CTRL_MAX_SPEED          80.0f
/* 架空调试阶段限速，稳定后再逐步恢复到 50/50/40。 */
//直接映射到遥控器
//50 50 40有点太快了，改小一点
#define CHASSIS_MAX_VX                  35.0f
#define CHASSIS_MAX_VY                  35.0f
#define CHASSIS_MAX_WZ                  25.0f
#define CHASSIS_TURN_CYCLE_SPEED        55.0f
#define CHASSIS_RC_DEADBAND             30.0f
#define CHASSIS_RC_AXIS_MAX             660.0f

#define CHASSIS_SPEED_KP                0.8f
#define CHASSIS_SPEED_KI                0.0f
#define CHASSIS_SPEED_KD                0.0f
#define CHASSIS_TEST_TORQUE_LIMIT_NM    1.5f
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
