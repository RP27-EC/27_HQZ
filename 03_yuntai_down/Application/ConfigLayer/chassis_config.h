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
#define CHASSIS_GIMBAL_FOLLOW_ENABLE    0u
#define CHASSIS_POWER_LIMIT_ENABLE      0u

#define CHASSIS_CONTROL_PERIOD_MS       1u

/* 原底盘代码参数。 */
#define CHASSIS_CTRL_MAX_SPEED          80.0f
#define CHASSIS_MAX_VX                  50.0f
#define CHASSIS_MAX_VY                  50.0f
#define CHASSIS_MAX_WZ                  40.0f
#define CHASSIS_TURN_CYCLE_SPEED        55.0f
#define CHASSIS_RC_DEADBAND             0.0f
#define CHASSIS_RC_AXIS_MAX             660.0f

#define CHASSIS_SPEED_KP                1.0f
#define CHASSIS_SPEED_KI                0.0f
#define CHASSIS_SPEED_KD                0.0f
#define CHASSIS_TEST_TORQUE_LIMIT_NM    5.4f
#define CHASSIS_FIXED_CURRENT_LIMIT_A   20.0f
#define CHASSIS_FIXED_TORQUE_LIMIT_NM   5.4f

/* 原底盘几何和重量参数。 */
#define CHASSIS_LENGTH_M                0.39994f
#define CHASSIS_WIDTH_M                 0.39990f
#define CHASSIS_DIAGONAL_LENGTH_M       0.56558f
#define CHASSIS_MASS_KG                 11.85f
#define CHASSIS_WHEEL_RADIUS_M          0.154f
#define CHASSIS_GRAVITY_MPS2            9.81f

#endif
