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
#define CHASSIS_FOLLOW_TRANSLATION_ENABLE 1u
#define CHASSIS_FOLLOW_CENTER_RAD         0.0f
/* 遥控与键鼠共用掉头基准。 */
#define CHASSIS_FOLLOW_YAW_REFERENCE_ENABLE 1u // 跟随前后基准使能，0/1
#define CHASSIS_FOLLOW_YAW_ANGLE_SIGN     1.0f
#define CHASSIS_FOLLOW_TRANSLATION_SIGN   1.0f
#define CHASSIS_FOLLOW_WZ_SIGN            -1.0f
#define CHASSIS_FOLLOW_KP                 20.0f
#define CHASSIS_FOLLOW_MAX_WZ             40.0f
#define CHASSIS_FOLLOW_WZ_STEP            0.4f
#define CHASSIS_FOLLOW_RATE_FF            -3.2f
#define CHASSIS_FOLLOW_RATE_PER_DEG_S     0.032f
#define CHASSIS_FOLLOW_FRICTION_FF        0.0f
#define CHASSIS_FOLLOW_DEADBAND_DEG       0.5f
#define CHASSIS_FOLLOW_TURN_LOCK_DEG      150.0f
#define CHASSIS_FOLLOW_TURN_UNLOCK_DEG    20.0f
#define CHASSIS_FOLLOW_TORQUE_LIMIT_NM    4.0f
#define CHASSIS_FOLLOW_YAW_JUMP_LIMIT_DEG 30.0f
#define CHASSIS_FOLLOW_TIMEOUT_MS         50u
#define CHASSIS_FOLLOW_BLEND_TIME_MS      10u
#define CHASSIS_FOLLOW_BLEND_STEP         ((float)CHASSIS_CONTROL_PERIOD_MS / (float)CHASSIS_FOLLOW_BLEND_TIME_MS)

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

