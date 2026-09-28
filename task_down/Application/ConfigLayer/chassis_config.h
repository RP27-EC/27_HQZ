/* chassis_config.h - 底盘参数配置 */

#ifndef __CHASSIS_CONFIG_H
#define __CHASSIS_CONFIG_H

#define CHASSIS_RC_INPUT_ENABLE         1u  // 遥控输入开关
#define CHASSIS_KEYBOARD_INPUT_ENABLE   1u  // 键鼠输入开关
#define CHASSIS_OWNS_RC_YAW             1u  // 底盘接管 Yaw

/* 键鼠输入参数。 */
#define CHASSIS_KEY_SPEED_BOOST         1.5f  // 键鼠加速倍率
#define CHASSIS_KEY_SPEED_SLOW          0.5f  // 键鼠减速倍率
#define CHASSIS_KEY_VX_SIGN             -1.0f  // 键鼠 X 方向修正
#define CHASSIS_KEY_VY_SIGN             -1.0f  // 键鼠 Y 方向修正
#define CHASSIS_KEY_WZ_SIGN              1.0f  // 键鼠转向修正

#define CHASSIS_GIMBAL_FOLLOW_ENABLE    1u  // 跟随云台开关
#define CHASSIS_SPIN_ENABLE              1u  // 小陀螺开关

#define CHASSIS_CONTROL_PERIOD_MS       1u  // 控制周期

/* 小陀螺参数。遥控 S2 下拨选择，键鼠按 C 选择，ch0 控制旋转速度。 */
#define CHASSIS_SPIN_MAX_WZ              20.0f  // 小陀螺最大角速度
#define CHASSIS_SPIN_BASE_WZ             20.0f  // 小陀螺基础角速度
#define CHASSIS_SPIN_TRIM_WZ             0.0f  // 小陀螺微调量
#define CHASSIS_SPIN_STEP                0.1f  // 小陀螺加减步长
#define CHASSIS_SPIN_DIRECTION           1.0f  // 小陀螺转向
#define CHASSIS_SPIN_RC_DEADBAND         30.0f  // 小陀螺遥控死区
#define CHASSIS_SPIN_TRANSLATION_ENABLE   1u  // 小陀螺平移开关
#define CHASSIS_SPIN_TRANSLATION_FRAME_GIMBAL 1u  // 平移参考系取云台
#define CHASSIS_SPIN_TRANSLATION_YAW_SIGN 1.0f  // 平移 Yaw 修正
#define CHASSIS_SPIN_TRANSLATION_SIGN     1.0f  // 平移方向修正
#define CHASSIS_SPIN_GIMBAL_TIMEOUT_MS    50u  // 云台数据超时
#define CHASSIS_SPIN_TORQUE_LIMIT_NM     4.0f  // 小陀螺力矩限幅

/* 底盘跟随云台参数。遥控 S2 上/中拨选择，键鼠按 Z 选择。 */
#define CHASSIS_FOLLOW_TRANSLATION_ENABLE 1u  // 跟随平移开关
#define CHASSIS_FOLLOW_CENTER_RAD         0.0f  // 跟随中心角
#define CHASSIS_FOLLOW_YAW_ANGLE_SIGN     1.0f  // 跟随角度修正
#define CHASSIS_FOLLOW_TRANSLATION_SIGN   1.0f  // 跟随平移修正
#define CHASSIS_FOLLOW_WZ_SIGN            -1.0f  // 跟随转向修正
#define CHASSIS_FOLLOW_KP                 20.0f  // 跟随 P
#define CHASSIS_FOLLOW_MAX_WZ             20.0f  // 跟随最大角速度
#define CHASSIS_FOLLOW_WZ_STEP            0.2f  // 跟随加减步长
#define CHASSIS_FOLLOW_FRICTION_FF        7.0f  // 跟随摩擦前馈
#define CHASSIS_FOLLOW_DEADBAND_DEG       7.0f  // 跟随死区
#define CHASSIS_FOLLOW_TURN_LOCK_DEG      150.0f  // 锁定转向角
#define CHASSIS_FOLLOW_TURN_UNLOCK_DEG    20.0f  // 解锁转向角
#define CHASSIS_FOLLOW_TORQUE_LIMIT_NM    3.0f  // 跟随力矩限幅
#define CHASSIS_FOLLOW_YAW_JUMP_LIMIT_DEG 30.0f  // 跟随跳变限制
#define CHASSIS_FOLLOW_TIMEOUT_MS         50u  // 跟随数据超时
#define CHASSIS_FOLLOW_BLEND_TIME_MS      200u  // 跟随切换过渡时长
#define CHASSIS_FOLLOW_BLEND_STEP         ((float)CHASSIS_CONTROL_PERIOD_MS / (float)CHASSIS_FOLLOW_BLEND_TIME_MS)  // 每周期过渡比例

/* 原底盘代码参数。 */
#define CHASSIS_CTRL_MAX_SPEED          80.0f  // 底盘最大速度
/* 架空调试阶段限速，稳定后再逐步恢复到 50/50/40。 */
//直接映射到遥控器
//50 50 40有点太快了，改小一点
#define CHASSIS_MAX_VX                  35.0f  // 遥控 X 最大速度
#define CHASSIS_MAX_VY                  35.0f  // 遥控 Y 最大速度
#define CHASSIS_MAX_WZ                  25.0f  // 遥控转向最大角速度
#define CHASSIS_TURN_CYCLE_SPEED        55.0f  // 原地转身速度
#define CHASSIS_RC_DEADBAND             30.0f  // 遥控死区
#define CHASSIS_RC_AXIS_MAX             660.0f  // 遥控通道量程

#define CHASSIS_SPEED_KP                0.8f  // 底盘速度环 P
#define CHASSIS_SPEED_KI                0.0f  // 底盘速度环 I
#define CHASSIS_SPEED_KD                0.0f  // 底盘速度环 D
#define CHASSIS_TEST_TORQUE_LIMIT_NM    2.0f  // 调试力矩限幅
#define CHASSIS_FIXED_CURRENT_LIMIT_A   20.0f  // 固定电流限幅
#define CHASSIS_FIXED_TORQUE_LIMIT_NM   5.4f  // 固定力矩限幅

/* 零速区域：目标和反馈都接近 0 时停止输出，避免速度环来回抽搐。 */
#define CHASSIS_ZERO_TARGET_BAND        0.5f  // 零速目标带
#define CHASSIS_STOP_SPEED_BAND         1.0f  // 零速反馈带

/* 原底盘几何和重量参数。 */
#define CHASSIS_LENGTH_M                0.39994f  // 车长
#define CHASSIS_WIDTH_M                 0.39990f  // 车宽
#define CHASSIS_DIAGONAL_LENGTH_M       0.56558f  // 对角线轮距
#define CHASSIS_MASS_KG                 11.85f  // 整车质量
#define CHASSIS_WHEEL_RADIUS_M          0.154f  // 轮半径
#define CHASSIS_GRAVITY_MPS2            9.81f  // 重力加速度

#endif

