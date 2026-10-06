#ifndef __LAUNCHER_CONFIG_H
#define __LAUNCHER_CONFIG_H

/* 摩擦轮常规速度控制 */
#define LAUNCHER_DIAL_ENABLE              1u // 拨盘控制使能，0/1
#define LAUNCHER_REPEAT_ENABLE            1u // 连发使能，0/1
#define LAUNCHER_FRIC_TARGET_RPM          1500.0f // 轮速目标，rpm
#define LAUNCHER_FRIC_RAMP_RPM_PER_MS     20.0f // 升速斜率，rpm/ms
#define LAUNCHER_FRIC_STOP_RAMP_RPM_PER_MS 20.0f // 降速斜率，rpm/ms
#define LAUNCHER_FRIC_STOP_SPEED_RPM      100.0f // 停转门槛，rpm
#define LAUNCHER_FRIC_STOP_CONFIRM_MS     1000u // 停转确认，ms
#define LAUNCHER_FRIC_READY_TOL_RPM       500.0f // 达速容差，rpm
#define LAUNCHER_FRIC_READY_TIME_MS       100u // 达速确认，ms
#define LAUNCHER_FRIC_KP                  2.0f // 比例，原始值/rpm
#define LAUNCHER_FRIC_KI                  1.0f // 积分，原始值/rpm每拍
#define LAUNCHER_FRIC_KD                  0.0f // 微分，原始值/rpm每拍
#define LAUNCHER_FRIC_INTEGRAL_MAX        500.0f // 误差积分限幅，rpm拍
#define LAUNCHER_FRIC_KFF                 0.0f // 前馈系数，非负
#define LAUNCHER_FRIC_OUT_MAX             5000.0f // 控制电流限幅，原始值
#define LAUNCHER_FRIC_L_DIRECTION         1.0f // 方向符号，仅±1
#define LAUNCHER_FRIC_R_DIRECTION         -1.0f // 方向符号，仅±1

/* 转轮堵转仅尝试一次增强 */
#define LAUNCHER_FRIC_START_GRACE_MS       500u // 启动检测宽限，ms
#define LAUNCHER_FRIC_JAM_SPEED_RPM        300.0f // 堵转速度门槛，rpm
#define LAUNCHER_FRIC_JAM_OUTPUT_RAW       3000.0f // 控制电流门槛，原始值
#define LAUNCHER_FRIC_JAM_CONFIRM_MS       200u // 堵转确认时间，ms
#define LAUNCHER_FRIC_BOOST_KP             4.0f // 增强比例，原始值/rpm
#define LAUNCHER_FRIC_BOOST_TIME_MS        200u // 单次增强上限，ms
#define LAUNCHER_DIAL_START_RETRY_MS       50u // 单发启动等待上限，ms

/* 拨盘：65536 count 为一圈，一发走一圈。 */
#define LAUNCHER_DIAL_AUTO_RESET_ENABLE   0u // 历史定位开关，固定0
#define LAUNCHER_DIAL_READY_HOLD_ENABLE   1u // 待发位置保持，0/1

#define LAUNCHER_DIAL_ANGLE_KP            0.08f // 比例，deg/s/count
#define LAUNCHER_DIAL_ANGLE_KI            0.0f // 积分，deg/s/count每拍
#define LAUNCHER_DIAL_ANGLE_KD            0.0f // 微分，deg/s/count每拍
#define LAUNCHER_DIAL_ANGLE_INTEGRAL_MAX  0.0f // 误差积分限幅，count拍
#define LAUNCHER_DIAL_ANGLE_DEADBAND      0.0f // 位置误差死区，count

#define LAUNCHER_DIAL_SPEED_KP            0.40f // 比例，原始值/(deg/s)
#define LAUNCHER_DIAL_SPEED_KI            0.05f // 积分增益，按1ms拍
#define LAUNCHER_DIAL_SPEED_KD            0.0f // 微分增益，按1ms拍
#define LAUNCHER_DIAL_SPEED_INTEGRAL_MAX  500.0f // 速度误差积分，deg/s拍
#define LAUNCHER_DIAL_SPEED_OUT_MAX       2000.0f // 电流限幅，原始值

#define LAUNCHER_DIAL_ANGLE_SIGN          1.0f // 方向符号，仅±1
#define LAUNCHER_DIAL_SPEED_SIGN          1.0f // 方向符号，仅±1
#define LAUNCHER_DIAL_OUTPUT_SIGN         1.0f // 方向符号，仅±1
#define LAUNCHER_DIAL_DIRECTION           1.0f // 方向符号，仅±1

#define LAUNCHER_DIAL_CURRENT_LIMIT       2000.0f // 电流限幅，原始值≤2000
#define LAUNCHER_DIAL_MAX_SPEED_DPS       7000u // 位置环速度上限，deg/s

#define LAUNCHER_DIAL_RESET_ANGLE         31259.0f // 定位目标，count
#define LAUNCHER_DIAL_RESET_TIMEOUT_MS    1000u // 定位超时，ms

#define LAUNCHER_DIAL_ONE_SHOT_ANGLE      65536.0f // 单发位移，count
#define LAUNCHER_DIAL_REVERSE_ANGLE       65536.0f // 反向退让，count
#define LAUNCHER_DIAL_STOP_ERROR          500.0f // 到位容差，count
#define LAUNCHER_DIAL_SINGLE_TIMEOUT_MS   500u // 单发运行超时，ms
#define LAUNCHER_DIAL_REVERSE_TIMEOUT_MS  200u // 反转超时，ms
#define LAUNCHER_DIAL_RELOAD_TIMEOUT_MS   200u // 退让复位超时，ms

/* 连发独立速度环：15 圈/s。 */
#define LAUNCHER_DIAL_REPEAT_SPEED_DPS    5400u // 连发速度上限，deg/s
#define LAUNCHER_DIAL_REPEAT_KP           0.35f // 比例，原始值/(deg/s)
#define LAUNCHER_DIAL_REPEAT_KI           0.0f // 积分增益，按1ms拍
#define LAUNCHER_DIAL_REPEAT_KD           0.0f // 微分增益，按1ms拍
#define LAUNCHER_DIAL_REPEAT_INTEGRAL_MAX 500.0f // 速度误差积分，deg/s拍
#define LAUNCHER_DIAL_REPEAT_OUT_MAX      2000.0f // 电流限幅，原始值

/* 堵转后反向退让，再回到原供弹目标。 */
#define LAUNCHER_DIAL_JAM_ENABLE          0u // 历史反转开关，固定0
#define LAUNCHER_DIAL_JAM_CURRENT_RAW     600 // 堵转反馈电流，原始值
#define LAUNCHER_DIAL_JAM_SPEED_DPS       10 // 堵转速度门槛，deg/s
#define LAUNCHER_DIAL_JAM_CONFIRM_TICKS   200u // 堵转确认拍数，1ms/拍

#define LAUNCHER_DIAL_SAFE_STOP_RETRY_MS  50u // 停机发送重试间隔，ms

/* 拨盘释放后的主动制动，避免直接断力后的惯性和回弹。 */
#define LAUNCHER_DIAL_BRAKE_KP            0.2f // 比例，原始值/(deg/s)
#define LAUNCHER_DIAL_BRAKE_KI            0.0f // 积分增益，按1ms拍
#define LAUNCHER_DIAL_BRAKE_KD            0.0f // 微分增益，按1ms拍
#define LAUNCHER_DIAL_BRAKE_INTEGRAL_MAX  0.0f // 速度误差积分，deg/s拍
#define LAUNCHER_DIAL_BRAKE_OUT_MAX       1500.0f // 电流限幅，原始值
#define LAUNCHER_DIAL_BRAKE_STOP_SPEED_DPS 20u // 制动停转门槛，deg/s
#define LAUNCHER_DIAL_BRAKE_TIMEOUT_MS    120u // 制动超时，ms

/* 17 mm热量预算与限频 */
#define LAUNCHER_HEAT_PER_SHOT             10.0f // 每发增热，热量单位
#define LAUNCHER_HEAT_WARN                200.0f // 降速起点，热量单位
#define LAUNCHER_HEAT_SATURATE             50.0f // 平衡区起点，热量单位
#define LAUNCHER_HEAT_MARGIN               20.0f // 安全余量，热量单位，至少10
#define LAUNCHER_HEAT_STOP LAUNCHER_HEAT_MARGIN // 连发停发余量，热量单位
#define LAUNCHER_HEAT_RESUME               30.0f // 恢复余量，热量单位
#define LAUNCHER_HEAT_MAX_RATE             15.0f // 最高射频，发/s
#define LAUNCHER_HEAT_D3_TIMEOUT_MS        100u // 热量链路超时，ms
#define LAUNCHER_HEAT_TRAINING_ENABLE        1u // 固定参数训练，0/1
#define LAUNCHER_HEAT_TRAINING_LIMIT     300.0f // 训练上限，热量单位
#define LAUNCHER_HEAT_TRAINING_COOLING   150.0f // 训练冷却，热量单位/s

#endif
