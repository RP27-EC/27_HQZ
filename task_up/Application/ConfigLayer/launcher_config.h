#ifndef __LAUNCHER_CONFIG_H
#define __LAUNCHER_CONFIG_H

/* 摩擦轮速度控制 */
#define LAUNCHER_DIAL_ENABLE              1u // 拨盘控制使能，0/1
#define LAUNCHER_REPEAT_ENABLE            1u // 连发控制使能，0/1
#define LAUNCHER_FRIC_TARGET_RPM          1500.0f // 摩擦轮目标，rpm
#define LAUNCHER_FRIC_RAMP_RPM_PER_MS     20.0f // 升速斜坡，rpm/ms
#define LAUNCHER_FRIC_STOP_RAMP_RPM_PER_MS 20.0f // 停机斜坡，rpm/ms
#define LAUNCHER_FRIC_STOP_SPEED_RPM      100.0f // 停转速度上限，rpm
#define LAUNCHER_FRIC_STOP_CONFIRM_MS     1000u // 停转确认时间，ms
#define LAUNCHER_FRIC_READY_TOL_RPM       500.0f // 达速允许误差，rpm
#define LAUNCHER_FRIC_READY_TIME_MS       100u // 达速确认时间，ms
#define LAUNCHER_FRIC_KP                  2.0f // 比例增益，原始量/rpm
#define LAUNCHER_FRIC_KI                  1.0f // 积分增益，原始量/累积误差
#define LAUNCHER_FRIC_KD                  0.0f // 微分增益，原始量/误差差值
#define LAUNCHER_FRIC_INTEGRAL_MAX        500.0f // 积分累积上限，rpm
#define LAUNCHER_FRIC_KFF                 0.0f // 前馈增益，原始量/rpm
#define LAUNCHER_FRIC_OUT_MAX             5000.0f // 电流输出限幅，原始量
#define LAUNCHER_FRIC_L_DIRECTION         1.0f // 左轮旋向，正负1
#define LAUNCHER_FRIC_R_DIRECTION         -1.0f // 右轮旋向，正负1

/* 单发位置控制 */
#define LAUNCHER_DIAL_ANGLE_KP            0.08f // 位置增益，(deg/s)/count
#define LAUNCHER_DIAL_ANGLE_KI            0.0f // 积分增益，速度/累积误差
#define LAUNCHER_DIAL_ANGLE_KD            0.0f // 微分增益，速度/误差差值
#define LAUNCHER_DIAL_ANGLE_INTEGRAL_MAX  0.0f // 位置积分上限，count
#define LAUNCHER_DIAL_ANGLE_DEADBAND      0.0f // 单发位置死区，count
#define LAUNCHER_DIAL_SPEED_KP            0.15f // 速度增益，原始量/(deg/s)
#define LAUNCHER_DIAL_SPEED_KI            0.05f // 积分增益，原始量/累积误差
#define LAUNCHER_DIAL_SPEED_KD            0.0f // 微分增益，原始量/误差差值
#define LAUNCHER_DIAL_SPEED_INTEGRAL_MAX  500.0f // 速度积分上限，deg/s
#define LAUNCHER_DIAL_SPEED_OUT_MAX       1500.0f // 单发电流限幅，原始量

/* 待发独立阻尼 */
#define LAUNCHER_DIAL_HOLD_ANGLE_KP        0.04f // 位置增益，(deg/s)/count
#define LAUNCHER_DIAL_HOLD_DEADBAND        100.0f // 保持误差死区，count
#define LAUNCHER_DIAL_HOLD_SPEED_KP        0.15f // 速度增益，原始量/(deg/s)
#define LAUNCHER_DIAL_HOLD_SPEED_KI        0.0f // 积分增益，原始量/累积误差
#define LAUNCHER_DIAL_HOLD_SPEED_KD        0.0f // 微分增益，原始量/误差差值
#define LAUNCHER_DIAL_SETTLE_SPEED_DPS     20u // 到位速度上限，deg/s
#define LAUNCHER_DIAL_SETTLE_TIME_MS       20u // 连续到位确认，ms
#define LAUNCHER_DIAL_START_TIMEOUT_MS     50u // 单发启动等待上限，ms

#define LAUNCHER_DIAL_ANGLE_SIGN          1.0f // 编码器极性，正负1
#define LAUNCHER_DIAL_SPEED_SIGN          1.0f // 转速反馈极性，正负1
#define LAUNCHER_DIAL_OUTPUT_SIGN         1.0f // 电流输出极性，正负1
#define LAUNCHER_DIAL_DIRECTION           1.0f // 供弹旋向，正负1
#define LAUNCHER_DIAL_CURRENT_LIMIT       2000.0f // 拨盘电流限幅，原始量
#define LAUNCHER_DIAL_MAX_SPEED_DPS       7000u // 位置环速度限幅，deg/s
#define LAUNCHER_DIAL_ONE_SHOT_ANGLE      65536.0f // 每发电机位移，count
#define LAUNCHER_DIAL_REVERSE_ANGLE       65536.0f // 堵转退让位移，count
#define LAUNCHER_DIAL_STOP_ERROR          500.0f // 到位位置容差，count
#define LAUNCHER_DIAL_SINGLE_TIMEOUT_MS   500u // 单发动作上限，ms
#define LAUNCHER_DIAL_REVERSE_TIMEOUT_MS  200u // 反向退让上限，ms
#define LAUNCHER_DIAL_RELOAD_TIMEOUT_MS   200u // 恢复定位上限，ms

/* 连发独立速度环 */
#define LAUNCHER_DIAL_REPEAT_SPEED_DPS    6000u // 连发机械速度上限，deg/s
#define LAUNCHER_DIAL_REPEAT_KP           0.35f // 速度增益，原始量/(deg/s)
#define LAUNCHER_DIAL_REPEAT_KI           0.0f // 积分增益，原始量/累积误差
#define LAUNCHER_DIAL_REPEAT_KD           0.0f // 微分增益，原始量/误差差值
#define LAUNCHER_DIAL_REPEAT_INTEGRAL_MAX 500.0f // 连发积分上限，deg/s
#define LAUNCHER_DIAL_REPEAT_OUT_MAX      1500.0f // 连发电流限幅，原始量

/* 可选堵转退让 */
#define LAUNCHER_DIAL_JAM_ENABLE          0u // 堵转退让使能，0/1
#define LAUNCHER_DIAL_JAM_CURRENT_RAW     600 // 堵转电流下限，原始量
#define LAUNCHER_DIAL_JAM_SPEED_DPS       10 // 堵转速度上限，deg/s
#define LAUNCHER_DIAL_JAM_CONFIRM_TICKS   200u // 堵转确认，1ms周期
#define LAUNCHER_DIAL_SAFE_STOP_RETRY_MS  50u // 失能停机重发间隔，ms

/* 制动后保持停止位置 */
#define LAUNCHER_DIAL_BRAKE_KP            0.1f // 制动增益，原始量/(deg/s)
#define LAUNCHER_DIAL_BRAKE_KI            0.0f // 积分增益，原始量/累积误差
#define LAUNCHER_DIAL_BRAKE_KD            0.0f // 微分增益，原始量/误差差值
#define LAUNCHER_DIAL_BRAKE_INTEGRAL_MAX  0.0f // 制动积分上限，deg/s
#define LAUNCHER_DIAL_BRAKE_OUT_MAX       1500.0f // 制动电流限幅，原始量
#define LAUNCHER_DIAL_BRAKE_STOP_SPEED_DPS 20u // 停止速度上限，deg/s
#define LAUNCHER_DIAL_BRAKE_TIMEOUT_MS    120u // 制动等待上限，ms

/* 17 mm热量预算与限频 */
#define LAUNCHER_HEAT_PER_SHOT             10.0f // 每发增热，热量单位
#define LAUNCHER_HEAT_WARN                200.0f // 降速起点，热量单位
#define LAUNCHER_HEAT_SATURATE             50.0f // 平衡区起点，热量单位
#define LAUNCHER_HEAT_MARGIN               20.0f // 安全余量，热量单位，至少10
#define LAUNCHER_HEAT_STOP LAUNCHER_HEAT_MARGIN // 连发停发余量，热量单位
#define LAUNCHER_HEAT_RESUME               30.0f // 恢复余量，热量单位
#define LAUNCHER_HEAT_MAX_RATE             15.0f // 最高射频，发/s
#define LAUNCHER_HEAT_D3_TIMEOUT_MS        100u // 热量链路超时，ms
#define LAUNCHER_HEAT_TRAINING_ENABLE        0u // 固定参数训练，0/1
#define LAUNCHER_HEAT_TRAINING_LIMIT       0.0f // 训练上限，热量单位
#define LAUNCHER_HEAT_TRAINING_COOLING     0.0f // 训练冷却，热量单位/s

#endif
