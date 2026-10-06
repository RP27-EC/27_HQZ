#ifndef __LAUNCHER_CONFIG_H
#define __LAUNCHER_CONFIG_H

/* 摩擦轮：与参考车参数一致。 */
#define LAUNCHER_DIAL_ENABLE              1u
#define LAUNCHER_REPEAT_ENABLE            1u
#define LAUNCHER_FRIC_TARGET_RPM          1500.0f
#define LAUNCHER_FRIC_RAMP_RPM_PER_MS     20.0f
#define LAUNCHER_FRIC_STOP_RAMP_RPM_PER_MS 20.0f
#define LAUNCHER_FRIC_STOP_SPEED_RPM      100.0f
#define LAUNCHER_FRIC_STOP_CONFIRM_MS     1000u
#define LAUNCHER_FRIC_READY_TOL_RPM       500.0f
#define LAUNCHER_FRIC_READY_TIME_MS       100u
#define LAUNCHER_FRIC_KP                  2.0f
#define LAUNCHER_FRIC_KI                  1.0f
#define LAUNCHER_FRIC_KD                  0.0f
#define LAUNCHER_FRIC_INTEGRAL_MAX        500.0f
#define LAUNCHER_FRIC_KFF                 0.0f
#define LAUNCHER_FRIC_OUT_MAX             5000.0f
#define LAUNCHER_FRIC_L_DIRECTION         1.0f
#define LAUNCHER_FRIC_R_DIRECTION         -1.0f

/* 拨盘：65536 count 为一圈，一发走一圈。 */
#define LAUNCHER_DIAL_AUTO_RESET_ENABLE   0u
#define LAUNCHER_DIAL_READY_HOLD_ENABLE   1u

#define LAUNCHER_DIAL_ANGLE_KP            0.08f
#define LAUNCHER_DIAL_ANGLE_KI            0.0f
#define LAUNCHER_DIAL_ANGLE_KD            0.0f
#define LAUNCHER_DIAL_ANGLE_INTEGRAL_MAX  0.0f
#define LAUNCHER_DIAL_ANGLE_DEADBAND      0.0f

#define LAUNCHER_DIAL_SPEED_KP            0.20f
#define LAUNCHER_DIAL_SPEED_KI            0.05f
#define LAUNCHER_DIAL_SPEED_KD            0.0f
#define LAUNCHER_DIAL_SPEED_INTEGRAL_MAX  500.0f
#define LAUNCHER_DIAL_SPEED_OUT_MAX       1500.0f

#define LAUNCHER_DIAL_ANGLE_SIGN          1.0f
#define LAUNCHER_DIAL_SPEED_SIGN          1.0f
#define LAUNCHER_DIAL_OUTPUT_SIGN         1.0f
#define LAUNCHER_DIAL_DIRECTION           1.0f

#define LAUNCHER_DIAL_CURRENT_LIMIT       2000.0f
#define LAUNCHER_DIAL_MAX_SPEED_DPS       7000u

#define LAUNCHER_DIAL_RESET_ANGLE         31259.0f
#define LAUNCHER_DIAL_RESET_TIMEOUT_MS    1000u

#define LAUNCHER_DIAL_ONE_SHOT_ANGLE      65536.0f
#define LAUNCHER_DIAL_REVERSE_ANGLE       65536.0f
#define LAUNCHER_DIAL_STOP_ERROR          500.0f
#define LAUNCHER_DIAL_SINGLE_TIMEOUT_MS   500u
#define LAUNCHER_DIAL_REVERSE_TIMEOUT_MS  200u
#define LAUNCHER_DIAL_RELOAD_TIMEOUT_MS   200u

/* 连发独立速度环：15 圈/s。 */
#define LAUNCHER_DIAL_REPEAT_SPEED_DPS    6000u
#define LAUNCHER_DIAL_REPEAT_KP           0.35f
#define LAUNCHER_DIAL_REPEAT_KI           0.0f
#define LAUNCHER_DIAL_REPEAT_KD           0.0f
#define LAUNCHER_DIAL_REPEAT_INTEGRAL_MAX 500.0f
#define LAUNCHER_DIAL_REPEAT_OUT_MAX      1500.0f

/* 堵转后反向退让，再回到原供弹目标。 */
#define LAUNCHER_DIAL_JAM_ENABLE          0u
#define LAUNCHER_DIAL_JAM_CURRENT_RAW     600
#define LAUNCHER_DIAL_JAM_SPEED_DPS       10
#define LAUNCHER_DIAL_JAM_CONFIRM_TICKS   200u

#define LAUNCHER_DIAL_SAFE_STOP_RETRY_MS  50u

/* 拨盘释放后的主动制动，避免直接断力后的惯性和回弹。 */
#define LAUNCHER_DIAL_BRAKE_KP            0.2f
#define LAUNCHER_DIAL_BRAKE_KI            0.0f
#define LAUNCHER_DIAL_BRAKE_KD            0.0f
#define LAUNCHER_DIAL_BRAKE_INTEGRAL_MAX  0.0f
#define LAUNCHER_DIAL_BRAKE_OUT_MAX       1500.0f
#define LAUNCHER_DIAL_BRAKE_STOP_SPEED_DPS 20u
#define LAUNCHER_DIAL_BRAKE_TIMEOUT_MS    120u

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
