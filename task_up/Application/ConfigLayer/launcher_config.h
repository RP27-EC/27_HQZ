/* launcher_config.h - 发射机构参数配置 */

#ifndef __LAUNCHER_CONFIG_H
#define __LAUNCHER_CONFIG_H

/* 摩擦轮：与参考车参数一致。 */
#define LAUNCHER_DIAL_ENABLE              1u  // 拨盘控制开关, 0=只控摩擦轮
#define LAUNCHER_REPEAT_ENABLE            1u  // 连发使能
#define LAUNCHER_FRIC_TARGET_RPM          1500.0f  // 摩擦轮目标转速
#define LAUNCHER_FRIC_RAMP_RPM_PER_MS     20.0f  // 升速斜坡, 每 ms 增量
#define LAUNCHER_FRIC_STOP_RAMP_RPM_PER_MS 20.0f  // 降速斜坡, 每 ms 减量
#define LAUNCHER_FRIC_STOP_SPEED_RPM      100.0f  // 判停转速阈值
#define LAUNCHER_FRIC_STOP_CONFIRM_MS     1000u  // 停稳确认时长
#define LAUNCHER_FRIC_READY_TOL_RPM       500.0f  // 转速达标容差
#define LAUNCHER_FRIC_READY_TIME_MS       100u  // 达标持续时间
#define LAUNCHER_FRIC_KP                  2.0f  // 摩擦轮速度环 P
#define LAUNCHER_FRIC_KI                  1.0f  // 摩擦轮速度环 I
#define LAUNCHER_FRIC_KD                  0.0f  // 摩擦轮速度环 D
#define LAUNCHER_FRIC_INTEGRAL_MAX        500.0f  // 速度环积分限幅
#define LAUNCHER_FRIC_KFF                 0.0f  // 转速前馈系数
#define LAUNCHER_FRIC_OUT_MAX             5000.0f  // 摩擦轮输出限幅
#define LAUNCHER_FRIC_L_DIRECTION         1.0f  // 左轮转向修正
#define LAUNCHER_FRIC_R_DIRECTION         -1.0f  // 右轮转向修正

/* 拨盘：65536 count 为一圈，一发走一圈。 */
#define LAUNCHER_DIAL_AUTO_RESET_ENABLE   0u  // 拨盘自动回零
#define LAUNCHER_DIAL_READY_HOLD_ENABLE   1u  // 待发时保持位置

#define LAUNCHER_DIAL_ANGLE_KP            0.08f  // 拨盘角度环 P
#define LAUNCHER_DIAL_ANGLE_KI            0.0f  // 拨盘角度环 I
#define LAUNCHER_DIAL_ANGLE_KD            0.0f  // 拨盘角度环 D
#define LAUNCHER_DIAL_ANGLE_INTEGRAL_MAX  0.0f  // 角度环积分限幅
#define LAUNCHER_DIAL_ANGLE_DEADBAND      0.0f  // 角度环死区

#define LAUNCHER_DIAL_SPEED_KP            0.15f  // 拨盘速度环 P
#define LAUNCHER_DIAL_SPEED_KI            0.05f  // 拨盘速度环 I
#define LAUNCHER_DIAL_SPEED_KD            0.0f  // 拨盘速度环 D
#define LAUNCHER_DIAL_SPEED_INTEGRAL_MAX  500.0f  // 速度环积分限幅
#define LAUNCHER_DIAL_SPEED_OUT_MAX       1500.0f  // 速度环输出限幅

#define LAUNCHER_DIAL_ANGLE_SIGN          1.0f  // 角度方向修正
#define LAUNCHER_DIAL_SPEED_SIGN          1.0f  // 速度方向修正
#define LAUNCHER_DIAL_OUTPUT_SIGN         1.0f  // 电流方向修正
#define LAUNCHER_DIAL_DIRECTION           1.0f  // 发弹方向

#define LAUNCHER_DIAL_CURRENT_LIMIT       2000.0f  // 拨盘电流限幅
#define LAUNCHER_DIAL_MAX_SPEED_DPS       7000u  // 拨盘最大转速

#define LAUNCHER_DIAL_RESET_ANGLE         31259.0f  // 回零目标角度
#define LAUNCHER_DIAL_RESET_TIMEOUT_MS    1000u  // 回零超时

#define LAUNCHER_DIAL_ONE_SHOT_ANGLE      65536.0f  // 单发转过角度
#define LAUNCHER_DIAL_REVERSE_ANGLE       65536.0f  // 退弹反转角度
#define LAUNCHER_DIAL_STOP_ERROR          500.0f  // 到位误差容限
#define LAUNCHER_DIAL_SINGLE_TIMEOUT_MS   500u  // 单发超时
#define LAUNCHER_DIAL_REVERSE_TIMEOUT_MS  200u  // 反转超时
#define LAUNCHER_DIAL_RELOAD_TIMEOUT_MS   200u  // 补弹超时

/* 连发独立速度环：15 圈/s。 */
#define LAUNCHER_DIAL_REPEAT_SPEED_DPS    5400u  // 连发转速
#define LAUNCHER_DIAL_REPEAT_KP           0.05f  // 连发速度环 P
#define LAUNCHER_DIAL_REPEAT_KI           0.0f  // 连发速度环 I
#define LAUNCHER_DIAL_REPEAT_KD           0.0f  // 连发速度环 D
#define LAUNCHER_DIAL_REPEAT_INTEGRAL_MAX 500.0f  // 连发积分限幅
#define LAUNCHER_DIAL_REPEAT_OUT_MAX      1500.0f  // 连发输出限幅

/* 堵转后反向退让，再回到原供弹目标。 */
#define LAUNCHER_DIAL_JAM_ENABLE          0u  // 卡弹检测开关
#define LAUNCHER_DIAL_JAM_CURRENT_RAW     600  // 卡弹判定电流阈值
#define LAUNCHER_DIAL_JAM_SPEED_DPS       10  // 卡弹判定转速阈值
#define LAUNCHER_DIAL_JAM_CONFIRM_TICKS   200u  // 卡弹确认节拍数

#define LAUNCHER_DIAL_SAFE_STOP_RETRY_MS  50u  // 安全停机重试间隔

/* 拨盘释放后的主动制动，避免直接断力后的惯性和回弹。 */
#define LAUNCHER_DIAL_BRAKE_KP            0.2f  // 拨盘制动环 P
#define LAUNCHER_DIAL_BRAKE_KI            0.0f  // 拨盘制动环 I
#define LAUNCHER_DIAL_BRAKE_KD            0.0f  // 拨盘制动环 D
#define LAUNCHER_DIAL_BRAKE_INTEGRAL_MAX  0.0f  // 制动积分限幅
#define LAUNCHER_DIAL_BRAKE_OUT_MAX       1500.0f  // 制动输出限幅
#define LAUNCHER_DIAL_BRAKE_STOP_SPEED_DPS 20u  // 制动停止转速
#define LAUNCHER_DIAL_BRAKE_TIMEOUT_MS    120u  // 制动超时

#endif
