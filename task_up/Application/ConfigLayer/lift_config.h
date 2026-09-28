/* lift_config.h - 狗洞升降参数 */

#ifndef __LIFT_CONFIG_H
#define __LIFT_CONFIG_H

#define LIFT_POSITION_COUNTS_PER_UNIT   1.0f  // 位置换算系数
/* 临时测试行程，验证后改回 316 圈 */
#define LIFT_TRAVEL_TURNS               280.0f
#define LIFT_TRAVEL_COUNTS              (LIFT_TRAVEL_TURNS * 8192.0f)  // 行程总计数
/* encoder_sum 向上增大用 1.0f，否则用 -1.0f */
#define LIFT_UP_DIRECTION               1.0f
/* 负输出能使机构下降用 1.0f，否则用 -1.0f */
#define LIFT_OUTPUT_DIRECTION           1.0f
/* 速度反馈与位置反馈同号用 1.0f，反号用 -1.0f */
#define LIFT_SPEED_DIRECTION            1.0f

/* 上限位找零 */
/* 找顶速度 */
#define LIFT_HOME_SPEED_RPM             2865.0f
/* 低于正常峰值、高于正常堵转点 */
#define LIFT_HOME_CURRENT_RAW           520.0f
/* 顶部低速判定，反馈噪声大可提高 */
#define LIFT_HOME_STALL_SPEED_RPM       1.0f
/* 启动宽限，避免上电零速误判 */
#define LIFT_HOME_SPEED_GRACE_MS        0u
#define LIFT_AUTO_HOME_DELAY_MS         2000u  // 上电自动找零延时
#define LIFT_HOME_STATIONARY_WINDOW_MS  500u  // 静止判定窗口
#define LIFT_HOME_STATIONARY_COUNTS     40.0f  // 静止位移阈值
/* 条件连续成立时间 */
#define LIFT_HOME_CONFIRM_MS            500u
/* 超时未找到顶部则故障 */
#define LIFT_HOME_TIMEOUT_MS            90000u
/* 找顶最大输出，需高于找顶阈值 */
#define LIFT_HOME_OUTPUT_LIMIT_RAW      4444.0f

/* 到顶后向下回转，回转量不清零 */
#define LIFT_RETRACT_COUNTS             (5.0f * 8192.0f)  // 到顶后回转量
#define LIFT_RETRACT_SPEED_RPM          2865.0f  // 回转速度
#define LIFT_RETRACT_OUTPUT_LIMIT_RAW   2500.0f  // 回转输出限幅

/* 正常位置环和速度环 */
#define LIFT_POS_KP                     0.0064f  // 升降位置环 P
#define LIFT_POS_KI                     0.0f  // 升降位置环 I
#define LIFT_POS_KD                     0.0f  // 升降位置环 D
#define LIFT_POS_INTEGRAL_MAX           0.0f  // 位置环积分限幅
#define LIFT_POS_OUT_MAX_RPM            9549.0f  // 位置环输出限速
#define LIFT_POS_MIN_SPEED_RPM          95.5f  // 最小爬行速度
#define LIFT_MOVE_SPEED_RPM             9549.0f  // 最大运动速度

/* 先调位置环，再调速度环 */
#define LIFT_SPEED_KP                   20.0f  // 升降速度环 P
#define LIFT_SPEED_KI                   10.0f  // 升降速度环 I
#define LIFT_SPEED_KD                   0.0f  // 升降速度环 D
#define LIFT_SPEED_INTEGRAL_MAX         0.0f  // 速度环积分限幅
#define LIFT_SPEED_OUT_MAX_RAW          4444.0f  // 速度环输出限幅

/* 到位判定 */
#define LIFT_POS_TOL_COUNTS             200.0f  // 到位位移容差
#define LIFT_SPEED_TOL_RPM              1.0f  // 到位速度容差
#define LIFT_STABLE_CONFIRM_MS          100u  // 稳定确认时长

/* 方向独立堵转保护，下行可更低 */
#define LIFT_MOVE_TIMEOUT_MS            90000u  // 运动超时
#define LIFT_DOWN_OVER_CURRENT_RAW      75.0f  // 下行堵转电流阈值
#define LIFT_UP_OVER_CURRENT_RAW        520.0f  // 上行堵转电流阈值
#define LIFT_OVER_CURRENT_CONFIRM_MS    500u  // 堵转确认时长
#define LIFT_DOWN_OVER_CURRENT_CONFIRM_MS 200u  // 下行堵转确认
#define LIFT_UP_OVER_CURRENT_CONFIRM_MS   500u  // 上行堵转确认
#define LIFT_STALL_PROGRESS_COUNTS        40.0f  // 无进展判定阈值

/* 云台对齐和行程保护 */
#define LIFT_ALIGN_TOL_DEG              5.0f  // 云台对齐容差
#define LIFT_OVERTRAVEL_COUNTS          40.0f  // 超程保护阈值

/* 手动调试最大输出 */
#define LIFT_DEBUG_OUTPUT_MAX_RAW       1500.0f  // 手动调试输出限幅
/* 云台对齐到位速度阈值 */
#define LIFT_ALIGN_SPEED_RAD_S          0.2f  // 对齐到位速度阈值

#endif
