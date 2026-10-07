/* lift_config.h - 狗洞升降参数 */

#ifndef __LIFT_CONFIG_H
#define __LIFT_CONFIG_H

#define LIFT_POSITION_COUNTS_PER_UNIT   1.0f     // 位置换算系数，count/单位
#define LIFT_TRAVEL_TURNS               280.0f   // 当前行程设置，电机圈
#define LIFT_TRAVEL_COUNTS              (LIFT_TRAVEL_TURNS * 8192.0f) // 行程，count
#define LIFT_UP_DIRECTION               1.0f     // 上行位置符号，仅±1
#define LIFT_OUTPUT_DIRECTION           1.0f     // 输出方向，仅±1
#define LIFT_SPEED_DIRECTION            1.0f     // 速度符号，仅±1

#define LIFT_HOME_SPEED_RPM             2865.0f  // 找顶速度，rpm
#define LIFT_HOME_CURRENT_RAW           520.0f   // 找顶电流阈值，原始量
#define LIFT_HOME_STALL_SPEED_RPM       1.0f     // 顶部低速阈值，rpm
#define LIFT_HOME_SPEED_GRACE_MS        0u       // 找顶启动宽限，ms
#define LIFT_AUTO_HOME_DELAY_MS         2000u    // 自动找顶延时，ms
#define LIFT_HOME_STATIONARY_WINDOW_MS  500u     // 位置观察窗，ms
#define LIFT_HOME_STATIONARY_COUNTS     40.0f    // 窗内最大位移，count
#define LIFT_HOME_CONFIRM_MS            500u     // 找顶条件确认时长，ms
#define LIFT_HOME_TIMEOUT_MS            90000u   // 找顶超时，ms
#define LIFT_HOME_OUTPUT_LIMIT_RAW      4444.0f  // 找顶输出上限，原始量

#define LIFT_RETRACT_COUNTS             (5.0f * 8192.0f) // 找顶回退量，count
#define LIFT_RETRACT_SPEED_RPM          2865.0f   // 回退速度，rpm
#define LIFT_RETRACT_OUTPUT_LIMIT_RAW   2500.0f   // 回退输出上限，原始量

#define LIFT_POS_KP                     0.0064f  // 比例增益，rpm/count
#define LIFT_POS_KI                     0.0f     // 积分增益，按周期累加
#define LIFT_POS_KD                     0.0f     // 微分增益，按周期差分
#define LIFT_POS_INTEGRAL_MAX           0.0f     // 误差累加限幅，count·周期
#define LIFT_POS_OUT_MAX_RPM            9549.0f  // 位置环输出上限，rpm
#define LIFT_POS_MIN_SPEED_RPM          95.5f    // 最低运动速度，rpm
#define LIFT_MOVE_SPEED_RPM             9549.0f  // 运动速度上限，rpm

#define LIFT_SPEED_KP                   20.0f    // 比例增益，原始量/rpm
#define LIFT_SPEED_KI                   10.0f    // 积分增益，按周期累加
#define LIFT_SPEED_KD                   0.0f     // 微分增益，按周期差分
#define LIFT_SPEED_INTEGRAL_MAX         0.0f     // 误差累加限幅，rpm·周期
#define LIFT_SPEED_OUT_MAX_RAW          4444.0f  // 速度环输出上限，原始量

#define LIFT_POS_TOL_COUNTS             200.0f   // 到位位置误差，count
#define LIFT_SPEED_TOL_RPM              1.0f     // 到位速度误差，rpm
#define LIFT_STABLE_CONFIRM_MS          100u     // 到位稳定时长，ms

#define LIFT_MOVE_TIMEOUT_MS            90000u   // 单次运动超时，ms
#define LIFT_DOWN_OVER_CURRENT_RAW      75.0f    // 下行过流阈值，原始量
#define LIFT_UP_OVER_CURRENT_RAW        520.0f   // 上行过流阈值，原始量
#define LIFT_OVER_CURRENT_CONFIRM_MS    500u     // 通用过流确认，ms
#define LIFT_DOWN_OVER_CURRENT_CONFIRM_MS 200u   // 下行过流确认，ms
#define LIFT_UP_OVER_CURRENT_CONFIRM_MS   500u   // 上行过流确认，ms
#define LIFT_STALL_PROGRESS_COUNTS        40.0f  // 堵转观察最小进展，count

#define LIFT_ALIGN_TOL_DEG              5.0f     // 云台对齐误差，deg
#define LIFT_FRONT_TOL_DEG              10.0f    // 前向角度容差，deg
#define LIFT_OVERTRAVEL_COUNTS          40.0f    // 允许越程，count

#define LIFT_DEBUG_OUTPUT_MAX_RAW       1500.0f  // 手动输出上限，原始量
#define LIFT_ALIGN_SPEED_RAD_S          0.2f     // 对齐速度上限，rad/s

#endif
