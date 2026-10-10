/* power_limit_config.h - 底盘功率限制参数 */
#ifndef __POWER_LIMIT_CONFIG_H
#define __POWER_LIMIT_CONFIG_H

#define CHASSIS_POWER_LIMIT_ENABLE  1u     // 0旁路，1启用
#define CHASSIS_POWER_FALLBACK_W    45.0f  // 固定回退预算，W
#define CHASSIS_POWER_MARGIN_W      5.0f   // 在线固定功率余量，W
#define CHASSIS_POWER_MIN_W         20.0f  // 合法裁判上限下界，W
#define CHASSIS_POWER_MAX_W         120.0f // 合法裁判上限上界，W
#define CHASSIS_POWER_BUFFER_FULL_J 60.0f  // 缓冲钳位上界，J
#define CHASSIS_POWER_BUFFER_TARGET_J 59.0f // 缓冲目标，J
#define CHASSIS_POWER_BUFFER_BAND_J 0.0f  // 目标半宽，J
#define CHASSIS_POWER_BUFFER_KP    2.0f   // 比例增益，W/J
#define CHASSIS_POWER_BUFFER_KI    0.5f   // 积分增益，W/(J·s)
#define CHASSIS_POWER_BUFFER_RELEASE_W_S 1.0f // 满缓冲扣减释放，W/s
#define CHASSIS_POWER_BUFFER_GUARD_J 45.0f // 强降额起点，J
#define CHASSIS_POWER_BUFFER_DT_MAX_MS 200u // 单帧积分上限，ms
#define CHASSIS_POWER_RECOVER_W_S  10.0f  // 预算恢复速度，W/s
#define CHASSIS_POWER_ACTIVE_V_M_S 0.001f // 平移有效阈值，m/s
#define CHASSIS_POWER_ACTIVE_W_RAD_S 0.001f // 转向有效阈值，rad/s
#define CHASSIS_POWER_SEARCH_STEPS  10u    // 公共比例二分次数，10

#endif
