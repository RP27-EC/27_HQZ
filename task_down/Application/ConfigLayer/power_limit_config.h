/* power_limit_config.h - 底盘功率限制参数 */
#ifndef __POWER_LIMIT_CONFIG_H
#define __POWER_LIMIT_CONFIG_H

#define CHASSIS_POWER_LIMIT_ENABLE  1u     // 0旁路，1启用
#define CHASSIS_POWER_SAFETY_K      1.0f   // 在线预算系数，0.1~1
#define CHASSIS_POWER_FALLBACK_W    45.0f  // 固定回退预算，W
#define CHASSIS_POWER_MIN_W         20.0f  // 合法裁判上限下界，W
#define CHASSIS_POWER_MAX_W         120.0f // 合法裁判上限上界，W
#define CHASSIS_POWER_BUFFER_FULL_J 60.0f  // 缓冲钳位上界，J
#define CHASSIS_POWER_BUFFER_FLOOR  0.5f   // 缓冲降额下界，0~1
#define CHASSIS_POWER_SEARCH_STEPS  10u    // 公共比例二分次数，10

#endif
