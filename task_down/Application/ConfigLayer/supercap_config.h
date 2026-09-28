/* supercap_config.h - 超电通信调试参数 */

#ifndef __SUPERCAP_CONFIG_H
#define __SUPERCAP_CONFIG_H

#define SUPERCAP_OFFLINE_TIMEOUT_MS    100u  // 离线判定超时

/* 仅保活通信，不让超电参与功率输出 */
#define SUPERCAP_CAP_SWITCH            0u  // 超电放电开关
#define SUPERCAP_TURBO_MODE            0u  // 涡轮模式
#define SUPERCAP_PRE_CHARGE_ENABLE     0u  // 预充电使能
#define SUPERCAP_POWER_BUFFER          0u  // 缓冲能量目标
#define SUPERCAP_POWER_LIMIT           0u  // 功率上限
#define SUPERCAP_POWER_OUT_LIMIT       0  // 输出功率上限
#define SUPERCAP_POWER_IN_LIMIT        0u  // 输入功率上限

#endif
