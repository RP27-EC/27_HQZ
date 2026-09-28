/* supercap_config.h - 超电通信调试参数 */

#ifndef __SUPERCAP_CONFIG_H
#define __SUPERCAP_CONFIG_H

#define SUPERCAP_OFFLINE_TIMEOUT_MS    100u

/* 仅保活通信，不让超电参与功率输出 */
#define SUPERCAP_CAP_SWITCH            0u
#define SUPERCAP_TURBO_MODE            0u
#define SUPERCAP_PRE_CHARGE_ENABLE     0u
#define SUPERCAP_POWER_BUFFER          0u
#define SUPERCAP_POWER_LIMIT           0u
#define SUPERCAP_POWER_OUT_LIMIT       0
#define SUPERCAP_POWER_IN_LIMIT        0u

#endif
