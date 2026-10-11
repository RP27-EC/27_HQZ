/* supercap.h - 超电通信设备 */

#ifndef __SUPERCAP_H
#define __SUPERCAP_H

#include "stm32h7xx_hal.h"
#include "supercap_config.h"
#include "supercap_protocol.h"

typedef enum
{
    SUPERCAP_STATE_OFFLINE = 0, // 未收有效反馈，状态0
    SUPERCAP_STATE_ONLINE = 1 // 收到有效反馈，状态1
} SuperCap_State_e;

typedef struct
{
    SuperCap_State_e state; // 超电在线状态，0/1
    SuperCap_Feedback_t feedback; // 原始反馈，量纲见协议类型
    int16_t chassis_power; // 底盘功率计数，量纲待确认
    float cap_voltage; // 解码电容电压，V
    float cap_current; // 解码电容电流，A
    uint32_t tx_count; // 发送尝试次数，uint32回绕
    uint32_t rx_count; // 接收次数，uint32回绕
    uint32_t last_rx_ms; // 最近接收时刻，ms
    uint16_t offline_count; // 心跳计数，饱和值为超时配置
} SuperCap_t;

extern SuperCap_t supercap;

void SuperCap_Init(void);
void SuperCap_Tx(void);
void SuperCap_Rx(const uint8_t *rx_buf);
void SuperCap_Heartbeat(void);

#endif
