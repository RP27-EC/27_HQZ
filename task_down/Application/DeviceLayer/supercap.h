/* supercap.h - 超电通信设备 */

#ifndef __SUPERCAP_H
#define __SUPERCAP_H

#include "stm32h7xx_hal.h"
#include "supercap_config.h"
#include "supercap_protocol.h"

typedef enum
{
    SUPERCAP_STATE_OFFLINE = 0,  // 链路离线
    SUPERCAP_STATE_ONLINE = 1  // 链路在线
} SuperCap_State_e;

typedef struct
{
    SuperCap_State_e state;  // 链路状态
    SuperCap_Feedback_t feedback;  // 原始反馈帧
    int16_t chassis_power;  // 底盘功率
    float cap_voltage;  // 电容电压(V)
    float cap_current;  // 电容电流(A)
    uint32_t tx_count;  // 发送计数
    uint32_t rx_count;  // 接收计数
    uint32_t last_rx_ms;  // 最近接收时刻
    uint16_t offline_count;  // 离线计数
} SuperCap_t;

extern SuperCap_t supercap;

void SuperCap_Init(void);
void SuperCap_Tx(void);
void SuperCap_Rx(const uint8_t *rx_buf);
void SuperCap_Heartbeat(void);

#endif
