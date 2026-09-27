/* supercap.h - 超电通信设备 */

#ifndef __SUPERCAP_H
#define __SUPERCAP_H

#include "stm32h7xx_hal.h"
#include "supercap_config.h"
#include "supercap_protocol.h"

typedef enum
{
    SUPERCAP_STATE_OFFLINE = 0,
    SUPERCAP_STATE_ONLINE = 1
} SuperCap_State_e;

typedef struct
{
    SuperCap_State_e state;
    SuperCap_Feedback_t feedback;
    int16_t chassis_power;
    float cap_voltage;
    float cap_current;
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t last_rx_ms;
    uint16_t offline_count;
} SuperCap_t;

extern SuperCap_t supercap;

void SuperCap_Init(void);
void SuperCap_Tx(void);
void SuperCap_Rx(const uint8_t *rx_buf);
void SuperCap_Heartbeat(void);

#endif
