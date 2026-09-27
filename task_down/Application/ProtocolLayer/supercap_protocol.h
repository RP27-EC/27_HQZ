/* supercap_protocol.h - 超电 CAN 帧定义 */

#ifndef __SUPERCAP_PROTOCOL_H
#define __SUPERCAP_PROTOCOL_H

#include <stdint.h>

#define SUPERCAP_CAN_TX_ID    0x222u
#define SUPERCAP_CAN_RX_ID    0x211u

typedef struct
{
    int16_t chassis_power;
    int16_t voltage_raw;
    int16_t current_raw;
    uint8_t ability;
    uint8_t pre_charge_mode;
} SuperCap_Feedback_t;

typedef struct
{
    uint8_t power_buffer;
    uint16_t power_limit;
    int16_t power_out_limit;
    uint16_t power_in_limit;
    uint8_t cap_switch;
    uint8_t turbo_mode;
    uint8_t pre_charge_enable;
} SuperCap_Control_t;

void SuperCap_Protocol_Decode(const uint8_t *data, SuperCap_Feedback_t *feedback);
void SuperCap_Protocol_Encode(const SuperCap_Control_t *control, uint8_t *data);

#endif
