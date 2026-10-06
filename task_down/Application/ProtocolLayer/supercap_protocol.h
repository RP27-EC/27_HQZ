/* supercap_protocol.h - 超电 CAN 帧定义 */

#ifndef __SUPERCAP_PROTOCOL_H
#define __SUPERCAP_PROTOCOL_H

#include <stdint.h>

#define SUPERCAP_CAN_TX_ID    0x222u // 控制帧ID，11位
#define SUPERCAP_CAN_RX_ID    0x211u // 反馈帧ID，11位

/* 16位字段按低字节在前 */

typedef struct
{
    int16_t chassis_power; // 底盘功率计数，量纲待确认
    int16_t voltage_raw; // 电压映射计数，±32000
    int16_t current_raw; // 电流映射计数，±32000
    uint8_t ability; // 放电能力标志，0/1
    uint8_t pre_charge_mode; // 预充状态，0/1
} SuperCap_Feedback_t;

typedef struct
{
    uint8_t power_buffer; // 裁判缓冲，J，0~255
    uint16_t power_limit; // 裁判功率上限，W
    int16_t power_out_limit; // 超电输出限制，int16计数
    uint16_t power_in_limit; // 超电输入限制，uint16计数
    uint8_t cap_switch; // 超电开关，0/1
    uint8_t turbo_mode; // 增强模式开关，0/1
    uint8_t pre_charge_enable; // 预充开关，0/1
} SuperCap_Control_t;

void SuperCap_Protocol_Decode(const uint8_t *data, SuperCap_Feedback_t *feedback);
void SuperCap_Protocol_Encode(const SuperCap_Control_t *control, uint8_t *data);

#endif
