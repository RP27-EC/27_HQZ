/* supercap_protocol.h - 超电 CAN 帧定义 */

#ifndef __SUPERCAP_PROTOCOL_H
#define __SUPERCAP_PROTOCOL_H

#include <stdint.h>

#define SUPERCAP_CAN_TX_ID    0x222u
#define SUPERCAP_CAN_RX_ID    0x211u

typedef struct
{
    int16_t chassis_power;  // 底盘功率
    int16_t voltage_raw;  // 电容电压原始值
    int16_t current_raw;  // 电容电流原始值
    uint8_t ability;  // 超电就绪标志
    uint8_t pre_charge_mode;  // 预充电状态
} SuperCap_Feedback_t;

typedef struct
{
    uint8_t power_buffer;  // 缓冲能量
    uint16_t power_limit;  // 功率上限
    int16_t power_out_limit;  // 输出功率上限
    uint16_t power_in_limit;  // 输入功率上限
    uint8_t cap_switch;  // 放电开关
    uint8_t turbo_mode;  // 涡轮模式
    uint8_t pre_charge_enable;  // 预充电使能
} SuperCap_Control_t;

void SuperCap_Protocol_Decode(const uint8_t *data, SuperCap_Feedback_t *feedback);
void SuperCap_Protocol_Encode(const SuperCap_Control_t *control, uint8_t *data);

#endif
