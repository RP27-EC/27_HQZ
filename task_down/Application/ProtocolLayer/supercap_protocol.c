/* supercap_protocol.c - 超电 CAN 帧编解码 */

#include "supercap_protocol.h"
#include <string.h>

static int16_t SuperCap_ReadInt16(const uint8_t *data)
{
    /* 模板按低字节在前 */
    return (int16_t)(((uint16_t)data[1] << 8) | data[0]);
}

void SuperCap_Protocol_Decode(const uint8_t *data, SuperCap_Feedback_t *feedback)
{
    feedback->chassis_power = SuperCap_ReadInt16(&data[0]);
    feedback->voltage_raw = SuperCap_ReadInt16(&data[2]);
    feedback->current_raw = SuperCap_ReadInt16(&data[4]);
    feedback->ability = data[6] & 0x01u;
    feedback->pre_charge_mode = (data[6] >> 1) & 0x01u;
}

void SuperCap_Protocol_Encode(const SuperCap_Control_t *control, uint8_t *data)
{
    memset(data, 0, 8);

    data[0] = control->power_buffer;
    data[1] = (uint8_t)control->power_limit;
    data[2] = (uint8_t)(control->power_limit >> 8);
    data[3] = (uint8_t)control->power_out_limit;
    data[4] = (uint8_t)((uint16_t)control->power_out_limit >> 8);
    data[5] = (uint8_t)control->power_in_limit;
    data[6] = (uint8_t)(control->power_in_limit >> 8);
    data[7] = (uint8_t)((control->cap_switch & 0x01u) |
                        ((control->turbo_mode & 0x01u) << 1) |
                        ((control->pre_charge_enable & 0x01u) << 2));
}
