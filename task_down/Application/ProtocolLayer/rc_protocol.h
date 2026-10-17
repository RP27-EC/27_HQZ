/* rc_protocol.h - 遥控器协议解析 */

#ifndef __RC_PROTOCOL_H
#define __RC_PROTOCOL_H
#include "rp_config.h"
#include "rc_sensor.h"

typedef struct
{
    int16_t raw_x; // 鼠标X计数，int16
    int16_t raw_y; // 鼠标Y计数，int16
    float filtered_x; // X窗口均值，count
    float filtered_y; // Y窗口均值，count
    float mech_wz; // 鼠标机械转向，rad/s
    float input_wz; // 仲裁前转向，rad/s
    float final_wz; // 仲裁后转向，rad/s
    float d5_yaw_rate; // D5目标转速，deg/s
    uint32_t input_tick; // 输入记录时刻，ms
    uint32_t final_tick; // 仲裁记录时刻，ms
    uint32_t d5_tick; // D5装箱时刻，ms
    uint8_t keyboard_active; // 键鼠输入有效，0/1
    uint8_t chassis_mode; // 跟随0机械1旋转2
    uint8_t d5_keyboard; // D5键鼠来源，0/1
} rc_mouse_debug_t;

extern volatile rc_mouse_debug_t rc_mouse_debug;

void rc_mouse_filter_reset(rc_dev_t *rc_sen);
void keyboard_update(rc_data_t *info);
void rc_interrupt_update(rc_dev_t *rc_sen);
void rc_init(rc_dev_t *rc_sen);

#endif

