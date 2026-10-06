/* rc_protocol.h - 遥控器协议解析 */

#ifndef __RC_PROTOCOL_H
#define __RC_PROTOCOL_H
#include "rp_config.h"
#include "rc_sensor.h"
typedef struct
{
    uint32_t press_seq; // 左键按下序号，uint32循环
    uint32_t release_seq; // 左键释放序号，uint32循环
    uint8_t pressed; // 当前左键电平，0/1
} rc_mouse_shot_t;

void Rc_GetMouseShotSnapshot(rc_mouse_shot_t *snapshot);
void keyboard_update(rc_data_t *info);
void rc_interrupt_update(rc_dev_t *rc_sen);
void rc_init(rc_dev_t *rc_sen);

#endif

