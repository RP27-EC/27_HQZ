/* rc_protocol.h - 遥控器协议解析 */

#ifndef __RC_PROTOCOL_H
#define __RC_PROTOCOL_H
#include "rp_config.h"
#include "rc_sensor.h"
void keyboard_update(rc_data_t *info);
void rc_interrupt_update(rc_dev_t *rc_sen);
void rc_init(rc_dev_t *rc_sen);

#endif


