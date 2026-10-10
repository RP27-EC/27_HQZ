/* cap.h - 超级电容设备 */

#ifndef __CAP_H
#define __CAP_H

#include "stm32h7xx_hal.h"
#include "cap_protocol.h"
#include "rp_config.h"
extern cap_t cap;


void Cap_Init(cap_t* my_cap);
#endif

