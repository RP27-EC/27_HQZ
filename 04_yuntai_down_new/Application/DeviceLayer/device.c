/**
 * @file  device.c
 * @brief Minimal down-board device initialization.
 */

#include "device.h"
#include "board_protocol.h"
#include "rc_sensor.h"

void DEVICE_Init(void)
{
    rc_sensor.init(&rc_sensor);
    board.init(&board);
}
