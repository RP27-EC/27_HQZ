/* device.c - 设备统一初始化 */

#include "device.h"
#include "board_protocol.h"
#include "rc_sensor.h"
#include "launch.h"
#include "motor.h"
#include "chassis_control.h"
#include "chassis_follow.h"
#include "chassis_spin.h"
#include "chassis_input.h"
#include "supercap.h"

void DEVICE_Init(void)
{
    SuperCap_Init();
    rc_dev.init(&rc_dev);
    board.init(&board);
    launch.init(&launch);
    rm_motor_list_init();
    Chassis_Follow_Init();
    Chassis_Spin_Init();
    Chassis_Input_Init();
    Chassis_Control_Init();
}
