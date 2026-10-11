/* device.c - 设备统一初始化 */
#include "device.h"
#include "judge.h"
#include "board_protocol.h"
#include "rc_sensor.h"
#include "launch.h"
#include "chassis_control.h"
#include "chassis_follow.h"
#include "chassis_spin.h"
#include "chassis_input.h"
#include "supercap.h"
void DEVICE_Init(void)
{
#if SUPERCAP_BRINGUP_ENABLE
    SuperCap_Init();
#endif
    /* 裁判心跳供功率预算使用 */
    judge.init(&judge);
    rc_dev.init(&rc_dev);
    board.init(&board);
    launch.init(&launch);
    rm_motor_list_init();
    Chassis_Follow_Init();
    Chassis_Spin_Init();
    Chassis_Input_Init();
    Chassis_Control_Init();
}
