/* device.c - 设备统一初始化 */
#include "device.h"
#include "judge.h"
#include "cap.h"
#include "board_protocol.h"
#include "rc_sensor.h"
#include "infantry.h"
#include "chassis.h"
#include "gimbal.h"
#include "launch.h"
#include "board_comm_config.h"
#include "chassis_config.h"
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
    /* 裁判对象初始化独立于调试分支，功率限制依赖它的在线状态与超时计数 */
    judge.init(&judge);
#if BOARD_COMM_DEBUG
    rc_dev.init(&rc_dev);
    board.init(&board);
#if CHASSIS_BRINGUP_ENABLE
    launch.init(&launch);
    rm_motor_list_init();
    Chassis_Follow_Init();
    Chassis_Spin_Init();
    Chassis_Input_Init();
    Chassis_Control_Init();
#endif
#else
    imu_dev.init(&imu_dev);
    rc_dev.init(&rc_dev);
    rm_motor_list_init();
    kt_motor_list_init();
    ht_motor_list_init();
    dm_motor_list_init();

    cap.init(&cap);
    board.init(&board);

    chassis.init(&chassis);
    gimbal.init(&gimbal);
    launch.init(&launch);
    infantry.init(&infantry);
#endif
}

