/* control_task.c - 控制任务 */

#include "control_task.h"
#include "communicate.h"
#include "imu_sensor.h"
#include "module.h"
#include "motor.h"
#include "rp_device_config.h"
#include "launcher.h"

/* 底盘失能时电机卸力 */
static void gimbal_can_send(void)
{
    if ((Board_HeartBeat.status == DEV_ONLINE) &&
        (Board_Rx_Info.state_pkt.car_state != 0))
    {
        DM_Group.group_set_torque(&DM_Group);
    }
    else
    {
        DM_Group.group_sleep(&DM_Group);
        DM_Group.group_set_torque(&DM_Group);
    }
}

/* 上板控制任务，IMU -> 模块 -> CAN -> 发射 -> 通信 */
void StartControlTask(void const *argument)
{
    (void)argument;
    uint32_t next_tick = osKernelSysTick();

    for (;;)
    {
        /* 更新 IMU 数据 */
        if ((imu_dev.work_state.err_code == IMU_E_NONE) ||
            (imu_dev.work_state.err_code == IMU_E_CALI))
        {
            imu_dev.update(&imu_dev);
        }

        Module_Work();
        gimbal_can_send();
        Launcher_Work();
        Send_To_Down_Board();

        if (osDelayUntil(&next_tick, 1u) != osOK)
        {
            next_tick = osKernelSysTick();
        }
    }
}



