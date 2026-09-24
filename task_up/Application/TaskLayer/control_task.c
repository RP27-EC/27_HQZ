/* control_task.c - 控制任务 */

#include "control_task.h"
#include "communicate.h"
#include "imu_sensor.h"
#include "module.h"
#include "motor.h"
#include "rc_protocol.h"
#include "rp_device_config.h"
#include "board_remote_config.h"
#include "launcher.h"

volatile imu_debug_t imu_dbg; /* IMU 在线调试快照 */

/* 将 IMU 内部数据同步到调试结构体 */

static void imu_debug_update(void)
{
    imu_data_t *info = imu_dev.info; /* 原始 IMU 数据源 */

    imu_dbg.acc_x = info->raw_info.acc_x;
    imu_dbg.acc_y = info->raw_info.acc_y;
    imu_dbg.acc_z = info->raw_info.acc_z;

    imu_dbg.gyro_x = info->raw_info.gyro_x;
    imu_dbg.gyro_y = info->raw_info.gyro_y;
    imu_dbg.gyro_z = info->raw_info.gyro_z;

    imu_dbg.yaw = info->base_info.yaw;
    imu_dbg.pitch = info->base_info.pitch;
    imu_dbg.roll = info->base_info.roll;

    imu_dbg.rate_yaw = info->base_info.rate_yaw;
    imu_dbg.rate_pitch = info->base_info.rate_pitch;
    imu_dbg.rate_roll = info->base_info.rate_roll;

    imu_dbg.ave_rate_yaw = info->base_info.ave_rate_yaw;
    imu_dbg.ave_rate_pitch = info->base_info.ave_rate_pitch;
    imu_dbg.ave_rate_roll = info->base_info.ave_rate_roll;

    imu_dbg.accx = info->base_info.accx;
    imu_dbg.accy = info->base_info.accy;
    imu_dbg.accz = info->base_info.accz;
    imu_dbg.temperature = info->base_info.temperature;

    imu_dbg.dev_state = (uint8_t)imu_dev.work_state.dev_state;
    imu_dbg.cali_end = imu_dev.work_state.cali_end;
    imu_dbg.err_code = (uint8_t)imu_dev.work_state.err_code;
}

/* 底盘失能时让云台电机卸力，保证安全 */
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

/* 上板 1 kHz 控制任务，顺序：IMU -> 模块 -> CAN -> 发射 -> 通信 */
void StartControlTask(void const *argument)
{
    (void)argument;

    for (;;)
    {
        /* 仅错误以外的状态允许惯导更新 */
        if ((imu_dev.work_state.err_code == IMU_E_NONE) ||
            (imu_dev.work_state.err_code == IMU_E_CALI))
        {
            imu_dev.update(&imu_dev);
        }

        imu_debug_update();
#if GIMBAL_LOCAL_RC_ENABLE
        rc_interrupt_update(&rc_dev);
#endif
        Module_Work();
        gimbal_can_send();
        Launcher_Work();
        Send_To_Down_Board();

        osDelay(1);
    }
}



