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

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
static uint8_t reset_preparing; // 停机准备进行中，0/1
static uint8_t reset_wait_idle; // 等待请求释放，0/1
static uint32_t reset_phase_tick; // 阶段起点，ms
static uint32_t reset_commit_tick; // 确认入队时刻，ms
static uint8_t reset_commit_sent; // 提交确认已入队，0/1

static void Board_Reset_Update(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t request = Board_Rx_Info.remote_cmd_pkt.reset_stage;
    uint8_t fresh = ((Board_HeartBeat.status == DEV_ONLINE) &&
        (Board_Rx_Info.remote_cmd_pkt.valid != 0u) &&
        ((now - Board_Rx_Info.remote_cmd_pkt.rx_tick) < BOARD_RESET_TIMEOUT_MS));

    if ((fresh != 0u) && (request == 0u)) reset_wait_idle = 0u;
    if ((fresh != 0u) && (request == BOARD_RESET_PREPARE) &&
        (board_reset_ack == 0u) && (reset_preparing == 0u) && (reset_wait_idle == 0u))
    {
        board_reset_stopped = 1u;
        reset_preparing = 1u;
        board_reset_ack_sent = 0u;
        reset_phase_tick = now;
        reset_commit_sent = 0u;
    }
    if ((reset_preparing != 0u) && ((now - reset_phase_tick) >= 1u) &&
        (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 3u) &&
        (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 3u))
    {
        reset_preparing = 0u;
        board_reset_ack = BOARD_RESET_PREPARE;
    }
    if ((fresh != 0u) && (request == BOARD_RESET_COMMIT) &&
        (board_reset_ack == BOARD_RESET_PREPARE))
    {
        board_reset_ack_sent = 0u;
        board_reset_ack = BOARD_RESET_COMMIT;
        reset_phase_tick = now;
    }
    if (((board_reset_ack != 0u) || (reset_preparing != 0u)) &&
        ((now - reset_phase_tick) >= BOARD_RESET_TIMEOUT_MS))
    {
        board_reset_fail_count++;
        board_reset_ack = 0u;
        reset_preparing = 0u;
        reset_commit_sent = 0u;
        reset_wait_idle = 1u;
    }
    if (board_reset_stopped != 0u)
    {
        Board_Rx_Info.state_pkt.car_state = 0u;
        Board_Rx_Info.shoot_pkt.launch_state = 0u;
        Board_Rx_Info.shoot_pkt.shoot_level = 0u;
        Board_Rx_Info.shoot_pkt.feed_permit = 0u;
    }
}

static void Board_Reset_Finish(void)
{
    uint32_t now = HAL_GetTick();
    if ((board_reset_ack == BOARD_RESET_COMMIT) &&
        (board_reset_ack_sent == BOARD_RESET_COMMIT) &&
        (reset_commit_sent == 0u))
    {
        reset_commit_sent = 1u;
        reset_commit_tick = now;
    }
    if ((reset_commit_sent != 0u) && ((now - reset_commit_tick) >= BOARD_RESET_REBOOT_DELAY_MS) &&
        (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 3u))
    {
        __DSB();
        NVIC_SystemReset();
    }
}

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
    uint32_t next_tick = osKernelSysTick();

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
        Board_Reset_Update();
        Module_Work();
        gimbal_can_send();
        Launcher_Work();
        Send_To_Down_Board();
        Board_Reset_Finish();

        if (osDelayUntil(&next_tick, 1u) != osOK)
        {
            next_tick = osKernelSysTick();
        }
    }
}



