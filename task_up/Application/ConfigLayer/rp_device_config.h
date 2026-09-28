/* rp_device_config.h - 设备 ID 与状态定义 */

#ifndef __RP_DEVICE_CONFIG_H
#define __RP_DEVICE_CONFIG_H

#include "stm32f4xx_hal.h"
#include "stdbool.h"
#include "rp_driver_config.h"

typedef enum {
    DEV_ID_IMU = 0,  // IMU 设备 ID
    DEV_ID_CNT = 4,  // 设备数量
} dev_id_t;

typedef enum {
    GIMB_P = 0,  // 云台 Pitch 电机
} dev_rm_motor_list_e;

typedef enum {
    DEV_ONLINE = 0,  // 在线
    DEV_OFFLINE = 1,  // 离线
} dev_work_state_t;

#endif
