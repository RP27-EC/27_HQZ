/* rp_config.h - 配置总入口 */


#ifndef __RP_CONFIG_H
#define __RP_CONFIG_H
#include "stm32f4xx_hal.h"
#include "stdbool.h"
#include "string.h"
// 驱动层配置
#include "rp_driver_config.h"
// 设备层配置
#include "rp_device_config.h"
// 用户层配置
#include "rp_user_config.h"
/* IMU 姿态解算算法选择: Mahony */
/* IMU 姿态解算算法选择: EKF */
#define IMU_USE_MAHONY  0
#define IMU_USE_EKF     1
#endif




