/* rp_config.h - 配置总入口 */


#ifndef __RP_CONFIG_H
#define __RP_CONFIG_H
#include "stm32h7xx_hal.h"
#include "stdbool.h"
#include "string.h"

#include "rp_driver_config.h"

#include "rp_device_config.h"

#include "rp_user_config.h"

#define   CAP_SWITCH             1  // 超电模块开关


#define   CHASSIS_SWITCH         1  // 底盘模块开关

#define   GIMBAL_SWITCH          1  // 云台模块开关

#define   LAUNCH_SWITCH          1  // 发射模块开关

#define   SLIP_SWITCH            1  // 打滑检测开关

#define   TURN_MODE              0  // 转向模式














#define IMU_USE_MAHONY  0  // Mahony 互补滤波, 0=关闭

#define IMU_USE_EKF 	1  // EKF 四元数解算, 1=启用
#endif



