
#ifndef __RP_CONFIG_H
#define __RP_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stdbool.h"
#include "string.h"
// 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
#include "rp_driver_config.h"
// 锟借备锟斤拷锟斤拷锟斤拷
#include "rp_device_config.h"
// 锟矫伙拷锟斤拷锟斤拷锟斤拷
#include "rp_user_config.h"

/* Exported macro ------------------------------------------------------------*/

/*选锟斤拷IMU锟斤拷锟斤拷锟姐法为Mahony*/
/*选锟斤拷IMU锟斤拷锟斤拷锟姐法为EKF*/
#define IMU_USE_MAHONY  0
#define IMU_USE_EKF     1


/* Exported types ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/


#endif

