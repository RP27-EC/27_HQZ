/* rp_device_config.h - 设备 ID 与状态定义 */

#ifndef __RP_DEVICE_CONFIG_H
#define __RP_DEVICE_CONFIG_H
#include "stm32h7xx_hal.h"
#include "stdbool.h"
#include "rp_driver_config.h"
/* 设备层 --------------------------------------------------------------------*/
/* enum */
typedef enum {
	DEV_ID_IMU = 0,
  DEV_ID_IMU_EX,
	DEV_ID_RC,
	DEV_ID_VISION,
	DEV_ID_CNT,
} dev_id_t;

/* enum */
typedef enum {
	DEV_OFFLINE,
	DEV_ONLINE,
	
} dev_work_state_t;


/* enum */
typedef enum DEV_RESET_STATE
{
	DEV_RESET_NO,
	DEV_RESET_OK,
}Dev_Reset_State_e;

/* enum */
typedef enum {
	NONE_ERR,		// 正常(无错误)
	DEV_ID_ERR,		// 设备ID错误
	DEV_INIT_ERR,	// 设备初始化错误
	DEV_DATA_ERR,	// 设备数据错误
} dev_errno_t;



#endif


