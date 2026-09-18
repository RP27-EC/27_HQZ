#ifndef __RP_DEVICE_CONFIG_H
#define __RP_DEVICE_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "stdbool.h"
#include "rp_driver_config.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* 设备层 --------------------------------------------------------------------*/
/**
 *	@brief	设备id列表
 *	@class	device
 */
typedef enum {
	DEV_ID_IMU = 0,
  DEV_ID_IMU_EX,
	DEV_ID_RC,
	DEV_ID_VISION,
	DEV_ID_CNT,
} dev_id_t;

/**
 *	@brief	设备工作状态(通用)
 *	@class	device
 */
typedef enum {
	DEV_OFFLINE,
	DEV_ONLINE,
	
} dev_work_state_t;


/**
 * @brief 未初始化：DEV_RESET_NO 初始化完成:DEV_RESET_OK
 * 
 */
typedef enum DEV_RESET_STATE
{
	DEV_RESET_NO,
	DEV_RESET_OK,
}Dev_Reset_State_e;

/**
 *	@brief	错误代码(通用)
 *  @note   可自定义设备错误代码类型并替换变量errno的变量类型，如
 *          typedef enum {
 *              IMU_NONE_ERR,
 *              IMU_ID_ERR,
 *              IMU_COM_FAILED,
 *              IMU_DEV_NOT_FOUND,
 *              ...
 *          } imu_errno_t;
 *          
 *          typedef struct imu_sensor_struct {
 *              ...
 *	            imu_errno_t errno;
 *              ...	
 *          } imu_sensor_t;
 *	@class	device
 */
typedef enum {
	NONE_ERR,		// 正常(无错误)
	DEV_ID_ERR,		// 设备ID错误
	DEV_INIT_ERR,	// 设备初始化错误
	DEV_DATA_ERR,	// 设备数据错误
} dev_errno_t;



#endif
