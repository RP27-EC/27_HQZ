/* rp_driver_config.h - 外设驱动配置 */

#ifndef __RP_DRIVER_CONFIG_H
#define __RP_DRIVER_CONFIG_H
#include "stm32f4xx_hal.h"
#include "stdbool.h"
#define configDRV_CAN_USE_MAIL  1
/* 驱动层配置 */
/**
 *	@brief 驱动层配置
 *	@class	driver
 */
typedef enum drv_type{
		DRV_CAN,
} drv_type_t;

/**
 *	@brief CAN 外设 id
 *	@class	driver
 */
typedef enum {
    DRV_CAN1,
    DRV_CAN2
} can_id_t;

/**
 *	@brief CAN 外设
 *	@class	driver
 */
typedef struct drv_can {
    can_id_t    can_id;  // CAN1 或 CAN2
    uint32_t    err_cnt;
	uint32_t	rx_id;  // 接收报文标识符
	uint32_t	tx_id;  // 发送报文标识符
	uint8_t		data_id;  // 数据下标
    uint16_t    tx_period;  // 定时发送间隔(ms)
	uint8_t		*CANx_XXX_DATA;  // 发送数据指针
} drv_can_t;

#endif



