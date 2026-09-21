/* drv_spi.h - SPI 驱动 */

#ifndef __DRV_SPI_H
#define __DRV_SPI_H
#include "stm32f4xx_hal.h"
#include "imu_sensor.h"

#include "main.h"
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
void SPI2_Init(void);
void SPI1_Init(void);

#endif




