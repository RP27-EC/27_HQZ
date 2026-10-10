/* drv_flash.h - Flash 读写 */

#ifndef __DRV_FLASH_H
#define __DRV_FLASH_H
#include "stm32h7xx_hal.h"
#include "main.h"
#define IMU_DATA_ADDR	0x080E0000
#define FLASH_TYPEPROGRAM_BYTE    ((uint32_t)0x00000001U)  // FLASHTYPEPROGRAM字节
#define FLASH_TYPEPROGRAM_HALFWORD ((uint32_t)0x00000002U)
#define FLASH_TYPEPROGRAM_WORD    ((uint32_t)0x00000000U)
void Flash_ReadData(uint32_t addr, uint32_t *buf, uint16_t len);
void Flash_WriteByteData(uint32_t addr,uint8_t *data,uint16_t num);
void Flash_WriteHalfWordData(uint32_t addr,uint16_t *data,uint16_t num);
void Flash_WriteWordData(uint32_t addr,uint32_t *data,uint16_t num);
void Flash_WriteDoubleWordData(uint32_t addr,uint64_t *data,uint16_t num);
uint32_t Flash_EraseSector(uint32_t SectorNum);
uint32_t Flash_EraseSector11(void);

#endif

