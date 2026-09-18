
#ifndef __JUDGE_POTOCOL_H
#define __JUDGE_POTOCOL_H

#include "stm32h7xx_hal.h"

/* 帧头 */
typedef struct 
{
	uint8_t SOF;  //数据帧起始字节，固定值为 0xA5
	uint16_t data_length;  //数据帧中 data 的长度
	uint8_t seq;  //包序号
	uint8_t CRC8;  //帧头 CRC8 校验
}judge_frame_header_t;

typedef struct 
{
	judge_frame_header_t *frame_header;
	uint16_t cmd_id;
	uint16_t frame_tail;
}drv_judge_info_t;

extern drv_judge_info_t drv_judge_info;

void judge_receive(uint8_t *rxBuf);


#endif
