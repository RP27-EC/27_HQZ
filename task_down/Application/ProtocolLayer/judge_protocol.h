/* judge_protocol.h - 裁判系统协议 */


#ifndef __JUDGE_POTOCOL_H
#define __JUDGE_POTOCOL_H

#include "stm32h7xx_hal.h"


typedef struct 
{
	uint8_t SOF;
	uint16_t data_length;  // 数据长度
	uint8_t seq;  // 序列号
	uint8_t CRC8;
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

