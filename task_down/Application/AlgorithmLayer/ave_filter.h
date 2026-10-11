/* ave_filter.h - 滑动平均滤波 */

#ifndef __AVE_FILTER_H
#define __AVE_FILTER_H

#include "stm32h7xx_hal.h"

#define ave_filter_times_max 30 // 环形缓存容量，30个样本

typedef struct
{
	int16_t index; // 写入下标，0~窗口长度-1
	float value[ave_filter_times_max]; // 样本环形缓存，单位随输入
	float value_ave; // 窗口均值，与输入同单位
	float filter_times; // 配置窗口长度，0~30
}ave_filter_t;

void ave_fil_init(ave_filter_t *ave_fil);
float avg_push(ave_filter_t *ave_fil, float value, uint16_t max);

#endif

