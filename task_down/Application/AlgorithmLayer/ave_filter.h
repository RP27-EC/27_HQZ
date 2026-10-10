/* ave_filter.h - 滑动平均滤波 */

#ifndef __AVE_FILTER_H
#define __AVE_FILTER_H

#include "stm32h7xx_hal.h"

#define ave_filter_times_max 30

typedef struct
{
	int16_t index;
	float value[ave_filter_times_max];
	float value_ave;
	float filter_times;
}ave_filter_t;

void ave_fil_init(ave_filter_t *ave_fil);
float avg_push(ave_filter_t *ave_fil, float value, uint16_t max);

#endif

