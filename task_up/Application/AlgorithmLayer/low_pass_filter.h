/* low_pass_filter.h - 一阶低通滤波 */

#ifndef __LOW_PASS_FILTER_H
#define __LOW_PASS_FILTER_H

#include "stm32f4xx_hal.h"

float low_pass_filter(float input, float prevOutput, float alpha) ;
#endif

