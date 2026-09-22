/* rp_math.c - 数学工具 */
#include "rp_math.h"
/* 一阶低通滤波 */
float Lowpass(float X_last, float X_new, float K)
{
	return (X_last + (X_new - X_last) * K);
}

/* 角度归一化到正负半圈 */
float motor_half_cycle(float angle, float max)
{
	if (abs(angle) > (max / 2))
	{
		if (angle >= 0)
			angle += -max;
		else
			angle += max;
	}
	return angle;
}

int16_t RampInt(int16_t final, int16_t now, int16_t ramp)
{
	int32_t buffer = 0;

	buffer = final - now;
	if (buffer > 0)
	{
		if (buffer > ramp)
			now += ramp;
		else
			now += buffer;
	}
	else
	{
		if (buffer < -ramp)
			now += -ramp;
		else
			now += buffer;
	}

	return now;
}

float RampFloat(float final, float now, float ramp)
{
	float buffer = 0;

	buffer = final - now;
	if (buffer > 0)
	{
		if (buffer > ramp)
			now += ramp;
		else
			now += buffer;
	}
	else
	{
		if (buffer < -ramp)
			now += -ramp;
		else
			now += buffer;
	}

	return now;
}




float deadzone(float input, float center, float death)
{
	if (abs(input - center) < death)
		return center;
	return input;
}
/* Time_Trigger_inloop */
void Time_Trigger_inloop(Time_trigger_t *Time_trigger_struct)
{
	if (Time_trigger_struct->private_flag == NULL || Time_trigger_struct->last_trigger_tick == NULL || Time_trigger_struct->ignore_first_trigger_flag == NULL)
	{
		return;
	}


	if (Time_trigger_struct->if_ignore_first == 1 && Time_trigger_struct->ignore_first_trigger_flag == 0)
	{
		*Time_trigger_struct->ignore_first_trigger_flag = 1;
		*Time_trigger_struct->last_trigger_tick = HAL_GetTick();
	}

	if ((HAL_GetTick() - *Time_trigger_struct->last_trigger_tick > Time_trigger_struct->delay_tick) && (Time_trigger_struct->ignore_first_trigger_flag != 0 || Time_trigger_struct->if_ignore_first == 0))
	{
		*Time_trigger_struct->private_flag = 1;
		*Time_trigger_struct->last_trigger_tick = HAL_GetTick();
	}

	else
	{
		*Time_trigger_struct->private_flag = Time_trigger_struct->flag_before_trigger;
	}
}


uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    
    return (uint16_t) ((x-offset)*((float)((1<<bits)-1))/span);
}

/* 定长整型转浮点 */
float uint_to_float(uint16_t x_int, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/* 限速斜坡滤波 */
float step_limit_filter(float new_value, float last_value, float max_step)
{
    float filtered_value;
    float difference = new_value - last_value;


    if (fabs(difference) > max_step) {
        filtered_value = last_value + sgn(difference) * max_step;
    } else {

        filtered_value = new_value;
    }
    
    return filtered_value; 
}




