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
