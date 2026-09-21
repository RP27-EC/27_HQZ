/* drv_tick.c - 系统时基与延时 */

#include "drv_tick.h"
uint32_t haltick = 0;
/**
  * @brief  读取当前时间(微秒)
 * @param  None
  * @retval 当前时间
 */
uint32_t micros(void)
{
	register uint32_t ms, us;
	
	ms = HAL_GetTick();
	/* TIM2 作为 HAL 时基 */
	/* Freq:1MHz => 1Tick = 1us */
	/* Period:1ms */
	us = TIM2->CNT;
	
    haltick = ms*1000 + us;
    
	return haltick;
}

void delay_us(uint32_t us)
{
	uint32_t now = micros();
	
	while((micros() - now) < us);
}

void delay_ms(uint32_t ms)
{
	while(ms--)
		delay_us(1000);
}





