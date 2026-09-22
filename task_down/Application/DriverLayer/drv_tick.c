/* drv_tick.c - 系统时基与延时 */
#include "drv_tick.h"
uint32_t haltick = 0;
/* 读当前时间(微秒) */
uint32_t micros(void)
{
	register uint32_t ms, us;
	
	ms = HAL_GetTick();
	/* 选用定时器2作为HAL时基的TimeBase */
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
/*!
 * @usage: 开启DWT模块
 */
void DWT_Init(void)
{
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk))
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 启用跟踪功能
    }
    
    DWT->CYCCNT = 0;                        // 清零周期计数器
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;    // 启用周期计数器
}

/*!
 * @usage: 获得时钟周期数
 */
uint32_t DWT_GetCycleCount(void)
{
    return DWT->CYCCNT;
}


