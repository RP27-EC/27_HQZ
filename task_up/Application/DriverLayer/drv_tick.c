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
    uint32_t irq = __get_PRIMASK();
    uint32_t ms;
    uint32_t us;
    uint32_t pending;
    uint32_t tick;

    __disable_irq();
    ms = HAL_GetTick();
    do
    {
        pending = TIM2->SR & TIM_SR_UIF;
        us = TIM2->CNT;
    } while (pending != (TIM2->SR & TIM_SR_UIF));

    // NOTE: 补偿尚未服务的溢出
    if (pending != 0u) ms++;
    tick = ms * 1000u + us;
    haltick = tick;
    __set_PRIMASK(irq);
    return tick;
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





