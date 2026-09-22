/* DWT.c - DWT 微秒计数 */

#include "DWT.h"
/*!

 */
void DWT_Init(void)
{
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk))
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }
    
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/*!

 */
uint32_t DWT_GetCycleCount(void)
{
    return DWT->CYCCNT;
}

