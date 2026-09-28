/* driver.c - 驱动层统一初始化 */

#include "driver.h"

void DRIVER_Init(void)
{
    USART5_Init();
    CAN2_Filter_Init();
    CAN1_Filter_Init();
}
