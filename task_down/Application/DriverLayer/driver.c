/* driver.c - 驱动层统一初始化 */
#include "driver.h"
void DRIVER_Init(void)
{
    /* 裁判数据供功率预算使用 */
    USART1_Init();

    USART5_Init();
    CAN2_Filter_Init();
    CAN1_Filter_Init();
}
