/* config_uart.c - 串口中断回调分发 */


#include "config_uart.h"
#include "rc_sensor.h"


/* USART1 回调(视觉, 当前未启用) */
//void USART1_rxDataHandler(uint8_t *rxBuf)
//{

//}

/* USART3 回调(遥控器) */
void USART3_rxDataHandler(uint8_t *rxBuf)
{
	// 解析遥控器数据
	rc_dev.update(&rc_dev, rxBuf);  // 更新数据
	rc_dev.check(&rc_dev);
}

/* USART6 回调 */

void USART6_rxDataHandler(uint8_t *rxBuf)
{
	
}


