/* config_uart.c - 串口中断回调分发 */


#include "rc_sensor.h"

/* USART3 回调(遥控器) */
void USART3_rxDataHandler(uint8_t *rxBuf)
{
	rc_dev.update(&rc_dev, rxBuf);
	rc_dev.check(&rc_dev);
}


