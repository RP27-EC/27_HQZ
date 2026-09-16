#include "led_task.h"


void StartLedTask(void const * argument)
{
	/* 开机绿灯常亮 0.5 秒 */
	led.colour = LED_colour_green;
	led.state  = LED_ON;
	led_work(&led);
	osDelay(500);

	/* 绿灯 5Hz 闪烁 */
	led.state     = LED_BLINK;
	led.blink_fre = 5;        //每秒闪 5 次

  for(;;)
  {		
	  led_work(&led);
		osDelay(1);
  }
}



