/* Command_Task.c - 指令任务 */

#include "Command_Task.h"
//决策层，遥控器输入然后去做
void StartCommandTask(void const * argument)
{
	for(;;)
	{
		rc_interrupt_update(&rc_dev);//遥控器数据解析
    keyboard_update(rc_dev.info); // 键鼠状态检测
	
	

    osDelay(1);
	}
}  


