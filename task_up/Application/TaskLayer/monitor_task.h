/* monitor_task.h - 设备监控任务 */

#ifndef __MONITOR_TASK
#define __MONITOR_TASK


#include "cmsis_os.h"
#include "device.h"

extern IWDG_HandleTypeDef hiwdg;
void StartMonitorTask(void const * argument);


#endif

