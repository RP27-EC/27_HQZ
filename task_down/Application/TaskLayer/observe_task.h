/* observe_task.h - 监测任务 */

#ifndef __OBSERVE_TASK
#define __OBSERVE_TASK

#include "main.h"
#include "cmsis_os.h"
#include "imu_sensor.h"
#include "Chassis.h"

void StartUpdataTask(void const * argument);

#endif

