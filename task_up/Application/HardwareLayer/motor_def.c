/* motor_def.c - 电机公共定义 */

#include "motor_def.h"

/* 滑动平均滤波 */
void motor_pid_init(motor_pid_t *motor_pid,motor_pid_t extern_motor_pid)
{ 
	if (motor_pid == NULL) return;
	*motor_pid=extern_motor_pid;
}

 



