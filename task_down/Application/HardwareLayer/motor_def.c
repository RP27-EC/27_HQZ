#include "motor_def.h"

/* 电机 PID 初始化 */
void motor_pid_init(motor_pid_t *motor_pid,motor_pid_t extern_motor_pid)
{ 
	if (motor_pid == NULL) return;
	*motor_pid=extern_motor_pid;
}

 


