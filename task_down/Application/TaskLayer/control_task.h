/* control_task.h - 控制任务 */

#ifndef __CONTROL_TASK
#define __CONTROL_TASK

#include "cmsis_os.h"
#include "main.h"
#include "device.h"

void StartControlTask(void const * argument);

extern volatile uint8_t board_hole_request;
extern volatile uint8_t board_hole_exit_pending;

uint8_t Board_Control_GetTurnEvent(void);
void Board_Control_NotifyLiftTx(uint8_t is_hole);
uint8_t Board_Lift_IsRestricted(void);
uint8_t Board_Lift_IsReady(void);


#endif

