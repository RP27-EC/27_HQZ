/* chassis_input.h - 底盘输入解析 */

#ifndef __CHASSIS_INPUT_H
#define __CHASSIS_INPUT_H

#include "chassis_control.h"

extern chassis_cmd_t chassis_input_cmd; /* 统一底盘输入 */

void Chassis_Input_Init(void);
void Chassis_Input_Update(void);
void Chassis_Input_SetSource(chassis_source_e source);
uint8_t Chassis_Input_IsKeyboardMode(void);

#endif

