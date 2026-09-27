/* chassis_input.h - 底盘输入解析 */

#ifndef __CHASSIS_INPUT_H
#define __CHASSIS_INPUT_H

#include "chassis_control.h"

extern chassis_cmd_t chassis_input_cmd; /* 统一底盘输入 */

typedef enum
{
    CHASSIS_KEY_MODE_FOLLOW = 0, /* Z：跟随 */
    CHASSIS_KEY_MODE_MECH,       /* X：机械 */
    CHASSIS_KEY_MODE_SPIN,       /* C：小陀螺 */
} chassis_key_mode_e;

void Chassis_Input_Init(void);
void Chassis_Input_Update(void);
void Chassis_Input_SetSource(chassis_source_e source);
uint8_t Chassis_Input_IsKeyboardMode(void);
chassis_key_mode_e Chassis_Input_GetKeyboardChassisMode(void);

#endif

