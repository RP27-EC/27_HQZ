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

/* 键鼠机械档（X 档）是否生效 */
uint8_t Chassis_Input_IsKeyboardMechMode(void);
/* 掉头基准是否在后方（180deg），机械档锁零位和 WASD 反向都按它走 */
uint8_t Chassis_Input_IsKeyboardYawRear(void);
/* 当前掉头基准角，rad。跟随中心用它，未进键鼠时为 0 */
float Chassis_Input_GetYawReferenceRad(void);
/* 强制把掉头基准打回前方（过洞强制云台回零位时用） */
void Chassis_Input_ResetYawReference(void);
/* 跟随档掉头动作是否进行中（进行中要冻结跟随环，底盘不许动） */
uint8_t Chassis_Input_IsUturnActive(void);
/* 取掉头动作的 Yaw 角速度指令，deg/s；1 = 动作中，0 = 无动作 */
uint8_t Chassis_Input_GetUturnYawRateDegS(float *rate_deg_s);

#endif

