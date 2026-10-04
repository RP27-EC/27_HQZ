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

typedef enum
{
    CHASSIS_UTURN_IDLE = 0,
    CHASSIS_UTURN_PREPARE,
    CHASSIS_UTURN_POSITION,
    CHASSIS_UTURN_RESTORE,
} chassis_uturn_state_e;

typedef enum
{
    CHASSIS_UTURN_NONE = 0,
    CHASSIS_UTURN_RUNNING,
    CHASSIS_UTURN_DONE,
    CHASSIS_UTURN_TIMEOUT,
    CHASSIS_UTURN_CANCELLED,
} chassis_uturn_result_e;

void Chassis_Input_Init(void);
void Chassis_Input_Update(void);
void Chassis_Input_SetSource(chassis_source_e source);
uint8_t Chassis_Input_IsKeyboardMode(void);
chassis_key_mode_e Chassis_Input_GetKeyboardChassisMode(void);

/* 键鼠机械档（X 档）是否生效 */
uint8_t Chassis_Input_IsKeyboardMechMode(void);
/* 固定前后方向不采纳超时停偏角 */
uint8_t Chassis_Input_IsKeyboardYawRear(void);
float Chassis_Input_GetKeyboardYawTargetRad(void);
/* 超时仅对齐跟随中心，不改变固定终点 */
float Chassis_Input_GetYawReferenceRad(void);
/* 过洞回零必须同步取消掉头 */
void Chassis_Input_ResetYawReference(void);
/* 包含预发送及恢复，期间屏蔽 Yaw 和自动跟转 */
uint8_t Chassis_Input_IsUturnActive(void);
chassis_uturn_state_e Chassis_Input_GetUturnState(void);
chassis_uturn_result_e Chassis_Input_GetUturnResult(void);
/* 只计完整成功发送周期，目标使用 D2 编码值 */
void Chassis_Input_NotifyUturnTxCycle(uint8_t gimbal_mode, uint16_t yaw_target_raw);

#endif

