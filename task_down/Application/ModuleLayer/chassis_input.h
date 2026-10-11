/* chassis_input.h - 底盘输入解析 */

#ifndef __CHASSIS_INPUT_H
#define __CHASSIS_INPUT_H

#include "chassis_control.h"

extern chassis_cmd_t chassis_input_cmd; /* 统一底盘输入 */

typedef enum
{
    CHASSIS_KEY_MODE_FOLLOW = 0, // Z跟随模式，值0
    CHASSIS_KEY_MODE_MECH, // X机械模式，值1
    CHASSIS_KEY_MODE_SPIN, // C小陀螺模式，值2
} chassis_key_mode_e;

typedef enum
{
    CHASSIS_UTURN_IDLE = 0, // 空闲阶段，值0
    CHASSIS_UTURN_PREPARE, // 预发送阶段，值1
    CHASSIS_UTURN_POSITION, // 定位阶段，值2
    CHASSIS_UTURN_RESTORE, // 恢复交接阶段，值3
} chassis_uturn_state_e;

typedef enum
{
    CHASSIS_UTURN_NONE = 0, // 尚无结果，值0
    CHASSIS_UTURN_RUNNING, // 掉头进行中，值1
    CHASSIS_UTURN_DONE, // 掉头到位，值2
    CHASSIS_UTURN_TIMEOUT, // 掉头超时，值3
    CHASSIS_UTURN_CANCELLED, // 掉头取消，值4
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
/* 交接期间屏蔽Yaw与跟转 */
uint8_t Chassis_Input_IsUturnActive(void);
chassis_uturn_state_e Chassis_Input_GetUturnState(void);
chassis_uturn_result_e Chassis_Input_GetUturnResult(void);
/* 仅完整成功发送周期计数 */
void Chassis_Input_NotifyUturnTxCycle(uint8_t gimbal_mode, uint16_t yaw_target_raw);

#endif

