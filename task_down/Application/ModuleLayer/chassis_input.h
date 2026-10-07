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
    CHASSIS_UTURN_IDLE = 0, // 空闲，状态0
    CHASSIS_UTURN_PREPARE, // 预发送，状态1
    CHASSIS_UTURN_POSITION, // 机械定位，状态2
    CHASSIS_UTURN_RESTORE, // 恢复跟随，状态3
} chassis_uturn_state_e;

typedef enum
{
    CHASSIS_UTURN_NONE = 0, // 无动作，结果0
    CHASSIS_UTURN_RUNNING, // 执行中，结果1
    CHASSIS_UTURN_DONE, // 定位完成，结果2
    CHASSIS_UTURN_TIMEOUT, // 定位超时，结果3
    CHASSIS_UTURN_CANCELLED, // 动作取消，结果4
} chassis_uturn_result_e;

void Chassis_Input_Init(void);
void Chassis_Input_Update(void);
void Chassis_Input_SetSource(chassis_source_e source);
uint8_t Chassis_Input_IsKeyboardMode(void);
chassis_key_mode_e Chassis_Input_GetKeyboardChassisMode(void);

/* 键鼠机械档（X 档）是否生效 */
uint8_t Chassis_Input_IsKeyboardMechMode(void);
/* 遥控与键鼠共用前后基准 */
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

