/* Command.h - 指令服务 */

#ifndef __COMMAND_H
#define __COMMAND_H


#include "stdbool.h"
#include "stdint.h"
#include "stm32h7xx_hal.h"
#define OUT_TIME_OFF  (0xFFFF)  // 输出时间OFF

/* enum */
typedef enum
{
 HIGH_TRIGER_C,  // 高TRIGERc
  LOW_TRIGER_C,  // 低TRIGERc
  RISE_TRIGER_C,  // RISETRIGERc
  FALL_TRIGER_C,  // FALLTRIGERc
  NO_CMD,  // NO命令
}Cmd_Type_e;

/* enum */
typedef enum 
{
  KEEP_U,
  SWITCH_HIGHT_U,
  SWITCH_LOW_U,  // SWITCH低U
}User_Status_e;

/* enum */
typedef enum 
{
	FINISH_C,  // FINISHc
	RUNING_C,  // RUNINGc
}Cmd_Status_e;

/* enum */
typedef enum
{
  DEINIT_C,  // DEINITc
  INIT_C,  // 初始化c
}Cmd_Init_e;
/* struct */
typedef struct 
{
  uint8_t Trigger_lock_on;
  uint32_t lock_time;

}Trigger_lock_t;

/* command_class_t */
typedef struct command_class_t
{
  bool cmd_value;  // 命令值
  Cmd_Status_e cmd_status;  // 命令状态

  Cmd_Type_e cmd_type;  // 命令类型
  
  bool user_value;  // user值
  bool user_value_last;  // user值last
  User_Status_e user_status;  // user状态

  uint16_t run_time;  // run时间
  uint16_t run_time_max;  // run时间最大

  Cmd_Init_e init_flag;  // 初始化标志
  Trigger_lock_t   Trigger_lock;
  void (*init)(struct command_class_t *command);  // 初始化
  void (*heartbeat)(struct command_class_t *command);
  void (*update)(struct command_class_t *command,bool condition);
  void (*clean)(struct command_class_t *command);
  void (*s_run)(struct command_class_t *command);
  void (*s_finish)(struct command_class_t *command);
}command_t;


void Cmd_Class_Init(command_t *commond);
#endif


