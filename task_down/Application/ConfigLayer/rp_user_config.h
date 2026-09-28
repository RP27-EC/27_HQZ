/* rp_user_config.h - 用户参数配置 */

#ifndef __RP_USER_CONFIG_H
#define __RP_USER_CONFIG_H
#include "stm32h7xx_hal.h"
#include "stdbool.h"

/* Remote Mode Enum */
typedef enum {
	RC = 0,
	KEY = 1,
	REMOTE_MODE_CNT = 2,
} remote_mode_t;

typedef enum {
	SYS_STATE_NORMAL,
	SYS_STATE_RCLOST,
	SYS_STATE_RCERR,
	SYS_STATE_WRONG,
} sys_state_t;

typedef enum {
	SYS_MODE_NORMAL,  // 正常模式
	SYS_MODE_CNT,
} sys_mode_t;

typedef struct {
	uint8_t reset_start;  // 起立开始
	uint8_t reset_ok;  // 起立完成
	uint8_t turn_start;  // 转向开始
	uint8_t turn_ok;  // 转向完成
	uint8_t forward;  // 前进
	uint8_t turn_right;  // 右转
	uint8_t turn_left;  // 左转
} gimbal_symbal_t;

typedef struct __symbal_struct
{
	gimbal_symbal_t   gim_sym;  // 云台状态标志
	uint8_t 					rc_update;  // 遥控更新
	uint8_t						slave_reset;  // 下板复位
} symbal_t;

typedef struct {
	remote_mode_t		remote_mode;  // 控制方式
	sys_state_t			state;  // 系统状态
	sys_mode_t			mode;  // 模式
} system_t;

extern symbal_t	symbal;
extern system_t sys;

#endif


