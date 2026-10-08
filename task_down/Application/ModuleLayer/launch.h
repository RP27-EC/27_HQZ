/* launch.h - 发射机构控制 */

#ifndef __LAUNCH_H
#define __LAUNCH_H

#include "stdint.h"

/* 发射许可状态 */
typedef enum{
	L_LOCK = 0, /* 摩擦轮关闭，0 */
  L_UNLOCK,   /* 摩擦轮开启，1 */
}Launch_State_e;


/* 发射模式 */
typedef enum{
	SINGLE_SHOT, /* 单发模式，0 */
	REPEAT_SHOT, /* 连发模式，1 */
}Launch_Mode_e;


/* 发射机构子设备在线状态 */
typedef struct{
	uint8_t r_fric_heart; /* 右轮在线，0/1 */
  uint8_t l_fric_heart; /* 左轮在线，0/1 */
	uint8_t dial_heart;   /* 拨盘在线，0/1 */
}Launch_Heart_t;


/* 发射机构对象 */
typedef struct Launch_Struct_t{
	Launch_State_e state; /* 摩擦轮使能，0/1 */
  Launch_Mode_e mode;   /* 单发/连发，0/1 */
	Launch_Heart_t heart; /* 三电机在线，各0/1 */

	uint8_t shoot_level;  /* 供弹触发，0/1 */
	uint8_t shoot_lock;   /* 供弹禁止，0/1 */
	uint8_t feed_permit;  /* 供弹许可，0/1 */

	void (*init)(struct Launch_Struct_t* launch);       /* 初始化对象，无量纲 */
	void (*work)(struct Launch_Struct_t* launch);       /* 控制更新，周期1 ms */
	void (*heart_beat)(struct Launch_Struct_t* launch); /* 在线采样，值0/1 */
}Launch_t;

extern  Launch_t  launch;
#endif
