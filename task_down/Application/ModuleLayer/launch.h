/* launch.h - 发射机构控制 */

#ifndef __LAUNCH_H
#define __LAUNCH_H

#include "stdint.h"

/* 发射许可状态 */
typedef enum{
	L_LOCK = 0, /* 禁止发射 */
  L_UNLOCK,   /* 允许发射 */
}Launch_State_e;


/* 发射模式 */
typedef enum{
	SINGLE_SHOT, /* 单发 */
	REPEAT_SHOT, /* 连发 */
}Launch_Mode_e;


/* 发射机构子设备在线状态 */
typedef struct{
	uint8_t r_fric_heart; /* 右摩擦轮在线 */
  uint8_t l_fric_heart; /* 左摩擦轮在线 */
	uint8_t dial_heart;   /* 拨盘在线 */
}Launch_Heart_t;


/* 发射机构对象 */
typedef struct Launch_Struct_t{
    Launch_State_e state; // 摩擦轮使能，0/1
    Launch_Mode_e mode; // 单发或连发，0/1
    Launch_Heart_t heart; // 子设备在线，成员0/1
    uint8_t shoot_level; // 发射触发电平，0/1
    uint8_t shoot_lock; // 发射锁定，0/1
    uint8_t feed_permit; // 过洞外供弹许可，0/1
    uint8_t s2_mid; // 消抖后S2中位，0/1
    uint8_t ctrl_source; // 同帧来源，0遥控1键鼠
    uint8_t block_reason; // 拦截原因，0~5
    void (*init)(struct Launch_Struct_t* launch); // 初始化回调，非空
    void (*work)(struct Launch_Struct_t* launch); // 周期控制回调，非空
    void (*heart_beat)(struct Launch_Struct_t* launch); // 心跳回调，非空
}Launch_t;

extern uint8_t launch_mouse_press_seq; // 鼠标按下序号，0~255
extern Launch_t launch;
#endif


