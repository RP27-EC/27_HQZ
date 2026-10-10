/* motor.h - 电机对象管理 */

#ifndef __MOTOR_H
#define __MOTOR_H

#include "rp_config.h"
#include "can_protocol.h"
#include "rm_motor.h"
#include "motor_def.h"
#include "drv_can.h"


/* 四轮索引与数量 */
typedef enum{
	WHEEL_LF = 0, /* 左前 */
	WHEEL_LB,     /* 左后 */
	WHEEL_RF,     /* 右前 */
  WHEEL_RB,     /* 右后 */
	WHEEL_CNT,    /* 轮数 */
}Wheel_List_e;


#define   ID_WHEEL_LF    0x201 /* 左前反馈 ID */
#define   ID_WHEEL_LB    0x202 /* 左后反馈 ID */
#define   ID_WHEEL_RF    0x203 /* 右前反馈 ID */
#define   ID_WHEEL_RB    0x204 /* 右后反馈 ID */

extern rm_motor_t wheel_motor[WHEEL_CNT]; /* 四轮单电机对象 */
extern rm_group_t wheel_group;            /* 四轮电机组对象 */
void rm_motor_list_init(void);
void rm_motor_list_heart_beat(void);
void kt_motor_list_init(void);
void ht_motor_list_init(void);
void dm_motor_list_init(void);
uint8_t rm_motor_list_workstate(void);

#endif


