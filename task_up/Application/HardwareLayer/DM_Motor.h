/* DM_Motor.h - 达妙电机驱动 */

#ifndef __DM_MOTOR_H
#define __DM_MOTOR_H

#include "rp_config.h"
#include "arm_math.h"
#include "HT_Motor.h"
#include "drv_can.h"
#include "drv_tick.h"
#include "motor_def.h"
#include "rp_math.h"
#ifndef __HT_MOTOR_H
/* MIT 命令集 */
typedef enum mit_cmd_enum_t
{
	Enter_Motor_Mode,  // 使能电机(指示灯变亮)
	Exit_Motor_Mode,  // 失能电机(指示灯变暗)
	Zero_Position_Sensor,  // 把当前位置设为零点
	
}mit_cmd_t;
#endif

#define P_MIN -PI    // Radians
#define P_MAX PI        
#define V_MIN -30.0f    // Rad/s
#define V_MAX 30.0f
#define KP_MIN 0.0f     // N-m/rad
#define KP_MAX 500.0f
#define KD_MIN 0.0f     // N-m/rad/s
#define KD_MAX 5.0f
#define T_MIN -10.0f    // N.m
#define T_MAX 10.0f
#define C_MIN -10.0f    // A
#define C_MAX 10.0f


/* 电机错误状态 */
typedef enum dm_err_enum_t
{
	Motor_Enable,  // 电机使能
	Motor_Unenable,  // 电机失能
	Over_Voltage,  // 过压
	Lack_Voltage,		// 欠压
	Over_Current,  // 过流
	MOS_OverTemp,  // 驱动 MOS 过温
	Motor_OverTemp,  // 电机过温
	Commun_Loss,  // 通信丢失
	Unknow_Err,  // 未知错误
}dm_err_t;


/* 电机初始化参数 */
typedef struct dm_cfg_struct_t
{	
    uint32_t stdId;  // 控制报文 ID


#ifdef __STM32F4xx_HAL_H
    CAN_HandleTypeDef *hcan;  // 使用的 CAN 口
#endif
	
#ifdef STM32H7xx_HAL_H
    FDCAN_HandleTypeDef *hcan;  // 使用的 CAN 口
#endif
	
	  int8_t order_correction;  // 方向修正
}dm_cfg_t;

/* 电机反馈信息 */
typedef struct dm_rx_struct_t
{
	float speed;  // 转速(rad/s)
	
	float torque;  // 输出转矩(N*m)
	
	float motor_angle_sum;

  float motor_angle;  // 电机绝对角度

	float motor_angle_last;
	
	uint8_t num;  // 电机序号
}dm_rx_t;

/* 电机发送信息 */
typedef struct dm_tx_struct_t
{
	float torque;  // 待发送转矩(N*m)
	
	float target_speed;  // 目标转速(rad/s)
	
	float target_angle;  // 目标角度(rad)
	
	float Kp;  // 位置环增益
	
	float Kd;  // 速度环增益
	
	uint8_t single_tx_buff[8];  // 单电机模式发送缓存
}dm_tx_t;
/* 参考输出 = torque + Kp*err_angle + Kd*err_speed */

/* 电机状态 */
typedef struct dm_state_struct_t
{
    uint32_t offline_cnt;

    uint32_t offline_cnt_max;

    dev_work_state_t status;
	
		dm_err_t motor_state;
	
		dm_err_t last_motor_state;
}dm_state_t;

/* 单电机对象 */
typedef struct dm_motor_struct_t
{
	dm_cfg_t* born_info;
	
	dm_rx_t* rx_info;
	
	dm_tx_t* tx_info;
	
	dm_state_t* state;


	
	void (*single_init)(struct dm_motor_struct_t *motor);
	
	void (*single_sleep)(struct dm_motor_struct_t *motor);
	
	void (*zero_position)(struct dm_motor_struct_t *motor);
	
	void (*single_set_torque)(struct dm_motor_struct_t *motor);
	
	void (*single_set_speed)(struct dm_motor_struct_t *motor);
	
	void (*single_set_angle)(struct dm_motor_struct_t *motor);
	
	void (*rx)(struct dm_motor_struct_t *motor, uint8_t *rxBuf);
	
	void (*single_heart_beat)(struct dm_motor_struct_t *motor);
}dm_motor_t;

/* 电机组对象, 统一管理组内电机 */
typedef struct dm_group_struct_t
{
	dm_motor_t* motor[4];
	
	uint8_t motor_num;  // 组内电机数量
	
	void (*group_set_torque)(struct dm_group_struct_t *group);
	
	void (*group_sleep)(struct dm_group_struct_t *group);
	
	void (*group_init)(struct dm_group_struct_t *group);
	
	void (*group_heartbeat)(struct dm_group_struct_t *group);
}dm_group_t;

void dm_motor_init(dm_motor_t *motor);
void dm_group_init(dm_group_t *group);

#endif

