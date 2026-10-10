/* HT_Motor.h - HT 电机驱动 */

#ifndef __HT_MOTOR_H
#define __HT_MOTOR_H

#include "rp_config.h"
#include "drv_can.h"
#include "drv_tick.h"
#include "rp_math.h"
#include "motor_def.h"
#include "arm_math.h"

#define HT_P_MIN -95.5f    // Radians
#define HT_P_MAX 95.5f        
#define HT_V_MIN -45.0f    // Rad/s
#define HT_V_MAX 45.0f
#define HT_KP_MIN 0.0f     // N-m/rad
#define HT_KP_MAX 500.0f
#define HT_KD_MIN 0.0f     // N-m/rad/s
#define HT_KD_MAX 5.0f
#define HT_T_MIN -18.0f    // N.m
#define HT_T_MAX 18.0f
#define HT_C_MIN -40.0f    // A
#define HT_C_MAX 40.0f
#define HT_TORQUE_CONSTANT 0.45f  // 转矩常数(N*m/A)
#define TIME_STEP 0.001
/* MIT 命令集 */
typedef enum mit_cmd_enum_t
{
	Enter_Motor_Mode,  // 使能电机(指示灯变亮)
	Exit_Motor_Mode,  // 失能电机(指示灯变暗)
	Zero_Position_Sensor,  // 把当前位置设为零点
	
}mit_cmd_t;

/* 电机模式 */
typedef enum ht_mode_enum_t
{
	Motor_Control,  // 电机受控
	Motor_UnControl,  // 电机不受控
}ht_mode_t;

/* 电机初始化参数 */
typedef struct ht_cfg_struct_t
{	
    uint32_t stdId;  // 控制报文 ID

#ifdef __STM32F4xx_HAL_H
    CAN_HandleTypeDef *hcan;  // 使用的 CAN 口
#endif
	
#ifdef STM32H7xx_HAL_H
    FDCAN_HandleTypeDef *hcan;  // 使用的 CAN 口
#endif
	  int8_t order_correction;  // 方向修正

}ht_cfg_t;

/* 电机反馈信息 */
typedef struct ht_rx_struct_t
{
	float encoder;  // 上电后角度累计(rad)
	
	float encoder_last;
	
	float encoder_err;
	
	float speed;  // 转速(rad/s)
	
	float torque;  // 输出转矩(N*m)
	
	float torque_current;  // 输出电流(A)
	
	float motor_angle_sum;
	
	float motor_angle_sum_vi;
	
	float motor_angle_sum_filter;

  float motor_angle;  // 电机绝对角度
	
	float motor_angle_last;
	
/* 上电时间戳 */
	uint32_t time_now;
	
	uint32_t time_last;
	
	float time;
}ht_rx_t;

/* 电机发送信息 */
typedef struct ht_tx_struct_t
{
	float torque;  // 待发送转矩(N*m)
	
	float target_speed;  // 目标转速(rad/s)
	
	float target_angle;  // 目标角度(rad)
	
	float Kp;  // 位置环增益
	
	float Kd;  // 速度环增益
	
	uint8_t single_tx_buff[8];  // 单电机模式发送缓存
}ht_tx_t;
/* 参考输出 = torque + Kp*err_angle + Kd*err_speed */

/* 电机状态 */
typedef struct ht_state_struct_t
{
    uint32_t offline_cnt;

    uint32_t offline_cnt_max;

    dev_work_state_t status;

		ht_mode_t mode;
}ht_state_t;

/* 单电机对象 */
typedef struct ht_motor_struct_t
{
	ht_cfg_t* born_info;
	
	ht_rx_t* rx_info;
	
	ht_tx_t* tx_info;
	
	ht_state_t* state;

	
	void (*single_init)(struct ht_motor_struct_t *motor);
	
	void (*single_sleep)(struct ht_motor_struct_t *motor);
	
	void (*zero_position)(struct ht_motor_struct_t *motor);
	
	void (*single_set_torque)(struct ht_motor_struct_t *motor);
	
	void (*single_set_speed)(struct ht_motor_struct_t *motor);
	
	void (*single_set_angle)(struct ht_motor_struct_t *motor);
	
	void (*rx)(struct ht_motor_struct_t *motor, uint8_t *rxBuf);
	
	void (*single_heart_beat)(struct ht_motor_struct_t *motor);
}ht_motor_t;

/* 电机组对象, 统一管理组内电机 */
typedef struct ht_group_struct_t
{
		ht_motor_t* motor[4];
	
	  void (*group_set_torque)(struct ht_group_struct_t *group);
	
		void (*group_sleep)(struct ht_group_struct_t *group);
	
	  void (*group_init)(struct ht_group_struct_t *group);
	
	  void (*group_heartbeat)(struct ht_group_struct_t *group);
	
}ht_group_t;

void ht_motor_init(ht_motor_t *motor);
void HT_Group_Motor_Init(ht_group_t *group);
extern uint8_t flag_rx;

#endif

