/* RM_motor.h - RM 电机驱动 */

#ifndef __RM_MOTOR_H
#define __RM_MOTOR_H
#include "rp_config.h"
#include "pid.h"
#include "drv_can.h"
#include "motor_def.h"
#define _3508_TORQUE_CONSTANT     0.3f //3508加减速箱的扭矩常数，N*m/A
#define _2006_TORQUE_CONSTANT     0.18f //2006的扭矩常数，N*m/A
#define _3508_MAX_CURRENT         20.f    //3508输出最大电流，手册-20~20A
#define _2006_MAX_CURRENT     		10.f //2006输出最大电流，手册-10~10A

#define _6020_TORQUE_CONSTANT     1.f //6020的转速常数，rpm/V
#define _6020_MAX_CURRENT         25000.f    //3508输出最大电流，手册-20~20A

#define _3508_REDUCT_RATIO        (19.f/1.f)
#define _2006_REDUCT_RATIO        (36.f/1.f)
/*电机模式*/
typedef enum rm_type
{
	_3508_Single,//3508不加减速箱
	_3508_Reduction,//3508加减速箱
	_6020_Single,//单6020电机
	_2006_Single,//单2006电机
}rm_type_t;

typedef struct rm_cfg_struct_t
{
	 int8_t order_correction;
		
	uint8_t rxId;//对应一拖四的序号0~3
	
	uint32_t stdId;
	
	rm_type_t type;//电机类型
	
#ifdef __STM32F4xx_HAL_H
    CAN_HandleTypeDef *hcan;//can口选择
#endif

#ifdef STM32H7xx_HAL_H
    FDCAN_HandleTypeDef *hcan;//can口选择
#endif
}rm_cfg_t;

typedef struct rm_rx_struct_t
{
		float torque;
	
	  float torque_current;
	
	  int16_t torque_current_raw;
	
		int16_t encoder_speed;//rpm(r/min)
	
		float speed;//rad/s
	
    uint16_t encoder;//0~8191
	
		int32_t encoder_sum;
	
		uint16_t encoder_last;

    float motor_angle_sum;

    float motor_angle;//电机弧度制绝对角度，0~2PI
	
	  float motor_angle_last;

    int8_t temperature;
}rm_rx_t;


typedef struct rm_ctrl_struct_t
{
	bool Speed_Input_Flag;//使用外部传感器的速度标志位：0不使用，1使用
	pid_ctrl_t* angle_ctrl_inner;//角度环内环
	
	bool Angle_Input_Flag;//使用外部传感器的角度标志位：0不使用，1使用
	bool Nearest_Return;//半圈处理标志位：0不使用，1使用
	pid_ctrl_t* angle_ctrl_outer;//角度环外环
	
	pid_ctrl_t* speed_ctrl;//速度环
}rm_ctrl_t;

typedef struct rm_tx_struct_t
{
		float	torque;//需要发送的转矩
	
	  float torque_current;
	
		int16_t torque_current_raw;
	
		uint8_t tx_buff[8];
	
}rm_tx_t;

typedef struct rm_state_struct_t
{
    uint32_t offline_cnt;

    uint32_t offline_cnt_max;

    dev_work_state_t status;
		

}rm_state_t;

typedef struct rm_motor_struct_t
{
    rm_cfg_t* born_info;
	
    rm_rx_t* rx_info;
	
		rm_tx_t* tx_info;

    rm_state_t* state;
	
		rm_ctrl_t* ctrl;
	
		void (*single_set_torque)(struct rm_motor_struct_t *motor);
	
		void (*single_set_speed)(struct rm_motor_struct_t *motor);
	
		void (*single_set_angle)(struct rm_motor_struct_t *motor);
	
	  void (*rx)(struct rm_motor_struct_t *motor, uint8_t *rxBuf);
	
	  void (*single_sleep)(struct rm_motor_struct_t *motor);
	
		void (*single_ctrl)(struct rm_motor_struct_t *group);
	
	  void (*single_init)(struct rm_motor_struct_t *motor);
	
	  void (*single_heart_beat)(struct rm_motor_struct_t *motor);
}rm_motor_t;

typedef struct rm_group_struct_t
{
	  rm_motor_t* motor[4];
	
		uint8_t tx_buff[8];
	
		uint32_t stdId;
	
    CAN_HandleTypeDef *hcan;
	
	  void (*group_set_torque)(struct rm_group_struct_t *group);
	
		void (*group_ctrl)(struct rm_group_struct_t *group);
	
		void (*group_sleep)(struct rm_group_struct_t *group);
	
	  void (*group_init)(struct rm_group_struct_t *group);
	
	  void (*group_heartbeat)(struct rm_group_struct_t *group);
	
}rm_group_t;
void rm_motor_init(rm_motor_t *motor);
void rm_group_init(rm_group_t *group);

#endif


