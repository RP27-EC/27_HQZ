#ifndef __INFANTRY_H
#define __INFANTRY_H

#include "motor.h"//电机定义部分

#define  WHEEL_UP_TO_ONCE        rc_info->thumbwheel.step[0] != last_thumbwheel_step[0] || rc_info->thumbwheel.step[1] != last_thumbwheel_step[1]    
#define  WHEEL_DOWN_TO_ONCE        rc_info->thumbwheel.step[2] != last_thumbwheel_step[2] || rc_info->thumbwheel.step[3] != last_thumbwheel_step[3]   


typedef enum{
	RC_CTRL,//遥控器控制
	KEY_CTRL,//键鼠控制

}Infantry_Ctrl_e;

typedef enum{
	I_SLEEP,//休眠模式
	I_INIT,//初始化模式
	I_MEC,//机械模式
	I_IMU,//陀螺仪模式
	I_TURN,//小陀螺模式
  I_HOLE,//过洞模式
}Infantry_Mode_e;



typedef enum{
	LOWING,//低电平 无动作
	RISING,//上升沿
	HIGHING,//高电平 进行中
	FALLING,//下降沿
	
}Signal_Form_e;

typedef struct{
	bool  value; //当前状态
	bool  last_value; //上一次状态
	Signal_Form_e    form; //信号波形状态
	uint16_t  tick; //计数器
	uint16_t  tick_max; //计数器最大值

}Flag_Class_t;



typedef struct{
	Flag_Class_t  U_turn_flag;  //180度掉头动作标志
	Flag_Class_t  L_turn_flag; //左转90度动作标志
	Flag_Class_t  R_turn_flag; //右转90度动作标志
//	Flag_Class_t    hole_flag;
	Flag_Class_t    chassis_reset;	//底盘复位动作标志
	
	bool    mec_flag;  //机械模式标志
	bool    imu_flag;  //陀螺仪模式标志
  bool    turn_flag;   //小陀螺模式标志
	bool    hole_flag;  //过洞模式标志
	uint8_t vision_flag;  //自瞄标志(0:无, 1:装甲板, 2:小符, 3:大符, 4:前哨站）
	bool    broken_flag;  //损坏标志
	
	bool    cap_use_flag; //超电使用标志

//  bool    U_turn_flag;
//	bool    L_turn_flag;
//	bool    R_turn_flag;
	
	bool    chassis_off; //底盘掉电标志
	bool    gimbal_off;  //云台掉电标志
	
//	bool    chassis_reset;
	bool    car_reset;  //车辆复位标志
	
}Infantry_Flag_t;



typedef struct Infantry_Struct_t{
	Infantry_Ctrl_e          ctrl;  //当前控制模式(遥控器/键鼠)
	Infantry_Ctrl_e          last_ctrl;  //上次控制模式
  Infantry_Mode_e          mode;  //当前工作模式
	Infantry_Mode_e          last_mode; //上次工作模式
	Infantry_Flag_t          flag;  //标志位

	//面向对象的接口函数
	void (* init)(struct Infantry_Struct_t* infantry);
  void (* work)(struct Infantry_Struct_t* infantry);
	void (* heart_beat)(struct Infantry_Struct_t* infantry);

}Infantry_t;



extern Infantry_t  infantry;


#endif


