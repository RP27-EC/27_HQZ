#ifndef __CHASSIS_H
#define __CHASSIS_H

#include  "motor.h" 

#define   CHASSIS_MAX_SPEED     80
#define   FRONT_MAX_SPEED       50
#define   LEFT_MAX_SPEED        50
#define   CYCLE_MAX_SPEED       40
#define   TURN_CYCLE_SPEED      55

#define  ROOM_ENOUGH_GIMBAL   0


typedef enum{
	SPEED_MODE,
	POSITION_MODE,
	
}Chassis_Pid_Mode_e;


typedef enum{
  C_SLEEP,
	C_INIT,
	C_BOSS,
	C_SLAVE,

}Chassis_Mode_e;


typedef struct{
	float  front_speed;
	float  left_speed;
	float  cycle_speed;
	
	float  front_location;
	float  left_location;
	float  cycle_location;
	
	float  motor_speed[WHEEL_CNT];
	float  motor_position[WHEEL_CNT];
	
}Chassis_Target_t;


typedef struct{
	
	float   front_speed;
	float   left_speed;
	float   cycle_speed;
	
	float  front_location;
	float  left_location;
	float  cycle_location;
	
}Chassis_Measure_t;



typedef struct{
  int16_t  w_s_now;
	int16_t  a_d_now;
	
	int16_t  w_s_last;
	int16_t  a_d_last;


}Chassis_Key_Info_t;


typedef struct{
	uint8_t  wheel_heart[WHEEL_CNT];

}Chassis_State_t;



typedef struct{
	float  wheel_feed_out[WHEEL_CNT];
	float  wheel_initial_out[WHEEL_CNT];
	float  wheel_powerd_out[WHEEL_CNT];
	float  wheel_end_out[WHEEL_CNT];

}Chassis_Out_t;

/**
 * @brief  底盘运动学逆解算，车速算轮速
 * @note   
 */
typedef struct{
	bool  slip_flag;
	bool  is_allot;
	float wheel_speed_max_difference;
  float slip_low_out;
}Chassis_Slip_t;


typedef struct Chassis_Struct_t{
	Motor_RM_Group_t*   wheel;
	Chassis_Pid_Mode_e  pid_mode; 
	Chassis_Mode_e      mode;
  Chassis_Target_t    target;
	Chassis_Measure_t   measure;
	Chassis_Key_Info_t  key;
	Chassis_State_t     state;
	Chassis_Slip_t      slip;
	float               power_coefficient[4][6];
	bool                burst_flag;
	
  Chassis_Out_t       out;
	
	void (*init)(struct Chassis_Struct_t* chassis);
	void (*work)(struct Chassis_Struct_t* chassis);
	void (*heart_beat)(struct Chassis_Struct_t* chassis);

}Chassis_t;


extern Chassis_t chassis;


#endif


