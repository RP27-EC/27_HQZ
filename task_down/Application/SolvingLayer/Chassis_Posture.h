/* Chassis_Posture.h - 底盘姿态解算 */

#ifndef __CHASSIS_POSTURE_H
#define __CHASSIS_POSTURE_H
#include "imu_sensor.h"
#include "arm_math.h"
#define Degree_to_rad 0.017453f

typedef struct Chassis_Posture_info_struct_t
{
	float pitch;
	
	float yaw;
	
	float roll;
	
	float pitch_v;
	
	float yaw_v;
	
	float roll_v;
	
	float a_x;
	
	float a_y;
	
	float a_z;
	
	float x_world;
	
	float y_world;
	
	float z_world;
}Chassis_Posture_info_t;


typedef struct Chassis_Posture_struct_t
{
	Chassis_Posture_info_t *info;
	void (*data_update)(struct Chassis_Posture_struct_t* My_Chassis_Posture);
}Chassis_Posture_t;
/* Servo functions */
extern Chassis_Posture_t Chassis_Posture;

#endif

