#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "stdint.h"
#include <stdbool.h>

#define   YAW_MEC_ZERO_ANGLE          -0.387884378
#define   PITCH_MEC_ZERO_ANGLE        0.f 
#define   PITCH_MEC_MAX_ANGLE         30.f*PI/180
#define   PITCH_MEC_MIN_ANGLE         -7.5f*PI/180

#define   PITCH_IMU_MAX_ANGLE         (gimbal->info.pitch_imu + (PITCH_MEC_MAX_ANGLE - gimbal->info.pitch_mec)/PI*180.f)
#define   PITCH_IMU_MIN_ANGLE         (gimbal->info.pitch_imu - (gimbal->info.pitch_mec - PITCH_MEC_MIN_ANGLE)/PI*180.f)

#define   PI      3.1415926

typedef enum{
	G_SLEEP,
	G_INIT,
	G_BOSS,
	G_SLAVE,

}Gimbal_Mode_e;


typedef enum{
	FRONT,
	RIGHT,
	BEHIND,
	LEFT,
	DIRECT_CNT,

}Gimbal_Direct_e;

typedef struct{
	float yaw_mec_tar;
	float pitch_mec_tar;
	float yaw_imu_tar;
	float pitch_imu_tar;
	
	float gimbal_height;


}Gimbal_Target_t;


typedef struct{
  float yaw_mec;
	float pitch_mec;
	float yaw_imu;
	float pitch_imu;

	float yaw_mec_err_raw;
	
	float pitch_mec_err_raw;
	
	float yaw_mec_err_act;
	
	float gimbal_height;

	

}Gimbal_Info_t;

typedef struct{
	float rc_yaw_mec_step;
	float rc_pitch_mec_step;
	float rc_yaw_imu_step;
	float rc_pitch_imu_step;
	float key_yaw_mec_step;
	float key_pitch_mec_step;
	float key_yaw_imu_step;
	float key_pitch_imu_step;
	
  float yaw_zero[DIRECT_CNT];
	
}Gimbal_Config_t;


typedef struct{
	uint8_t  yaw_heart;
	uint8_t  pitch_heart;
	uint8_t  left_heart;

}Gimbal_State_t;


typedef struct Gimbal_Struct_t{
  Gimbal_Mode_e      mode;
  Gimbal_Target_t    target;
	Gimbal_Info_t      info;
	Gimbal_Config_t    config;
	Gimbal_State_t     state;
  
	bool        gimbal_reset_flag;
	
	void (*init)(struct Gimbal_Struct_t* gimbal);
	void (*work)(struct Gimbal_Struct_t* gimbal);
	void (*heart_beat)(struct Gimbal_Struct_t* gimbal);
	
	
}Gimbal_t;

extern  Gimbal_t   gimbal;


#endif

