/* gimbal.h - 云台控制 */

#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "stdint.h"
#include <stdbool.h>

#define   YAW_MEC_ZERO_ANGLE          -0.387884378 /* Yaw 机械零点，rad */
#define   PITCH_MEC_ZERO_ANGLE        0.f          /* Pitch 机械零点，rad */
#define   PITCH_MEC_MAX_ANGLE         30.f*PI/180  /* Pitch 机械上限，rad */
#define   PITCH_MEC_MIN_ANGLE         -7.5f*PI/180 /* Pitch 机械下限，rad */

#define   PITCH_IMU_MAX_ANGLE         (gimbal->info.pitch_imu + (PITCH_MEC_MAX_ANGLE - gimbal->info.pitch_mec)/PI*180.f)
#define   PITCH_IMU_MIN_ANGLE         (gimbal->info.pitch_imu - (gimbal->info.pitch_mec - PITCH_MEC_MIN_ANGLE)/PI*180.f)

#define   PI      3.1415926

/* 云台控制模式 */
typedef enum{
	G_SLEEP, /* 休眠 */
	G_INIT,  /* 初始化 */
	G_BOSS,  /* 主控模式 */
	G_SLAVE, /* 从控模式 */
}Gimbal_Mode_e;


/* 云台朝向 */
typedef enum{
	FRONT,     /* 前 */
	RIGHT,     /* 右 */
	BEHIND,    /* 后 */
	LEFT,      /* 左 */
	DIRECT_CNT,/* 方向数量 */
}Gimbal_Direct_e;

/* 云台目标量 */
typedef struct{
	float yaw_mec_tar;   /* Yaw 机械目标，rad */
	float pitch_mec_tar; /* Pitch 机械目标，rad */
	float yaw_imu_tar;   /* Yaw IMU 目标，deg */
	float pitch_imu_tar; /* Pitch IMU 目标，deg */
	float gimbal_height; /* 云台高度 */
}Gimbal_Target_t;


/* 云台反馈与误差 */
typedef struct{
  float yaw_mec;           /* Yaw 机械角，rad */
	float pitch_mec;         /* Pitch 机械角，rad */
	float yaw_imu;           /* Yaw IMU 角，deg */
	float pitch_imu;         /* Pitch IMU 角，deg */
	float yaw_mec_err_raw;   /* Yaw 原始机械误差 */
	float pitch_mec_err_raw; /* Pitch 原始机械误差 */
	float yaw_mec_err_act;   /* Yaw 生效机械误差 */
	float gimbal_height;     /* 云台高度 */
}Gimbal_Info_t;

/* 遥控与键鼠步长配置 */
typedef struct{
	float rc_yaw_mec_step;   /* 遥控 Yaw 机械步长 */
	float rc_pitch_mec_step; /* 遥控 Pitch 机械步长 */
	float rc_yaw_imu_step;   /* 遥控 Yaw IMU 步长 */
	float rc_pitch_imu_step; /* 遥控 Pitch IMU 步长 */
	float key_yaw_mec_step;  /* 键鼠 Yaw 机械步长 */
	float key_pitch_mec_step;/* 键鼠 Pitch 机械步长 */
	float key_yaw_imu_step;  /* 键鼠 Yaw IMU 步长 */
	float key_pitch_imu_step;/* 键鼠 Pitch IMU 步长 */
  float yaw_zero[DIRECT_CNT]; /* 各朝向 Yaw 零点 */
}Gimbal_Config_t;


/* 云台设备在线状态 */
typedef struct{
	uint8_t yaw_heart;   /* Yaw 电机在线 */
	uint8_t pitch_heart; /* Pitch 电机在线 */
	uint8_t left_heart;  /* 左侧设备在线 */
}Gimbal_State_t;


/* 云台控制对象 */
typedef struct Gimbal_Struct_t{
  Gimbal_Mode_e mode;       /* 控制模式 */
  Gimbal_Target_t target;   /* 目标量 */
	Gimbal_Info_t info;       /* 反馈量 */
	Gimbal_Config_t config;   /* 输入配置 */
	Gimbal_State_t state;     /* 在线状态 */
	bool gimbal_reset_flag;   /* 初始化复位请求 */
	void (*init)(struct Gimbal_Struct_t* gimbal);       /* 初始化 */
	void (*work)(struct Gimbal_Struct_t* gimbal);       /* 周期控制 */
	void (*heart_beat)(struct Gimbal_Struct_t* gimbal); /* 心跳更新 */
}Gimbal_t;

extern  Gimbal_t   gimbal;


#endif


