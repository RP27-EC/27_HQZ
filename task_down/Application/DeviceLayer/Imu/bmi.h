/* bmi.h - Mahony 互补滤波姿态解算 */
#ifndef __BMI_MAHONY_H
#define __BMI_MAHONY_H
#include <stdio.h>
#include "arm_math.h"
#include "rp_config.h"
#define ACCD_X_LSB 0x0c
#define ACCD_X_MSB 0x0d
#define ACCD_Y_LSB 0x0e
#define ACCD_Y_MSB 0x0f
#define ACCD_Z_LSB 0x10
#define ACCD_Z_MSB 0x11
#define GYR_X_LSB 0x12
#define GYR_X_MSB 0x13
#define GYR_Y_LSB 0x14
#define GYR_Y_MSB 0x15
#define GYR_Z_LSB 0x16
#define GYR_Z_MSB 0x17
#define TEMPERATURE_0 0x22
#define TEMPERATURE_1 0x23
#define TEMP_RATIO (0.001953125f)
/* 陀螺仪坐标变换为云台坐标结构体 */
typedef struct
{
    float arz;
    float ary;
    float arx;
    float trans[9];
} imu_frame_t;

/* 陀螺仪数据解算结构体 */
typedef struct
{
    float Kp;               //Kp越大，越信任加速度。
    float norm;             //归一化
    float halfT;            //解算周期的一半
    float gx, gy, gz;       //陀螺仪数据
    float ax, ay, az;       //加速度计数据
    float vx, vy, vz;       //算出来的加速度
    float ex, ey, ez;       //加速度计和陀螺仪数据解算的误差
    float q0, q1, q2, q3;   //四元数
    float q0_temp, q1_temp, q2_temp, q3_temp;//四元数临时变量
    float sintemp, sintemp_, costemp, costemp_;
} bmi_t;
extern imu_frame_t imu_frame;
void imu_frame_init(imu_frame_t *imu_frame);
void imu_frame_rotate(float gx, float gy, float gz,\
	                  float ax, float ay, float az,\
	                  float *ggx, float *ggy, float *ggz,\
					  float *aax, float *aay, float *aaz);
uint8_t imu_mahony_euler(float *pitch,float *roll,float *yaw,\
						   float *gx,float *gy,float *gz,\
						   float *ax,float *ay,float *az);										 
void imu_world_accel(float pitch, float roll, float yaw,\
						  float ax, float ay, float az,\
						  float *accx, float *accy, float *accz);
void BMI_Change_Kp(void);                          
extern bmi_t bmi;
							
#endif

