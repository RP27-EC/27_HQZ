/* imu_sensor.h - IMU 设备抽象与姿态解算 */

#ifndef __IMU_H
#define __IMU_H
#include "rp_config.h"
#include "BMI088driver.h"
#include "BMI088reg.h"
#include "BMI088Middleware.h"
#include "drv_tim.h"
#include "PID.h"
#include "rp_math.h"
#include "ave_filter.h"
//#define IMU_Set_PWM(x) TIM3_Set_PWM(x) 
typedef enum{
  IMU_E_NONE,
  IMU_E_TYPE,
  IMU_E_ID,
  IMU_E_INIT,
  IMU_E_DATA,
  IMU_E_CALI,
} imu_errno_t;

typedef enum{
	DR_SPI1,
	DR_SPI2,
	DR_SPI3,
	DR_IIC,
	
}imu_bus_t;

typedef struct drive_str{
	imu_bus_t tpye;
	
	int8_t (*send)(struct drive_str *self, uint8_t *Txbuff, uint16_t len);
	int8_t (*read)(struct drive_str *self, uint8_t *Rxbuff, uint16_t len);
	int8_t (*sendread)(struct drive_str *self, uint8_t *Txbuff, uint8_t *Rxbuff, uint16_t len);
}imu_bus_ops_t;

typedef struct imu_state {
	dev_work_state_t dev_state;
	imu_errno_t	       err_code;
	
	uint8_t		err_cnt;
	int8_t		init_code;
	
	uint8_t		cali_end;
	
	uint8_t   offline_cnt;
	uint8_t   offline_max_cnt;
	
} imu_state_t;



typedef struct{
	float acc_x;
	float acc_y;
	float acc_z;
	
	float gyro_x;
	float gyro_y;
	float gyro_z;	

}	imu_raw_t;

typedef struct{
	float accx;
	float accy;
	float accz;
	
	float yaw;
	float pitch;
	float roll;
	float yaw_total_angle;
	
	float rate_yaw;
	float rate_pitch;
	float rate_roll;	
	
	float ave_rate_yaw;
	float ave_rate_pitch;	
	float ave_rate_roll;

    float temperature;

}	imu_fused_t;

typedef struct{
	float gx_offset;
	float gy_offset;
	float gz_offset;	
}	imu_offset_t;

typedef struct imu_data_struct {

	imu_raw_t  	raw_info;
	imu_fused_t 	base_info;
	imu_offset_t	offset_info;
	
	uint8_t		    init_flag;

} imu_data_t;

typedef struct imu_dev {
	
	imu_data_t		*info;
	imu_bus_ops_t	  	driver;
	pid_ctrl_t		*temp_pid;
	void			(*init)(struct imu_dev *self);
	void			(*update)(struct imu_dev *self);
	void			(*heart_beat)(struct imu_state *self);
    	void            (*set_temperature)(struct imu_dev *self, float temp);
	
	imu_state_t  	work_state;
	dev_id_t		id;	
} imu_dev_t;

extern imu_dev_t imu_dev;
#endif

