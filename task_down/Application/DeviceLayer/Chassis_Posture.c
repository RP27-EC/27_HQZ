/* Chassis_Posture.c - 底盘姿态设备 */

#include "Chassis_Posture.h"
#include "arm_math.h"
static void Chassis_Posture_Update(Chassis_Posture_t* My_Chassis_Posture);
static void Chassis_Slope_Update(Chassis_Posture_t* My_Chassis_Posture, float yaw, float pitch, float roll);
Chassis_Posture_info_t Chassis_Posture_info;

Chassis_Posture_t Chassis_Posture = {
	.data_update = Chassis_Posture_Update,
	.info = &Chassis_Posture_info,
};

static void Chassis_Posture_Update(Chassis_Posture_t* My_Chassis_Posture)
{
	Chassis_Posture_info_t* info = My_Chassis_Posture->info;
	

	if(imu_dev.info->base_info.roll > 0)
	{
		info->pitch = imu_dev.info->base_info.roll * Degree_to_rad - PI;
	}
	else
	{
		info->pitch = imu_dev.info->base_info.roll * Degree_to_rad + PI;
	}
	info->pitch *= -1.f;

	info->roll = imu_dev.info->base_info.pitch * Degree_to_rad + 0.004f;


	info->yaw = imu_dev.info->base_info.yaw * Degree_to_rad;
	

	info->roll_v = imu_dev.info->base_info.ave_rate_pitch * Degree_to_rad;
	info->pitch_v = -imu_dev.info->base_info.ave_rate_roll * Degree_to_rad;
	info->yaw_v = -imu_dev.info->base_info.ave_rate_yaw * Degree_to_rad;
	

	info->a_x = imu_dev.info->raw_info.acc_y;
	info->a_y = imu_dev.info->raw_info.acc_x;
	info->a_z = imu_dev.info->raw_info.acc_z;
	

	info->x_world = imu_dev.info->base_info.accy;
	info->y_world = imu_dev.info->base_info.accx;
	info->z_world = imu_dev.info->base_info.accz;
	

	Chassis_Slope_Update(My_Chassis_Posture, info->yaw, info->pitch, info->roll);
}
float test_yaw;
/* 坡道姿态刷新 */
static void Chassis_Slope_Update(Chassis_Posture_t* My_Chassis_Posture, float yaw, float pitch, float roll)
{

	float z_world[3];
	z_world[0] = arm_sin_f32(yaw)*arm_sin_f32(roll) + arm_cos_f32(yaw)*arm_sin_f32(pitch)*arm_cos_f32(roll);
	z_world[1] = -arm_cos_f32(yaw)*arm_sin_f32(roll) + arm_sin_f32(yaw)*arm_sin_f32(pitch)*arm_cos_f32(roll);
	z_world[2] = arm_cos_f32(pitch)*arm_cos_f32(roll);
	

	float temp;
	arm_sqrt_f32(z_world[0]*z_world[0]+z_world[1]*z_world[1], &temp);
	arm_atan2_f32(temp, z_world[2], &My_Chassis_Posture->info->slope_pitch);
	

	float sin_yaw, cos_yaw;
	if(My_Chassis_Posture->info->slope_pitch > 0.08f)
	{
		sin_yaw = z_world[1];
		cos_yaw = z_world[0];
		arm_atan2_f32(sin_yaw, cos_yaw, &My_Chassis_Posture->info->slope_yaw);
		test_yaw = yaw - My_Chassis_Posture->info->slope_yaw;
	}
	else
	{
		My_Chassis_Posture->info->slope_yaw = yaw;
	}
}


