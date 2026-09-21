/* bmi.c - Mahony 互补滤波姿态解算 */

#include "bmi.h"
#include "rp_math.h"
#include "ave_filter.h"

#if IMU_USE_MAHONY==1
/* Macros to select the sensors */

/* 重力加速度 */
#define GRAVITY_EARTH  (9.80665f)
/* 矩阵实例定义 */
arm_matrix_instance_f32 Trans;
arm_matrix_instance_f32 Src;
arm_matrix_instance_f32 Dst;

/**
 * @brief   坐标变换采用Z-Y-X欧拉角描述，即从陀螺仪坐标系向云台坐标系变换中，
 *          坐标系按照绕陀螺仪Z轴、Y轴、X轴的顺序旋转
 *          每一次旋转的参考坐标系为当前陀螺仪坐标系  
 *  @param
 *  @arz
 *      陀螺仪x轴与roll轴之间的夹角，单位为度
 *  @ary
 *      陀螺仪x轴与yaw轴之间的夹角，单位为度
 *  @arx
 *      陀螺仪y轴与yaw轴之间的夹角，单位为度
 */
imu_frame_t imu_frame = {
    .arz = 90.0f,
    .ary = 0.0f,
    .arx = 0.0f,
    .trans = {0.0f},
};

/**
 * @param
 * @Kp
 *     越大表示越信任加速度，但快速晃动时，yaw轴角度可能会变化或者快速漂移。Kp越大，初始化的时候数据稳定越快。
 * @halfT
 *     解算周期的一半，比如1ms解算1次则halfT为0.0005f
 */
bmi_t bmi = {
    .Kp = 0.125f,
    .norm = 0.0f,
    .halfT = 0.00025f,
    .gx = 0.0f, .gy = 0.0f, .gz = 0.0f,
    .ax = 0.0f, .ay = 0.0f, .az = 0.0f,
    .vx = 0.0f, .vy = 0.0f, .vz = 0.0f,
    .ex = 0.0f, .ey = 0.0f, .ez = 0.0f,
    .q0 = 1.0f, .q1 = 0.0f, .q2 = 0.0f, .q3 = 0.0f,
    .q0_temp = 0.0f, .q1_temp = 0.0f, .q2_temp = 0.0f, .q3_temp = 0.0f,
    .sintemp = 0.0f, .sintemp_ = 0.0f, .costemp = 0.0f, .costemp_ = 0.0f,
};
/* 快速平方根倒数 */
float inVSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

/* 由安装角生成坐标变换矩阵 */
void imu_frame_init(imu_frame_t *imu_frame)
{
    float arz, ary, arx;

	/* 角度单位转换（to弧度） */
	arz = imu_frame->arz * (double)0.017453;
	ary = imu_frame->ary * (double)0.017453;
	arx = imu_frame->arx * (double)0.017453;

	/* 旋转矩阵赋值（三个旋转矩阵叠加） */
	imu_frame->trans[0] = arm_cos_f32(arz)*arm_cos_f32(ary);
	imu_frame->trans[1] = arm_cos_f32(arz)*arm_sin_f32(ary)*arm_sin_f32(arx) - arm_sin_f32(arz)*arm_cos_f32(arx);
	imu_frame->trans[2] = arm_cos_f32(arz)*arm_sin_f32(ary)*arm_cos_f32(arx) + arm_sin_f32(arz)*arm_sin_f32(arx);
	imu_frame->trans[3] = arm_sin_f32(arz)*arm_cos_f32(ary);
	imu_frame->trans[4] = arm_sin_f32(arz)*arm_sin_f32(ary)*arm_sin_f32(arx) + arm_cos_f32(arz)*arm_cos_f32(arx);
	imu_frame->trans[5] = arm_sin_f32(arz)*arm_sin_f32(ary)*arm_cos_f32(arx) - arm_cos_f32(arz)*arm_sin_f32(arx);
	imu_frame->trans[6] = -arm_sin_f32(ary);
	imu_frame->trans[7] = arm_cos_f32(ary)*arm_sin_f32(arx);
	imu_frame->trans[8] = arm_cos_f32(ary)*arm_cos_f32(arx);
	
    /* 3x3变换矩阵初始化 */
	arm_mat_init_f32(&Trans, 3, 3, (float *)imu_frame->trans); 
}

/* 传感器系转云台系 */
void imu_frame_rotate(float gx, float gy, float gz,\
	                  float ax, float ay, float az,\
	                  float *ggx, float *ggy, float *ggz,\
					  float *aax, float *aay, float *aaz)
{
    /* 陀螺仪输入输出数组定义 */
    float gyro_in[3], gyro_out[3];
    /* 加速度输入输出数组定义 */
    float acc_in[3], acc_out[3];

	/* 陀螺仪赋值 */
	gyro_in[0] = (float)gx, gyro_in[1] = (float)gy, gyro_in[2] = (float)gz;
	/* 加速度计赋值 */
	acc_in[0] = (float)ax, acc_in[1] = (float)ay, acc_in[2] = (float)az;
	
	/* 陀螺仪坐标变换 */
	arm_mat_init_f32(&Src, 1, 3, gyro_in);
	arm_mat_init_f32(&Dst, 1, 3, gyro_out);
	arm_mat_mult_f32(&Src, &Trans, &Dst);
	*ggx = gyro_out[0], *ggy = gyro_out[1], *ggz = gyro_out[2];
	
	/* 加速度计坐标变换 */
	arm_mat_init_f32(&Src, 1, 3, acc_in);
	arm_mat_init_f32(&Dst, 1, 3, acc_out);
	arm_mat_mult_f32(&Src, &Trans, &Dst);
	*aax = acc_out[0], *aay = acc_out[1], *aaz = acc_out[2];
}

/* Mahony 互补滤波解算欧拉角 */
uint8_t imu_mahony_euler(float *pitch,float *roll,float *yaw,\
						   float *gx,float *gy,float *gz,\
						   float *ax,float *ay,float *az)
{
	/* 角速度赋值 */
    bmi.gx = *gx;
    bmi.gy = *gy;
    bmi.gz = *gz;
	
	/* 加速度赋值 */
    bmi.ax = *ax;
    bmi.ay = *ay;
    bmi.az = *az;
	
	/* 角度解算start */
	/* 加速度计数据检查 */
	if(bmi.ax * bmi.ay * bmi.az != 0)
	{
        /* 加速度计测得加速度并归一化 */
        bmi.norm = inVSqrt(bmi.ax*bmi.ax + bmi.ay*bmi.ay + bmi.az*bmi.az);
        bmi.ax *= bmi.norm;
        bmi.ay *= bmi.norm;
        bmi.az *= bmi.norm;
		
        /* 四元数解算出的加速度 */
        /* -sin(Pitch) cos(K,i) */
        bmi.vx = -2*(bmi.q1*bmi.q3 - bmi.q0*bmi.q2);
        /* sin(Roll)cos(Pitch) cos(K,j) */
        bmi.vy = -2*(bmi.q0*bmi.q1 + bmi.q2*bmi.q3);   
        /* cos(Roll)cos(Pitch) cos(K,k) */
        bmi.vz = -(bmi.q0*bmi.q0 - bmi.q1*bmi.q1 - bmi.q2*bmi.q2 + bmi.q3*bmi.q3);  
		
        /* θ较小时，θ约等于sinθ约等于两向量外积 */
        bmi.ex = (bmi.az*bmi.vy - bmi.ay*bmi.vz);
        bmi.ey = (bmi.ax*bmi.vz - bmi.az*bmi.vx);
        bmi.ez = (bmi.ay*bmi.vx - bmi.ax*bmi.vy);
		
        /* 误差补偿，Kp越大，越信任加速度计 */
        bmi.gx = bmi.gx + bmi.ex*bmi.Kp;
        bmi.gy = bmi.gy + bmi.ey*bmi.Kp;
        bmi.gz = bmi.gz + bmi.ez*bmi.Kp;
	}
	
    /* 保存临时变量 */
    bmi.q0_temp = bmi.q0;
    bmi.q1_temp = bmi.q1;
    bmi.q2_temp = bmi.q2;
    bmi.q3_temp = bmi.q3;

    /* 四元数解算更新 */
    bmi.q0 = bmi.q0_temp + (-bmi.q1_temp*bmi.gx - bmi.q2_temp*bmi.gy -bmi.q3_temp*bmi.gz)*bmi.halfT;
    bmi.q1 = bmi.q1_temp + ( bmi.q0_temp*bmi.gx + bmi.q2_temp*bmi.gz -bmi.q3_temp*bmi.gy)*bmi.halfT;
    bmi.q2 = bmi.q2_temp + ( bmi.q0_temp*bmi.gy - bmi.q1_temp*bmi.gz +bmi.q3_temp*bmi.gx)*bmi.halfT;
    bmi.q3 = bmi.q3_temp + ( bmi.q0_temp*bmi.gz + bmi.q1_temp*bmi.gy -bmi.q2_temp*bmi.gx)*bmi.halfT;

    bmi.norm = inVSqrt(bmi.q0*bmi.q0 + bmi.q1*bmi.q1 + bmi.q2*bmi.q2 + bmi.q3*bmi.q3);
    bmi.q0 *= bmi.norm;
    bmi.q1 *= bmi.norm;
    bmi.q2 *= bmi.norm;
    bmi.q3 *= bmi.norm;
	
    /* asin(x) = atan(x/sqrt(1-x*x)) */
    bmi.sintemp = 2*bmi.q1*bmi.q3 - 2*bmi.q0*bmi.q2;
    arm_sqrt_f32(1 - bmi.sintemp*bmi.sintemp, &bmi.costemp); 

    /* 四元数解出欧拉角 */
    /* pitch = -asin(2*q1*q3 - 2*q0*q2)*57.295773f; */
    arm_atan2_f32(bmi.sintemp, bmi.costemp, pitch);
	/* yaw =  atan2(2*(q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3)*57.295773f; */
    arm_atan2_f32(2*(bmi.q1*bmi.q2 + bmi.q0*bmi.q3), 
                     bmi.q0*bmi.q0 + bmi.q1*bmi.q1 - bmi.q2*bmi.q2 - bmi.q3*bmi.q3, yaw);
    /* roll = atan2(2*q2*q3 + 2*q0*q1, q0*q0 - q1*q1 - q2*q2 + q3*q3)*57.295773f; */
    arm_atan2_f32(2*bmi.q2*bmi.q3 + 2*bmi.q0*bmi.q1, 
                    bmi.q0*bmi.q0 - bmi.q1*bmi.q1 - bmi.q2*bmi.q2 + bmi.q3*bmi.q3, roll);

    /* 转回角度制 */
	*roll  *=  57.295773f;
	*pitch *= -57.295773f;
	*yaw   *=  57.295773f;
	/* 角度解算end */
	
	return 0;
}

/* 机体系加速度转世界系 */
void imu_world_accel(float pitch, float roll, float yaw,\
						  float ax, float ay, float az,\
						  float *accx, float *accy, float *accz)
{
	float imu_accx, imu_accy, imu_accz;
	
    /* 角度制to弧度制 */
	pitch *= (double)0.017453;
	yaw   *= (double)0.017453;
	roll  *= (double)0.017453;

	imu_accx = ax + arm_sin_f32(pitch) * GRAVITY_EARTH;
	imu_accy = ay - arm_sin_f32(roll) * arm_cos_f32(pitch) * GRAVITY_EARTH;
	imu_accz = az - arm_cos_f32(roll) * arm_cos_f32(pitch) * GRAVITY_EARTH;
	
	*accx = imu_accx * arm_cos_f32(pitch) + imu_accz * arm_sin_f32(pitch);
	*accy = imu_accy * arm_cos_f32(roll) - imu_accz * arm_sin_f32(roll);
	*accz = imu_accz * arm_cos_f32(pitch) * arm_cos_f32(roll) - imu_accx * arm_sin_f32(pitch) * arm_cos_f32(roll) \
			+ imu_accy * arm_sin_f32(roll) * arm_cos_f32(pitch);
	
}

void BMI_Change_Kp(void)
{
    float *kp = &bmi.Kp;

	if(HAL_GetTick() <= 1000)
	{
		*kp = 1.0f;
	}
	else if(HAL_GetTick() > 1000)
	{
		*kp = 0.1f;
	}
}

#endif



