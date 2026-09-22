#include "imu_sensor.h"

#include "drv_gpio.h"

#include "drv_tick.h"



#if IMU_USE_EKF == 1

#include "bmi_EKF.h"

#endif

#if IMU_USE_MAHONY == 1

#include "bmi.h"

#endif
void imu_init(imu_dev_t *self);

void imu_heart_beat(imu_state_t *heart);

void imu_update(imu_dev_t *self);

//void imu_set_temperature(imu_dev_t *self, float temp);

#if IMU_USE_EKF == 1

static void InitQuaternion(float *init_q4);

static float Sqrt(float x);

#endif //IMU_USE_EKF
pid_ctrl_t imu_temp_pid = {

    .kp = 1000.0f,

    .ki = 0.0f,

    .out_max = 4500,

};

 

imu_data_t imu_data = 

{

	.offset_info.gx_offset = 0.f,

	.offset_info.gy_offset = 0.f,

	.offset_info.gz_offset = 0.f,

	.init_flag = 0,

};
imu_dev_t imu_dev = {



	.info = &imu_data,

	.driver.tpye = DR_SPI2,

	.work_state.dev_state = DEV_OFFLINE,

	.id = DEV_ID_IMU,	

	.work_state.cali_end = 0,	

	.work_state.offline_max_cnt = 50,	

	.work_state.err_cnt = 0,

    .temp_pid = &imu_temp_pid,

	

	.init = &imu_init,

	.update = &imu_update,

    .heart_beat = &imu_heart_beat,

//    .set_temperature = &imu_set_temperature,

};
float imu_read[3];

uint8_t init_cnt = 200;

/* IMU 初始化 */
void imu_init(struct imu_dev *self)

{

	uint32_t tickstart = HAL_GetTick();



	self->work_state.dev_state = DEV_OFFLINE;

	self->work_state.init_code = BMI088_init();



	while(self->work_state.init_code)

	{

		if (++self->work_state.err_cnt == init_cnt)

		{

			__set_FAULTMASK(1); 

			NVIC_SystemReset();

			break;

		}

        self->work_state.err_code = IMU_E_INIT;

        self->work_state.init_code = BMI088_init();

	}	



	

	if (self->work_state.init_code == 0)

	{

		self->work_state.dev_state = DEV_ONLINE;

		

//		self->work_state.err_cnt = 0;

		self->info->init_flag = 1;

		

#if IMU_USE_MAHONY == 1

		/* Mahony初始化 */

		imu_frame_init(&imu_frame);

#endif //IMU_USE_MAHONY

		

#if IMU_USE_EKF == 1

		/* EKF初始化 */

		imu_frame_init(&ekf_imu_frame);

		// float init_quaternion[4] = {0.999019921, -0.0315267481, -0.0310692526};

		float init_quaternion[4] = {0};

		InitQuaternion(init_quaternion);

		ekf_init(init_quaternion, 15, 0.001, 100000, 1);

#endif //IMU_USE_EKF

		

		self->work_state.err_code = IMU_E_NONE;

		imu_dev.info->offset_info.gx_offset = 0.f;

		imu_dev.info->offset_info.gy_offset = 0.f;

		imu_dev.info->offset_info.gz_offset = 0.f;

	}

	else

	{

		self->work_state.dev_state = DEV_OFFLINE;

		self->work_state.err_code = IMU_E_INIT;

		self->work_state.offline_cnt = self->work_state.offline_max_cnt;

		self->info->init_flag = 0;

	}

	

}



/* IMU 离线检测 */
void imu_heart_beat(imu_state_t *heart)

{

	heart->offline_cnt++;

	if(heart->offline_cnt > heart->offline_max_cnt) 

	{

		heart->offline_cnt = heart->offline_max_cnt;

		heart->dev_state = DEV_OFFLINE;

	}

	else 

	{

		if(heart->dev_state == DEV_OFFLINE)

        {

            heart->dev_state = DEV_ONLINE;

        }

	}

}



/* 接口说明 */
//void imu_set_temperature(imu_dev_t *self, float temp)

//{

//    self->temp_pid->err = temp - imu_data.base_info.temperature;

//    single_pid_ctrl(self->temp_pid);

//	/* 温度异常值保护 */

//    if(imu_data.base_info.temperature > 50 || imu_data.base_info.temperature < 0

//       || self->temp_pid->out < 0)

//    {

//        self->temp_pid->out = 0;

//    }



//    IMU_Set_PWM(self->temp_pid->out);

//}



ave_filter_t imu_pitch_dif_speed_ave_filter;

ave_filter_t imu_roll_dif_speed_ave_filter;

ave_filter_t imu_yaw_dif_speed_ave_filter;

/* 临时变量 */

static float pitch, roll, yaw;

static float gyrox, gyroy, gyroz;

static float accx, accy, accz;

static float gyrox_, gyroy_, gyroz_;

static float accx_, accy_, accz_;

static float gyro[3], accel[3], temp;

static int16_t imu_cnt = 0;

#if IMU_USE_EKF == 1

// 采样时间

static float imu_dt;

static uint32_t imu_tick_now, imu_tick_last;

#endif

void imu_update(imu_dev_t *imu_sen)

{



    imu_data_t *imu_data = imu_sen->info;

	

	/* 获取陀螺仪数据 */

	BMI088_read(gyro, accel, &temp);

	if(imu_dev.info->base_info.temperature >= 40)
			imu_dev.info->offset_info.gz_offset = 0.0007f;

	imu_data->raw_info.acc_x = accel[0];

	imu_data->raw_info.acc_y = accel[1];

	imu_data->raw_info.acc_z = accel[2];

	imu_data->raw_info.gyro_x = gyro[0];

	imu_data->raw_info.gyro_y = gyro[1];

	imu_data->raw_info.gyro_z = gyro[2];

	

	/* 坐标系变换 */

	imu_frame_rotate(gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2],\

	                 &gyrox, &gyroy, &gyroz, &accx, &accy, &accz);

	

	/* 陀螺仪校正 */

	if (imu_sen->work_state.err_code == IMU_E_CALI)

	{

		if (imu_cnt < 2000)

		{

			imu_data->offset_info.gx_offset -= gyrox * 0.0005f;

			imu_data->offset_info.gy_offset -= gyroy * 0.0005f;

			imu_data->offset_info.gz_offset -= gyroz * 0.0005f;

			imu_cnt++;

		}

		else

		{

			imu_cnt = 0;

			imu_sen->work_state.err_code = IMU_E_NONE;

			

			if (fabs(imu_data->offset_info.gx_offset) > 5.f)

				imu_data->offset_info.gx_offset = 0;

			if (fabs(imu_data->offset_info.gy_offset) > 5.f)

				imu_data->offset_info.gy_offset = 0;

			if (fabs(imu_data->offset_info.gz_offset) > 5.f)

				imu_data->offset_info.gz_offset = 0;

			imu_sen->work_state.cali_end = 1;

		}

	}

	else

	{

		gyrox += imu_data->offset_info.gx_offset;

		gyroy += imu_data->offset_info.gy_offset;

		gyroz += imu_data->offset_info.gz_offset;

	}

	

	/* 原始数据低通滤波 */

	gyrox_ = Lowpass(gyrox_, gyrox, 1);

	gyroy_ = Lowpass(gyroy_, gyroy, 1);

	gyroz_ = Lowpass(gyroz_, gyroz, 1);

	accx_ = Lowpass(accx_, accx, 0.8);

	accy_ = Lowpass(accy_, accy, 0.2);

	accz_ = Lowpass(accz_, accz, 0.2);

	

	/* 解算陀螺仪数据 */

#if IMU_USE_MAHONY == 1

	imu_mahony_euler(&imu_data->base_info.pitch, &imu_data->base_info.roll, &imu_data->base_info.yaw,\

										 &gyrox_, &gyroy_, &gyroz_, \

										 &accx_, &accy_, &accz_);

#endif



#if IMU_USE_EKF == 1

	// TODO: ins 替换为 imu

	// 采样时间

	imu_tick_now = micros();

	if(imu_tick_last == 0) // 第一次特殊处理

	{

		imu_tick_last = imu_tick_now - 1000; // 间隔1ms

	}

	imu_dt = (imu_tick_now - imu_tick_last) * 0.000001f; // us to s

	imu_tick_last = imu_tick_now;



    // 核心函数,EKF更新四元数

    ekf_update(gyrox, gyroy, gyroz, accx, accy, accz, imu_dt);



		imu_data->base_info.yaw = g_ekf.Yaw;

    imu_data->base_info.pitch = g_ekf.Pitch;

    imu_data->base_info.roll = g_ekf.Roll;

    imu_data->base_info.yaw_total_angle = g_ekf.YawTotalAngle;

#endif

	/* 获取世界坐标系的加速度 */						 

	pitch = imu_data->base_info.pitch, roll = imu_data->base_info.roll, yaw = imu_data->base_info.yaw;

	imu_world_accel(pitch, roll, yaw,\

											 accx_, accy_, accz_,\

											 &imu_data->base_info.accx, &imu_data->base_info.accy, &imu_data->base_info.accz);

	

	/* 计算陀螺仪数据 */

	//pitch

	imu_data->base_info.rate_pitch = gyroy_ / (double)0.017453;

	imu_data->base_info.ave_rate_pitch = avg_push(&imu_pitch_dif_speed_ave_filter, imu_data->base_info.rate_pitch, 3);

	

	//roll

	imu_data->base_info.rate_roll = gyrox_ / (double)0.017453;

	imu_data->base_info.ave_rate_roll = avg_push(&imu_roll_dif_speed_ave_filter, imu_data->base_info.rate_roll, 3);

	

	//yaw

	imu_data->base_info.rate_yaw = gyroz_ / (double)0.017453;

	imu_data->base_info.ave_rate_yaw = avg_push(&imu_yaw_dif_speed_ave_filter, imu_data->base_info.rate_yaw, 3);

	

	imu_sen->work_state.offline_cnt = 0;

	

	/* imu读取数据判断  */

	if ((accel[0] == 0) && (accel[1] == 0) && (accel[2] == 0) \

		 && (gyro[0] == 0) && (gyro[1] == 0) && (gyro[2]== 0))

	{

		if(++imu_sen->work_state.err_cnt >= 100)

		{

			imu_sen->work_state.dev_state = DEV_OFFLINE;

			imu_sen->work_state.err_code = IMU_E_DATA;

			imu_sen->work_state.offline_cnt = imu_sen->work_state.offline_max_cnt;

			imu_sen->work_state.err_cnt = 100;

		}

	}



    /* imu获取温度 */

    imu_data->base_info.temperature = temp;



}



#if IMU_USE_EKF == 1

// 使用加速度计的数据初始化Roll和Pitch,而Yaw置0,这样可以避免在初始时候的姿态估计误差

static void InitQuaternion(float *init_q4)

{

    float acc_sum[3] = {0};

	float gyro[3], acc_init[3], temp;



    // 读取100次加速度计数据,取平均值作为初始值

    for (uint8_t i = 0; i < 100; ++i)

    {

        BMI088_read(gyro, acc_init, &temp);

        acc_sum[0] += acc_init[0];

        acc_sum[1] += acc_init[1];

        acc_sum[2] += acc_init[2];

        delay_ms(1);

    }

    for (uint8_t i = 0; i < 3; ++i)

        acc_init[i] = acc_sum[i]/100;

		float pitch = atan2(-acc_init[0], Sqrt(acc_init[1] * acc_init[1] + acc_init[2] * acc_init[2]));

		float roll = atan2(acc_init[1], acc_init[2]);

		

		float half_pitch = pitch / 2.0f;

    float half_roll = roll / 2.0f;



    init_q4[0] = cos(half_pitch) * cos(half_roll);

    init_q4[1] = sin(half_pitch) * cos(half_roll);

    init_q4[2] = cos(half_pitch) * sin(half_roll);

    init_q4[3] = sin(half_pitch) * sin(half_roll);

}



// 快速开方

static float Sqrt(float x)

{

    float y;

    float delta;

    float maxError;



    if (x <= 0)

    {

        return 0;

    }



    // initial guess

    y = x / 2;



    // refine

    maxError = x * 0.001f;



    do

    {

        delta = (y * y) - x;

        y -= delta / (2 * y);

    } while (delta > maxError || delta < -maxError);



    return y;

}





#endif //IMU_USE_EKF




