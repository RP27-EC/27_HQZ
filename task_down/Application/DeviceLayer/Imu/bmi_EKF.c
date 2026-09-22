/* bmi_EKF.c - 四元数 EKF 姿态解算 */

#include "bmi_EKF.h"

#include "rp_config.h"

#if IMU_USE_EKF==1





#define GRAVITY_EARTH  (9.80665f)



arm_matrix_instance_f32 ekf_trans;

arm_matrix_instance_f32 ekf_src;

arm_matrix_instance_f32 ekf_dst;



/* imu_frame_t */
imu_frame_t ekf_imu_frame = {

    .arz = 180.0f,

    .ary = 0.0f,

    .arx = 0.0f,

    .trans = {0.0f},

};



ekf_att_t g_ekf;



const float ekf_F_mat[36] = {1, 0, 0, 0, 0, 0,

                                       0, 1, 0, 0, 0, 0,

                                       0, 0, 1, 0, 0, 0,

                                       0, 0, 0, 1, 0, 0,

                                       0, 0, 0, 0, 1, 0,

                                       0, 0, 0, 0, 0, 1};

float ekf_P_mat[36] = {100000, 0.1, 0.1, 0.1, 0.1, 0.1,

                                 0.1, 100000, 0.1, 0.1, 0.1, 0.1,

                                 0.1, 0.1, 100000, 0.1, 0.1, 0.1,

                                 0.1, 0.1, 0.1, 100000, 0.1, 0.1,

                                 0.1, 0.1, 0.1, 0.1, 100, 0.1,

                                 0.1, 0.1, 0.1, 0.1, 0.1, 100};

float ekf_K_mat[18];

float ekf_H_mat[18];



static float invSqrt(float x);

static void ekf_observe(KalmanFilter_t *kf);

static void ekf_linearize(KalmanFilter_t *kf);

static void ekf_set_H(KalmanFilter_t *kf);

static void ekf_update_xhat(KalmanFilter_t *kf);



/* 初始化 EKF 参数与矩阵 */
void ekf_init(float* init_quaternion,float process_noise1, float process_noise2, float measure_noise, float lambda)

{

	  

    g_ekf.Initialized = 1;

    g_ekf.Q1 = process_noise1;

    g_ekf.Q2 = process_noise2;

    g_ekf.R = measure_noise;

    g_ekf.ChiSquareTestThreshold = 1e-8;

    g_ekf.ConvergeFlag = 0;

    g_ekf.ErrorCount = 0;

    g_ekf.UpdateCount = 0;

    if (lambda > 1)

    {

        lambda = 1;

    }

    g_ekf.lambda = lambda;





    kf_init(&g_ekf.IMU_QuaternionEKF, 6, 0, 3);

    mat_init(&g_ekf.ChiSquare, 1, 1, (float *)g_ekf.ChiSquare_Data);





    for(int i = 0; i < 4; i++)

    {

        g_ekf.IMU_QuaternionEKF.xhat_data[i] = init_quaternion[i];

    }





    g_ekf.IMU_QuaternionEKF.User_Func0_f = ekf_observe;

    g_ekf.IMU_QuaternionEKF.User_Func1_f = ekf_linearize;

    g_ekf.IMU_QuaternionEKF.User_Func2_f = ekf_set_H;

    g_ekf.IMU_QuaternionEKF.User_Func3_f = ekf_update_xhat;





    g_ekf.IMU_QuaternionEKF.SkipEq3 = TRUE;

    g_ekf.IMU_QuaternionEKF.SkipEq4 = TRUE;



    memcpy(g_ekf.IMU_QuaternionEKF.F_data, ekf_F_mat, sizeof(ekf_F_mat));

    memcpy(g_ekf.IMU_QuaternionEKF.P_data, ekf_P_mat, sizeof(ekf_P_mat));

}



/* EKF 一步递推 */
void ekf_update(float gx, float gy, float gz, float ax, float ay, float az, float dt)

{



    static float halfgxdt, halfgydt, halfgzdt;

    static float accelInvNorm;



    /*   F, number with * represent vals to be set

     0      1*     2*     3*     4     5

     6*     7      8*     9*    10    11

    12*    13*    14     15*    16    17

    18*    19*    20*    21     22    23

    24     25     26     27     28    29

    30     31     32     33     34    35

    */

    g_ekf.dt = dt;



    g_ekf.Gyro[0] = gx - g_ekf.GyroBias[0];

    g_ekf.Gyro[1] = gy - g_ekf.GyroBias[1];

    g_ekf.Gyro[2] = gz - g_ekf.GyroBias[2];



    // set F

    halfgxdt = 0.5f * g_ekf.Gyro[0] * dt;

    halfgydt = 0.5f * g_ekf.Gyro[1] * dt;

    halfgzdt = 0.5f * g_ekf.Gyro[2] * dt;







    memcpy(g_ekf.IMU_QuaternionEKF.F_data, ekf_F_mat, sizeof(ekf_F_mat));



    g_ekf.IMU_QuaternionEKF.F_data[1] = -halfgxdt;

    g_ekf.IMU_QuaternionEKF.F_data[2] = -halfgydt;

    g_ekf.IMU_QuaternionEKF.F_data[3] = -halfgzdt;



    g_ekf.IMU_QuaternionEKF.F_data[6] = halfgxdt;

    g_ekf.IMU_QuaternionEKF.F_data[8] = halfgzdt;

    g_ekf.IMU_QuaternionEKF.F_data[9] = -halfgydt;



    g_ekf.IMU_QuaternionEKF.F_data[12] = halfgydt;

    g_ekf.IMU_QuaternionEKF.F_data[13] = -halfgzdt;

    g_ekf.IMU_QuaternionEKF.F_data[15] = halfgxdt;



    g_ekf.IMU_QuaternionEKF.F_data[18] = halfgzdt;

    g_ekf.IMU_QuaternionEKF.F_data[19] = halfgydt;

    g_ekf.IMU_QuaternionEKF.F_data[20] = -halfgxdt;



		g_ekf.Accel[0] = ax;

		g_ekf.Accel[1] = ay;

		g_ekf.Accel[2] = az;



    accelInvNorm = invSqrt(g_ekf.Accel[0] * g_ekf.Accel[0] + g_ekf.Accel[1] * g_ekf.Accel[1] + g_ekf.Accel[2] * g_ekf.Accel[2]);

    for (uint8_t i = 0; i < 3; ++i)

    {

        g_ekf.IMU_QuaternionEKF.MeasuredVector[i] = g_ekf.Accel[i] * accelInvNorm;

    }





    g_ekf.gyro_norm = 1.0f / invSqrt(g_ekf.Gyro[0] * g_ekf.Gyro[0] +

                                        g_ekf.Gyro[1] * g_ekf.Gyro[1] +

                                        g_ekf.Gyro[2] * g_ekf.Gyro[2]);

    g_ekf.accl_norm = 1.0f / accelInvNorm;







    if (g_ekf.gyro_norm < 2.0f && g_ekf.accl_norm > 9.8f - 0.5f && g_ekf.accl_norm < 9.8f + 0.5f)

    {

        g_ekf.StableFlag = 1;

    }

    else

    {

        g_ekf.StableFlag = 0;

    }





    g_ekf.IMU_QuaternionEKF.Q_data[0] = g_ekf.Q1 * g_ekf.dt;

    g_ekf.IMU_QuaternionEKF.Q_data[7] = g_ekf.Q1 * g_ekf.dt;

    g_ekf.IMU_QuaternionEKF.Q_data[14] = g_ekf.Q1 * g_ekf.dt;

    g_ekf.IMU_QuaternionEKF.Q_data[21] = g_ekf.Q1 * g_ekf.dt;

    g_ekf.IMU_QuaternionEKF.Q_data[28] = g_ekf.Q2 * g_ekf.dt;

    g_ekf.IMU_QuaternionEKF.Q_data[35] = g_ekf.Q2 * g_ekf.dt;

    g_ekf.IMU_QuaternionEKF.R_data[0] = g_ekf.R;

    g_ekf.IMU_QuaternionEKF.R_data[4] = g_ekf.R;

    g_ekf.IMU_QuaternionEKF.R_data[8] = g_ekf.R;





    kf_update(&g_ekf.IMU_QuaternionEKF);





    g_ekf.q[0] = g_ekf.IMU_QuaternionEKF.FilteredValue[0];

    g_ekf.q[1] = g_ekf.IMU_QuaternionEKF.FilteredValue[1];

    g_ekf.q[2] = g_ekf.IMU_QuaternionEKF.FilteredValue[2];

    g_ekf.q[3] = g_ekf.IMU_QuaternionEKF.FilteredValue[3];

    g_ekf.GyroBias[0] = g_ekf.IMU_QuaternionEKF.FilteredValue[4];

    g_ekf.GyroBias[1] = g_ekf.IMU_QuaternionEKF.FilteredValue[5];

    g_ekf.GyroBias[2] = 0;  // 陀螺Bias





    g_ekf.Yaw = atan2f(2.0f * (g_ekf.q[0] * g_ekf.q[3] + g_ekf.q[1] * g_ekf.q[2]), 2.0f * (g_ekf.q[0] * g_ekf.q[0] + g_ekf.q[1] * g_ekf.q[1]) - 1.0f) * 57.295779513f;

    g_ekf.Roll = atan2f(2.0f * (g_ekf.q[0] * g_ekf.q[1] + g_ekf.q[2] * g_ekf.q[3]), 2.0f * (g_ekf.q[0] * g_ekf.q[0] + g_ekf.q[3] * g_ekf.q[3]) - 1.0f) * 57.295779513f;

    g_ekf.Pitch = asinf(-2.0f * (g_ekf.q[1] * g_ekf.q[3] - g_ekf.q[0] * g_ekf.q[2])) * 57.295779513f;





    if (g_ekf.Yaw - g_ekf.YawAngleLast > 180.0f)

    {

        g_ekf.YawRoundCount--;

    }

    else if (g_ekf.Yaw - g_ekf.YawAngleLast < -180.0f)

    {

        g_ekf.YawRoundCount++;

    }

    g_ekf.YawTotalAngle = 360.0f * g_ekf.YawRoundCount + g_ekf.Yaw;

    g_ekf.YawAngleLast = g_ekf.Yaw;

    g_ekf.UpdateCount++;

}



/* 状态转移线性化与渐消 */
static void ekf_linearize(KalmanFilter_t *kf)

{

    static float q0, q1, q2, q3;

    static float qInvNorm;



    q0 = kf->xhatminus_data[0];

    q1 = kf->xhatminus_data[1];

    q2 = kf->xhatminus_data[2];

    q3 = kf->xhatminus_data[3];





    qInvNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);

    for (uint8_t i = 0; i < 4; ++i)

    {

        kf->xhatminus_data[i] *= qInvNorm;

    }

    /*  F, number with * represent vals to be set

     0     1     2     3     4*     5*

     6     7     8     9    10*    11*

    12    13    14    15    16*    17*

    18    19    20    21    22*    23*

    24    25    26    27    28     29

    30    31    32    33    34     35

    */

    // set F

    kf->F_data[4] = q1 * g_ekf.dt / 2;

    kf->F_data[5] = q2 * g_ekf.dt / 2;



    kf->F_data[10] = -q0 * g_ekf.dt / 2;

    kf->F_data[11] = q3 * g_ekf.dt / 2;



    kf->F_data[16] = -q3 * g_ekf.dt / 2;

    kf->F_data[17] = -q0 * g_ekf.dt / 2;



    kf->F_data[22] = q2 * g_ekf.dt / 2;

    kf->F_data[23] = -q1 * g_ekf.dt / 2;





    kf->P_data[28] /= g_ekf.lambda;

    kf->P_data[35] /= g_ekf.lambda;





    if (kf->P_data[28] > 10000)

    {

        kf->P_data[28] = 10000;

    }

    if (kf->P_data[35] > 10000)

    {

        kf->P_data[35] = 10000;

    }

}



/* 构造量测矩阵 H */
static void ekf_set_H(KalmanFilter_t *kf)

{

    static float doubleq0, doubleq1, doubleq2, doubleq3;

    /* H

     0     1     2     3     4     5

     6     7     8     9    10    11

    12    13    14    15    16    17

    last two cols are zero

    */

    // set H

    doubleq0 = 2 * kf->xhatminus_data[0];

    doubleq1 = 2 * kf->xhatminus_data[1];

    doubleq2 = 2 * kf->xhatminus_data[2];

    doubleq3 = 2 * kf->xhatminus_data[3];



    memset(kf->H_data, 0, sizeof_float * kf->zSize * kf->xhatSize);



    kf->H_data[0] = -doubleq2;

    kf->H_data[1] = doubleq3;

    kf->H_data[2] = -doubleq0;

    kf->H_data[3] = doubleq1;



    kf->H_data[6] = doubleq1;

    kf->H_data[7] = doubleq0;

    kf->H_data[8] = doubleq3;

    kf->H_data[9] = doubleq2;



    kf->H_data[12] = doubleq0;

    kf->H_data[13] = -doubleq1;

    kf->H_data[14] = -doubleq2;

    kf->H_data[15] = doubleq3;

}



/* 状态更新 */
static void ekf_update_xhat(KalmanFilter_t *kf)

{

    static float q0, q1, q2, q3;



    kf->MatStatus = mat_trans(&kf->H, &kf->HT); // z|x => x|z

    kf->temp_matrix.numRows = kf->H.numRows;

    kf->temp_matrix.numCols = kf->Pminus.numCols;

    kf->MatStatus = mat_mul(&kf->H, &kf->Pminus, &kf->temp_matrix);  // Mat状态

    kf->temp_matrix1.numRows = kf->temp_matrix.numRows;

    kf->temp_matrix1.numCols = kf->HT.numCols;

    kf->MatStatus = mat_mul(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1);  // Mat状态

    kf->S.numRows = kf->R.numRows;

    kf->S.numCols = kf->R.numCols;

    kf->MatStatus = mat_add(&kf->temp_matrix1, &kf->R, &kf->S); // S = H P'(k) HT + R

    kf->MatStatus = mat_inv(&kf->S, &kf->temp_matrix1);  // Mat状态



    q0 = kf->xhatminus_data[0];

    q1 = kf->xhatminus_data[1];

    q2 = kf->xhatminus_data[2];

    q3 = kf->xhatminus_data[3];



    kf->temp_vector.numRows = kf->H.numRows;

    kf->temp_vector.numCols = 1;



    kf->temp_vector_data[0] = 2 * (q1 * q3 - q0 * q2);

    kf->temp_vector_data[1] = 2 * (q0 * q1 + q2 * q3);

    kf->temp_vector_data[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3; // temp_vector = h(xhat'(k))





    for (uint8_t i = 0; i < 3; ++i)

    {

        g_ekf.OrientationCosine[i] = acosf(fabsf(kf->temp_vector_data[i]));

    }





    kf->temp_vector1.numRows = kf->z.numRows;

    kf->temp_vector1.numCols = 1;

    kf->MatStatus = mat_sub(&kf->z, &kf->temp_vector, &kf->temp_vector1); // temp_vector1 = z(k) - h(xhat'(k))





    kf->temp_matrix.numRows = kf->temp_vector1.numRows;

    kf->temp_matrix.numCols = 1;

    kf->MatStatus = mat_mul(&kf->temp_matrix1, &kf->temp_vector1, &kf->temp_matrix);  // Mat状态

    kf->temp_vector.numRows = 1;

    kf->temp_vector.numCols = kf->temp_vector1.numRows;

    kf->MatStatus = mat_trans(&kf->temp_vector1, &kf->temp_vector); // temp_vector = z(k) - h(xhat'(k))'

    kf->MatStatus = mat_mul(&kf->temp_vector, &kf->temp_matrix, &g_ekf.ChiSquare);



    if (g_ekf.ChiSquare_Data[0] < 0.5f * g_ekf.ChiSquareTestThreshold)

    {

        g_ekf.ConvergeFlag = 1;

    }



    if (g_ekf.ChiSquare_Data[0] > g_ekf.ChiSquareTestThreshold && g_ekf.ConvergeFlag)

    {

        if (g_ekf.StableFlag)

        {

            g_ekf.ErrorCount++;

        }

        else

        {

            g_ekf.ErrorCount = 0;

        }



        if (g_ekf.ErrorCount > 50)

        {



            g_ekf.ConvergeFlag = 0;

            kf->SkipEq5 = FALSE;

        }

        else

        {



            //  xhat(k) = xhat'(k)

            //  P(k) = P'(k)

            memcpy(kf->xhat_data, kf->xhatminus_data, sizeof_float * kf->xhatSize);

            memcpy(kf->P_data, kf->Pminus_data, sizeof_float * kf->xhatSize * kf->xhatSize);

            kf->SkipEq5 = TRUE;

            return;

        }

    }

    else

    {



        if (g_ekf.ChiSquare_Data[0] > 0.1f * g_ekf.ChiSquareTestThreshold && g_ekf.ConvergeFlag)

        {

            g_ekf.AdaptiveGainScale = (g_ekf.ChiSquareTestThreshold - g_ekf.ChiSquare_Data[0]) / (0.9f * g_ekf.ChiSquareTestThreshold);

        }

        else

        {

            g_ekf.AdaptiveGainScale = 1;

        }

        g_ekf.ErrorCount = 0;

        kf->SkipEq5 = FALSE;

    }





    kf->temp_matrix.numRows = kf->Pminus.numRows;

    kf->temp_matrix.numCols = kf->HT.numCols;

    kf->MatStatus = mat_mul(&kf->Pminus, &kf->HT, &kf->temp_matrix);  // Mat状态

    kf->MatStatus = mat_mul(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);





    for (uint8_t i = 0; i < kf->K.numRows * kf->K.numCols; ++i)

    {

        kf->K_data[i] *= g_ekf.AdaptiveGainScale;

    }

    for (uint8_t i = 4; i < 6; ++i)

    {

        for (uint8_t j = 0; j < 3; ++j)

        {

            kf->K_data[i * 3 + j] *= g_ekf.OrientationCosine[i - 4] / 1.5707963f; // 1 rad

        }

    }



    kf->temp_vector.numRows = kf->K.numRows;

    kf->temp_vector.numCols = 1;

    kf->MatStatus = mat_mul(&kf->K, &kf->temp_vector1, &kf->temp_vector);  // Mat状态





    if (g_ekf.ConvergeFlag)

    {

        for (uint8_t i = 4; i < 6; ++i)

        {

            if (kf->temp_vector.pData[i] > 1e-2f * g_ekf.dt)

            {

                kf->temp_vector.pData[i] = 1e-2f * g_ekf.dt;

            }

            if (kf->temp_vector.pData[i] < -1e-2f * g_ekf.dt)

            {

                kf->temp_vector.pData[i] = -1e-2f * g_ekf.dt;

            }

        }

    }





//    kf->temp_vector.pData[3] = 0;

    kf->MatStatus = mat_add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);

}



/* 保存 P/K/H 供调试 */
static void ekf_observe(KalmanFilter_t *kf)

{

    memcpy(ekf_P_mat, kf->P_data, sizeof(ekf_P_mat));

    memcpy(ekf_K_mat, kf->K_data, sizeof(ekf_K_mat));

    memcpy(ekf_H_mat, kf->H_data, sizeof(ekf_H_mat));

}



/* 快速平方根倒数 */
static float invSqrt(float x)

{

    float halfx = 0.5f * x;

    float y = x;

    long i = *(long *)&y;

    i = 0x5f375a86 - (i >> 1);

    y = *(float *)&i;

    y = y * (1.5f - (halfx * y * y));

    return y;

}



/* 由安装角生成坐标变换矩阵 */
void imu_frame_init(imu_frame_t *imu_frame)

{

    float arz, ary, arx;





	arz = imu_frame->arz * (double)0.017453;

	ary = imu_frame->ary * (double)0.017453;

	arx = imu_frame->arx * (double)0.017453;





	imu_frame->trans[0] = arm_cos_f32(arz)*arm_cos_f32(ary);

	imu_frame->trans[1] = arm_cos_f32(arz)*arm_sin_f32(ary)*arm_sin_f32(arx) - arm_sin_f32(arz)*arm_cos_f32(arx);

	imu_frame->trans[2] = arm_cos_f32(arz)*arm_sin_f32(ary)*arm_cos_f32(arx) + arm_sin_f32(arz)*arm_sin_f32(arx);

	imu_frame->trans[3] = arm_sin_f32(arz)*arm_cos_f32(ary);

	imu_frame->trans[4] = arm_sin_f32(arz)*arm_sin_f32(ary)*arm_sin_f32(arx) + arm_cos_f32(arz)*arm_cos_f32(arx);

	imu_frame->trans[5] = arm_sin_f32(arz)*arm_sin_f32(ary)*arm_cos_f32(arx) - arm_cos_f32(arz)*arm_sin_f32(arx);

	imu_frame->trans[6] = -arm_sin_f32(ary);

	imu_frame->trans[7] = arm_cos_f32(ary)*arm_sin_f32(arx);

	imu_frame->trans[8] = arm_cos_f32(ary)*arm_cos_f32(arx);

	



	arm_mat_init_f32(&ekf_trans, 3, 3, (float *)imu_frame->trans); 

}



/* 传感器系转云台系 */
void imu_frame_rotate(float gx, float gy, float gz,\

	                  float ax, float ay, float az,\

	                  float *ggx, float *ggy, float *ggz,\

					  float *aax, float *aay, float *aaz)

{



    float gyro_in[3], gyro_out[3];



    float acc_in[3], acc_out[3];





	gyro_in[0] = (float)gx, gyro_in[1] = (float)gy, gyro_in[2] = (float)gz;



	acc_in[0] = (float)ax, acc_in[1] = (float)ay, acc_in[2] = (float)az;

	



	arm_mat_init_f32(&ekf_src, 1, 3, gyro_in);

	arm_mat_init_f32(&ekf_dst, 1, 3, gyro_out);

	arm_mat_mult_f32(&ekf_src, &ekf_trans, &ekf_dst);

	*ggx = gyro_out[0], *ggy = gyro_out[1], *ggz = gyro_out[2];

	



	arm_mat_init_f32(&ekf_src, 1, 3, acc_in);

	arm_mat_init_f32(&ekf_dst, 1, 3, acc_out);

	arm_mat_mult_f32(&ekf_src, &ekf_trans, &ekf_dst);

	*aax = acc_out[0], *aay = acc_out[1], *aaz = acc_out[2];

}



/* 机体系加速度转世界系 */
void imu_world_accel(float pitch, float roll, float yaw,\

						  float ax, float ay, float az,\

						  float *accx, float *accy, float *accz)

{

	float imu_accx, imu_accy, imu_accz;

	



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

#endif




