/* kalman_filter.h - 卡尔曼滤波器 */

#ifndef __KALMAN_FILTER_H
#define __KALMAN_FILTER_H

// cortex-m4 DSP lib
/*
#define __CC_ARM    // Keil
#define ARM_MATH_CM4
#define ARM_MATH_MATRIX_CHECK
#define ARM_MATH_ROUNDING
#define ARM_MATH_DSP    // define in arm_math.h
*/

#include "stm32h7xx_hal.h"
#include "arm_math.h"
//#include "dsp/matrix_functions.h"
#include "math.h"
#include "cmsis_os.h"


#define mat arm_matrix_instance_f32
#define mat_init arm_mat_init_f32
#define mat_add arm_mat_add_f32
#define mat_sub arm_mat_sub_f32
#define mat_mul arm_mat_mult_f32
#define mat_trans arm_mat_trans_f32
#define mat_inv arm_mat_inverse_f32

#define sizeof_float sizeof(float)
#define sizeof_double sizeof(double)

typedef struct kf_t
{
    float *FilteredValue;
    float *MeasuredVector;
    float *ControlVector;

    uint8_t xhatSize;
    uint8_t uSize;
    uint8_t zSize;

    uint8_t UseAutoAdjustment;
    uint8_t MeasurementValidNum;

    uint8_t *MeasurementMap;  // Measurement地图
    float *MeasurementDegree;
    float *MatR_DiagonalElements;
    float *StateMinVariance;  // State最小Variance
    uint8_t *temp;


    uint8_t SkipEq1, SkipEq2, SkipEq3, SkipEq4, SkipEq5;


    mat xhat;
    mat xhatminus;
    mat u;
    mat z;  // z
    mat P;
    mat Pminus;
    mat F, FT;
    mat B;  // 蓝
    mat H, HT;
    mat Q;
    mat R;  // 半径
    mat K;
    mat S, temp_matrix, temp_matrix1, temp_vector, temp_vector1;

    int8_t MatStatus;


    void (*User_Func0_f)(struct kf_t *kf);
    void (*User_Func1_f)(struct kf_t *kf);
    void (*User_Func2_f)(struct kf_t *kf);
    void (*User_Func3_f)(struct kf_t *kf);
    void (*User_Func4_f)(struct kf_t *kf);
    void (*User_Func5_f)(struct kf_t *kf);
    void (*User_Func6_f)(struct kf_t *kf);


    float *xhat_data, *xhatminus_data;
    float *u_data;
    float *z_data;
    float *P_data, *Pminus_data;
    float *F_data, *FT_data;
    float *B_data;
    float *H_data, *HT_data;
    float *Q_data;
    float *R_data;
    float *K_data;
    float *S_data, *temp_matrix_data, *temp_matrix_data1, *temp_vector_data, *temp_vector_data1;
} KalmanFilter_t;


void kf_init(KalmanFilter_t *kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize);
void kf_measure(KalmanFilter_t *kf);
void kf_predict_state(KalmanFilter_t *kf);
void kf_predict_cov(KalmanFilter_t *kf);
void kf_calc_gain(KalmanFilter_t *kf);
void kf_update_state(KalmanFilter_t *kf);
void kf_update_cov(KalmanFilter_t *kf);
float *kf_update(KalmanFilter_t *kf);

#endif //__KALMAN_FILTER_H

