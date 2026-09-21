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

#include "stm32f407xx.h"
#include "arm_math.h"
//#include "dsp/matrix_functions.h"
#include "math.h"
#include "cmsis_os.h"

// 定点 q31 运算会降低精度, 这里用 f32
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

    uint8_t *MeasurementMap;  // 量测与状态的对应关系
    float *MeasurementDegree;  // 量测在 H 中的系数
    float *MatR_DiagonalElements;  // 各量测的方差
    float *StateMinVariance;  // 状态最小方差, 防止过度收敛
    uint8_t *temp;

    // 用户自定义函数, 以标志位判断是否跳过标准 KF 的某一步
    uint8_t SkipEq1, SkipEq2, SkipEq3, SkipEq4, SkipEq5;

    // 矩阵结构: 行列数与数据指针
    mat xhat;  // x(k|k) 状态估计
    mat xhatminus;  // x(k|k-1) 状态预测
    mat u;         // 控制向量 u
    mat z;         // 量测向量 z
    mat P;  // P(k|k) 协方差
    mat Pminus;  // P(k|k-1) 协方差预测
    mat F, FT;  // 状态转移矩阵 F 及其转置
    mat B;  // 控制矩阵 B
    mat H, HT;  // 量测矩阵 H 及其转置
    mat Q;  // 过程噪声协方差 Q
    mat R;  // 量测噪声协方差 R
    mat K;  // 卡尔曼增益 K
    mat S, temp_matrix, temp_matrix1, temp_vector, temp_vector1;

    int8_t MatStatus;

    // 用户自定义函数, 可替换标准 KF 的对应步骤
    void (*User_Func0_f)(struct kf_t *kf);
    void (*User_Func1_f)(struct kf_t *kf);
    void (*User_Func2_f)(struct kf_t *kf);
    void (*User_Func3_f)(struct kf_t *kf);
    void (*User_Func4_f)(struct kf_t *kf);
    void (*User_Func5_f)(struct kf_t *kf);
    void (*User_Func6_f)(struct kf_t *kf);

    // 矩阵存储空间指针
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

