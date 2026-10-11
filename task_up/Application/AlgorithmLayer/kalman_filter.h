/* kalman_filter.h - 卡尔曼滤波器 */

#ifndef __KALMAN_FILTER_H
#define __KALMAN_FILTER_H

// CMSIS单精度矩阵接口
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
    float *FilteredValue; // 滤波状态，单位随状态分量
    float *MeasuredVector; // 输入量测，单位随量测分量
    float *ControlVector; // 控制输入，单位随控制分量

    uint8_t xhatSize; // 状态维数，1~255
    uint8_t uSize; // 控制维数，0~255
    uint8_t zSize; // 量测维数，1~255

    uint8_t UseAutoAdjustment; // 自动筛选有效量测，0/1
    uint8_t MeasurementValidNum; // 有效量测数，0~zSize

    uint8_t *MeasurementMap; // 量测映射状态编号，1起始
    float *MeasurementDegree; // 量测系数，量测单位/状态单位
    float *MatR_DiagonalElements; // 量测噪声方差，量测单位²
    float *StateMinVariance; // 状态方差下限，状态单位²
    uint8_t *temp; // 有效量测索引缓存，字节

    // 用户自定义函数, 以标志位判断是否跳过标准 KF 的某一步
    uint8_t SkipEq1, SkipEq2, SkipEq3, SkipEq4, SkipEq5; // 跳过五个标准步骤，各0/1

    // 矩阵结构: 行列数与数据指针
    mat xhat; // 后验状态，单位随状态分量
    mat xhatminus; // 预测状态，单位随状态分量
    mat u; // 控制向量，单位随控制分量
    mat z; // 量测向量，单位随量测分量
    mat P; // 后验协方差，状态单位乘积
    mat Pminus; // 预测协方差，状态单位乘积
    mat F, FT; // 状态转移及转置，状态单位比
    mat B; // 控制矩阵，状态单位/控制单位
    mat H, HT; // 量测及转置，量测单位/状态单位
    mat Q; // 过程噪声，状态单位乘积
    mat R; // 量测噪声，量测单位乘积
    mat K; // 卡尔曼增益，状态单位/量测单位
    mat S, temp_matrix, temp_matrix1, temp_vector, temp_vector1; // 新息与暂存，量纲随运算

    int8_t MatStatus; // 最近矩阵运算结果，见CMSIS

    // 用户自定义函数, 可替换标准 KF 的对应步骤
    void (*User_Func0_f)(struct kf_t *kf); // 量测整理后钩子，参数为状态
    void (*User_Func1_f)(struct kf_t *kf); // 状态预测后钩子，参数为状态
    void (*User_Func2_f)(struct kf_t *kf); // 协方差预测后钩子，参数为状态
    void (*User_Func3_f)(struct kf_t *kf); // 增益计算后钩子，参数为状态
    void (*User_Func4_f)(struct kf_t *kf); // 状态校正后钩子，参数为状态
    void (*User_Func5_f)(struct kf_t *kf); // 协方差更新后钩子，参数为状态
    void (*User_Func6_f)(struct kf_t *kf); // 输出同步后钩子，参数为状态

    // 矩阵存储空间指针
    float *xhat_data, *xhatminus_data; // 后验与预测状态存储，float
    float *u_data; // 控制向量存储，float
    float *z_data; // 量测向量存储，float
    float *P_data, *Pminus_data; // 后验与预测协方差存储，float
    float *F_data, *FT_data; // 状态转移及转置存储，float
    float *B_data; // 控制矩阵存储，float
    float *H_data, *HT_data; // 量测及转置存储，float
    float *Q_data; // 过程噪声存储，float
    float *R_data; // 量测噪声存储，float
    float *K_data; // 卡尔曼增益存储，float
    float *S_data, *temp_matrix_data, *temp_matrix_data1, *temp_vector_data, *temp_vector_data1; // 新息与各暂存区，float
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

