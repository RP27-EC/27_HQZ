/* Filter.c - 滤波参数 */

#include "Filter.h"

KalmanFilter_t vaEstimateKF;

float vaEstimateKF_F[4] = {1.0f, 0.001f, 
                           0.0f, 1.0f};

float vaEstimateKF_P[4] = {1.0f, 0.0f,
                           0.0f, 1.0f};

float vaEstimateKF_Q[4] = {0.1f, 0.0f, 
                           0.0f, 0.1f};

float vaEstimateKF_R[4] = {200.0f, 0.0f, 
                            0.0f,  200.0f}; 	
														
float vaEstimateKF_K[4];
													 
const float vaEstimateKF_H[4] = {1.0f, 0.0f,
                                 0.0f, 1.0f};

																 
void xvEstimateKF_Init(KalmanFilter_t *EstimateKF)
{
    kf_init(EstimateKF, 2, 0, 2);
	
		memcpy(EstimateKF->F_data, vaEstimateKF_F, sizeof(vaEstimateKF_F));
    memcpy(EstimateKF->P_data, vaEstimateKF_P, sizeof(vaEstimateKF_P));
    memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
    memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
    memcpy(EstimateKF->H_data, vaEstimateKF_H, sizeof(vaEstimateKF_H));

}

void xvEstimateKF_Update(KalmanFilter_t *EstimateKF ,float acc,float vel)
{   	

    EstimateKF->MeasuredVector[0] =	vel;
    EstimateKF->MeasuredVector[1] = acc;
    		

    kf_update(EstimateKF);

}


KalmanFilter_t XEstimateKF;

float XEstimateKF_F[4] = {1.0f, 0.001f, 
                           0.0f, 1.0f};

float XEstimateKF_P[4] = {1.0f, 0.0f,
                           0.0f, 1.0f};

float XEstimateKF_Q[4] = {0.1f, 0.0f, 
                           0.0f, 0.1f};

float XEstimateKF_R[4] = {200.0f, 0.0f, 
                            0.0f,  100.0f}; 	
														
float XEstimateKF_K[4];
													 
const float XEstimateKF_H[4] = {1.0f, 0.0f,
                                 0.0f, 1.0f};

																 
void XEstimateKF_Init(KalmanFilter_t *EstimateKF)
{
	 kf_init(EstimateKF, 2, 0, 2);
	
		memcpy(EstimateKF->F_data, XEstimateKF_F, sizeof(XEstimateKF_F));
    memcpy(EstimateKF->P_data, XEstimateKF_P, sizeof(XEstimateKF_P));
    memcpy(EstimateKF->H_data, XEstimateKF_H, sizeof(XEstimateKF_H));
		memcpy(EstimateKF->K_data, XEstimateKF_K, sizeof(XEstimateKF_K));
}

void XEstimateKF_Clear(KalmanFilter_t *EstimateKF)
{
		memcpy(EstimateKF->F_data, XEstimateKF_F, sizeof(XEstimateKF_F));
    memcpy(EstimateKF->P_data, XEstimateKF_P, sizeof(XEstimateKF_P));
    memcpy(EstimateKF->Q_data, XEstimateKF_Q, sizeof(XEstimateKF_Q));
    memcpy(EstimateKF->R_data, XEstimateKF_R, sizeof(XEstimateKF_R));
    memcpy(EstimateKF->H_data, XEstimateKF_H, sizeof(XEstimateKF_H));
		memset(EstimateKF->Pminus_data, 0, sizeof_float * 4);
}

void XEstimateKF_Update(KalmanFilter_t *EstimateKF ,float vel,float s)
{   	

    EstimateKF->MeasuredVector[0] =	s;
    EstimateKF->MeasuredVector[1] = vel;
    		

    kf_update(EstimateKF);

}




