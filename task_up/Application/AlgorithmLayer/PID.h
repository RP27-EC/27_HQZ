#ifndef PID_H
#define PID_H

#include <stdint.h>

/**
 * @brief PID 
 */
typedef struct
{
    float kp;
    float ki;
    float kd;

    float integral_max;      // 积分项限幅
    float out_max;           // 总输出限幅
    float deadband;          // 误差死区

    float d_filter_alpha;    // 低通滤波系数

    float target;
    float measure;
    float err;
    float last_err;
    float integral;

    /* 各项输出分量 */
    float pout;
    float iout;
    float dout;
    float last_dout;
    float out;               // 总输出
} pid_ctrl_t;

void single_pid_ctrl(pid_ctrl_t *pid);
void integral_to_zero(pid_ctrl_t *pid);
void pid_err_cal(pid_ctrl_t *pid);
float all_pid_calc(pid_ctrl_t *out, pid_ctrl_t *inn, float target, float mea_out, float mea_in, float rate_feedforward, float inner_scale, uint8_t err_cal_mode);

#endif /* PID_H */
