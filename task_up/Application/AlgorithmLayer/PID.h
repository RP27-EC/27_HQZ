/* PID.h - 单环 PID */

#ifndef PID_H
#define PID_H

#include <stdint.h>

/* 周期累加与时间积分共用状态 */
typedef struct
{
    float kp; // 比例增益，输出/输入单位
    float ki; // 积分增益，量纲随算法
    float kd; // 微分增益，量纲随算法

    float integral_max; // 积分状态限幅，量纲随算法
    float out_max; // 输出幅值上限，≥0
    float deadband; // 误差死区，与目标同单位

    float d_filter_alpha; // 微分历史权重，0~1

    float target; // 目标，单位随控制对象
    float measure; // 反馈，与目标同单位
    float err; // 目标减反馈，与目标同单位
    float last_err; // 上周期误差，与目标同单位
    float integral; // 误差积分，量纲随算法

    /* 各项输出分量 */
    float pout; // 比例分量，与输出同单位
    float iout; // 积分分量，与输出同单位
    float dout; // 微分分量，与输出同单位
    float last_dout; // 微分历史，与输出同单位
    float out; // 限幅后输出，±out_max
} pid_ctrl_t;

void single_pid_ctrl(pid_ctrl_t *pid);
void integral_to_zero(pid_ctrl_t *pid);
void pid_err_cal(pid_ctrl_t *pid);
float all_pid_calc(pid_ctrl_t *out, pid_ctrl_t *inn, float target, float mea_out, float mea_in, float rate_feedforward, float inner_scale, uint8_t err_cal_mode);

#endif /* PID_H */

