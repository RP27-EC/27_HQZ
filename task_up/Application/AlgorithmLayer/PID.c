/* PID.c - 单环 PID */

#include "pid.h"
#include "rp_math.h"
#include <math.h>

/* 单环 PID 计算，调用前需刷新 err */
void single_pid_ctrl(pid_ctrl_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    // 误差死区
    if (pid->deadband > 0.0f && fabsf(pid->err) < pid->deadband) /* 误差死区 */
    {
        pid->err = 0.0f;
    }

    // (P)
    pid->pout = pid->kp * pid->err; /* 比例项 */

    // 抗积分饱和 
    uint8_t anti_windup = 0; /* 1 = 停止积分 */
    if ((pid->out >= pid->out_max && pid->err > 0.0f) ||
        (pid->out <= -pid->out_max && pid->err < 0.0f))
    {
        anti_windup = 1;
    }

    if (!anti_windup)
    {
        pid->integral += pid->err; /* 累加误差 */
        pid->integral = constrain(pid->integral, -pid->integral_max, pid->integral_max); /* 积分限幅 */
    }
    pid->iout = pid->ki * pid->integral; /* 积分项 */

    //  一阶低通滤波 
    float raw_dout = pid->kd * (pid->err - pid->last_err); /* 微分项 */
    pid->last_dout = pid->dout;
    if (pid->d_filter_alpha > 0.0f && pid->d_filter_alpha < 1.0f)
    {
        pid->dout = pid->d_filter_alpha * pid->last_dout + (1.0f - pid->d_filter_alpha) * raw_dout;
    }
    else
    {
        pid->dout = raw_dout;
    }

    // 总输出
    pid->out = pid->pout + pid->iout + pid->dout; /* 总输出 */
    pid->out = constrain(pid->out, -pid->out_max, pid->out_max); /* 输出限幅 */

    // 状态更新
    pid->last_err = pid->err; /* 保存微分历史 */
}

/* 清除积分和全部历史输出 */
void integral_to_zero(pid_ctrl_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->integral = 0.0f; /* 清积分 */
    pid->iout = 0.0f;
    pid->dout = 0.0f;
    pid->last_dout = 0.0f;
    pid->last_err = 0.0f;
    pid->out = 0.0f;
}

/* 目标减反馈刷新误差 */
void pid_err_cal(pid_ctrl_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->err = pid->target - pid->measure; /* 误差刷新 */
}

/* 串级 PID：外环角度/位置，内环速度 */
float all_pid_calc(pid_ctrl_t *out, pid_ctrl_t *inn, float target, float mea_out, float mea_in, float rate_feedforward, float inner_scale, uint8_t err_cal_mode)
{
    if (inn == NULL)
    {
        return 0.0f;
    }

    if (out == NULL)
    {
        inn->target = target + rate_feedforward; /* 内环目标 */
        inn->measure = mea_in * inner_scale;
        inn->err = inn->target - inn->measure;
        single_pid_ctrl(inn);
        return inn->out;
    }

    // 外环位置计算
    out->target = target; /* 外环目标 */
    out->measure = mea_out; /* 外环反馈 */
    out->err = out->target - out->measure; /* 外环误差 */

    switch (err_cal_mode)
    {
    case 1: 
        out->err = motor_half_cycle(out->err, 8192.0f);
        break;
    case 2: 
        out->err = motor_half_cycle(out->err, 4096.0f);
        break;
    case 3: 
        out->err = motor_half_cycle(out->err, 360.0f);
        break;
    case 4:
        out->err = motor_half_cycle(out->err, 65536.0f);
        break;
    case 5: 
        out->err = motor_half_cycle(out->err, 191.0f);
        break;
    case 0: 
    default:
        break;
    }

    single_pid_ctrl(out);

    //内环速度计算
    inn->target = out->out + rate_feedforward; /* 内环目标 */
    inn->measure = mea_in * inner_scale;       /* 内环反馈 */
    inn->err = inn->target - inn->measure;     /* 内环误差 */
    single_pid_ctrl(inn);

    return inn->out;
}


