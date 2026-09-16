#include "pid.h"
#include "rp_math.h"
#include <math.h>

/**
 * @brief 单环 PID 
 * @param pid PID 
 */
void single_pid_ctrl(pid_ctrl_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    // 误差死区
    if (pid->deadband > 0.0f && fabsf(pid->err) < pid->deadband)
    {
        pid->err = 0.0f;
    }

    // (P)
    pid->pout = pid->kp * pid->err;

    // 抗积分饱和 
    uint8_t anti_windup = 0;
    if ((pid->out >= pid->out_max && pid->err > 0.0f) ||
        (pid->out <= -pid->out_max && pid->err < 0.0f))
    {
        anti_windup = 1;
    }

    if (!anti_windup)
    {
        pid->integral += pid->err;
        pid->integral = constrain(pid->integral, -pid->integral_max, pid->integral_max);
    }
    pid->iout = pid->ki * pid->integral;

    //  一阶低通滤波 
    float raw_dout = pid->kd * (pid->err - pid->last_err);
    pid->last_dout = pid->dout;
    if (pid->d_filter_alpha > 0.0f && pid->d_filter_alpha < 1.0f)
    {
        pid->dout = pid->d_filter_alpha * pid->last_dout + (1.0f - pid->d_filter_alpha) * raw_dout;
    }
    else
    {
        pid->dout = raw_dout;
    }

    // 总输出  （外部注入前馈量）
    pid->out = pid->pout + pid->iout + pid->dout + pid->feedforward;
    pid->out = constrain(pid->out, -pid->out_max, pid->out_max);

    // 6. 状态更新
    pid->last_err = pid->err;
}

/**
 * @brief 积分器与状态清零
 * @param pid PID 
 */
void integral_to_zero(pid_ctrl_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->integral = 0.0f;
    pid->iout = 0.0f;
    pid->dout = 0.0f;
    pid->last_dout = 0.0f;
    pid->last_err = 0.0f;
    pid->feedforward = 0.0f;
    pid->out = 0.0f;
}

/**
 * @brief 误差计算
 * @param pid PID
 */
void pid_err_cal(pid_ctrl_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->err = pid->target - pid->measure;
}

/**
 * @brief  PID 计算
 * @param out           外环 PID 指针
 * @param inn           内环 PID 指针
 * @param target        控制目标值
 * @param mea_out       外环观测反馈值
 * @param mea_in        内环观测反馈值
 * @param inner_scale   内环极性确定
 * @param err_cal_mode  外环就近转位
 * @return float        内环最终输出量
 */
float all_pid_calc(pid_ctrl_t *out, pid_ctrl_t *inn, float target, float mea_out, float mea_in, float inner_scale, uint8_t err_cal_mode)
{
    if (inn == NULL)
    {
        return 0.0f;
    }

    if (out == NULL)
    {
        inn->target = target;
        inn->measure = mea_in * inner_scale;
        inn->err = inn->target - inn->measure;
        single_pid_ctrl(inn);
        return inn->out;
    }

    // 外环位置计算
    out->target = target;
    out->measure = mea_out;
    out->err = out->target - out->measure;

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
    inn->target = out->out;
    inn->measure = mea_in * inner_scale;
    inn->err = inn->target - inn->measure;
    single_pid_ctrl(inn);

    return inn->out;
}