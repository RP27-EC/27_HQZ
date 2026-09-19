#include "gimbal.h"
#include "imu_sensor.h"
#include "rc_sensor.h"
#include "board_remote_config.h"
#include "rp_math.h"
#include <math.h> 

gimbal_t Gimbal;

//重力补偿现在为一个比较小的数值，还未调试数值
//重力补偿较小目前pitch可能会下垂，pitch使用速控
//feedback后续会在遥控器声明，作为速度前馈加入内环目标值，目前还未加入
//阻力补偿先留着，后续需要再加,下方有伪代码
//机械中值也需确定，把车拨到中间，看编码器反馈作为中值
//自瞄暂时不需要，先删去
//有一些地方加了斜坡，如果响应过慢可以适当调整数值
//小陀螺模式的角速度前馈还没写，后续与底盘部分一起写
//键鼠和遥控器部分复制的源码，还未调试，这部分并非正确
//陀螺仪部分需去拓展下协议，目前协议还是照搬的源码，目前我完成的部分是云台，其它的后续再说
//键鼠的映射记得改，目前也是简单配置而已，数值也未标定
//死区和滤波的值也未标定，需测试再标定


//绝对值函数，工具函数
static float gimbal_abs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

//限幅函数，工具函数
static float gimbal_clamp(float value, float min_value, float max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

//  步进斜坡平滑函数
static float gimbal_ramp(float current, float target, float step)
{
    if (step <= 0.0f) return target;
    if (target - current > step) return current + step;
    if (target - current < -step) return current - step;
    return target;
}

static float gimbal_wrap_deg(float angle);

//就近转位的实现
static float gimbal_ramp_wrapped(float current, float target, float step)
{
    float error;

    if (step <= 0.0f) return gimbal_wrap_deg(target);

    error = gimbal_wrap_deg(target - current);
    if (error > step)
    {
        error = step;
    }
    else if (error < -step)
    {
        error = -step;
    }

    return gimbal_wrap_deg(current + error);
}

/*归一化到 [-180, 180) */
static float gimbal_wrap_deg(float angle)
{
    while (angle >= 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}


/* 遥控器通道转换为目标角速度 */
#if GIMBAL_LOCAL_RC_ENABLE
static float gimbal_axis_to_rate(int16_t axis, float max_rate)
{
    float value = (float)axis;

    if (gimbal_abs(value) <= GIMBAL_RC_AXIS_DEADBAND)
    {
        return 0.0f;
    }

    value -= (value > 0.0f) ? GIMBAL_RC_AXIS_DEADBAND : -GIMBAL_RC_AXIS_DEADBAND;
    value /= (GIMBAL_RC_AXIS_MAX - GIMBAL_RC_AXIS_DEADBAND);
    value = gimbal_clamp(value, -1.0f, 1.0f);

    return value * max_rate;
}
#endif

/* 更新遥控器和键鼠角速度前馈 不太清楚要用遥控还是键鼠，索性两个一起写了 */
static void gimbal_manual_input_update(gimbal_t *gimbal)
{
    float yaw_rate = 0.0f;
    float pitch_rate = 0.0f;

#if GIMBAL_DOWN_RC_ENABLE
    if ((Board_HeartBeat.offline_cnt_5 < Board_HeartBeat.offline_cnt_max) &&
        (Board_Rx_Info.remote_cmd_pkt.valid != 0u) &&
        (Board_Rx_Info.state_pkt.car_state != 0u))
    {
        yaw_rate = gimbal_clamp(Board_Rx_Info.remote_cmd_pkt.yaw_rate_deg_s,
                                -GIMBAL_MANUAL_YAW_RATE_DEG_S,
                                GIMBAL_MANUAL_YAW_RATE_DEG_S);
        pitch_rate = gimbal_clamp(Board_Rx_Info.remote_cmd_pkt.pitch_rate_deg_s,
                                  -GIMBAL_MANUAL_PITCH_RATE_DEG_S,
                                  GIMBAL_MANUAL_PITCH_RATE_DEG_S);
    }
#elif GIMBAL_LOCAL_RC_ENABLE
    if (rc_sensor.work_state == DEV_ONLINE)
    {
        if (Board_Rx_Info.state_pkt.car_state == 1u)
        {
            yaw_rate = gimbal_axis_to_rate(RC_RIGH_CH_LR_VALUE,
                                           GIMBAL_MANUAL_YAW_RATE_DEG_S);
            pitch_rate = gimbal_axis_to_rate(RC_RIGH_CH_UD_VALUE,
                                             GIMBAL_MANUAL_PITCH_RATE_DEG_S);
        }
        else if (Board_Rx_Info.state_pkt.car_state == 2u)
        {
            yaw_rate = gimbal_clamp(rc_sensor_info.mouse_x * GIMBAL_MOUSE_YAW_RATE_GAIN,
                                    -GIMBAL_MANUAL_YAW_RATE_DEG_S,
                                    GIMBAL_MANUAL_YAW_RATE_DEG_S);
            pitch_rate = gimbal_clamp(rc_sensor_info.mouse_y * GIMBAL_MOUSE_PITCH_RATE_GAIN,
                                      -GIMBAL_MANUAL_PITCH_RATE_DEG_S,
                                      GIMBAL_MANUAL_PITCH_RATE_DEG_S);
        }
    }
#else
    (void)gimbal;
#endif

    gimbal->feedforward.yaw_rate_cmd_deg_s = yaw_rate;
    gimbal->feedforward.pitch_rate_cmd_deg_s = pitch_rate;
}

/* 重置单个 PID 历史状态 */
static void gimbal_pid_clear(pid_ctrl_t *pid)
{
    integral_to_zero(pid);
    pid->target = 0.0f;
    pid->measure = 0.0f;
    pid->err = 0.0f;
    pid->pout = 0.0f;
    pid->iout = 0.0f;
    pid->dout = 0.0f;
    pid->last_dout = 0.0f;
}

//清空所有 PID 历史状态
static void gimbal_clear_all_pid(gimbal_t *gimbal)
{
    gimbal_pid_clear(&gimbal->pid_info.yaw_gyro_outer);
    gimbal_pid_clear(&gimbal->pid_info.yaw_gyro_inner);
    gimbal_pid_clear(&gimbal->pid_info.yaw_mec_outer);
    gimbal_pid_clear(&gimbal->pid_info.yaw_mec_inner);
    gimbal_pid_clear(&gimbal->pid_info.pitch_gyro_outer);
    gimbal_pid_clear(&gimbal->pid_info.pitch_gyro_inner);
    gimbal_pid_clear(&gimbal->pid_info.pitch_mec_outer);
    gimbal_pid_clear(&gimbal->pid_info.pitch_mec_inner);
}

/* 参数默认值 */
static void gimbal_pid_init(gimbal_t *gimbal)
{
    pid_ctrl_t *pid;

    /* Yaw 陀螺仪串级 */
    pid = &gimbal->pid_info.yaw_gyro_outer;//  外环
    pid->kp = 20.0f; pid->ki = 0.05f; pid->kd = 0.0f;
    pid->integral_max = 200.0f; pid->out_max = 500.0f;

    pid = &gimbal->pid_info.yaw_gyro_inner;//  内环
    pid->kp = 0.04f; pid->ki = 0.0f; pid->kd = 0.0f;
    pid->integral_max = 0.0f; pid->out_max = 10.0f;

    /* Pitch 陀螺仪串级 */
    pid = &gimbal->pid_info.pitch_gyro_outer;//  外环
    pid->kp = 56.0f; pid->ki = 0.0f; pid->kd = 0.0f;
    pid->integral_max = 0.0f; pid->out_max = 100.0f;

    pid = &gimbal->pid_info.pitch_gyro_inner;//  内环
    pid->kp = 0.03f; pid->ki = 0.0f; pid->kd = 0.0f;
    pid->integral_max = 0.0f; pid->out_max = 10.0f;

    /* Yaw 机械编码器串级 */
    pid = &gimbal->pid_info.yaw_mec_outer;//下面的也是同理
    pid->kp = 20.0f; pid->ki = 0.0f; pid->kd = 0.0f;
    pid->integral_max = 0.0f; pid->out_max = 500.0f;

    pid = &gimbal->pid_info.yaw_mec_inner;
    pid->kp = 0.1f; pid->ki = 0.0f; pid->kd = 0.0f;
    pid->integral_max = 0.0f; pid->out_max = 10.0f;

    /* Pitch 机械编码器串级 */
    pid = &gimbal->pid_info.pitch_mec_outer;
    pid->kp = 1.6f; pid->ki = 0.0f; pid->kd = 0.0f;
    pid->integral_max = 500.0f; pid->out_max = 10.0f;

    pid = &gimbal->pid_info.pitch_mec_inner;
    pid->kp = 1.2f; pid->ki = 0.0f; pid->kd = 0.0f;
    pid->integral_max = 0.0f; pid->out_max = 10.0f;

    gimbal_clear_all_pid(gimbal);
}

/* 传感器数据同步与坐标偏置换算 */
static void gimbal_info_update(gimbal_t *gimbal)
{
    imu_info_t *imu = imu_sensor.info;

    /* IMU 姿态与角速度 */
    //获取IMU的角度和角速度信息
    gimbal->base_info.yaw_imu_angle = imu->base_info.yaw;
    gimbal->base_info.yaw_imu_speed = imu->base_info.rate_yaw;
    gimbal->base_info.pitch_imu_angle = imu->base_info.pitch;
    gimbal->base_info.pitch_imu_speed = imu->base_info.rate_pitch;

    /* 电机编码器角度换算 */
    //获取电机的角度和速度信息，并进行换算
    gimbal->base_info.yaw_mec_angle = gimbal_wrap_deg(
        gimbal->yaw_motor->rx_info->motor_angle * GIMBAL_RAD_TO_DEG -
        GIMBAL_YAW_MIDDLE_DEG);
    gimbal->base_info.yaw_mec_speed = gimbal->yaw_motor->rx_info->speed;

    gimbal->base_info.pitch_mec_angle = gimbal_wrap_deg(
        gimbal->pitch_motor->rx_info->motor_angle * GIMBAL_RAD_TO_DEG -
        GIMBAL_PITCH_MIDDLE_DEG);
    gimbal->base_info.pitch_mec_speed = gimbal->pitch_motor->rx_info->speed;

    //获取上位机发送的目标角度信息，并进行换算
    gimbal->pid_info.yaw_mec_target_raw =
        Board_Rx_Info.gimbal_target_pkt.yaw_mec_tar * GIMBAL_RAD_TO_DEG;
    gimbal->pid_info.pitch_mec_target_raw =
        Board_Rx_Info.gimbal_target_pkt.pitch_mec_tar * GIMBAL_RAD_TO_DEG;
    gimbal->pid_info.yaw_imu_target_raw =
        Board_Rx_Info.gimbal_target_pkt.yaw_imu_tar;
    gimbal->pid_info.pitch_imu_target_raw =
        Board_Rx_Info.gimbal_target_pkt.pitch_imu_tar;

    gimbal_manual_input_update(gimbal);
}

/* 硬件与数据流 */
static uint8_t gimbal_safety_allows_control(gimbal_t *gimbal)
{
    (void)gimbal;
    /* Keep the same enable logic as the working reference project. */
    if (Board_HeartBeat.status != DEV_ONLINE) return 0;
    if (Board_Rx_Info.state_pkt.car_state == 0) return 0;

    return 1;
}

/* 控制模式仲裁 */
static gimbal_mode_e gimbal_select_mode(gimbal_t *gimbal)
{
    // 任意安全条件不满足，进入休眠模式
    if (!gimbal_safety_allows_control(gimbal))
    {
        gimbal->init_info.init_flag = 0;
        gimbal->init_info.init_time = 0;
        return G_SLEEP;
    }

    // 重新初始化
    if (gimbal->init_info.init_flag == 0)
    {
        return G_INIT;
    }

    // 通信包模式字分发 (0: 机械环, 其他: 陀螺仪自稳)
    if (Board_Rx_Info.state_pkt.gimbal_mode == 0)
    {
        return G_MEC;
    }

    return G_RATE;// 速控模式
}

/* 获取当前模式最终目标 */
static void gimbal_get_command_target(gimbal_t *gimbal, float *yaw_target, float *pitch_target)
{
    switch (gimbal->gimbal_mode)
    {
    case G_INIT:
    case G_MEC:
        *yaw_target = gimbal_wrap_deg(gimbal->pid_info.yaw_mec_target_raw);
        *pitch_target = gimbal->pid_info.pitch_mec_target_raw;
        break;

    case G_GYRO:
    case G_AUTO:
        *yaw_target = gimbal_wrap_deg(gimbal->pid_info.yaw_imu_target_raw);
        *pitch_target = gimbal->pid_info.pitch_imu_target_raw;
        break;

    case G_SLEEP:
    default:
        *yaw_target = gimbal->pid_info.yaw_target;
        *pitch_target = gimbal->pid_info.pitch_target;
        break;
    }
}

/* 对最终 Pitch 目标限位 */
static float gimbal_limit_pitch_target(gimbal_t *gimbal, float pitch_target)
{
    if ((gimbal->gimbal_mode == G_GYRO) ||
        (gimbal->gimbal_mode == G_AUTO))
    {
        float delta = pitch_target - gimbal->base_info.pitch_imu_angle;
        float predicted_mec_angle = gimbal->base_info.pitch_mec_angle + delta;

        if (predicted_mec_angle > GIMBAL_PITCH_MAX_DEG)
        {
            delta -= predicted_mec_angle - GIMBAL_PITCH_MAX_DEG;
        }
        else if (predicted_mec_angle < GIMBAL_PITCH_MIN_DEG)
        {
            delta += GIMBAL_PITCH_MIN_DEG - predicted_mec_angle;
        }

        return gimbal->base_info.pitch_imu_angle + delta;
    }

    return gimbal_clamp(pitch_target, GIMBAL_PITCH_MIN_DEG, GIMBAL_PITCH_MAX_DEG);
}

/* 归中到位判定与超时退出 */
static void gimbal_update_init(gimbal_t *gimbal)
{
    float yaw_error = gimbal_abs(gimbal_wrap_deg(
        gimbal->base_info.yaw_mec_angle - gimbal->pid_info.yaw_mec_target_raw));
    float pitch_error = gimbal_abs(
        gimbal->base_info.pitch_mec_angle - gimbal->pid_info.pitch_mec_target_raw);
    uint8_t position_ok = (yaw_error <= gimbal->init_info.yaw_angle_tolerance) &&
                          (pitch_error <= gimbal->init_info.pitch_angle_tolerance);
    uint8_t speed_ok = (gimbal_abs(gimbal->base_info.yaw_mec_speed) <=
                        gimbal->init_info.yaw_speed_tolerance) &&
                       (gimbal_abs(gimbal->base_info.pitch_mec_speed) <=
                        gimbal->init_info.pitch_speed_tolerance);
    uint8_t target_ok = (gimbal_abs(gimbal_wrap_deg(
                           gimbal->pid_info.yaw_target - gimbal->pid_info.yaw_mec_target_raw)) <=
                        GIMBAL_TARGET_ARRIVE_EPS_DEG) &&
                       (gimbal_abs(gimbal->pid_info.pitch_target -
                                    gimbal->pid_info.pitch_mec_target_raw) <=
                        GIMBAL_TARGET_ARRIVE_EPS_DEG);

    if (position_ok && speed_ok && target_ok)
    {
        if (gimbal->init_info.stable_time < gimbal->init_info.stable_time_max)
        {
            gimbal->init_info.stable_time++;
        }
    }
    else
    {
        gimbal->init_info.stable_time = 0;
    }

    if (gimbal->init_info.stable_time >= gimbal->init_info.stable_time_max)
    {
        gimbal->init_info.init_flag = 1;
        gimbal->init_info.init_time = 0;
        gimbal->init_info.stable_time = 0;
        gimbal->init_info.mode_transition_active = 0;
        return;
    }

    if (gimbal->init_info.init_time >= gimbal->init_info.init_time_max)
    {
        gimbal->init_info.init_flag = 1;
        gimbal->init_info.init_time = 0;
        gimbal->init_info.stable_time = 0;
        gimbal->init_info.mode_transition_active = 0;
        return;
    }

    gimbal->init_info.init_time++;
}

/* 目标值更新与限位 */
static void gimbal_update_targets(gimbal_t *gimbal)
{
    float yaw_final;
    float pitch_final;

    if (gimbal->gimbal_mode == G_SLEEP)
    {
        gimbal->pid_info.yaw_target = 0.0f;
        gimbal->pid_info.pitch_target = 0.0f;
        return;
    }

    if (gimbal->gimbal_mode == G_RATE)
    {
        gimbal->init_info.mode_transition_active = 0;
        return;
    }

    gimbal_get_command_target(gimbal, &yaw_final, &pitch_final);
    pitch_final = gimbal_limit_pitch_target(gimbal, pitch_final);

    if (gimbal->gimbal_mode == G_INIT)
    {
        gimbal->pid_info.yaw_target = gimbal_ramp_wrapped(
            gimbal->pid_info.yaw_target, yaw_final, gimbal->init_info.yaw_ramp_step);
        gimbal->pid_info.pitch_target = gimbal_ramp(
            gimbal->pid_info.pitch_target, pitch_final, gimbal->init_info.pitch_ramp_step);
    }
    else if (gimbal->init_info.mode_transition_active)
    {
        gimbal->pid_info.yaw_target = gimbal_ramp_wrapped(
            gimbal->pid_info.yaw_target, yaw_final, gimbal->init_info.mode_yaw_ramp_step);
        gimbal->pid_info.pitch_target = gimbal_ramp(
            gimbal->pid_info.pitch_target, pitch_final, gimbal->init_info.mode_pitch_ramp_step);

        if (gimbal_abs(gimbal_wrap_deg(yaw_final - gimbal->pid_info.yaw_target)) <=
                GIMBAL_TARGET_ARRIVE_EPS_DEG &&
            gimbal_abs(pitch_final - gimbal->pid_info.pitch_target) <=
                GIMBAL_TARGET_ARRIVE_EPS_DEG)
        {
            gimbal->init_info.mode_transition_active = 0;
        }
    }
    else
    {
        gimbal->pid_info.yaw_target = yaw_final;
        gimbal->pid_info.pitch_target = pitch_final;
    }
}

/* 更新速控模式的目标角速度 */
static void gimbal_update_rate_targets(gimbal_t *gimbal)
{
    if (gimbal_abs(gimbal->feedforward.yaw_rate_cmd_deg_s) <
        GIMBAL_RATE_HOLD_DEADBAND_DEG_S)
    {
        gimbal->feedforward.yaw_rate_target_deg_s =
            GIMBAL_RATE_HOLD_KP * gimbal_wrap_deg(
                gimbal->feedforward.yaw_hold_angle_deg -
                gimbal->base_info.yaw_imu_angle);
    }
    else
    {
        gimbal->feedforward.yaw_hold_angle_deg = gimbal->base_info.yaw_imu_angle;
        gimbal->feedforward.yaw_rate_target_deg_s =
            gimbal->feedforward.yaw_rate_cmd_deg_s;
    }

    if (gimbal_abs(gimbal->feedforward.pitch_rate_cmd_deg_s) <
        GIMBAL_RATE_HOLD_DEADBAND_DEG_S)
    {
        gimbal->feedforward.pitch_rate_target_deg_s =
            GIMBAL_RATE_HOLD_KP * (
                gimbal->feedforward.pitch_hold_angle_deg -
                gimbal->base_info.pitch_imu_angle);
    }
    else
    {
        gimbal->feedforward.pitch_hold_angle_deg = gimbal->base_info.pitch_imu_angle;
        gimbal->feedforward.pitch_rate_target_deg_s =
            gimbal->feedforward.pitch_rate_cmd_deg_s;
    }
}

/* Pitch 轴重力力矩补偿 */
//重力补偿，需调试数值
static float gimbal_gravity_compensation(gimbal_t *gimbal)
{
#if GIMBAL_GRAVITY_ENABLE
    float angle_rad =
        (gimbal->base_info.pitch_mec_angle - GIMBAL_GRAVITY_MIDDLE_DEG) *
        GIMBAL_DEG_TO_RAD;

    return GIMBAL_GRAVITY_K_NM * cosf(angle_rad) + GIMBAL_GRAVITY_B_NM;
#else
    (void)gimbal;
    return 0.0f;
#endif
}

//伪代码，yaw的阻力补偿
/*
 * Yaw axis drag compensation placeholder.
 *
 * tau = Coulomb * sign(omega) + Viscous * omega
 *
 * static float gimbal_yaw_drag_compensation(gimbal_t *gimbal)
 * {
 *     const float coulomb_nm = 0.05f;
 *     const float viscous = 0.001f;
 *     const float deadband_deg_s = 20.0f;
 *     const float output_limit_nm = 0.2f;
 *     float omega = gimbal->base_info.yaw_imu_speed;
 *     float torque;
 *
 *     if (gimbal_abs(omega) < deadband_deg_s)
 *     {
 *         return 0.0f;
 *     }
 *
 *     torque = (omega > 0.0f ? coulomb_nm : -coulomb_nm) + viscous * omega;
 *
 *     return gimbal_clamp(torque, -output_limit_nm, output_limit_nm);
 * }
 */

/*  PID 计算  力矩合成输出 */
static void gimbal_calc_output(gimbal_t *gimbal)
{
    float gravity = gimbal_gravity_compensation(gimbal);


    switch (gimbal->gimbal_mode)
    {
    case G_INIT:
        gimbal_update_init(gimbal);
        // 穿透至 G_MEC 闭环复用机械环运算
        /* fall through */
    case G_MEC:
        // Pitch 机械环
        gimbal->base_info.output_gimbal_p =
            all_pid_calc(&gimbal->pid_info.pitch_mec_outer,
                         &gimbal->pid_info.pitch_mec_inner,
                         gimbal->pid_info.pitch_target,
                         gimbal->base_info.pitch_mec_angle,
                         gimbal->base_info.pitch_mec_speed,
                         0.0f,
                         1.0f,
                         0) + gravity;

        // Yaw 机械环
        gimbal->base_info.output_gimbal_y =
            all_pid_calc(&gimbal->pid_info.yaw_mec_outer,
                         &gimbal->pid_info.yaw_mec_inner,
                         gimbal->pid_info.yaw_target,
                         gimbal->base_info.yaw_mec_angle,
                         gimbal->base_info.yaw_mec_speed,
                         0.0f,
                         1.0f, 
                         3);
        break;

    case G_GYRO:
        // Pitch 陀螺仪环
        gimbal->base_info.output_gimbal_p =
            all_pid_calc(&gimbal->pid_info.pitch_gyro_outer,
                         &gimbal->pid_info.pitch_gyro_inner,
                         gimbal->pid_info.pitch_target,
                         gimbal->base_info.pitch_imu_angle,
                         gimbal->base_info.pitch_imu_speed,
                         gimbal->feedforward.pitch_rate_cmd_deg_s,
                         1.0f, 
                         0) + gravity;

        // Yaw 陀螺仪环
        gimbal->base_info.output_gimbal_y =
            all_pid_calc(&gimbal->pid_info.yaw_gyro_outer,
                         &gimbal->pid_info.yaw_gyro_inner,
                         gimbal->pid_info.yaw_target,
                         gimbal->base_info.yaw_imu_angle,
                         gimbal->base_info.yaw_imu_speed,
                         gimbal->feedforward.yaw_rate_cmd_deg_s,
                         1.0f, 
                         3);
        break;

    case G_AUTO:
        gimbal->base_info.output_gimbal_p =
            all_pid_calc(&gimbal->pid_info.pitch_gyro_outer,
                         &gimbal->pid_info.pitch_gyro_inner,
                         gimbal->pid_info.pitch_target,
                         gimbal->base_info.pitch_imu_angle,
                         gimbal->base_info.pitch_imu_speed,
                         0.0f,
                         1.0f,
                         0) + gravity;

        gimbal->base_info.output_gimbal_y =
            all_pid_calc(&gimbal->pid_info.yaw_gyro_outer,
                         &gimbal->pid_info.yaw_gyro_inner,
                         gimbal->pid_info.yaw_target,
                         gimbal->base_info.yaw_imu_angle,
                         gimbal->base_info.yaw_imu_speed,
                         0.0f,
                         1.0f,
                         3);
        break;

    case G_RATE:
        gimbal_update_rate_targets(gimbal);

        gimbal->base_info.output_gimbal_p =
            all_pid_calc(NULL,
                         &gimbal->pid_info.pitch_gyro_inner,
                         gimbal->feedforward.pitch_rate_target_deg_s,
                         0.0f,
                         gimbal->base_info.pitch_imu_speed,
                         0.0f,
                         1.0f,
                         0) + gravity;

        gimbal->base_info.output_gimbal_y =
            all_pid_calc(NULL,
                         &gimbal->pid_info.yaw_gyro_inner,
                         gimbal->feedforward.yaw_rate_target_deg_s,
                         0.0f,
                         gimbal->base_info.yaw_imu_speed,
                         0.0f,
                         1.0f,
                         0);
        break;

    case G_SLEEP://休眠一定是无力的
    default:
        gimbal->base_info.output_gimbal_p = 0.0f;
        gimbal->base_info.output_gimbal_y = 0.0f;
        break;
    }


    /* 硬件输出力矩限幅 */
    if (gimbal->gimbal_mode != G_SLEEP)
    {
        gimbal->base_info.output_gimbal_p +=
            gimbal->feedforward.pitch_torque_ff_nm;
        gimbal->base_info.output_gimbal_y +=
            gimbal->feedforward.yaw_torque_ff_nm;

        gimbal->base_info.output_gimbal_p =
            gimbal_clamp(gimbal->base_info.output_gimbal_p,
                         -GIMBAL_TORQUE_LIMIT,
                         GIMBAL_TORQUE_LIMIT);
        gimbal->base_info.output_gimbal_y =
            gimbal_clamp(gimbal->base_info.output_gimbal_y,
                         -GIMBAL_TORQUE_LIMIT,
                         GIMBAL_TORQUE_LIMIT);
    }
    else
    {
        gimbal->base_info.output_gimbal_p = 0.0f;
        gimbal->base_info.output_gimbal_y = 0.0f;
    }
}

void Gimbal_Init(gimbal_t *gimbal)
{
    if (gimbal == NULL) return;

    /* 绑定电机驱动 */
    gimbal->pitch_motor = &dm_motor[PITCH];
    gimbal->yaw_motor = &dm_motor[YAW];
    gimbal->init = Gimbal_Init;
    gimbal->work = Gimbal_Work;

    gimbal->gimbal_mode = G_SLEEP;
    gimbal->last_gimbal_mode = G_SLEEP;

    /* 归中参数初始化 */
    gimbal->init_info.init_flag = 0;
    gimbal->init_info.init_time = 0;
    gimbal->init_info.init_time_max = 6000;
    gimbal->init_info.pitch_angle_tolerance = 2.0f;
    gimbal->init_info.yaw_angle_tolerance = 2.0f;
    gimbal->init_info.pitch_ramp_step = 0.05f; //  归中速率限制
    gimbal->init_info.yaw_ramp_step = 0.05f;   // 归中速率限制
    gimbal->init_info.mode_transition_active = 0;
    gimbal->init_info.mode_pitch_ramp_step = 0.1f;
    gimbal->init_info.mode_yaw_ramp_step = 0.1f;
    gimbal->init_info.pitch_speed_tolerance = 0.5f;
    gimbal->init_info.yaw_speed_tolerance = 0.5f;
    gimbal->init_info.stable_time = 0;
    gimbal->init_info.stable_time_max = 30;

    gimbal->feedforward.yaw_rate_cmd_deg_s = 0.0f;
    gimbal->feedforward.pitch_rate_cmd_deg_s = 0.0f;
    gimbal->feedforward.yaw_rate_target_deg_s = 0.0f;
    gimbal->feedforward.pitch_rate_target_deg_s = 0.0f;
    gimbal->feedforward.yaw_torque_ff_nm = 0.0f;
    gimbal->feedforward.pitch_torque_ff_nm = 0.0f;
    gimbal->feedforward.yaw_hold_angle_deg = 0.0f;
    gimbal->feedforward.pitch_hold_angle_deg = 0.0f;

    gimbal_pid_init(gimbal);
    gimbal->base_info.output_gimbal_p = 0.0f;
    gimbal->base_info.output_gimbal_y = 0.0f;
}

/* 主执行入口，放control任务中 */
void Gimbal_Work(gimbal_t *gimbal)
{
    gimbal_mode_e selected_mode;

    if (gimbal == NULL) return;

    //  采集更新
    gimbal_info_update(gimbal);
    selected_mode = gimbal_select_mode(gimbal);

    //  模式切换保护
    if (selected_mode != gimbal->gimbal_mode)
    {
        gimbal->gimbal_mode = selected_mode;
        gimbal_clear_all_pid(gimbal);
        gimbal->last_gimbal_mode = selected_mode;
        gimbal->init_info.mode_transition_active = 1;
        gimbal->init_info.stable_time = 0;

        // 切换时将目标位置对齐到当前实际位置
        if ((selected_mode == G_GYRO) || (selected_mode == G_AUTO))
        {
            gimbal->pid_info.yaw_target = gimbal->base_info.yaw_imu_angle;
            gimbal->pid_info.pitch_target = gimbal->base_info.pitch_imu_angle;
        }
        else
        {
            gimbal->pid_info.yaw_target = gimbal->base_info.yaw_mec_angle;
            gimbal->pid_info.pitch_target = gimbal->base_info.pitch_mec_angle;
        }

        gimbal->feedforward.yaw_hold_angle_deg = gimbal->base_info.yaw_imu_angle;
        gimbal->feedforward.pitch_hold_angle_deg = gimbal->base_info.pitch_imu_angle;

        // 切换帧先卸力，下一帧从当前角度开始是斜坡步进
        gimbal->base_info.output_gimbal_p = 0.0f;
        gimbal->base_info.output_gimbal_y = 0.0f;
    }
    else
    {
        // 闭环运算
        gimbal_update_targets(gimbal);
        gimbal_calc_output(gimbal);
    }

    //  休眠下电置零
    if (gimbal->gimbal_mode == G_SLEEP)
    {
        gimbal->base_info.output_gimbal_p = 0.0f;
        gimbal->base_info.output_gimbal_y = 0.0f;
    }

    //  写入电机底层发送缓冲区
    gimbal->pitch_motor->tx_info->torque = gimbal->base_info.output_gimbal_p;
    gimbal->yaw_motor->tx_info->torque = gimbal->base_info.output_gimbal_y;
}
