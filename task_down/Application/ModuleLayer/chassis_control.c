/* chassis_control.c - 底盘控制 */

#include "chassis_control.h"

#include <math.h>
#include <stddef.h>

#include "rp_math.h"

//目前纯P的控制器，目前响应速度还可以，跟随得也还行
//但可以牺牲了一些操作手感，后续再看看
//后续可以根据情况去加速度规划器和前馈力控方案等

chassis_control_t chassis_ctrl; /* 底盘控制对象 */

/* 防 NaN 和异常大值 */
static uint8_t Chassis_Control_ValueValid(float value)
{
    return (value == value) && (value < 1000000.0f) && (value > -1000000.0f);
}

/* 检查四轮对象和在线状态 */
static uint8_t Chassis_Control_CheckOnline(void)
{
    if (chassis_ctrl.wheel == NULL)
    {
        chassis_ctrl.state.all_online = 0u;
        return 0u;
    }

    uint8_t online = 1u; /* 四轮组合在线标志 */

    for (uint8_t i = 0u; i < WHEEL_CNT; i++)
    {
        chassis_ctrl.state.wheel_online[i] =
            (chassis_ctrl.wheel->motor[i] != NULL &&
             chassis_ctrl.wheel->motor[i]->state != NULL &&
             chassis_ctrl.wheel->motor[i]->ctrl != NULL &&
             chassis_ctrl.wheel->motor[i]->ctrl->speed_ctrl != NULL &&
             chassis_ctrl.wheel->motor[i]->state->status == DEV_ONLINE) ? 1u : 0u;

        if (chassis_ctrl.state.wheel_online[i] == 0u)
        {
            online = 0u;
        }
    }

    chassis_ctrl.state.all_online = online;
    return online;
}

/* 底盘运动学逆解，输出四轮速度目标 */
static void Chassis_Control_KinematicsInverse(const chassis_cmd_t *cmd)
{
    float front = cmd->vx; /* 前后分量 */
    float left = cmd->vy;  /* 左右分量 */
    float cycle = cmd->wz; /* 旋转分量 */
    float trans = fabsf(front) + fabsf(left); /* 平移权重 */
    float rotate = fabsf(cycle);               /* 旋转权重 */
    float total = trans + rotate;              /* 总权重 */
    /* 总权重超限时优先保留旋转 */
    if (total > CHASSIS_CTRL_MAX_SPEED)
    {
        float rotate_limit = CHASSIS_CTRL_MAX_SPEED * 0.6f; /* 旋转保留量 */

        if (rotate > rotate_limit)
        {
            cycle *= rotate_limit / rotate; /* 旋转限幅 */
        }

        float remain = CHASSIS_CTRL_MAX_SPEED - fabsf(cycle); /* 平移余量 */
        if (trans > 0.0001f)
        {
            float k = remain / trans; /* 平移缩放 */
            if (k < 1.0f)
            {
                front *= k;
                left *= k;
            }
        }
    }

    chassis_ctrl.state.wheel_target[WHEEL_LF] = -front + left + cycle; /* 左前 */
    chassis_ctrl.state.wheel_target[WHEEL_LB] = -front - left + cycle; /* 左后 */
    chassis_ctrl.state.wheel_target[WHEEL_RF] =  front + left + cycle; /* 右前 */
    chassis_ctrl.state.wheel_target[WHEEL_RB] =  front - left + cycle; /* 右后 */
}

/* 四轮速度环计算，按控制源限制力矩 */
static uint8_t Chassis_Control_PidUpdate(void)
{
    float torque_limit = CHASSIS_TEST_TORQUE_LIMIT_NM; /* 默认调试限矩 */

    if (chassis_ctrl.state.cmd.source == CHASSIS_SRC_RC_FOLLOW)
    {
        torque_limit = CHASSIS_FOLLOW_TORQUE_LIMIT_NM;
    }
    else if (chassis_ctrl.state.cmd.source == CHASSIS_SRC_SPIN)
    {
        torque_limit = CHASSIS_SPIN_TORQUE_LIMIT_NM;
    }

    for (uint8_t i = 0u; i < WHEEL_CNT; i++)
    {
        if (chassis_ctrl.wheel == NULL ||
            chassis_ctrl.wheel->motor[i] == NULL ||
            chassis_ctrl.wheel->motor[i]->ctrl == NULL ||
            chassis_ctrl.wheel->motor[i]->ctrl->speed_ctrl == NULL ||
            chassis_ctrl.wheel->motor[i]->rx_info == NULL)
        {
            chassis_ctrl.state.fault = 1u;
            return 0u;
        }

        pid_ctrl_t *pid = chassis_ctrl.wheel->motor[i]->ctrl->speed_ctrl; /* 当前速度环 */

        chassis_ctrl.state.wheel_speed[i] =
            chassis_ctrl.wheel->motor[i]->rx_info->speed;
        pid->target = chassis_ctrl.state.wheel_target[i]; /* 轮速目标 */
        pid->measure = chassis_ctrl.state.wheel_speed[i]; /* 轮速反馈 */
        pid->out_max = torque_limit; /* 模式限矩 */
        if ((fabsf(pid->target) < CHASSIS_ZERO_TARGET_BAND) &&
            (fabsf(pid->measure) < CHASSIS_STOP_SPEED_BAND))
        {
            pid->err = 0.0f; /* 零速区清 PID */
            pid->last_err = 0.0f;
            pid->integral = 0.0f;
            pid->pout = 0.0f;
            pid->iout = 0.0f;
            pid->dout = 0.0f;
            pid->last_dout = 0.0f;
            pid->out = 0.0f;
        }
        else
        {
            pid->err = pid->target - pid->measure;
            single_pid_ctrl(pid);
        }

        chassis_ctrl.state.wheel_torque_out[i] = /* 力矩限幅 */
            constrain(pid->out,
                      -torque_limit,
                      torque_limit);
    }

    return 1u;
}

/* 将四轮力矩写入电机并整组发送 */
static uint8_t Chassis_Control_Output(void)
{
    if (chassis_ctrl.wheel == NULL || chassis_ctrl.wheel->group_set_torque == NULL)
    {
        chassis_ctrl.state.fault = 1u;
        return 0u;
    }

    for (uint8_t i = 0u; i < WHEEL_CNT; i++)
    {
        if (chassis_ctrl.wheel->motor[i] == NULL ||
            chassis_ctrl.wheel->motor[i]->tx_info == NULL)
        {
            chassis_ctrl.state.fault = 1u;
            return 0u;
        }

        chassis_ctrl.wheel->motor[i]->tx_info->torque = /* 写入组帧缓存 */
            chassis_ctrl.state.wheel_torque_out[i];
    }

    chassis_ctrl.wheel->group_set_torque(chassis_ctrl.wheel);

    return 1u;
}

/* 初始化底盘对象、四轮 PID 与安全状态 */
void Chassis_Control_Init(void)
{
    uint8_t init_ok = 1u; /* 四轮对象完整性 */

    chassis_ctrl.wheel = &wheel_group;

    for (uint8_t i = 0u; i < WHEEL_CNT; i++)
    {
        chassis_ctrl.state.wheel_target[i] = 0.0f;
        chassis_ctrl.state.wheel_speed[i] = 0.0f;
        chassis_ctrl.state.wheel_torque_out[i] = 0.0f;
        chassis_ctrl.state.wheel_online[i] = 0u;

        if (chassis_ctrl.wheel == NULL ||
            chassis_ctrl.wheel->motor[i] == NULL ||
            chassis_ctrl.wheel->motor[i]->ctrl == NULL ||
            chassis_ctrl.wheel->motor[i]->ctrl->speed_ctrl == NULL ||
            chassis_ctrl.wheel->motor[i]->tx_info == NULL)
        {
            init_ok = 0u;
            continue;
        }

        pid_ctrl_t *pid = chassis_ctrl.wheel->motor[i]->ctrl->speed_ctrl;

        pid->kp = CHASSIS_SPEED_KP;
        pid->ki = CHASSIS_SPEED_KI;
        pid->kd = CHASSIS_SPEED_KD;
        pid->integral = 0.0f;
        pid->integral_max = 0.0f;
        pid->out_max = CHASSIS_TEST_TORQUE_LIMIT_NM;
        pid->target = 0.0f;
        pid->measure = 0.0f;
        pid->err = 0.0f;
        pid->out = 0.0f;

        chassis_ctrl.wheel->motor[i]->tx_info->torque = 0.0f;
    }

    chassis_ctrl.state.cmd.vx = 0.0f;
    chassis_ctrl.state.cmd.vy = 0.0f;
    chassis_ctrl.state.cmd.wz = 0.0f;
    chassis_ctrl.state.cmd.valid = 0u;
    chassis_ctrl.state.cmd.source = CHASSIS_SRC_NONE;
    chassis_ctrl.state.all_online = 0u;
    chassis_ctrl.state.enabled = init_ok;
    chassis_ctrl.state.fault = (init_ok == 0u) ? 1u : 0u;
}

/* 使能或关闭底盘，关闭时立即卸力 */
void Chassis_Control_SetEnable(uint8_t enable)
{
    chassis_ctrl.state.enabled = enable;

    if (enable == 0u)
    {
        Chassis_Control_Stop();
    }
}

/* 四轮输出清零 */
void Chassis_Control_Stop(void)
{
    if (chassis_ctrl.wheel == NULL)
    {
        return;
    }

    for (uint8_t i = 0u; i < WHEEL_CNT; i++)
    {
        if (chassis_ctrl.wheel->motor[i] == NULL ||
            chassis_ctrl.wheel->motor[i]->tx_info == NULL)
        {
            continue;
        }

        chassis_ctrl.state.wheel_torque_out[i] = 0.0f;
        chassis_ctrl.wheel->motor[i]->tx_info->torque = 0.0f;
    }

    if (chassis_ctrl.wheel->group_set_torque != NULL)
    {
        chassis_ctrl.wheel->group_set_torque(chassis_ctrl.wheel);
    }
}

/* 底盘周期更新，任何异常均回到停机 */
void Chassis_Control_Update(const chassis_cmd_t *cmd)
{
    if (cmd == NULL)
    {
        chassis_ctrl.state.fault = 1u;
        Chassis_Control_Stop();
        return;
    }

    chassis_ctrl.state.cmd = *cmd;

    if (chassis_ctrl.state.enabled == 0u || cmd->valid == 0u)
    {
        chassis_ctrl.state.fault = 1u;
        Chassis_Control_Stop();
        return;
    }

    if (!Chassis_Control_ValueValid(cmd->vx) ||
        !Chassis_Control_ValueValid(cmd->vy) ||
        !Chassis_Control_ValueValid(cmd->wz))
    {
        chassis_ctrl.state.fault = 1u;
        Chassis_Control_Stop();
        return;
    }

    if (Chassis_Control_CheckOnline() == 0u)
    {
        chassis_ctrl.state.fault = 1u;
        Chassis_Control_Stop();
        return;
    }

    Chassis_Control_KinematicsInverse(cmd);
    if (Chassis_Control_PidUpdate() == 0u)
    {
        Chassis_Control_Stop();
        return;
    }

    if (Chassis_Control_Output() == 0u)
    {
        Chassis_Control_Stop();
        return;
    }

    chassis_ctrl.state.fault = 0u;
}

