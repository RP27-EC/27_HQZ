#include "chassis_control.h"

#include <math.h>
#include <stddef.h>

#include "rp_math.h"

chassis_control_t chassis_ctrl;
//检查浮点数的有效性
static uint8_t Chassis_Control_ValueValid(float value)
{
    return (value == value) && (value < 1000000.0f) && (value > -1000000.0f);
}

//检查底盘是否在线
static uint8_t Chassis_Control_CheckOnline(void)
{
    if (chassis_ctrl.wheel == NULL)
    {
        chassis_ctrl.state.all_online = 0u;
        return 0u;
    }

    uint8_t online = 1u;

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

//底盘运动学逆解
static void Chassis_Control_KinematicsInverse(const chassis_cmd_t *cmd)
{
    float front = cmd->vx;
    float left = cmd->vy;
    float cycle = cmd->wz;
    //曼哈顿距离近似
    float trans = fabsf(front) + fabsf(left);
    float rotate = fabsf(cycle);
    float total = trans + rotate;

    if (total > CHASSIS_CTRL_MAX_SPEED)
    {
        float rotate_limit = CHASSIS_CTRL_MAX_SPEED * 0.6f;

        if (rotate > rotate_limit)
        {
            cycle *= rotate_limit / rotate;
        }

        float remain = CHASSIS_CTRL_MAX_SPEED - fabsf(cycle);
        if (trans > 0.0001f)
        {
            float k = remain / trans;
            if (k < 1.0f)
            {
                front *= k;
                left *= k;
            }
        }
    }

    chassis_ctrl.state.wheel_target[WHEEL_LF] = -front + left + cycle;
    chassis_ctrl.state.wheel_target[WHEEL_LB] = -front - left + cycle;
    chassis_ctrl.state.wheel_target[WHEEL_RF] =  front + left + cycle;
    chassis_ctrl.state.wheel_target[WHEEL_RB] =  front - left + cycle;
}

static uint8_t Chassis_Control_PidUpdate(void)
{
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

        pid_ctrl_t *pid = chassis_ctrl.wheel->motor[i]->ctrl->speed_ctrl;

        chassis_ctrl.state.wheel_speed[i] =
            chassis_ctrl.wheel->motor[i]->rx_info->speed;
        pid->target = chassis_ctrl.state.wheel_target[i];
        pid->measure = chassis_ctrl.state.wheel_speed[i];
        pid->err = pid->target - pid->measure;
        single_pid_ctrl(pid);

        chassis_ctrl.state.wheel_torque_out[i] =
            constrain(pid->out,
                      -CHASSIS_TEST_TORQUE_LIMIT_NM,
                      CHASSIS_TEST_TORQUE_LIMIT_NM);
    }

    return 1u;
}

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

        chassis_ctrl.wheel->motor[i]->tx_info->torque =
            chassis_ctrl.state.wheel_torque_out[i];
    }

    chassis_ctrl.wheel->group_set_torque(chassis_ctrl.wheel);

    return 1u;
}

void Chassis_Control_Init(void)
{
    uint8_t init_ok = 1u;

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

void Chassis_Control_SetEnable(uint8_t enable)
{
    chassis_ctrl.state.enabled = enable;

    if (enable == 0u)
    {
        Chassis_Control_Stop();
    }
}

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
