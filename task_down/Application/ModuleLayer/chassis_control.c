/* chassis_control.c - 底盘控制 */

#include "chassis_control.h"

#include <math.h>
#include <stddef.h>

#include "rp_math.h"
#include "power_limit.h"
#include "supercap.h"

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
    uint8_t i;

    if (chassis_ctrl.state.cmd.source == CHASSIS_SRC_RC_FOLLOW)
    {
        torque_limit = CHASSIS_FOLLOW_TORQUE_LIMIT_NM;
    }
    else if (chassis_ctrl.state.cmd.source == CHASSIS_SRC_SPIN)
    {
        torque_limit = CHASSIS_SPIN_TORQUE_LIMIT_NM;
    }

    for (i = 0u; i < WHEEL_CNT; i++)
    {
        pid_ctrl_t *pid;

        if (chassis_ctrl.wheel == NULL ||
            chassis_ctrl.wheel->motor[i] == NULL ||
            chassis_ctrl.wheel->motor[i]->ctrl == NULL ||
            chassis_ctrl.wheel->motor[i]->ctrl->speed_ctrl == NULL ||
            chassis_ctrl.wheel->motor[i]->rx_info == NULL)
        {
            chassis_ctrl.state.fault = 1u;
            return 0u;
        }

        pid = chassis_ctrl.wheel->motor[i]->ctrl->speed_ctrl; /* 当前速度环 */

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

    /* 公共比例不增大候选力矩 */
    Power_Limit_Apply(chassis_ctrl.state.wheel_torque_out, chassis_ctrl.wheel->motor);

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

    Power_Limit_Init(); /* 功率限制运行时状态清零 */
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
        Power_Limit_Apply(chassis_ctrl.state.wheel_torque_out, NULL);
        return;
    }

    for (uint8_t i = 0u; i < WHEEL_CNT; i++)
    {
        chassis_ctrl.state.wheel_torque_out[i] = 0.0f;
        if (chassis_ctrl.wheel->motor[i] == NULL ||
            chassis_ctrl.wheel->motor[i]->tx_info == NULL)
        {
            continue;
        }

        chassis_ctrl.wheel->motor[i]->tx_info->torque = 0.0f;
    }

    /* 停机观测对应零输出 */
    Power_Limit_Apply(chassis_ctrl.state.wheel_torque_out, chassis_ctrl.wheel->motor);

    if (chassis_ctrl.wheel->group_set_torque != NULL)
    {
        chassis_ctrl.wheel->group_set_torque(chassis_ctrl.wheel);
    }
}

/* 底盘周期更新，任何异常均回到停机 */
void Chassis_Control_Update(const chassis_cmd_t *cmd)
{
    judge_power_snapshot_t power_snapshot;
    uint8_t all_online = Chassis_Control_CheckOnline();
    uint8_t active = 0u;

    if ((cmd != NULL) && (cmd->valid != 0u) &&
        (chassis_ctrl.state.enabled != 0u) && (all_online != 0u) &&
        Chassis_Control_ValueValid(cmd->vx) &&
        Chassis_Control_ValueValid(cmd->vy) &&
        Chassis_Control_ValueValid(cmd->wz))
    {
        active = ((fabsf(cmd->vx) > CHASSIS_POWER_ACTIVE_V_M_S) ||
                  (fabsf(cmd->vy) > CHASSIS_POWER_ACTIVE_V_M_S) ||
                  (fabsf(cmd->wz) > CHASSIS_POWER_ACTIVE_W_RAD_S)) ? 1u : 0u;
    }

    /* 预算先于力矩限幅刷新 */
    Judge_GetPowerSnapshot(&power_snapshot);
    Power_Limit_GetTarget(&power_snapshot, active);

    /* 超电反馈仅作观测，不参与限功闭环 */
    Power_Limit_SetCapFeedback(supercap.chassis_power,
                               supercap.cap_voltage,
                               supercap.cap_current,
                               supercap.feedback.ability,
                               (supercap.state == SUPERCAP_STATE_ONLINE) ? 1u : 0u);

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

    if (all_online == 0u)
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

