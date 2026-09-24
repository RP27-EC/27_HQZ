/* chassis_spin.c - 底盘小陀螺 */

#include "chassis_spin.h"

#include <math.h>
#include <stddef.h>

#include "board_protocol.h"
#include "main.h"
#include "rc_sensor.h"
#include "rp_math.h"

chassis_spin_state_t chassis_spin; /* 小陀螺状态 */

static float spin_ramp_wz; /* 旋转斜坡输出，rad/s */
static uint8_t spin_last_selected; /* 上拍小陀螺选中状态 */

/* 将遥控通道映射到 [-1, 1]，含死区 */

static float Chassis_Spin_AxisValue(int16_t axis)
{
    float value = (float)axis; /* 去死区并归一化 */

    if ((value > -CHASSIS_SPIN_RC_DEADBAND) && (value < CHASSIS_SPIN_RC_DEADBAND))
    {
        return 0.0f;
    }

    value -= (value > 0.0f) ? CHASSIS_SPIN_RC_DEADBAND : -CHASSIS_SPIN_RC_DEADBAND; /* 剔除死区 */
    value /= (CHASSIS_RC_AXIS_MAX - CHASSIS_SPIN_RC_DEADBAND); /* 归一化 */

    return constrain(value, -1.0f, 1.0f);
}

/* 按步长斜坡到目标角速度 */
static float Chassis_Spin_Ramp(float current, float target, float step)
{
    float diff = target - current; /* 剩余斜坡量 */

    if (diff > step)
    {
        return current + step;
    }

    if (diff < -step)
    {
        return current - step;
    }

    return target;
}

/* 云台反馈未超时且有效才允许系变换 */
static uint8_t Chassis_Spin_GimbalValid(void)
{
    uint32_t age; /* 云台反馈年龄，ms */

    if ((board.status == NULL) || (board.status->gimbal_data_valid == 0u))
    {
        return 0u;
    }

    age = HAL_GetTick() - board.status->gimbal_rx_time_ms; /* 反馈年龄 */
    return (age <= CHASSIS_SPIN_GIMBAL_TIMEOUT_MS) ? 1u : 0u;
}

/* 小陀螺平移按云台朝向旋转 */
static void Chassis_Spin_UpdateTranslation(chassis_cmd_t *cmd)
{
    float vx_gimbal; /* 旋转前纵向速度 */
    float vy_gimbal; /* 旋转前横向速度 */
    float yaw_mec;   /* 云台机械角，rad */
    float theta;     /* 平移旋转角，rad */

    if (cmd == NULL)
    {
        return;
    }

#if !CHASSIS_SPIN_TRANSLATION_ENABLE
    cmd->vx = 0.0f;
    cmd->vy = 0.0f;
    /* 云台数据缺失时零平移，只保留自转 */
#elif CHASSIS_SPIN_TRANSLATION_FRAME_GIMBAL
    if (Chassis_Spin_GimbalValid() == 0u)
    {
        cmd->vx = 0.0f;
        cmd->vy = 0.0f;
        return;
    }

    yaw_mec = board.rx_meg->gimbal_meg.yaw_mec; /* 云台机械角 */
    if ((yaw_mec != yaw_mec) || (fabsf(yaw_mec) > 4.0f)) /* 排除 NaN/越界 */
    {
        cmd->vx = 0.0f;
        cmd->vy = 0.0f;
        return;
    }

    vx_gimbal = cmd->vx;
    vy_gimbal = cmd->vy;
    theta = CHASSIS_SPIN_TRANSLATION_YAW_SIGN * yaw_mec; /* 平移转角 */

    cmd->vx = CHASSIS_SPIN_TRANSLATION_SIGN * /* 旋转纵向 */
              ((cosf(theta) * vx_gimbal) - (sinf(theta) * vy_gimbal));
    cmd->vy = CHASSIS_SPIN_TRANSLATION_SIGN * /* 旋转横向 */
              ((sinf(theta) * vx_gimbal) + (cosf(theta) * vy_gimbal));
#else
    (void)vx_gimbal;
    (void)vy_gimbal;
    (void)yaw_mec;
    (void)theta;
#endif
}

/* 初始化小陀螺状态 */
void Chassis_Spin_Init(void)
{
    chassis_spin.target_wz = 0.0f; /* 清目标 */
    chassis_spin.output_wz = 0.0f; /* 清输出 */
    chassis_spin.selected = 0u;
    chassis_spin.active = 0u;

    spin_ramp_wz = 0.0f;
    spin_last_selected = 0u;
}

/* S1 上拨、S2 下拨选择小陀螺 */
void Chassis_Spin_UpdateMode(void)
{
    uint8_t selected = 0u; /* 小陀螺档位选择 */

#if CHASSIS_SPIN_ENABLE
    if ((rc_dev.work_state == DEV_ONLINE) &&
        (rc_dev.info != NULL) &&
        (rc_dev.info->s1.value == RC_SW_UP) &&
        (rc_dev.info->s2.value == RC_SW_DOWN))
    {
        selected = 1u; /* S1 上、S2 下 */
    }
#endif

    chassis_spin.selected = selected; /* 本拍选择 */
    chassis_spin.active = selected;   /* 小陀螺生效 */
}

/* 根据 ch0 调节旋转速度并接管底盘指令 */
void Chassis_Spin_Update(chassis_cmd_t *cmd)
{
    float axis;     /* ch0 归一化输入 */
    float target_wz; /* 目标旋转角速度，rad/s */

    if (cmd == NULL)
    {
        return;
    }

    /* 退出档位后斜坡归零，不瞬间切断旋转 */
    if (chassis_spin.selected == 0u)
    {
        if (spin_last_selected != 0u)
        {
            spin_ramp_wz = Chassis_Spin_Ramp(spin_ramp_wz, 0.0f, CHASSIS_SPIN_STEP); /* 斜坡归零 */

            if (fabsf(spin_ramp_wz) <= CHASSIS_SPIN_STEP)
            {
                spin_ramp_wz = 0.0f;
                spin_last_selected = 0u;
            }

            if (spin_ramp_wz != 0.0f)
            {
                cmd->wz = spin_ramp_wz;
                cmd->source = CHASSIS_SRC_SPIN;
            }
        }

        chassis_spin.output_wz = spin_ramp_wz;
        return;
    }

    /* 进入档位时承接当前角速度，避免跳变 */
    if (spin_last_selected == 0u)
    {
        spin_ramp_wz = cmd->wz; /* 承接当前角速度 */
        spin_last_selected = 1u;
    }

    axis = Chassis_Spin_AxisValue(rc_dev.info->ch0); /* 调节量 */
    target_wz = CHASSIS_SPIN_DIRECTION *
                (CHASSIS_SPIN_BASE_WZ + (axis * CHASSIS_SPIN_TRIM_WZ)); /* 旋转目标 */
    target_wz = constrain(target_wz, /* 目标限幅 */
                        -CHASSIS_SPIN_MAX_WZ,
                        CHASSIS_SPIN_MAX_WZ);

    spin_ramp_wz = Chassis_Spin_Ramp(spin_ramp_wz, target_wz, CHASSIS_SPIN_STEP); /* 平滑旋转 */

    Chassis_Spin_UpdateTranslation(cmd);

    cmd->wz = spin_ramp_wz; /* 接管旋转 */
    cmd->valid = 1u;
    cmd->source = CHASSIS_SRC_SPIN; /* 标记来源 */

    chassis_spin.target_wz = target_wz; /* 调参观测 */
    chassis_spin.output_wz = spin_ramp_wz;
}

/* 小陀螺档位是否选中 */
uint8_t Chassis_Spin_IsSelected(void)
{
    return chassis_spin.selected;
}

/* 小陀螺控制是否生效 */
uint8_t Chassis_Spin_IsActive(void)
{
    return chassis_spin.active;
}

