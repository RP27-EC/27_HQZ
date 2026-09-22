/* chassis_spin.c - 底盘小陀螺 */

#include "chassis_spin.h"

#include <math.h>
#include <stddef.h>

#include "board_protocol.h"
#include "main.h"
#include "rc_sensor.h"
#include "rp_math.h"

chassis_spin_state_t chassis_spin;

static float spin_ramp_wz;
static uint8_t spin_last_selected;

static float Chassis_Spin_AxisValue(int16_t axis)
{
    float value = (float)axis;

    if ((value > -CHASSIS_SPIN_RC_DEADBAND) && (value < CHASSIS_SPIN_RC_DEADBAND))
    {
        return 0.0f;
    }

    value -= (value > 0.0f) ? CHASSIS_SPIN_RC_DEADBAND : -CHASSIS_SPIN_RC_DEADBAND;
    value /= (CHASSIS_RC_AXIS_MAX - CHASSIS_SPIN_RC_DEADBAND);

    return constrain(value, -1.0f, 1.0f);
}

static float Chassis_Spin_Ramp(float current, float target, float step)
{
    float diff = target - current;

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

static uint8_t Chassis_Spin_GimbalValid(void)
{
    uint32_t age;

    if ((board.status == NULL) || (board.status->gimbal_data_valid == 0u))
    {
        return 0u;
    }

    age = HAL_GetTick() - board.status->gimbal_rx_time_ms;
    return (age <= CHASSIS_SPIN_GIMBAL_TIMEOUT_MS) ? 1u : 0u;
}

static void Chassis_Spin_UpdateTranslation(chassis_cmd_t *cmd)
{
    float vx_gimbal;
    float vy_gimbal;
    float yaw_mec;
    float theta;

    if (cmd == NULL)
    {
        return;
    }

#if !CHASSIS_SPIN_TRANSLATION_ENABLE
    cmd->vx = 0.0f;
    cmd->vy = 0.0f;
#elif CHASSIS_SPIN_TRANSLATION_FRAME_GIMBAL
    if (Chassis_Spin_GimbalValid() == 0u)
    {
        cmd->vx = 0.0f;
        cmd->vy = 0.0f;
        return;
    }

    yaw_mec = board.rx_meg->gimbal_meg.yaw_mec;
    if ((yaw_mec != yaw_mec) || (fabsf(yaw_mec) > 4.0f))
    {
        cmd->vx = 0.0f;
        cmd->vy = 0.0f;
        return;
    }

    vx_gimbal = cmd->vx;
    vy_gimbal = cmd->vy;
    theta = CHASSIS_SPIN_TRANSLATION_YAW_SIGN * yaw_mec;

    cmd->vx = CHASSIS_SPIN_TRANSLATION_SIGN *
              ((cosf(theta) * vx_gimbal) - (sinf(theta) * vy_gimbal));
    cmd->vy = CHASSIS_SPIN_TRANSLATION_SIGN *
              ((sinf(theta) * vx_gimbal) + (cosf(theta) * vy_gimbal));
#else
    (void)vx_gimbal;
    (void)vy_gimbal;
    (void)yaw_mec;
    (void)theta;
#endif
}

void Chassis_Spin_Init(void)
{
    chassis_spin.target_wz = 0.0f;
    chassis_spin.output_wz = 0.0f;
    chassis_spin.selected = 0u;
    chassis_spin.active = 0u;

    spin_ramp_wz = 0.0f;
    spin_last_selected = 0u;
}

void Chassis_Spin_UpdateMode(void)
{
    uint8_t selected = 0u;

#if CHASSIS_SPIN_ENABLE
    if ((rc_dev.work_state == DEV_ONLINE) &&
        (rc_dev.info != NULL) &&
        (rc_dev.info->s1.value == RC_SW_UP) &&
        (rc_dev.info->s2.value == RC_SW_DOWN))
    {
        selected = 1u;
    }
#endif

    chassis_spin.selected = selected;
    chassis_spin.active = selected;
}

void Chassis_Spin_Update(chassis_cmd_t *cmd)
{
    float axis;
    float target_wz;

    if (cmd == NULL)
    {
        return;
    }

    if (chassis_spin.selected == 0u)
    {
        if (spin_last_selected != 0u)
        {
            spin_ramp_wz = Chassis_Spin_Ramp(spin_ramp_wz, 0.0f, CHASSIS_SPIN_STEP);

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

    if (spin_last_selected == 0u)
    {
        spin_ramp_wz = cmd->wz;
        spin_last_selected = 1u;
    }

    axis = Chassis_Spin_AxisValue(rc_dev.info->ch0);
    target_wz = CHASSIS_SPIN_DIRECTION *
                (CHASSIS_SPIN_BASE_WZ + (axis * CHASSIS_SPIN_TRIM_WZ));
    target_wz = constrain(target_wz,
                        -CHASSIS_SPIN_MAX_WZ,
                        CHASSIS_SPIN_MAX_WZ);

    spin_ramp_wz = Chassis_Spin_Ramp(spin_ramp_wz, target_wz, CHASSIS_SPIN_STEP);

    Chassis_Spin_UpdateTranslation(cmd);

    cmd->wz = spin_ramp_wz;
    cmd->valid = 1u;
    cmd->source = CHASSIS_SRC_SPIN;

    chassis_spin.target_wz = target_wz;
    chassis_spin.output_wz = spin_ramp_wz;
}

uint8_t Chassis_Spin_IsSelected(void)
{
    return chassis_spin.selected;
}

uint8_t Chassis_Spin_IsActive(void)
{
    return chassis_spin.active;
}

