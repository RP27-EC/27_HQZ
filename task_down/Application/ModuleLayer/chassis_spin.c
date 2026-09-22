/* chassis_spin.c - 底盘小陀螺 */

#include "chassis_spin.h"

#include <math.h>
#include <stddef.h>

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

    cmd->vx = 0.0f;
    cmd->vy = 0.0f;
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

