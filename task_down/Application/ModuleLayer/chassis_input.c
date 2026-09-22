/* chassis_input.c - 底盘输入解析 */

#include "chassis_input.h"

#include "rc_sensor.h"
#include "rp_math.h"

chassis_cmd_t chassis_input_cmd;

static float Chassis_RcAxisValue(int16_t axis)
{
    float value = (float)axis;

    if ((value > -CHASSIS_RC_DEADBAND) && (value < CHASSIS_RC_DEADBAND))
    {
        return 0.0f;
    }

    if (CHASSIS_RC_DEADBAND > 0.0f)
    {
        value -= (value > 0.0f) ? CHASSIS_RC_DEADBAND : -CHASSIS_RC_DEADBAND;
        value /= (CHASSIS_RC_AXIS_MAX - CHASSIS_RC_DEADBAND);
    }
    else
    {
        value /= CHASSIS_RC_AXIS_MAX;
    }

    return constrain(value, -1.0f, 1.0f);
}

void Chassis_Input_Init(void)
{
    chassis_input_cmd.vx = 0.0f;
    chassis_input_cmd.vy = 0.0f;
    chassis_input_cmd.wz = 0.0f;
    chassis_input_cmd.valid = 0u;
    chassis_input_cmd.source = CHASSIS_SRC_NONE;
}

void Chassis_Input_SetSource(chassis_source_e source)
{
    chassis_input_cmd.source = source;
}

void Chassis_Input_Update(void)
{
    chassis_cmd_t cmd;

    cmd.vx = 0.0f;
    cmd.vy = 0.0f;
    cmd.wz = 0.0f;
    cmd.valid = 0u;
    cmd.source = CHASSIS_SRC_NONE;

#if CHASSIS_RC_INPUT_ENABLE
    /* 未使能时底盘保持停止。 */
    if ((rc_dev.work_state == DEV_ONLINE) &&
        (rc_dev.info != NULL) &&
        (rc_dev.info->s1.value == RC_SW_UP))
    {
        cmd.vx = -Chassis_RcAxisValue(rc_dev.info->ch3) * CHASSIS_MAX_VX;
        cmd.vy = Chassis_RcAxisValue(rc_dev.info->ch2) * CHASSIS_MAX_VY;
#if CHASSIS_OWNS_RC_YAW
        cmd.wz = Chassis_RcAxisValue(rc_dev.info->ch0) * CHASSIS_MAX_WZ;
#else
        cmd.wz = 0.0f;
#endif
        cmd.valid = 1u;
        cmd.source = CHASSIS_SRC_RC;
    }
#endif

#if CHASSIS_KEYBOARD_INPUT_ENABLE
    /* 键鼠输入后续在这复用同一个 chassis_cmd_t。 */
#endif

    chassis_input_cmd = cmd;
}

