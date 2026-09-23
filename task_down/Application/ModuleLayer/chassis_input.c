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

/* 键鼠模式：S1 上拨使能，S2 中位选择。 */
uint8_t Chassis_Input_IsKeyboardMode(void)
{
#if CHASSIS_KEYBOARD_INPUT_ENABLE
    if ((rc_dev.work_state != DEV_ONLINE) || (rc_dev.info == NULL))
    {
        return 0u;
    }

    if ((rc_dev.info->s1.value != RC_SW_UP) ||
        (rc_dev.info->s2.value != RC_SW_MID))
    {
        return 0u;
    }

    return 1u;
#else
    return 0u;
#endif
}

#if CHASSIS_KEYBOARD_INPUT_ENABLE
/*
 * 键位到速度：W/S 前后，A/D 左右，Shift 加速，Ctrl 减速。
 * 松开即回零，不做锁存，避免失联后残留速度。
 */
static void Chassis_Input_Keyboard(chassis_cmd_t *cmd, const rc_data_t *rc)
{
    float forward = 0.0f;
    float left = 0.0f;
    float spin = 0.0f;
    float scale = 1.0f;

    if ((rc->key_v & KEY_PRESSED_OFFSET_W) != 0u)
    {
        forward += 1.0f;
    }
    if ((rc->key_v & KEY_PRESSED_OFFSET_S) != 0u)
    {
        forward -= 1.0f;
    }
    if ((rc->key_v & KEY_PRESSED_OFFSET_A) != 0u)
    {
        left += 1.0f;
    }
    if ((rc->key_v & KEY_PRESSED_OFFSET_D) != 0u)
    {
        left -= 1.0f;
    }

    if ((rc->key_v & KEY_PRESSED_OFFSET_SHIFT) != 0u)
    {
        scale = CHASSIS_KEY_SPEED_BOOST;
    }
    else if ((rc->key_v & KEY_PRESSED_OFFSET_CTRL) != 0u)
    {
        scale = CHASSIS_KEY_SPEED_SLOW;
    }

#if CHASSIS_KEY_YAW_ENABLE
    /* 键鼠模式底盘不跟云台，Q/E 手动自转。 */
    if ((rc->key_v & KEY_PRESSED_OFFSET_Q) != 0u)
    {
        spin += 1.0f;
    }
    if ((rc->key_v & KEY_PRESSED_OFFSET_E) != 0u)
    {
        spin -= 1.0f;
    }
#endif

    cmd->vx = CHASSIS_KEY_VX_SIGN * forward * scale * CHASSIS_MAX_VX;
    cmd->vy = CHASSIS_KEY_VY_SIGN * left * scale * CHASSIS_MAX_VY;
    cmd->wz = CHASSIS_KEY_WZ_SIGN * spin * scale * CHASSIS_MAX_WZ;
    cmd->valid = 1u;
    cmd->source = CHASSIS_SRC_KEYBOARD;
}
#endif

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

#if CHASSIS_KEYBOARD_INPUT_ENABLE
    /* 键鼠优先；拨杆离开键鼠位立即落回遥控分支。 */
    if (Chassis_Input_IsKeyboardMode() != 0u)
    {
        Chassis_Input_Keyboard(&cmd, rc_dev.info);
        chassis_input_cmd = cmd;
        return;
    }
#endif

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

    chassis_input_cmd = cmd;
}

