/* chassis_input.c - 底盘输入解析 */

#include "chassis_input.h"

#include "rc_sensor.h"
#include "rp_math.h"

chassis_cmd_t chassis_input_cmd; /* 输入解析后的底盘指令 */

static uint8_t keyboard_source_active; /* 键鼠输入源已开启 */
static uint8_t last_f_pressed; /* 上拍 F 键状态 */

/* 遥控通道归一化，含死区 */
static float Chassis_RcAxisValue(int16_t axis)
{
    float value = (float)axis; /* 去死区输入 */

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

/* 当前是否由键盘模式接管 */
uint8_t Chassis_Input_IsKeyboardMode(void)
{
#if !CHASSIS_KEYBOARD_INPUT_ENABLE
    return 0u;
#else
    if ((keyboard_source_active == 0u) || (rc_dev.work_state != DEV_ONLINE) ||
        (rc_dev.info == NULL))
    {
        return 0u;
    }

    return (rc_dev.info->s1.value == RC_SW_UP) ? 1u : 0u;
#endif
}

/* WASD 平移、QE 旋转，Shift/Ctrl 调速 */
static void Chassis_Input_Keyboard(chassis_cmd_t *cmd, const rc_data_t *rc)
{
    float forward = 0.0f; /* 前后输入 */
    float left = 0.0f;    /* 左右输入 */
    float spin = 0.0f;    /* 旋转输入 */
    float scale = 1.0f;   /* 速度倍率 */

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
    if ((rc->key_v & KEY_PRESSED_OFFSET_Q) != 0u)
    {
        spin += 1.0f;
    }
    if ((rc->key_v & KEY_PRESSED_OFFSET_E) != 0u)
    {
        spin -= 1.0f;
    }

    if ((rc->key_v & KEY_PRESSED_OFFSET_SHIFT) != 0u)
    {
        scale = CHASSIS_KEY_SPEED_BOOST;
    }
    else if ((rc->key_v & KEY_PRESSED_OFFSET_CTRL) != 0u)
    {
        scale = CHASSIS_KEY_SPEED_SLOW;
    }

    cmd->vx = CHASSIS_KEY_VX_SIGN * forward * scale * CHASSIS_MAX_VX;
    cmd->vy = CHASSIS_KEY_VY_SIGN * left * scale * CHASSIS_MAX_VY;
    cmd->wz = CHASSIS_KEY_WZ_SIGN * spin * scale * CHASSIS_MAX_WZ;
    cmd->valid = 1u;
    cmd->source = CHASSIS_SRC_KEYBOARD;
}

/* 初始化输入缓存与按键状态 */
void Chassis_Input_Init(void)
{
    chassis_input_cmd.vx = 0.0f;
    chassis_input_cmd.vy = 0.0f;
    chassis_input_cmd.wz = 0.0f;
    chassis_input_cmd.valid = 0u;
    chassis_input_cmd.source = CHASSIS_SRC_NONE;
    keyboard_source_active = 0u;
    last_f_pressed = 0u;
}

/* 上层模式覆盖输入来源 */
void Chassis_Input_SetSource(chassis_source_e source)
{
    chassis_input_cmd.source = source;
}

/* 周期解析键鼠或遥控，输出统一底盘指令 */
void Chassis_Input_Update(void)
{
    chassis_cmd_t cmd;                 /* 本周期输出 */
    const rc_data_t *rc = rc_dev.info; /* 遥控数据源 */
    uint8_t f_pressed = 0u;            /* F 键当前状态 */

    cmd.vx = 0.0f;
    cmd.vy = 0.0f;
    cmd.wz = 0.0f;
    cmd.valid = 0u;
    cmd.source = CHASSIS_SRC_NONE;

    /* 遥控离线时清零输入，避免保留旧速度 */
    if ((rc_dev.work_state != DEV_ONLINE) || (rc == NULL))
    {
        keyboard_source_active = 0u;
        last_f_pressed = 0u;
        chassis_input_cmd = cmd;
        return;
    }

    f_pressed = ((rc->key_v & KEY_PRESSED_OFFSET_F) != 0u) ? 1u : 0u;
    /* F 切换输入源 */
    if ((f_pressed != 0u) && (last_f_pressed == 0u) && (rc->s1.value == RC_SW_UP))
    {
        keyboard_source_active ^= 1u;
    }
    last_f_pressed = f_pressed;

    /* 键鼠模式优先于遥控摇杆 */
#if CHASSIS_KEYBOARD_INPUT_ENABLE
    if (Chassis_Input_IsKeyboardMode() != 0u)
    {
        Chassis_Input_Keyboard(&cmd, rc);
        chassis_input_cmd = cmd;
        return;
    }
#endif

#if CHASSIS_RC_INPUT_ENABLE
    /* S1 上拨才允许遥控底盘直控 */
    if (rc->s1.value == RC_SW_UP)
    {
        cmd.vx = -Chassis_RcAxisValue(rc->ch3) * CHASSIS_MAX_VX;
        cmd.vy = Chassis_RcAxisValue(rc->ch2) * CHASSIS_MAX_VY;
#if CHASSIS_OWNS_RC_YAW
        cmd.wz = Chassis_RcAxisValue(rc->ch0) * CHASSIS_MAX_WZ;
#else
        cmd.wz = 0.0f;
#endif
        cmd.valid = 1u;
        cmd.source = CHASSIS_SRC_RC;
    }
#else
    (void)rc;
#endif

    chassis_input_cmd = cmd;
}
