/* chassis_input.c - 底盘输入解析 */

#include "chassis_input.h"

#include <math.h>

#include "board_protocol.h"
#include "main.h"
#include "rc_sensor.h"
#include "rp_math.h"

#define CHASSIS_INPUT_PI      3.14159265358979323846f /* 圆周率 */
#define CHASSIS_INPUT_HALF_PI (CHASSIS_INPUT_PI * 0.5f) /* 前后基准分界 */

chassis_cmd_t chassis_input_cmd; /* 输入解析后的底盘指令 */

static uint8_t keyboard_source_active; /* 键鼠输入源已开启 */
static uint8_t last_f_pressed; /* 上拍 F 键状态 */
static chassis_key_mode_e keyboard_chassis_mode; /* 键鼠底盘模式 */
static uint8_t last_z_pressed; /* Z 键上次状态 */
static uint8_t last_x_pressed; /* X 键上次状态 */
static uint8_t last_c_pressed; /* C 键上次状态 */

/*
 * 掉头基准角，rad。0 = 前方，pi = 后方，整车的"前方"就由它定义。
 *
 * 机械档按 R 直接翻到另一头（云台锁 0/pi，底盘不动）；跟随档按 R 走掉头动作，
 * 动作收尾时取实际相对角，这样跟随环接手时误差正好是 0，底盘一下都不会动。
 * 只在键鼠模式内保留，退出键鼠或遥控离线复位回前方。
 */
static float yaw_reference_rad;
static uint8_t last_r_pressed;    /* R 键上次状态 */
static uint8_t uturn_active;      /* 1 = 跟随档掉头动作进行中 */
static uint8_t uturn_stopping;    /* 1 = 已转够角度，正在减速 */
static float uturn_rate_deg_s;    /* 当前下发的 Yaw 角速度指令，deg/s */
static float uturn_start_yaw_deg; /* 动作起始 Yaw IMU 角，deg */
static uint32_t uturn_start_tick; /* 动作起始时刻，ms */

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

/* Z/X/C 选择键鼠底盘模式，只在键鼠源生效 */
static void Chassis_Input_KeyboardModeUpdate(const rc_data_t *rc)
{
    uint8_t z_pressed = ((rc->key_v & KEY_PRESSED_OFFSET_Z) != 0u) ? 1u : 0u;
    uint8_t x_pressed = ((rc->key_v & KEY_PRESSED_OFFSET_X) != 0u) ? 1u : 0u;
    uint8_t c_pressed = ((rc->key_v & KEY_PRESSED_OFFSET_C) != 0u) ? 1u : 0u;

    if (Chassis_Input_IsKeyboardMode() != 0u)
    {
        if ((z_pressed != 0u) && (last_z_pressed == 0u))
        {
            keyboard_chassis_mode = CHASSIS_KEY_MODE_FOLLOW;
        }
        else if ((x_pressed != 0u) && (last_x_pressed == 0u))
        {
            keyboard_chassis_mode = CHASSIS_KEY_MODE_MECH;
        }
        else if ((c_pressed != 0u) && (last_c_pressed == 0u))
        {
            keyboard_chassis_mode = CHASSIS_KEY_MODE_SPIN;
        }
    }

    last_z_pressed = z_pressed;
    last_x_pressed = x_pressed;
    last_c_pressed = c_pressed;
}

/* 获取键鼠选择的底盘模式 */
chassis_key_mode_e Chassis_Input_GetKeyboardChassisMode(void)
{
    return keyboard_chassis_mode;
}

/* 将角度归一化到 [-pi, pi] */
static float Chassis_Input_WrapPi(float angle)
{
    return atan2f(sinf(angle), cosf(angle));
}

/* 将角度归一化到 [-180, 180]，单位 deg */
static float Chassis_Input_Wrap180Deg(float angle_deg)
{
    float value = fmodf(angle_deg, 360.0f); /* 取余 */

    if (value > 180.0f)
    {
        value -= 360.0f;
    }
    else if (value < -180.0f)
    {
        value += 360.0f;
    }

    return value;
}

/* 当前掉头基准角，跟随中心用它 */
float Chassis_Input_GetYawReferenceRad(void)
{
    return yaw_reference_rad;
}

/* 强制把掉头基准打回前方：过洞强制云台回零位时，基准必须跟着回零位，
 * 否则跟随中心仍停在 180deg，云台被拉回前方时跟随环会把底盘转过去。 */
void Chassis_Input_ResetYawReference(void)
{
    yaw_reference_rad = 0.0f;
}

/* 掉头基准是否在后方 */
uint8_t Chassis_Input_IsKeyboardYawRear(void)
{
    return (fabsf(Chassis_Input_WrapPi(yaw_reference_rad)) > CHASSIS_INPUT_HALF_PI) ? 1u : 0u;
}

/* 键鼠 + X 机械档是否生效 */
uint8_t Chassis_Input_IsKeyboardMechMode(void)
{
    if ((Chassis_Input_IsKeyboardMode() == 0u) ||
        (keyboard_chassis_mode != CHASSIS_KEY_MODE_MECH))
    {
        return 0u;
    }

    return 1u;
}

/* 跟随档掉头动作是否进行中 */
uint8_t Chassis_Input_IsUturnActive(void)
{
    return uturn_active;
}

/* 取掉头动作的 Yaw 角速度指令，deg/s */
uint8_t Chassis_Input_GetUturnYawRateDegS(float *rate_deg_s)
{
    if ((uturn_active == 0u) || (rate_deg_s == NULL))
    {
        return 0u;
    }

    *rate_deg_s = uturn_rate_deg_s;
    return 1u;
}

/*
 * 掉头处理：机械档按 R 翻基准；跟随档按 R 走"云台世界角转 180deg"的动作。
 *
 * 跟随档云台在上板是 IMU 自稳的，D2 的机械目标角根本不参与，只能靠 D5 的 Yaw
 * 角速度指令推它转。所以这里给一段恒定角速度，并用上板回传的 yaw_imu 闭环判断
 * 转够了没有；动作期间跟随环只冻结"底盘跟转"（见 chassis_follow.c），
 * 平移旋转照旧，底盘不会自己转。
 */
static void Chassis_Input_UturnUpdate(const rc_data_t *rc)
{
    uint8_t r_pressed;   /* R 键当前状态 */
    uint8_t mech;        /* 机械档 */
    uint8_t follow;      /* 跟随档 */
    uint8_t data_valid;  /* 云台反馈可用 */
    float delta_deg;     /* 动作已转过的角度，deg */
    float target_rate;   /* 本拍角速度目标，deg/s */

    r_pressed = ((rc->key_v & KEY_PRESSED_OFFSET_R) != 0u) ? 1u : 0u;

    if (Chassis_Input_IsKeyboardMode() == 0u)
    {
        /* 退出键鼠：基准复位回前方，动作取消。
         * R 的按键记忆要留着：按着 R 切进键鼠的那一拍不许被当成一次新边沿。 */
        yaw_reference_rad = 0.0f;
        last_r_pressed = r_pressed;
        uturn_active = 0u;
        uturn_stopping = 0u;
        uturn_rate_deg_s = 0.0f;
        return;
    }

    mech = (keyboard_chassis_mode == CHASSIS_KEY_MODE_MECH) ? 1u : 0u;
    follow = (keyboard_chassis_mode == CHASSIS_KEY_MODE_FOLLOW) ? 1u : 0u;
    data_valid = 0u;
    /* gimbal_data_valid 只在收到 C2 时置位、不会自己清零，所以还要看反馈年龄 */
    if ((board.status != NULL) && (board.status->gimbal_data_valid != 0u))
    {
        if ((HAL_GetTick() - board.status->gimbal_rx_time_ms) <=
            CHASSIS_KEY_UTURN_FEEDBACK_TIMEOUT_MS)
        {
            data_valid = 1u;
        }
    }

    if ((r_pressed != 0u) && (last_r_pressed == 0u) && (uturn_active == 0u))
    {
        if (mech != 0u)
        {
            /* 机械档：基准直接翻到另一头，云台锁 0/pi，底盘不动 */
            yaw_reference_rad = (Chassis_Input_IsKeyboardYawRear() != 0u) ?
                                0.0f : CHASSIS_INPUT_PI;
        }
        else if ((follow != 0u) && (data_valid != 0u))
        {
            /* 跟随档：起一个掉头动作 */
            uturn_active = 1u;
            uturn_stopping = 0u;
            uturn_rate_deg_s = 0.0f;
            /* NOTE: 上板 yaw_imu 单位是 deg（yaw_mec 才是 rad），别混用 */
            uturn_start_yaw_deg = board.rx_meg->gimbal_meg.yaw_imu;
            uturn_start_tick = HAL_GetTick();
        }
    }
    last_r_pressed = r_pressed;

    if (uturn_active == 0u)
    {
        return;
    }

    /* 档位切走或反馈失效：取消动作，基准保持原值 */
    if ((follow == 0u) || (data_valid == 0u))
    {
        uturn_active = 0u;
        uturn_stopping = 0u;
        uturn_rate_deg_s = 0.0f;
        return;
    }

    if (uturn_stopping == 0u)
    {
        delta_deg = Chassis_Input_Wrap180Deg(board.rx_meg->gimbal_meg.yaw_imu -
                                            uturn_start_yaw_deg);

        if ((fabsf(delta_deg) >= (CHASSIS_KEY_UTURN_ANGLE_DEG - CHASSIS_KEY_UTURN_TOL_DEG)) ||
            ((HAL_GetTick() - uturn_start_tick) >= CHASSIS_KEY_UTURN_TIMEOUT_MS))
        {
            uturn_stopping = 1u;
        }
    }

    target_rate = (uturn_stopping != 0u) ? 0.0f :
                  (CHASSIS_KEY_UTURN_RATE_DEG_S * CHASSIS_KEY_UTURN_SIGN);

    /* 角速度斜坡，起停都不突跳 */
    if (uturn_rate_deg_s < target_rate)
    {
        uturn_rate_deg_s += CHASSIS_KEY_UTURN_RATE_STEP_DEG_S;
        if (uturn_rate_deg_s > target_rate)
        {
            uturn_rate_deg_s = target_rate;
        }
    }
    else if (uturn_rate_deg_s > target_rate)
    {
        uturn_rate_deg_s -= CHASSIS_KEY_UTURN_RATE_STEP_DEG_S;
        if (uturn_rate_deg_s < target_rate)
        {
            uturn_rate_deg_s = target_rate;
        }
    }
    else
    {
        /* 已在目标角速度上 */
    }

    if ((uturn_stopping != 0u) && (uturn_rate_deg_s == 0.0f))
    {
        /* 收尾：基准取当前实际相对角，跟随环接手时误差为 0，底盘不会动 */
        yaw_reference_rad = Chassis_Input_WrapPi(board.rx_meg->gimbal_meg.yaw_mec);
        uturn_active = 0u;
        uturn_stopping = 0u;
    }
}

/* WASD 平移、QE 旋转，Shift/Ctrl 调速 */
static void Chassis_Input_Keyboard(chassis_cmd_t *cmd, const rc_data_t *rc)
{
    float forward = 0.0f; /* 前后输入 */
    float left = 0.0f;    /* 左右输入 */
    float spin = 0.0f;    /* 旋转输入 */
    float scale = 1.0f;   /* 速度倍率 */
    float wz_key = 0.0f;  /* 机械档鼠标转向量，rad/s */

    if ((rc->key_v & KEY_PRESSED_OFFSET_W) != 0u)
    {
        forward += 0.7f;
    }
    if ((rc->key_v & KEY_PRESSED_OFFSET_S) != 0u)
    {
        forward -= 0.7f;
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

    /* 机械档的平移是纯底盘系，没人再帮它转向，所以基准在后方时这里整体反向，
     * 保证 W 始终朝云台（视线）方向开。
     * 跟随/小陀螺档的平移本来就按云台实际相对角旋转（chassis_follow/spin），
     * 这里不能重复反向，否则两次旋转互相抵消。 */
    if ((keyboard_chassis_mode == CHASSIS_KEY_MODE_MECH) &&
        (Chassis_Input_IsKeyboardYawRear() != 0u))
    {
        forward = -forward;
        left = -left;
    }

    cmd->vx = CHASSIS_KEY_VX_SIGN * forward * scale * CHASSIS_MAX_VX;
    cmd->vy = CHASSIS_KEY_VY_SIGN * left * scale * CHASSIS_MAX_VY;
    cmd->wz = CHASSIS_KEY_WZ_SIGN * spin * scale * CHASSIS_MAX_WZ;

    /* 机械档：鼠标 X 直接转底盘，等价遥控 S1 下位时 ch0 的转向作用。
     * 云台在机械档锁机械零位，底盘一转它就跟着转，视觉上像云台跟随底盘。 */
    if (keyboard_chassis_mode == CHASSIS_KEY_MODE_MECH)
    {
        /* 只对鼠标项限幅，Q/E 的 Shift 加速口径与改动前保持一致，不能被这里压回去 */
        wz_key = constrain(rc->mouse_x * CHASSIS_KEY_MECH_MOUSE_WZ_GAIN,
                           -CHASSIS_KEY_MECH_MOUSE_WZ_MAX,
                           CHASSIS_KEY_MECH_MOUSE_WZ_MAX);
        cmd->wz += CHASSIS_KEY_MECH_MOUSE_WZ_SIGN * wz_key;
    }

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
    keyboard_chassis_mode = CHASSIS_KEY_MODE_FOLLOW;
    last_z_pressed = 0u;
    last_x_pressed = 0u;
    last_c_pressed = 0u;
    yaw_reference_rad = 0.0f;
    last_r_pressed = 0u;
    uturn_active = 0u;
    uturn_stopping = 0u;
    uturn_rate_deg_s = 0.0f;
    uturn_start_yaw_deg = 0.0f;
    uturn_start_tick = 0u;
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
        keyboard_chassis_mode = CHASSIS_KEY_MODE_FOLLOW;
        last_z_pressed = 0u;
        last_x_pressed = 0u;
        last_c_pressed = 0u;
        yaw_reference_rad = 0.0f;
        last_r_pressed = 0u;
        uturn_active = 0u;
        uturn_stopping = 0u;
        uturn_rate_deg_s = 0.0f;
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

    Chassis_Input_KeyboardModeUpdate(rc);
    /* R 掉头（机械档翻基准 / 跟随档起掉头动作）必须在驱动之前更新，
     * 同拍的 WASD 反向才跟得上 */
    Chassis_Input_UturnUpdate(rc);

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
    /* S1 上/下都允许底盘直控，机械模式也保留平移和转向 */
    if ((rc->s1.value == RC_SW_UP) ||
        (rc->s1.value == RC_SW_DOWN))
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
