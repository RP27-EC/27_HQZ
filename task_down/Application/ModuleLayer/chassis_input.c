/* chassis_input.c - 底盘输入解析 */

#include "chassis_input.h"

#include <math.h>

#include "board_protocol.h"
#include "board_comm_config.h"
#include "control_task.h"
#include "main.h"
#include "rc_sensor.h"
#include "rp_math.h"

#define CHASSIS_INPUT_PI      3.14159265358979323846f /* 圆周率 */
#define CHASSIS_INPUT_DEG_TO_RAD (CHASSIS_INPUT_PI / 180.0f)

chassis_cmd_t chassis_input_cmd; /* 输入解析后的底盘指令 */

static uint8_t keyboard_source_active; /* 键鼠输入源已开启 */
static uint8_t last_f_pressed; /* 上拍 F 键状态 */
static chassis_key_mode_e keyboard_chassis_mode; /* 键鼠底盘模式 */
static uint8_t last_z_pressed; /* Z 键上次状态 */
static uint8_t last_x_pressed; /* X 键上次状态 */
static uint8_t last_c_pressed; /* C 键上次状态 */

/* 超时仅移动跟随中心，固定前后方向独立保存。 */
static float yaw_reference_rad;
static volatile uint8_t yaw_rear;
static uint8_t last_r_pressed;
static volatile chassis_uturn_state_e uturn_state;
static chassis_uturn_result_e uturn_result;
static uint32_t uturn_start_tick;
static uint32_t uturn_phase_tick;
static uint32_t uturn_last_feedback_tick;
static uint32_t uturn_stable_start_tick;
static uint8_t uturn_stable_started;
static volatile uint8_t uturn_handoff_cycles;
static volatile uint8_t uturn_handoff_tx_started;
static volatile uint32_t uturn_handoff_tx_tick;

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

float Chassis_Input_GetYawReferenceRad(void)
{
    return yaw_reference_rad;
}

static void Chassis_Input_CancelUturn(void)
{
    if (uturn_state != CHASSIS_UTURN_IDLE)
    {
        uturn_result = CHASSIS_UTURN_CANCELLED;
    }
    uturn_state = CHASSIS_UTURN_IDLE;
    uturn_stable_started = 0u;
    uturn_handoff_cycles = 0u;
    uturn_handoff_tx_started = 0u;
}

void Chassis_Input_ResetYawReference(void)
{
    Chassis_Input_CancelUturn();
    yaw_reference_rad = BOARD_MEC_YAW_FRONT_RAD;
    yaw_rear = 0u;
}

uint8_t Chassis_Input_IsKeyboardYawRear(void)
{
    return yaw_rear;
}

float Chassis_Input_GetKeyboardYawTargetRad(void)
{
    return (yaw_rear != 0u) ? BOARD_MEC_YAW_REAR_RAD : BOARD_MEC_YAW_FRONT_RAD;
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

uint8_t Chassis_Input_IsUturnActive(void)
{
    return (uturn_state != CHASSIS_UTURN_IDLE) ? 1u : 0u;
}

chassis_uturn_state_e Chassis_Input_GetUturnState(void)
{
    return uturn_state;
}

chassis_uturn_result_e Chassis_Input_GetUturnResult(void)
{
    return uturn_result;
}

void Chassis_Input_NotifyUturnTxCycle(uint8_t gimbal_mode, uint16_t yaw_target_raw)
{
    uint32_t now = HAL_GetTick();

    if (((uturn_state != CHASSIS_UTURN_PREPARE) &&
         (uturn_state != CHASSIS_UTURN_RESTORE)) || (gimbal_mode == 0u) ||
        (yaw_target_raw != float_to_uint(Chassis_Input_GetKeyboardYawTargetRad(), -4.0f, 4.0f, 16u)))
    {
        return;
    }

    if ((uturn_handoff_tx_started == 0u) ||
        ((now - uturn_handoff_tx_tick) >= BOARD_COMM_D1D2_PERIOD_MS))
    {
        uturn_handoff_tx_tick = now;
        uturn_handoff_tx_started = 1u;
        if (uturn_handoff_cycles < CHASSIS_KEY_UTURN_HANDOFF_PERIODS)
        {
            uturn_handoff_cycles++;
        }
    }
}

static void Chassis_Input_UturnUpdate(const rc_data_t *rc)
{
    uint32_t now = HAL_GetTick();
    uint32_t feedback_tick = 0u;
    uint32_t handoff_ms = CHASSIS_KEY_UTURN_HANDOFF_PERIODS * BOARD_COMM_D1D2_PERIOD_MS;
    uint8_t r_pressed = ((rc->key_v & KEY_PRESSED_OFFSET_R) != 0u) ? 1u : 0u;
    uint8_t r_edge = ((r_pressed != 0u) && (last_r_pressed == 0u)) ? 1u : 0u;
    uint8_t was_active = Chassis_Input_IsUturnActive();
    uint8_t data_valid = 0u;
    float yaw_mec = 0.0f;
    float error_rad;

    last_r_pressed = r_pressed;
    if (Chassis_Input_IsKeyboardMode() == 0u)
    {
        Chassis_Input_ResetYawReference();
        return;
    }

    if ((board_hole_request != 0u) || (board_hole_exit_pending != 0u) ||
        ((rc->s2.value == RC_SW_MID) && (rc->B.value != 0u)))
    {
        Chassis_Input_CancelUturn();
        return;
    }

    if ((board.status != NULL) && (board.rx_meg != NULL) &&
        (board.status->gimbal_data_valid != 0u))
    {
        feedback_tick = board.status->gimbal_rx_time_ms;
        yaw_mec = board.rx_meg->gimbal_meg.yaw_mec;
        if (((now - feedback_tick) <= CHASSIS_KEY_UTURN_FEEDBACK_TIMEOUT_MS) &&
            (yaw_mec == yaw_mec) && (fabsf(yaw_mec) <= (CHASSIS_INPUT_PI + 0.01f)))
        {
            data_valid = 1u;
        }
    }

    if (keyboard_chassis_mode != CHASSIS_KEY_MODE_FOLLOW)
    {
        Chassis_Input_CancelUturn();
        if ((keyboard_chassis_mode == CHASSIS_KEY_MODE_MECH) &&
            (r_edge != 0u) && (was_active == 0u))
        {
            yaw_rear ^= 1u;
        }
        if (keyboard_chassis_mode == CHASSIS_KEY_MODE_MECH)
        {
            yaw_reference_rad = Chassis_Input_GetKeyboardYawTargetRad();
        }
        return;
    }

    if (data_valid == 0u)
    {
        Chassis_Input_CancelUturn();
        return;
    }

    if ((r_edge != 0u) && (was_active == 0u))
    {
        yaw_rear ^= 1u;
        uturn_handoff_cycles = 0u;
        uturn_handoff_tx_started = 0u;
        uturn_state = CHASSIS_UTURN_PREPARE;
        uturn_result = CHASSIS_UTURN_RUNNING;
        uturn_start_tick = now;
        uturn_phase_tick = now;
        uturn_last_feedback_tick = feedback_tick;
        uturn_stable_started = 0u;
    }

    if (uturn_state == CHASSIS_UTURN_PREPARE)
    {
        /* NOTE: 时间与发送计数同时满足才交接。 */
        if (((now - uturn_phase_tick) >= handoff_ms) &&
            (uturn_handoff_cycles >= CHASSIS_KEY_UTURN_HANDOFF_PERIODS))
        {
            uturn_state = CHASSIS_UTURN_POSITION;
            uturn_last_feedback_tick = feedback_tick;
        }
    }
    else if (uturn_state == CHASSIS_UTURN_POSITION)
    {
        if (feedback_tick != uturn_last_feedback_tick)
        {
            if ((feedback_tick - uturn_last_feedback_tick) > CHASSIS_KEY_UTURN_FEEDBACK_TIMEOUT_MS)
            {
                uturn_stable_started = 0u;
            }
            uturn_last_feedback_tick = feedback_tick;
            error_rad = Chassis_Input_WrapPi(Chassis_Input_GetKeyboardYawTargetRad() - yaw_mec);
            if (fabsf(error_rad) <= (CHASSIS_KEY_UTURN_TOL_DEG * CHASSIS_INPUT_DEG_TO_RAD))
            {
                if (uturn_stable_started == 0u)
                {
                    uturn_stable_start_tick = feedback_tick;
                    uturn_stable_started = 1u;
                }
                if ((feedback_tick - uturn_stable_start_tick) >= CHASSIS_KEY_UTURN_STABLE_MS)
                {
                    yaw_reference_rad = Chassis_Input_GetKeyboardYawTargetRad();
                    uturn_result = CHASSIS_UTURN_DONE;
                    uturn_handoff_cycles = 0u;
                    uturn_handoff_tx_started = 0u;
                    uturn_state = CHASSIS_UTURN_RESTORE;
                    uturn_phase_tick = now;
                }
            }
            else
            {
                uturn_stable_started = 0u;
            }
        }
    }

    if (((uturn_state == CHASSIS_UTURN_PREPARE) ||
         (uturn_state == CHASSIS_UTURN_POSITION)) &&
        ((now - uturn_start_tick) >= CHASSIS_KEY_UTURN_TIMEOUT_MS))
    {
        yaw_reference_rad = Chassis_Input_WrapPi(yaw_mec);
        uturn_result = CHASSIS_UTURN_TIMEOUT;
        uturn_handoff_cycles = 0u;
        uturn_handoff_tx_started = 0u;
        uturn_state = CHASSIS_UTURN_RESTORE;
        uturn_phase_tick = now;
    }

    if ((uturn_state == CHASSIS_UTURN_RESTORE) &&
        ((now - uturn_phase_tick) >= handoff_ms) &&
        (uturn_handoff_cycles >= CHASSIS_KEY_UTURN_HANDOFF_PERIODS))
    {
        uturn_state = CHASSIS_UTURN_IDLE;
        uturn_stable_started = 0u;
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
    yaw_reference_rad = BOARD_MEC_YAW_FRONT_RAD;
    yaw_rear = 0u;
    last_r_pressed = 0u;
    uturn_state = CHASSIS_UTURN_IDLE;
    uturn_result = CHASSIS_UTURN_NONE;
    uturn_start_tick = 0u;
    uturn_phase_tick = 0u;
    uturn_last_feedback_tick = 0u;
    uturn_stable_start_tick = 0u;
    uturn_stable_started = 0u;
    uturn_handoff_cycles = 0u;
    uturn_handoff_tx_started = 0u;
    uturn_handoff_tx_tick = 0u;
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
        Chassis_Input_ResetYawReference();
        last_r_pressed = (rc != NULL && (rc->key_v & KEY_PRESSED_OFFSET_R) != 0u) ? 1u : 0u;
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
