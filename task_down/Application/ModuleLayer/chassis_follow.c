/* chassis_follow.c - 底盘跟随 */

#include "chassis_follow.h"

#include <math.h>
#include <stddef.h>

#include "board_protocol.h"
#include "chassis_input.h"
#include "chassis_spin.h"
#include "main.h"
#include "rc_sensor.h"
#include "rp_math.h"

#define CHASSIS_FOLLOW_PI          3.14159265358979323846f /* 圆周率 */
#define CHASSIS_FOLLOW_DEG_TO_RAD (CHASSIS_FOLLOW_PI / 180.0f) /* 角度转弧度 */
#define CHASSIS_FOLLOW_BLEND_EPS  0.0001f /* 融合结束阈值 */

chassis_follow_state_t chassis_follow; /* 底盘跟随状态 */

static float follow_last_yaw_rot; /* 上拍云台实际相对角（含符号），rad */
static float follow_last_wz; /* 上拍输出角速度 */
static uint8_t follow_have_last_yaw; /* 1 = 已有上拍 Yaw */
static uint8_t follow_last_selected; /* 上拍是否选中跟随 */
static uint8_t follow_last_spin_selected;
static uint8_t follow_stable_timing;
static uint32_t follow_stable_since_ms;
static uint32_t follow_recovery_since_ms;

static void Chassis_Follow_ResetRecovery(void)
{
    chassis_follow.correction_stopped = 0u;
    chassis_follow.disturbance_armed = 0u;
    chassis_follow.recovery_pending = 0u;
    chassis_follow.recovery_active = 0u;
    chassis_follow.recovery_ff = 0.0f;
    follow_stable_timing = 0u;
    follow_stable_since_ms = 0u;
    follow_recovery_since_ms = 0u;
}

/* 原始输入覆盖缓存延迟。 */
static uint8_t Chassis_Follow_ManualTurn(const chassis_cmd_t *cmd)
{
    if ((fabsf(cmd->wz) > CHASSIS_FOLLOW_RECOVERY_INPUT_EPS) ||
        (fabsf(board_manual_yaw_rate_deg_s) > CHASSIS_FOLLOW_RECOVERY_RATE_EPS))
    {
        return 1u;
    }
    if ((rc_dev.work_state != DEV_ONLINE) || (rc_dev.info == NULL))
    {
        return 1u;
    }
#if CHASSIS_KEYBOARD_INPUT_ENABLE
    if (Chassis_Input_IsKeyboardMode() != 0u)
    {
        return (rc_dev.info->mouse_x != 0) ? 1u : 0u;
    }
#endif
    return (fabsf((float)rc_dev.info->ch0) > BOARD_RC_AXIS_DEADBAND) ? 1u : 0u;
}

static uint8_t Chassis_Follow_WheelsStopped(void)
{
    uint8_t i;

    if ((chassis_ctrl.state.enabled == 0u) || (chassis_ctrl.state.all_online == 0u) ||
        (chassis_ctrl.state.fault != 0u))
    {
        return 0u;
    }
    for (i = 0u; i < WHEEL_CNT; ++i)
    {
        if (!(fabsf(chassis_ctrl.state.wheel_speed[i]) < CHASSIS_STOP_SPEED_BAND))
        {
            return 0u;
        }
    }
    return 1u;
}

/* 停稳后才允许受扰辅助。 */
static void Chassis_Follow_UpdateRecovery(const chassis_cmd_t *cmd,
                                         float abs_error_deg,
                                         uint8_t manual_turn,
                                         uint8_t spin_tail,
                                         uint8_t center_changed)
{
    uint32_t now = HAL_GetTick();
    uint8_t translating = ((fabsf(cmd->vx) > CHASSIS_FOLLOW_RECOVERY_INPUT_EPS) ||
                           (fabsf(cmd->vy) > CHASSIS_FOLLOW_RECOVERY_INPUT_EPS)) ? 1u : 0u;

    if ((manual_turn != 0u) || (Chassis_Input_IsUturnActive() != 0u) ||
        (center_changed != 0u) || (cmd->valid == 0u) ||
        (chassis_ctrl.state.enabled == 0u) || (chassis_ctrl.state.all_online == 0u) ||
        (chassis_ctrl.state.fault != 0u))
    {
        Chassis_Follow_ResetRecovery();
        return;
    }

    if (abs_error_deg <= CHASSIS_FOLLOW_DEADBAND_DEG)
    {
        chassis_follow.correction_stopped = 1u;
        chassis_follow.recovery_active = 0u;
    }
    else if (abs_error_deg >= CHASSIS_FOLLOW_RESUME_DEG)
    {
        chassis_follow.correction_stopped = 0u;
    }

    if (chassis_follow.recovery_pending != 0u)
    {
        if ((now - follow_recovery_since_ms) >= CHASSIS_FOLLOW_RECOVERY_TIMEOUT_MS)
        {
            chassis_follow.recovery_pending = 0u;
        }
        else if (spin_tail == 0u)
        {
            chassis_follow.recovery_pending = 0u;
            if (chassis_follow.correction_stopped == 0u)
            {
                chassis_follow.recovery_active = 1u;
                follow_recovery_since_ms = now;
            }
        }
    }

    if (spin_tail != 0u)
    {
        chassis_follow.disturbance_armed = 0u;
        follow_stable_timing = 0u;
        return;
    }

    if (translating != 0u)
    {
        chassis_follow.disturbance_armed = 0u;
        follow_stable_timing = 0u;
    }
    else if ((chassis_follow.disturbance_armed != 0u) &&
             (abs_error_deg >= CHASSIS_FOLLOW_RECOVERY_TRIGGER_DEG))
    {
        chassis_follow.disturbance_armed = 0u;
        chassis_follow.recovery_active = 1u;
        follow_recovery_since_ms = now;
        follow_stable_timing = 0u;
    }
    else if ((abs_error_deg <= CHASSIS_FOLLOW_DEADBAND_DEG) &&
             (chassis_follow.correction_stopped != 0u) &&
             (fabsf(chassis_follow.wz_output) <= CHASSIS_FOLLOW_RECOVERY_INPUT_EPS) &&
             (Chassis_Follow_WheelsStopped() != 0u))
    {
        if (follow_stable_timing == 0u)
        {
            follow_stable_timing = 1u;
            follow_stable_since_ms = now;
        }
        else if ((now - follow_stable_since_ms) >= CHASSIS_FOLLOW_RECOVERY_STABLE_MS)
        {
            chassis_follow.disturbance_armed = 1u;
        }
    }
    else
    {
        follow_stable_timing = 0u;
    }

    if ((chassis_follow.recovery_active != 0u) &&
        ((now - follow_recovery_since_ms) >= CHASSIS_FOLLOW_RECOVERY_TIMEOUT_MS))
    {
        chassis_follow.recovery_active = 0u;
    }
}

/* 前馈在停止门限处收零。 */
static float Chassis_Follow_RecoveryFeedforward(float yaw_error)
{
    float abs_error_deg = fabsf(yaw_error) / CHASSIS_FOLLOW_DEG_TO_RAD;
    float fade;
    float direction;

    if ((chassis_follow.recovery_active == 0u) ||
        (chassis_follow.correction_stopped != 0u) ||
        (abs_error_deg <= CHASSIS_FOLLOW_DEADBAND_DEG))
    {
        return 0.0f;
    }

    fade = constrain(
        (abs_error_deg - CHASSIS_FOLLOW_DEADBAND_DEG) /
        (CHASSIS_FOLLOW_RECOVERY_FULL_DEG - CHASSIS_FOLLOW_DEADBAND_DEG),
        0.0f, 1.0f);
    direction = (chassis_follow.turn_direction != 0) ?
                (float)chassis_follow.turn_direction : ((yaw_error >= 0.0f) ? 1.0f : -1.0f);

    return direction * CHASSIS_FOLLOW_RECOVERY_FF_WZ * fade;
}

/* 将角度归一化到 [-pi, pi] */
static float Chassis_Follow_WrapPi(float angle)
{
    return atan2f(sinf(angle), cosf(angle));
}

/* 按步长斜坡到目标值 */
static float Chassis_Follow_Ramp(float current, float target, float step)
{
    float diff = target - current;

    if (step <= 0.0f)
    {
        return target;
    }

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

/* 排除 NaN 和越界角度 */
static uint8_t Chassis_Follow_YawValid(float yaw_rad)
{
    return ((yaw_rad == yaw_rad) && (fabsf(yaw_rad) <= (CHASSIS_FOLLOW_PI + 0.01f))) ? 1u : 0u;
}

/* 云台反馈必须在线且未超时 */
static uint8_t Chassis_Follow_DataValid(void)
{
    uint32_t now; /* 当前时刻，ms */
    uint32_t age; /* 反馈年龄，ms */

    if (board.status == NULL)
    {
        return 0u;
    }

    if (board.status->gimbal_data_valid == 0u)
    {
        return 0u;
    }

    now = HAL_GetTick();
    age = now - board.status->gimbal_rx_time_ms;

    if (age > CHASSIS_FOLLOW_TIMEOUT_MS)
    {
        return 0u;
    }

    return 1u;
}

/* 当前跟随中心：键鼠掉头后取掉头基准（0 或 180deg），否则用配置中心。
 * 基准由 chassis_input.c 维护，退出键鼠时它会复位回 0。 */
static float Chassis_Follow_CenterRad(void)
{
#if CHASSIS_FOLLOW_YAW_REFERENCE_ENABLE
    return Chassis_Input_GetYawReferenceRad();
#else
    return CHASSIS_FOLLOW_CENTER_RAD;
#endif
}

/* 初始化跟随状态和差分缓存 */
void Chassis_Follow_Init(void)
{
    chassis_follow.yaw_mec_rad = 0.0f;
    chassis_follow.yaw_error_rad = 0.0f;
    chassis_follow.center_rad = CHASSIS_FOLLOW_CENTER_RAD;
    chassis_follow.wz_target = 0.0f;
    chassis_follow.wz_output = 0.0f;
    chassis_follow.manual_yaw_rate = 0.0f;
    chassis_follow.command_ff = 0.0f;
    chassis_follow.blend = 0.0f;
    chassis_follow.selected = 0u;
    chassis_follow.data_valid = 0u;
    chassis_follow.active = 0u;
    chassis_follow.fault_latched = 0u;
    chassis_follow.turn_direction = 0;

    follow_last_yaw_rot = 0.0f;
    follow_last_wz = 0.0f;
    follow_have_last_yaw = 0u;
    follow_last_selected = 0u;
    follow_last_spin_selected = 0u;
    Chassis_Follow_ResetRecovery();
}

/* S1 上拨且 S2 上/中拨选择跟随，云台失联则锁存故障 */
void Chassis_Follow_UpdateMode(void)
{
    uint8_t rc_ready;      /* 遥控器在线且数据有效 */
    uint8_t selected = 0u; /* 跟随档位选择 */

    rc_ready = ((rc_dev.work_state == DEV_ONLINE) && (rc_dev.info != NULL)) ? 1u : 0u; /* 遥控可用 */

    if ((rc_ready == 0u) || (rc_dev.info->s1.value != RC_SW_UP))
    {
        chassis_follow.fault_latched = 0u; /* 退出后清故障 */
        chassis_follow.turn_direction = 0;
        follow_have_last_yaw = 0u;
    }

#if CHASSIS_GIMBAL_FOLLOW_ENABLE
#if CHASSIS_KEYBOARD_INPUT_ENABLE
    if (Chassis_Input_IsKeyboardMode() != 0u)
    {
        selected = (Chassis_Input_GetKeyboardChassisMode() == CHASSIS_KEY_MODE_FOLLOW) ? 1u : 0u;
    }
    else
#endif
    if ((rc_ready != 0u) &&
        (rc_dev.info->s1.value == RC_SW_UP) &&
        ((rc_dev.info->s2.value == RC_SW_UP) ||
         (rc_dev.info->s2.value == RC_SW_MID)))
    {
        selected = 1u; /* S1 上拨且 S2 上/中拨 */
    }
#else
    (void)rc_ready;
#endif

    chassis_follow.selected = selected; /* 本拍选择 */
    chassis_follow.active = 0u;
    chassis_follow.data_valid = 0u;

    if (selected == 0u)
    {
        chassis_follow.turn_direction = 0;
        return;
    }

    if ((Chassis_Follow_DataValid() == 0u) ||
        (Chassis_Follow_YawValid(board.rx_meg->gimbal_meg.yaw_mec) == 0u))
    {
        chassis_follow.fault_latched = 1u; /* 锁存故障 */
        chassis_follow.turn_direction = 0;
        follow_have_last_yaw = 0u;
        return;
    }

    chassis_follow.data_valid = 1u; /* 云台数据有效 */

    if (chassis_follow.fault_latched == 0u)
    {
        chassis_follow.active = 1u;
    }
}

/* 将云台相对角转为底盘旋转速度，并旋转平移分量 */
void Chassis_Follow_Update(chassis_cmd_t *cmd)
{
    float yaw_mec;   /* 云台机械角，rad */
    float yaw_rot;   /* 实际相对角，rad */
    float yaw_error; /* 相对跟随中心误差，rad */
    float center_rad;/* 本拍跟随中心，rad */
    float auto_wz;   /* 自动跟随转速控制量 */
    float target_wz; /* 融合后转速控制量 */
    float manual_wz; /* 手动转速控制量 */
    float vx_gimbal; /* 旋转前纵向速度 */
    float vy_gimbal; /* 旋转前横向速度 */
    float feedback_wz; /* 角度反馈项 */
    float command_ff; /* 指令前馈项 */
    uint8_t manual_turn;
    uint8_t spin_tail;
    uint8_t center_changed;

    if (cmd == NULL)
    {
        return;
    }

    if ((follow_last_spin_selected != 0u) && (chassis_spin.selected == 0u) &&
        (chassis_follow.selected != 0u))
    {
        chassis_follow.recovery_pending = 1u;
        follow_recovery_since_ms = HAL_GetTick();
    }
    follow_last_spin_selected = chassis_spin.selected;

    /* 退出跟随：融合回手动旋转，避免角速度跳变 */
    if (chassis_follow.selected == 0u)
    {
        Chassis_Follow_ResetRecovery();
        if (follow_last_selected != 0u)
        {
            manual_wz = cmd->wz; /* 保留手动旋转 */
            chassis_follow.blend -= CHASSIS_FOLLOW_BLEND_STEP;

            if (chassis_follow.blend < 0.0f)
            {
                chassis_follow.blend = 0.0f;
            }

            target_wz = manual_wz * (1.0f - chassis_follow.blend); /* 退出融合 */
            cmd->wz = Chassis_Follow_Ramp(follow_last_wz, target_wz, CHASSIS_FOLLOW_WZ_STEP);
            follow_last_wz = cmd->wz;
            follow_last_selected = (chassis_follow.blend > CHASSIS_FOLLOW_BLEND_EPS) ? 1u : 0u;
        }
        else
        {
            follow_last_wz = cmd->wz;
        }

        chassis_follow.wz_output = cmd->wz;
        chassis_follow.manual_yaw_rate = 0.0f;
        chassis_follow.command_ff = 0.0f;
        follow_have_last_yaw = 0u;
        return;
    }

    follow_last_selected = 1u;

    /* 云台失联或故障时直接封锁底盘输出 */
    if ((chassis_follow.active == 0u) || (chassis_follow.fault_latched != 0u))
    {
        Chassis_Follow_ResetRecovery();
        cmd->vx = 0.0f;
        cmd->vy = 0.0f;
        cmd->wz = 0.0f;
        cmd->valid = 0u;

        chassis_follow.wz_output = 0.0f;
        follow_last_wz = 0.0f;
        chassis_follow.manual_yaw_rate = 0.0f;
        chassis_follow.command_ff = 0.0f;
        chassis_follow.turn_direction = 0;
        follow_have_last_yaw = 0u;
        return;
    }

    yaw_mec = board.rx_meg->gimbal_meg.yaw_mec; /* 云台相对角 */
    center_rad = Chassis_Follow_CenterRad(); /* 跟随中心 = 掉头基准 */
    center_changed = ((follow_have_last_yaw != 0u) &&
                      (fabsf(Chassis_Follow_WrapPi(center_rad - chassis_follow.center_rad)) >
                       CHASSIS_FOLLOW_BLEND_EPS)) ? 1u : 0u;
    yaw_rot = Chassis_Follow_WrapPi( /* 平移旋转角：始终按实际相对角 */
        CHASSIS_FOLLOW_YAW_ANGLE_SIGN * yaw_mec);
    yaw_error = Chassis_Follow_WrapPi( /* 跟随误差 = 旋转角 - 中心 */
        yaw_rot - center_rad);
    chassis_follow.manual_yaw_rate = board_manual_yaw_rate_deg_s;
    command_ff = CHASSIS_FOLLOW_RATE_FF * CHASSIS_FOLLOW_RATE_PER_DEG_S *
                 chassis_follow.manual_yaw_rate;
    chassis_follow.command_ff = command_ff;
    cmd->source = CHASSIS_SRC_RC_FOLLOW; /* 标记跟随源 */

    if (follow_have_last_yaw != 0u)
    {
        /* 跳变判据排除软件基准。 */
        float yaw_delta = Chassis_Follow_WrapPi(yaw_rot - follow_last_yaw_rot);

        /* 跳变过大说明反馈异常，锁存故障 */
        if (fabsf(yaw_delta) >
              (CHASSIS_FOLLOW_YAW_JUMP_LIMIT_DEG * CHASSIS_FOLLOW_DEG_TO_RAD))
        {
            Chassis_Follow_ResetRecovery();
            chassis_follow.fault_latched = 1u;
            chassis_follow.active = 0u;
            chassis_follow.data_valid = 0u;
            cmd->vx = 0.0f;
            cmd->vy = 0.0f;
            cmd->wz = 0.0f;
            cmd->valid = 0u;
            follow_last_wz = 0.0f;
            chassis_follow.manual_yaw_rate = 0.0f;
            chassis_follow.command_ff = 0.0f;
            chassis_follow.turn_direction = 0;
            follow_have_last_yaw = 0u;
            return;
        }
    }

    follow_last_yaw_rot = yaw_rot; /* 缓存上拍实际相对角 */
    follow_have_last_yaw = 1u;
    chassis_follow.yaw_mec_rad = yaw_mec;
    chassis_follow.yaw_error_rad = yaw_error;
    chassis_follow.center_rad = center_rad;

    /* 平移始终使用实际相对角。 */
#if CHASSIS_FOLLOW_TRANSLATION_ENABLE
    vx_gimbal = cmd->vx;
    vy_gimbal = cmd->vy;

    cmd->vx = CHASSIS_FOLLOW_TRANSLATION_SIGN * /* 旋转平移 */
              ((cosf(yaw_rot) * vx_gimbal) - (sinf(yaw_rot) * vy_gimbal));
    cmd->vy = CHASSIS_FOLLOW_TRANSLATION_SIGN * /* 旋转平移 */
              ((sinf(yaw_rot) * vx_gimbal) + (cosf(yaw_rot) * vy_gimbal));
#else
    (void)vx_gimbal;
    (void)vy_gimbal;
    (void)yaw_rot;
#endif

    manual_turn = Chassis_Follow_ManualTurn(cmd);
    spin_tail = (fabsf(chassis_spin.output_wz) > CHASSIS_FOLLOW_RECOVERY_SPIN_EPS) ? 1u : 0u;
    Chassis_Follow_UpdateRecovery(cmd, fabsf(yaw_error) / CHASSIS_FOLLOW_DEG_TO_RAD,
                                 manual_turn, spin_tail, center_changed);

    /* 旋转减速期间不积累跟随斜坡。 */
    if (spin_tail != 0u)
    {
        follow_last_wz = chassis_spin.output_wz;
    }

    if ((chassis_follow.correction_stopped != 0u) ||
        (fabsf(yaw_error) <= (CHASSIS_FOLLOW_DEADBAND_DEG * CHASSIS_FOLLOW_DEG_TO_RAD)))
    {
        chassis_follow.turn_direction = 0;
        feedback_wz = 0.0f;
    }
    else
    {
        float abs_error = fabsf(yaw_error); /* 误差幅值 */
        int8_t error_direction = (yaw_error >= 0.0f) ? 1 : -1; /* 误差方向 */

        /* 大误差锁定转向，小误差释放，抑制中心抖动 */
        if (abs_error >
            (CHASSIS_FOLLOW_TURN_LOCK_DEG * CHASSIS_FOLLOW_DEG_TO_RAD))
        {
            if (chassis_follow.turn_direction == 0)
            {
                chassis_follow.turn_direction = error_direction;
            }
        }
        else if (abs_error <
                 (CHASSIS_FOLLOW_TURN_UNLOCK_DEG * CHASSIS_FOLLOW_DEG_TO_RAD))
        {
            chassis_follow.turn_direction = 0;
        }

        if (chassis_follow.turn_direction != 0)
        {
            feedback_wz = (float)chassis_follow.turn_direction *
                          ((CHASSIS_FOLLOW_KP * abs_error) + CHASSIS_FOLLOW_FRICTION_FF);
        }
        else
        {
            feedback_wz = (CHASSIS_FOLLOW_KP * yaw_error) +
                          (CHASSIS_FOLLOW_FRICTION_FF * (float)error_direction);
        }

    }

    chassis_follow.recovery_ff = Chassis_Follow_RecoveryFeedforward(yaw_error);
    auto_wz = CHASSIS_FOLLOW_WZ_SIGN *
              (feedback_wz + command_ff + chassis_follow.recovery_ff);
    auto_wz = constrain(auto_wz,
                        -CHASSIS_FOLLOW_MAX_WZ,
                        CHASSIS_FOLLOW_MAX_WZ);

    /* 自动定位不接管 QE，鼠标 Yaw 已在 D5 屏蔽。 */
    if (Chassis_Input_IsUturnActive() != 0u)
    {
        auto_wz = 0.0f;
        chassis_follow.command_ff = 0.0f;
        chassis_follow.turn_direction = 0;
    }

    chassis_follow.blend += CHASSIS_FOLLOW_BLEND_STEP; /* 逐步接管 */
    if (chassis_follow.blend > 1.0f)
    {
        chassis_follow.blend = 1.0f;
    }

    target_wz = (Chassis_Input_IsUturnActive() != 0u) ?
                cmd->wz : auto_wz * chassis_follow.blend;
    cmd->wz = Chassis_Follow_Ramp(follow_last_wz, target_wz, CHASSIS_FOLLOW_WZ_STEP);

    chassis_follow.wz_target = auto_wz; /* 自动目标 */
    chassis_follow.wz_output = cmd->wz; /* 最终输出 */
    follow_last_wz = cmd->wz;
}

/* 跟随档位是否选中 */
uint8_t Chassis_Follow_IsSelected(void)
{
    return chassis_follow.selected;
}

/* 跟随闭环是否生效 */
uint8_t Chassis_Follow_IsActive(void)
{
    return chassis_follow.active;
}

/* 是否锁存跟随故障 */
uint8_t Chassis_Follow_HasFault(void)
{
    return chassis_follow.fault_latched;
}

