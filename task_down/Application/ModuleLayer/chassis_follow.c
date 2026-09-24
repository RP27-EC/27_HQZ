/* chassis_follow.c - 底盘跟随 */

#include "chassis_follow.h"

#include <math.h>
#include <stddef.h>

#include "board_protocol.h"
#include "main.h"
#include "rc_sensor.h"
#include "rp_math.h"

#define CHASSIS_FOLLOW_PI          3.14159265358979323846f /* 圆周率 */
#define CHASSIS_FOLLOW_DEG_TO_RAD (CHASSIS_FOLLOW_PI / 180.0f) /* 角度转弧度 */
#define CHASSIS_FOLLOW_BLEND_EPS  0.0001f /* 融合结束阈值 */

chassis_follow_state_t chassis_follow; /* 底盘跟随状态 */

static float follow_last_yaw_rad; /* 上拍 Yaw 误差 */
static float follow_last_wz; /* 上拍输出角速度 */
static uint8_t follow_have_last_yaw; /* 1 = 已有上拍 Yaw */
static uint8_t follow_last_selected; /* 上拍是否选中跟随 */

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

/* 初始化跟随状态和差分缓存 */
void Chassis_Follow_Init(void)
{
    chassis_follow.yaw_mec_rad = 0.0f;
    chassis_follow.yaw_error_rad = 0.0f;
    chassis_follow.wz_target = 0.0f;
    chassis_follow.wz_output = 0.0f;
    chassis_follow.blend = 0.0f;
    chassis_follow.selected = 0u;
    chassis_follow.data_valid = 0u;
    chassis_follow.active = 0u;
    chassis_follow.fault_latched = 0u;
    chassis_follow.turn_direction = 0;

    follow_last_yaw_rad = 0.0f;
    follow_last_wz = 0.0f;
    follow_have_last_yaw = 0u;
    follow_last_selected = 0u;
}

/* S1/S2 上拨选择跟随，云台失联则锁存故障 */
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
    if ((rc_ready != 0u) &&
        (rc_dev.info->s1.value == RC_SW_UP) &&
        (rc_dev.info->s2.value == RC_SW_UP))
    {
        selected = 1u; /* S1/S2 上拨 */
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
    float yaw_error; /* 相对跟随中心误差，rad */
    float auto_wz;   /* 自动跟随输出，rad/s */
    float target_wz; /* 融合目标，rad/s */
    float manual_wz; /* 退出时保留的手动旋转 */
    float vx_gimbal; /* 旋转前纵向速度 */
    float vy_gimbal; /* 旋转前横向速度 */

    if (cmd == NULL)
    {
        return;
    }

    /* 退出跟随：融合回手动旋转，避免角速度跳变 */
    if (chassis_follow.selected == 0u)
    {
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
        follow_have_last_yaw = 0u;
        return;
    }

    follow_last_selected = 1u;

    /* 云台失联或故障时直接封锁底盘输出 */
    if ((chassis_follow.active == 0u) || (chassis_follow.fault_latched != 0u))
    {
        cmd->vx = 0.0f;
        cmd->vy = 0.0f;
        cmd->wz = 0.0f;
        cmd->valid = 0u;

        chassis_follow.wz_output = 0.0f;
        follow_last_wz = 0.0f;
        chassis_follow.turn_direction = 0;
        follow_have_last_yaw = 0u;
        return;
    }

    yaw_mec = board.rx_meg->gimbal_meg.yaw_mec; /* 云台相对角 */
    yaw_error = Chassis_Follow_WrapPi( /* 跟随误差 */
        (CHASSIS_FOLLOW_YAW_ANGLE_SIGN * yaw_mec) - CHASSIS_FOLLOW_CENTER_RAD);
    cmd->source = CHASSIS_SRC_RC_FOLLOW; /* 标记跟随源 */

    if (follow_have_last_yaw != 0u)
    {
        float yaw_delta = Chassis_Follow_WrapPi(yaw_error - follow_last_yaw_rad);

        /* 跳变过大说明反馈异常，锁存故障 */
        if (fabsf(yaw_delta) >
              (CHASSIS_FOLLOW_YAW_JUMP_LIMIT_DEG * CHASSIS_FOLLOW_DEG_TO_RAD))
        {
            chassis_follow.fault_latched = 1u;
            chassis_follow.active = 0u;
            chassis_follow.data_valid = 0u;
            cmd->vx = 0.0f;
            cmd->vy = 0.0f;
            cmd->wz = 0.0f;
            cmd->valid = 0u;
            follow_last_wz = 0.0f;
            chassis_follow.turn_direction = 0;
            follow_have_last_yaw = 0u;
            return;
        }
    }

    follow_last_yaw_rad = yaw_error; /* 缓存上拍误差 */
    follow_have_last_yaw = 1u;
    chassis_follow.yaw_mec_rad = yaw_mec;
    chassis_follow.yaw_error_rad = yaw_error;

    /* 跟随误差存在时，平移按云台方向旋转 */
#if CHASSIS_FOLLOW_TRANSLATION_ENABLE
    vx_gimbal = cmd->vx;
    vy_gimbal = cmd->vy;

    cmd->vx = CHASSIS_FOLLOW_TRANSLATION_SIGN * /* 旋转平移 */
              ((cosf(yaw_error) * vx_gimbal) - (sinf(yaw_error) * vy_gimbal));
    cmd->vy = CHASSIS_FOLLOW_TRANSLATION_SIGN * /* 旋转平移 */
              ((sinf(yaw_error) * vx_gimbal) + (cosf(yaw_error) * vy_gimbal));
#else
    (void)vx_gimbal;
    (void)vy_gimbal;
#endif

      if (fabsf(yaw_error) < (CHASSIS_FOLLOW_DEADBAND_DEG * CHASSIS_FOLLOW_DEG_TO_RAD))
    {
        chassis_follow.turn_direction = 0;
        auto_wz = 0.0f;
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
            auto_wz = CHASSIS_FOLLOW_WZ_SIGN *
                      (float)chassis_follow.turn_direction *
                      ((CHASSIS_FOLLOW_KP * abs_error) + CHASSIS_FOLLOW_FRICTION_FF);
        }
        else
        {
            auto_wz = CHASSIS_FOLLOW_WZ_SIGN *
                      ((CHASSIS_FOLLOW_KP * yaw_error) +
                       (CHASSIS_FOLLOW_FRICTION_FF * (float)error_direction));
        }

        auto_wz = constrain(auto_wz, /* 输出限幅 */
                            -CHASSIS_FOLLOW_MAX_WZ,
                            CHASSIS_FOLLOW_MAX_WZ);
    }

    chassis_follow.blend += CHASSIS_FOLLOW_BLEND_STEP; /* 逐步接管 */
    if (chassis_follow.blend > 1.0f)
    {
        chassis_follow.blend = 1.0f;
    }

    target_wz = auto_wz * chassis_follow.blend; /* 融合旋转 */
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

