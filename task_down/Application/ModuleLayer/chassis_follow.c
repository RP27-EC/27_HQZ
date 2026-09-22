/* chassis_follow.c - 底盘跟随 */

#include "chassis_follow.h"

#include <math.h>
#include <stddef.h>

#include "board_protocol.h"
#include "main.h"
#include "rc_sensor.h"
#include "rp_math.h"

#define CHASSIS_FOLLOW_PI          3.14159265358979323846f
#define CHASSIS_FOLLOW_DEG_TO_RAD (CHASSIS_FOLLOW_PI / 180.0f)
#define CHASSIS_FOLLOW_BLEND_EPS  0.0001f

chassis_follow_state_t chassis_follow;

static float follow_last_yaw_rad;
static float follow_last_wz;
static uint8_t follow_have_last_yaw;
static uint8_t follow_last_selected;

static float Chassis_Follow_WrapPi(float angle)
{
    return atan2f(sinf(angle), cosf(angle));
}

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

static uint8_t Chassis_Follow_YawValid(float yaw_rad)
{
    return ((yaw_rad == yaw_rad) && (fabsf(yaw_rad) <= (CHASSIS_FOLLOW_PI + 0.01f))) ? 1u : 0u;
}

static uint8_t Chassis_Follow_DataValid(void)
{
    uint32_t now;
    uint32_t age;

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

void Chassis_Follow_UpdateMode(void)
{
    uint8_t rc_ready;
    uint8_t selected = 0u;

    rc_ready = ((rc_dev.work_state == DEV_ONLINE) && (rc_dev.info != NULL)) ? 1u : 0u;

    if ((rc_ready == 0u) || (rc_dev.info->s1.value != RC_SW_UP))
    {
        chassis_follow.fault_latched = 0u;
        chassis_follow.turn_direction = 0;
        follow_have_last_yaw = 0u;
    }

#if CHASSIS_GIMBAL_FOLLOW_ENABLE
    if ((rc_ready != 0u) &&
        (rc_dev.info->s1.value == RC_SW_UP) &&
        (rc_dev.info->s2.value == RC_SW_UP))
    {
        selected = 1u;
    }
#else
    (void)rc_ready;
#endif

    chassis_follow.selected = selected;
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
        chassis_follow.fault_latched = 1u;
        chassis_follow.turn_direction = 0;
        follow_have_last_yaw = 0u;
        return;
    }

    chassis_follow.data_valid = 1u;

    if (chassis_follow.fault_latched == 0u)
    {
        chassis_follow.active = 1u;
    }
}

void Chassis_Follow_Update(chassis_cmd_t *cmd)
{
    float yaw_mec;
    float yaw_error;
    float auto_wz;
    float target_wz;
    float manual_wz;
    float vx_gimbal;
    float vy_gimbal;

    if (cmd == NULL)
    {
        return;
    }

    if (chassis_follow.selected == 0u)
    {
        if (follow_last_selected != 0u)
        {
            manual_wz = cmd->wz;
            chassis_follow.blend -= CHASSIS_FOLLOW_BLEND_STEP;

            if (chassis_follow.blend < 0.0f)
            {
                chassis_follow.blend = 0.0f;
            }

            target_wz = manual_wz * (1.0f - chassis_follow.blend);
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

    yaw_mec = board.rx_meg->gimbal_meg.yaw_mec;
    yaw_error = Chassis_Follow_WrapPi(
        (CHASSIS_FOLLOW_YAW_ANGLE_SIGN * yaw_mec) - CHASSIS_FOLLOW_CENTER_RAD);
    cmd->source = CHASSIS_SRC_RC_FOLLOW;

    if (follow_have_last_yaw != 0u)
    {
        float yaw_delta = Chassis_Follow_WrapPi(yaw_error - follow_last_yaw_rad);

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

    follow_last_yaw_rad = yaw_error;
    follow_have_last_yaw = 1u;
    chassis_follow.yaw_mec_rad = yaw_mec;
    chassis_follow.yaw_error_rad = yaw_error;

#if CHASSIS_FOLLOW_TRANSLATION_ENABLE
    vx_gimbal = cmd->vx;
    vy_gimbal = cmd->vy;

    cmd->vx = CHASSIS_FOLLOW_TRANSLATION_SIGN *
              ((cosf(yaw_error) * vx_gimbal) - (sinf(yaw_error) * vy_gimbal));
    cmd->vy = CHASSIS_FOLLOW_TRANSLATION_SIGN *
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
        float abs_error = fabsf(yaw_error);
        int8_t error_direction = (yaw_error >= 0.0f) ? 1 : -1;

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

        auto_wz = constrain(auto_wz,
                            -CHASSIS_FOLLOW_MAX_WZ,
                            CHASSIS_FOLLOW_MAX_WZ);
    }

    chassis_follow.blend += CHASSIS_FOLLOW_BLEND_STEP;
    if (chassis_follow.blend > 1.0f)
    {
        chassis_follow.blend = 1.0f;
    }

    target_wz = auto_wz * chassis_follow.blend;
    cmd->wz = Chassis_Follow_Ramp(follow_last_wz, target_wz, CHASSIS_FOLLOW_WZ_STEP);

    chassis_follow.wz_target = auto_wz;
    chassis_follow.wz_output = cmd->wz;
    follow_last_wz = cmd->wz;
}

uint8_t Chassis_Follow_IsSelected(void)
{
    return chassis_follow.selected;
}

uint8_t Chassis_Follow_IsActive(void)
{
    return chassis_follow.active;
}

uint8_t Chassis_Follow_HasFault(void)
{
    return chassis_follow.fault_latched;
}

