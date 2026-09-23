#include "launcher.h"

#include <math.h>

#include "communicate.h"
#include "launcher_config.h"
#include "main.h"
#include "motor.h"
#include "pid.h"
#include "rp_math.h"

launcher_t launcher;

static uint16_t launcher_fric_ready_count;
static uint8_t launcher_jam_count;
static uint16_t launcher_fric_stop_count;
static uint8_t launcher_dial_last_online;
static pid_ctrl_t launcher_dial_angle_pid;
static pid_ctrl_t launcher_dial_speed_pid;

static float Launcher_Ramp(float current, float target, float step)
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

static uint8_t Launcher_FricOnline(uint8_t index)
{
    return ((rm_motor[index].state != NULL) &&
            (rm_motor[index].state->status == DEV_ONLINE)) ? 1u : 0u;
}

static uint8_t Launcher_DialOnline(void)
{
    return (dail_motor.KT_motor_info.state_info.work_state == M_ONLINE) ? 1u : 0u;
}

static int32_t Launcher_DialAngle(void)
{
    return (int32_t)(LAUNCHER_DIAL_ANGLE_SIGN *
                      (float)dail_motor.KT_motor_info.rx_info.encoder_sum);
}

static int32_t Launcher_DialEncoder(void)
{
    return (int32_t)dail_motor.KT_motor_info.rx_info.encoder;
}

static uint8_t Launcher_DialAtTarget(void)
{
    return (fabsf((float)(launcher.dial_angle - launcher.dial_target_angle)) <=
            LAUNCHER_DIAL_STOP_ERROR) ? 1u : 0u;
}

#if LAUNCHER_DIAL_ENABLE
static void Launcher_DialClearPid(void)
{
    launcher_dial_angle_pid.integral = 0.0f;
    launcher_dial_angle_pid.last_err = 0.0f;
    launcher_dial_angle_pid.out = 0.0f;

    launcher_dial_speed_pid.integral = 0.0f;
    launcher_dial_speed_pid.last_err = 0.0f;
    launcher_dial_speed_pid.out = 0.0f;
}
#endif

#if LAUNCHER_DIAL_ENABLE
static void Launcher_DialStop(void)
{
    Launcher_DialClearPid();
    if ((dail_motor.W_iqControl != NULL) && (dail_motor.tx_W_cmd != NULL))
    {
        dail_motor.W_iqControl(&dail_motor, 0);
        dail_motor.tx_W_cmd(&dail_motor, TORQUE_CLOSE_LOOP_ID);
    }
}
#endif

#if LAUNCHER_DIAL_ENABLE
static void Launcher_DialControl(void)
{
    float speed_target;
    int16_t current_output;

#if !LAUNCHER_DIAL_READY_HOLD_ENABLE
    if (launcher.state == LAUNCHER_READY)
    {
        Launcher_DialStop();
        return;
    }
#endif

    if (launcher.state == LAUNCHER_INIT)
    {
        launcher_dial_angle_pid.target = LAUNCHER_DIAL_RESET_ANGLE;
        launcher_dial_angle_pid.measure = (float)Launcher_DialEncoder();
    }
    else
    {
        launcher_dial_angle_pid.target = (float)launcher.dial_target_angle;
        launcher_dial_angle_pid.measure = (float)launcher.dial_angle;
    }
    launcher_dial_angle_pid.err =
        launcher_dial_angle_pid.target - launcher_dial_angle_pid.measure;
    single_pid_ctrl(&launcher_dial_angle_pid);

    speed_target = constrain(launcher_dial_angle_pid.out,
                             -(float)LAUNCHER_DIAL_MAX_SPEED_DPS,
                             (float)LAUNCHER_DIAL_MAX_SPEED_DPS);

    launcher_dial_speed_pid.target = speed_target;
    launcher_dial_speed_pid.measure =
        LAUNCHER_DIAL_SPEED_SIGN *
        (float)dail_motor.KT_motor_info.rx_info.speed;
    launcher_dial_speed_pid.err =
        launcher_dial_speed_pid.target - launcher_dial_speed_pid.measure;
    single_pid_ctrl(&launcher_dial_speed_pid);

    current_output = (int16_t)constrain(LAUNCHER_DIAL_OUTPUT_SIGN *
                                       launcher_dial_speed_pid.out,
                                       -LAUNCHER_DIAL_CURRENT_LIMIT,
                                       LAUNCHER_DIAL_CURRENT_LIMIT);

    if ((dail_motor.W_iqControl != NULL) && (dail_motor.tx_W_cmd != NULL))
    {
        dail_motor.W_iqControl(&dail_motor, current_output);
        dail_motor.tx_W_cmd(&dail_motor, TORQUE_CLOSE_LOOP_ID);
    }
}
#endif

#if !LAUNCHER_DIAL_ENABLE
static void Launcher_DialDisable(void)
{
    if (dail_motor.tx_W_cmd != NULL)
    {
        dail_motor.tx_W_cmd(&dail_motor, MOTOR_CLOSE_ID);
    }
}
#endif

static void Launcher_DialIdle(void)
{
#if LAUNCHER_DIAL_ENABLE
    Launcher_DialStop();
#else
    Launcher_DialDisable();
#endif
}

static void Launcher_FricControl(uint8_t enable)
{
    for (uint8_t i = 0u; i < SHOOT_FRIC_NUM; i++)
    {
        pid_ctrl_t *pid = rm_motor[i].ctrl->speed_ctrl;
        float feedforward;
        float direction = (i == SHOOT_FRIC_L) ?
                          LAUNCHER_FRIC_L_DIRECTION :
                          LAUNCHER_FRIC_R_DIRECTION;

        if ((enable != 0u) && (Launcher_FricOnline(i) != 0u))
        {
            pid->target = direction * launcher.fric_target_rpm;
            pid->measure = (float)rm_motor[i].rx_info->encoder_speed;
            pid->err = pid->target - pid->measure;

            if (launcher.state == LAUNCHER_STOPPING)
            {
                pid->integral = 0.0f;
            }

            single_pid_ctrl(pid);
            feedforward = LAUNCHER_FRIC_KFF * fabsf(pid->target);
            if (pid->target < 0.0f)
            {
                feedforward = -feedforward;
            }

            rm_motor[i].tx_info->torque =
                constrain(pid->out + feedforward,
                          -LAUNCHER_FRIC_OUT_MAX,
                          LAUNCHER_FRIC_OUT_MAX);
        }
        else
        {
            pid->integral = 0.0f;
            pid->last_err = 0.0f;
            pid->out = 0.0f;
            rm_motor[i].tx_info->torque = 0.0f;
        }
    }

    RM_Group.group_set_torque(&RM_Group);
}

static void Launcher_UpdateFrictionReady(uint8_t enabled)
{
    float l_speed;
    float r_speed;

    l_speed = fabsf((float)rm_motor[SHOOT_FRIC_L].rx_info->encoder_speed);
    r_speed = fabsf((float)rm_motor[SHOOT_FRIC_R].rx_info->encoder_speed);

    launcher.fric_l_speed_rpm = l_speed;
    launcher.fric_r_speed_rpm = r_speed;

    if ((enabled != 0u) &&
        (Launcher_FricOnline(SHOOT_FRIC_L) != 0u) &&
        (Launcher_FricOnline(SHOOT_FRIC_R) != 0u) &&
        (launcher.fric_target_rpm >= (LAUNCHER_FRIC_TARGET_RPM * 0.5f)) &&
        (fabsf(l_speed - launcher.fric_target_rpm) <= LAUNCHER_FRIC_READY_TOL_RPM) &&
        (fabsf(r_speed - launcher.fric_target_rpm) <= LAUNCHER_FRIC_READY_TOL_RPM))
    {
        if (launcher_fric_ready_count < LAUNCHER_FRIC_READY_TIME_MS)
        {
            launcher_fric_ready_count++;
        }
    }
    else
    {
        launcher_fric_ready_count = 0u;
    }

    launcher.fric_ready = (launcher_fric_ready_count >= LAUNCHER_FRIC_READY_TIME_MS) ? 1u : 0u;
}

void Launcher_Init(void)
{
    pid_ctrl_t *pid;

    launcher.state = LAUNCHER_SLEEP;
    launcher.state_tick = 0u;
    launcher.last_repeat_tick = 0u;
    launcher.jam_tick = 0u;
    launcher.fric_target_rpm = 0.0f;
    launcher.fric_l_speed_rpm = 0.0f;
    launcher.fric_r_speed_rpm = 0.0f;
    launcher.dial_angle = 0;
    launcher.dial_target_angle = 0;
    launcher.dial_zero_angle = 0;
    launcher.enabled = 0u;
    launcher.fric_ready = 0u;
    launcher.dial_online = 0u;
    launcher.single_pending = 0u;
    launcher.last_shoot_level = 0u;
    launcher.fault = 0u;

    launcher_fric_ready_count = 0u;
    launcher_jam_count = 0u;
    launcher_fric_stop_count = 0u;
    launcher_dial_last_online = 0u;

    for (uint8_t i = 0u; i < SHOOT_FRIC_NUM; i++)
    {
        pid = rm_motor[i].ctrl->speed_ctrl;
        pid->kp = LAUNCHER_FRIC_KP;
        pid->ki = LAUNCHER_FRIC_KI;
        pid->kd = LAUNCHER_FRIC_KD;
        pid->integral = 0.0f;
        pid->integral_max = LAUNCHER_FRIC_INTEGRAL_MAX;
        pid->out_max = LAUNCHER_FRIC_OUT_MAX;
        pid->out = 0.0f;
    }

    launcher_dial_angle_pid.kp = LAUNCHER_DIAL_ANGLE_KP;
    launcher_dial_angle_pid.ki = LAUNCHER_DIAL_ANGLE_KI;
    launcher_dial_angle_pid.kd = LAUNCHER_DIAL_ANGLE_KD;
    launcher_dial_angle_pid.integral = 0.0f;
    launcher_dial_angle_pid.integral_max = LAUNCHER_DIAL_ANGLE_INTEGRAL_MAX;
    launcher_dial_angle_pid.out_max = (float)LAUNCHER_DIAL_MAX_SPEED_DPS;
    launcher_dial_angle_pid.out = 0.0f;

    launcher_dial_speed_pid.kp = LAUNCHER_DIAL_SPEED_KP;
    launcher_dial_speed_pid.ki = LAUNCHER_DIAL_SPEED_KI;
    launcher_dial_speed_pid.kd = LAUNCHER_DIAL_SPEED_KD;
    launcher_dial_speed_pid.integral = 0.0f;
    launcher_dial_speed_pid.integral_max = LAUNCHER_DIAL_SPEED_INTEGRAL_MAX;
    launcher_dial_speed_pid.out_max = LAUNCHER_DIAL_CURRENT_LIMIT;
    launcher_dial_speed_pid.out = 0.0f;
}

void Launcher_Work(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t launch_on;
    uint8_t shoot_level;
    uint8_t shoot_mode;
    uint8_t single_edge;
    uint8_t jam_now;
    uint8_t dial_ready;
    uint8_t dial_online_rising;
    int32_t current_angle;

    current_angle = Launcher_DialAngle();
    launcher.dial_angle = current_angle;
    launcher.dial_online = Launcher_DialOnline();

    if ((launcher_dial_last_online == 0u) && (launcher.dial_online != 0u))
    {
        dial_online_rising = 1u;
        launcher.dial_zero_angle = current_angle;
        launcher.dial_target_angle = current_angle;
        launcher.single_pending = 0u;
    }
    else
    {
        dial_online_rising = 0u;
    }
    launcher_dial_last_online = launcher.dial_online;

#if LAUNCHER_DIAL_ENABLE
    dial_ready = launcher.dial_online;
#else
    dial_ready = 1u;
#endif

    launch_on = ((Board_HeartBeat.status == DEV_ONLINE) &&
                 (Board_Rx_Info.shoot_pkt.launch_state != 0u) &&
                 (Launcher_FricOnline(SHOOT_FRIC_L) != 0u) &&
                 (Launcher_FricOnline(SHOOT_FRIC_R) != 0u) &&
                 (dial_ready != 0u)) ? 1u : 0u;

    if ((launch_on != 0u) && (dial_online_rising != 0u))
    {
        launcher.state = LAUNCHER_SPINUP;
        launcher.state_tick = now;
        launcher.jam_tick = now; 
        launcher.fault = 0u;
    }

    if (launcher.fault != 0u)
    {
        if (launch_on != 0u)
        {
            launcher.enabled = 0u;
            Launcher_FricControl(0u);
            Launcher_DialIdle();
            return;
        }
        launcher.fault = 0u;
    }

    if (launch_on == 0u)
    {
        launcher.enabled = 0u;
        launcher.single_pending = 0u;
        launcher.last_shoot_level = 0u;
        Launcher_UpdateFrictionReady(0u);

        if (launcher.state == LAUNCHER_SLEEP)
        {
            launcher.fric_target_rpm = 0.0f;
            launcher_fric_stop_count = 0u;
            Launcher_FricControl(0u);
            Launcher_DialIdle();
            return;
        }

        launcher.state = LAUNCHER_STOPPING;
        launcher.fric_target_rpm = Launcher_Ramp(launcher.fric_target_rpm,
                                                 0.0f,
                                                 LAUNCHER_FRIC_STOP_RAMP_RPM_PER_MS);
        Launcher_FricControl(1u);
        Launcher_DialIdle();

        if ((fabsf(launcher.fric_l_speed_rpm) <= LAUNCHER_FRIC_STOP_SPEED_RPM) &&
            (fabsf(launcher.fric_r_speed_rpm) <= LAUNCHER_FRIC_STOP_SPEED_RPM))
        {
            if (launcher_fric_stop_count < LAUNCHER_FRIC_STOP_CONFIRM_MS)
            {
                launcher_fric_stop_count++;
            }
        }
        else
        {
            launcher_fric_stop_count = 0u;
        }

        if (launcher_fric_stop_count >= LAUNCHER_FRIC_STOP_CONFIRM_MS)
        {
            launcher.fric_target_rpm = 0.0f;
            launcher.state = LAUNCHER_SLEEP;
            launcher_fric_stop_count = 0u;
            Launcher_FricControl(0u);
        }

        return;
    }

    launcher.enabled = 1u;
    launcher_fric_stop_count = 0u;

    if ((launcher.state == LAUNCHER_SLEEP) ||
        (launcher.state == LAUNCHER_STOPPING))
    {
        launcher.dial_zero_angle = current_angle;
        launcher.dial_target_angle = current_angle;
        launcher.state = LAUNCHER_SPINUP;
        launcher.state_tick = now;
        launcher.last_repeat_tick = now;
        launcher.jam_tick = now;
        launcher_jam_count = 0u;
    }

    launcher.fric_target_rpm = Launcher_Ramp(launcher.fric_target_rpm,
                                             LAUNCHER_FRIC_TARGET_RPM,
                                             LAUNCHER_FRIC_RAMP_RPM_PER_MS);
    Launcher_UpdateFrictionReady(1u);

    shoot_level = Board_Rx_Info.shoot_pkt.shoot_level;
    shoot_mode = Board_Rx_Info.shoot_pkt.shoot_mode;
#if !LAUNCHER_DIAL_ENABLE
    shoot_level = 0u;
    shoot_mode = 0u;
#endif
    single_edge = ((shoot_level != 0u) &&
                   (launcher.last_shoot_level == 0u) &&
                   (shoot_mode == 0u)) ? 1u : 0u;
    if (single_edge != 0u)
    {
        launcher.single_pending = 1u;
    }
    launcher.last_shoot_level = shoot_level;

    jam_now = 0u;
    if (fabsf((float)dail_motor.KT_motor_info.rx_info.current) >=
        (float)LAUNCHER_DIAL_JAM_CURRENT_RAW)
    {
        if (fabsf((float)dail_motor.KT_motor_info.rx_info.speed) <=
            (float)LAUNCHER_DIAL_JAM_SPEED_DPS)
        {
            jam_now = 1u;
        }
    }

    switch (launcher.state)
    {
    case LAUNCHER_SPINUP:
        if (launcher.fric_ready != 0u)
        {
#if LAUNCHER_DIAL_AUTO_RESET_ENABLE
            launcher.state = LAUNCHER_INIT;
#else
            launcher.dial_zero_angle = current_angle;
            launcher.dial_target_angle = current_angle;
            Launcher_DialClearPid();
            launcher.state = LAUNCHER_READY;
            launcher.jam_tick = now;
            launcher_jam_count = 0u;
#endif
            launcher.state_tick = now;
        }
        break;

    case LAUNCHER_INIT:
        if ((fabsf(LAUNCHER_DIAL_RESET_ANGLE -
                     (float)Launcher_DialEncoder()) <= LAUNCHER_DIAL_STOP_ERROR) ||
            ((now - launcher.state_tick) >= LAUNCHER_DIAL_RESET_TIMEOUT_MS))
        {
            launcher.dial_zero_angle = current_angle;
            launcher.dial_target_angle = current_angle;
            Launcher_DialClearPid();
            launcher.state = LAUNCHER_READY;
            launcher.state_tick = now;
            launcher.jam_tick = now;
            launcher_jam_count = 0u;
        }
        break;

    case LAUNCHER_READY:
        if (launcher.single_pending != 0u)
        {
            launcher.single_pending = 0u;
            launcher.dial_target_angle +=
                (int32_t)(LAUNCHER_DIAL_DIRECTION * LAUNCHER_DIAL_ONE_SHOT_ANGLE);
            launcher.state = LAUNCHER_SINGLE;
            launcher.state_tick = now;
            launcher.jam_tick = now;
        }
#if LAUNCHER_REPEAT_ENABLE
        else if ((shoot_mode != 0u) && (shoot_level != 0u) &&
                 ((now - launcher.last_repeat_tick) >=
                  LAUNCHER_DIAL_REPEAT_INTERVAL_MS))
        {
            launcher.dial_target_angle +=
                (int32_t)(LAUNCHER_DIAL_DIRECTION * LAUNCHER_DIAL_ONE_SHOT_ANGLE);
            launcher.state = LAUNCHER_REPEAT;
            launcher.state_tick = now;
            launcher.jam_tick = now;
            launcher.last_repeat_tick = now;
        }
#endif
        break;

    case LAUNCHER_SINGLE:
    case LAUNCHER_REPEAT:
        if (Launcher_DialAtTarget() != 0u)
        {
            Launcher_DialClearPid();
            launcher.state = LAUNCHER_READY;
            launcher.state_tick = now;
            launcher.jam_tick = now;
            launcher_jam_count = 0u;
        }
        else if ((jam_now != 0u) &&
                 ((now - launcher.jam_tick) >= LAUNCHER_DIAL_JAM_TIME_MS))
        {
            launcher_jam_count++;
            if (launcher_jam_count >= LAUNCHER_DIAL_JAM_MAX_RETRY)
            {
                launcher.state = LAUNCHER_FAULT;
                launcher.fault = 1u;
                break;
            }

            launcher.dial_target_angle = current_angle -
                (int32_t)(LAUNCHER_DIAL_DIRECTION * LAUNCHER_DIAL_REVERSE_ANGLE);
            launcher.state = LAUNCHER_REVERSE;
            Launcher_DialClearPid();
            launcher.state_tick = now;
            launcher.jam_tick = now;
        }
        else if ((now - launcher.state_tick) >= LAUNCHER_DIAL_SINGLE_TIMEOUT_MS)
        {
            Launcher_DialClearPid();
            launcher.dial_target_angle = current_angle;
            launcher.state = LAUNCHER_READY;
            launcher.state_tick = now;
            launcher.jam_tick = now;
            break;
        }
        else
        {
            if (jam_now == 0u)
            {
                launcher.jam_tick = now;
            }
        }
        break;

    case LAUNCHER_REVERSE:
        if ((Launcher_DialAtTarget() != 0u) ||
            ((now - launcher.state_tick) >= LAUNCHER_DIAL_REVERSE_TIMEOUT_MS))
        {
            launcher.dial_target_angle = current_angle +
                (int32_t)(LAUNCHER_DIAL_DIRECTION * LAUNCHER_DIAL_ONE_SHOT_ANGLE);
            launcher.state = LAUNCHER_RELOAD;
            Launcher_DialClearPid();
            launcher.state_tick = now;
        }
        break;

    case LAUNCHER_RELOAD:
        if ((Launcher_DialAtTarget() != 0u) ||
            ((now - launcher.state_tick) >= LAUNCHER_DIAL_RELOAD_TIMEOUT_MS))
        {
            launcher.state = LAUNCHER_READY;
            Launcher_DialClearPid();
            launcher.state_tick = now;
            launcher.jam_tick = now;
        }
        break;

    case LAUNCHER_FAULT:
    case LAUNCHER_SLEEP:
    default:
        break;
    }

    if (launcher.state == LAUNCHER_FAULT)
    {
        launcher.fric_target_rpm = 0.0f;
        Launcher_FricControl(0u);
        Launcher_DialIdle();
        return;
    }

    Launcher_FricControl(1u);
#if LAUNCHER_DIAL_ENABLE
    Launcher_DialControl();
#else
    Launcher_DialIdle();
#endif
}
