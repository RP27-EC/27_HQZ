/* control_task.c - 控制任务 */

#include "control_task.h"
#include "board_protocol.h"
#include "rc_sensor.h"
#include "board_comm_config.h"
#include "chassis_config.h"
#include "chassis_input.h"
#include "chassis_control.h"
#include "chassis_follow.h"
#include "chassis_spin.h"
#include "launch.h"
#include "rp_math.h"
#include "supercap.h"
#include <math.h>

typedef struct
{
    volatile uint8_t rc_state; // 遥控在线状态，见设备枚举
    volatile uint8_t s1; // 右拨杆编码，1/2/3
    volatile uint8_t s2; // 左拨杆编码，1/2/3
    volatile uint8_t b_value; // B键当前电平，0/1
    volatile uint8_t button_event; // B键按下边沿，0/1
    volatile uint8_t is_hole; // 有效过洞请求，0/1
    volatile uint8_t exit_pending; // 退出等待有效，0/1
    volatile uint8_t hole_rear_blocked; // 后向过洞请求拒绝，0/1
} board_lift_debug_t;

volatile board_lift_debug_t board_lift_dbg;
volatile uint8_t board_hole_request;
volatile uint8_t board_hole_exit_pending;

/* 遥控前后基准，0前1后 */
static uint8_t board_gimbal_yaw_rear = 0u;

/* 按输入源读取有效前后基准 */
static uint8_t Board_Debug_YawRear(void)
{
    if (Chassis_Input_IsKeyboardMode() != 0u)
    {
        return Chassis_Input_IsKeyboardYawRear();
    }

    return board_gimbal_yaw_rear;
}


static uint8_t Board_Debug_Hole_Command(rc_data_t *rc_info)
{
    static uint8_t last_b_value = 0u;
    static uint32_t hole_exit_tick = 0u;
    uint32_t now = HAL_GetTick();
    uint8_t b_now;
    uint8_t button_event; // B键按下边沿，0/1

    board_lift_dbg.rc_state = (uint8_t)rc_dev.work_state;
    board_lift_dbg.s1 = (uint8_t)rc_info->s1.value;
    board_lift_dbg.s2 = (uint8_t)rc_info->s2.value;

    if (rc_dev.work_state != DEV_ONLINE)
    {
        last_b_value = 0u;
        board_hole_request = 0u;
        board_hole_exit_pending = 0u;
        board_lift_dbg.is_hole = 0u;
        board_lift_dbg.exit_pending = 0u;
        board_lift_dbg.hole_rear_blocked = 0u;
        board.tx_pkt->gimbal_target_pkt.is_hole = 0u;
        return 0u;
    }

    b_now = (rc_info->B.value != 0u) ? 1u : 0u;
    button_event = ((b_now != 0u) && (last_b_value == 0u)) ? 1u : 0u;
    last_b_value = b_now;

    if ((rc_info->s1.value == RC_SW_UP) &&
        (rc_info->s2.value == RC_SW_MID) &&
        (button_event != 0u))
    {
        if (board_hole_request == 0u)
        {
            /* 后向基准拒绝过洞请求 */
            if (Board_Debug_YawRear() == 0u)
            {
                board_hole_request = 1u;
                board_hole_exit_pending = 0u;
                board_lift_dbg.hole_rear_blocked = 0u;
            }
            else
            {
                board_lift_dbg.hole_rear_blocked = 1u;
            }
        }
        else
        {
            board_hole_request = 0u;
            board_hole_exit_pending = 1u;
            hole_exit_tick = now;
        }
    }

    board.tx_pkt->gimbal_target_pkt.is_hole = board_hole_request;
    board_lift_dbg.b_value = b_now;
    board_lift_dbg.button_event = button_event;
    board_lift_dbg.is_hole = (uint8_t)board_hole_request;

    if (board_hole_exit_pending != 0u)
    {
        if ((board.rx_meg->state_meg.is_down == 2u) ||
            ((now - hole_exit_tick) >= BOARD_HOLE_EXIT_TIMEOUT_MS))
        {
            board_hole_exit_pending = 0u;
        }
    }

    board_lift_dbg.exit_pending = (uint8_t)board_hole_exit_pending;
    if ((board_hole_request != 0u) || (board_hole_exit_pending != 0u))
    {
        board.tx_pkt->car_pkt.gimbal_mode = 0u;
        /* 过洞同步复位掉头与跟随基准 */
        Chassis_Input_ResetYawReference();
        board.tx_pkt->gimbal_target_pkt.yaw_mec_tar = BOARD_MEC_YAW_FRONT_RAD;
        board.tx_pkt->gimbal_target_pkt.pitch_mec_tar =
            BOARD_HOLE_PITCH_TARGET_RAD;
        return 1u;
    }

    return 0u;
}

static void Board_R_Turn_Update(uint8_t start, uint8_t cancel)
{
    static uint8_t active;
    static uint8_t source;
    static uint8_t stable;
    static uint32_t start_tick;
    static uint32_t stable_tick;
    uint32_t now = HAL_GetTick();
    uint32_t feedback_tick = board.status->gimbal_rx_time_ms;
    uint8_t keyboard = Chassis_Input_IsKeyboardMode();
    float error;
    float yaw = board.rx_meg->gimbal_meg.yaw_mec;

    board.tx_pkt->car_pkt.r_turn_active = 0u;
    if ((cancel != 0u) || (board.tx_pkt->car_pkt.gimbal_mode != 0u))
    {
        active = 0u;
        stable = 0u;
        return;
    }

    /* 跟随掉头状态接管完成判据 */
    if (Chassis_Input_IsUturnActive() != 0u)
    {
        active = 0u;
        stable = 0u;
        board.tx_pkt->car_pkt.r_turn_active =
            (Chassis_Input_GetUturnState() == CHASSIS_UTURN_POSITION) ? 1u : 0u;
        return;
    }

    if (source != keyboard)
    {
        active = 0u;
        stable = 0u;
    }
    source = keyboard;
    if (start != 0u)
    {
        active = 1u;
        stable = 0u;
        start_tick = now;
    }

    if ((board.status->gimbal_data_valid == 0u) ||
        ((now - feedback_tick) > CHASSIS_KEY_UTURN_FEEDBACK_TIMEOUT_MS) ||
        (yaw != yaw) || (fabsf(yaw) > 3.1515927f) ||
        ((now - start_tick) >= CHASSIS_KEY_UTURN_TIMEOUT_MS))
    {
        active = 0u;
        stable = 0u;
    }
    if (active == 0u)
    {
        return;
    }

    error = board.tx_pkt->gimbal_target_pkt.yaw_mec_tar - yaw;
    if (error > 3.1415927f) { error -= 6.2831853f; }
    else if (error < -3.1415927f) { error += 6.2831853f; }
    if (fabsf(error) <= CHASSIS_KEY_UTURN_TOL_DEG * 0.0174532925f)
    {
        if (stable == 0u)
        {
            stable = 1u;
            stable_tick = feedback_tick;
        }
        if ((feedback_tick - stable_tick) >= CHASSIS_KEY_UTURN_STABLE_MS)
        {
            active = 0u;
        }
    }
    else
    {
        stable = 0u;
    }
    board.tx_pkt->car_pkt.r_turn_active = active;
}

/* 调试模式映射云台目标 */
static void Board_Debug_Gimbal_Command(void)
{
    static uint8_t mec_mode_active = 0u; /* 机械角模式已激活 */
    static uint8_t last_r_pressed = 0u;   /* R 键上次状态 */
    static float pitch_mec_target = 0.0f;/* Pitch 机械目标角，rad */
    rc_data_t *rc_info = rc_dev.info;     /* 遥控数据源 */
    uint8_t r_pressed;
    uint8_t keyboard_active; /* 键鼠模式生效 */
    uint8_t keyboard_mech;   /* 键鼠 X 机械档生效 */
    uint8_t uturn_mech;
    uint8_t r_edge;
    float previous_yaw_target = board.tx_pkt->gimbal_target_pkt.yaw_mec_tar;

    if (rc_dev.work_state != DEV_ONLINE)
    {
        board_gimbal_yaw_rear = 0u;
        last_r_pressed = 0u;
        mec_mode_active = 0u;
        board.tx_pkt->car_pkt.car_state = 0u;
        board.tx_pkt->car_pkt.gimbal_mode = 0u;
        board.tx_pkt->gimbal_target_pkt.yaw_mec_tar = 0.0f;
        board.tx_pkt->gimbal_target_pkt.pitch_mec_tar = 0.0f;
        board.tx_pkt->gimbal_target_pkt.yaw_imu_tar = 0.0f;
        board.tx_pkt->gimbal_target_pkt.pitch_imu_tar = 0.0f;
        board.tx_pkt->gimbal_target_pkt.is_hole = 0u;
        Board_R_Turn_Update(0u, 1u);
        return;
    }

    board.tx_pkt->car_pkt.car_state = 1u;
    keyboard_active = Chassis_Input_IsKeyboardMode();
    keyboard_mech = Chassis_Input_IsKeyboardMechMode();
    uturn_mech = (Chassis_Input_GetUturnState() == CHASSIS_UTURN_POSITION) ? 1u : 0u;
    r_pressed = ((rc_info->key_v & KEY_PRESSED_OFFSET_R) != 0u) ? 1u : 0u;
    r_edge = ((r_pressed != 0u) && (last_r_pressed == 0u)) ? 1u : 0u;
#if BOARD_LIFT_ENABLE
    if (Board_Debug_Hole_Command(rc_info) != 0u)
    {
        /* 过洞期间屏蔽R掉头 */
        mec_mode_active = 0u;
        last_r_pressed = r_pressed;
        Board_R_Turn_Update(0u, 1u);
        return;
    }
#endif

    /* 键鼠与遥控独立维护掉头状态 */
    if (keyboard_active == 0u)
    {
        if ((r_pressed != 0u) && (last_r_pressed == 0u))
        {
            board_gimbal_yaw_rear ^= 1u;
        }
    }
    last_r_pressed = r_pressed;

    /* 预发送与定位共用固定终点。 */
    if (keyboard_active != 0u)
    {
        board.tx_pkt->gimbal_target_pkt.yaw_mec_tar = Chassis_Input_GetKeyboardYawTargetRad();
    }
    else
    {
        board.tx_pkt->gimbal_target_pkt.yaw_mec_tar =
            (board_gimbal_yaw_rear != 0u) ?
            BOARD_MEC_YAW_REAR_RAD : BOARD_MEC_YAW_FRONT_RAD;
    }

    /* 遥控下位或键鼠X档机械控制 */
    if ((rc_info->s1.value == RC_SW_DOWN) || (keyboard_mech != 0u) || (uturn_mech != 0u))
    {
        board.tx_pkt->car_pkt.gimbal_mode = 0u;
        /* 机械模式入口对齐反馈 */
        if (mec_mode_active == 0u)
        {
            pitch_mec_target = board.rx_meg->gimbal_meg.pitch_mec;
            mec_mode_active = 1u;
        }

        /* 遥控Pitch积分机械目标
         * 键鼠Pitch使用D5输入 */
        if (keyboard_active == 0u)
        {
            pitch_mec_target += (float)rc_info->ch1 / BOARD_RC_AXIS_MAX *
                                BOARD_MEC_PITCH_STEP_RAD;
            if (pitch_mec_target > BOARD_MEC_PITCH_MAX_RAD)
            {
                pitch_mec_target = BOARD_MEC_PITCH_MAX_RAD;
            }
            else if (pitch_mec_target < BOARD_MEC_PITCH_MIN_RAD)
            {
                pitch_mec_target = BOARD_MEC_PITCH_MIN_RAD;
            }
        }

        board.tx_pkt->gimbal_target_pkt.pitch_mec_tar = pitch_mec_target;
    }
    /* 机械模式退出后交回 IMU 角度环 */
    else
    {
        board.tx_pkt->car_pkt.gimbal_mode = 1u;
        mec_mode_active = 0u;
    }
    Board_R_Turn_Update(
        ((r_edge != 0u) &&
         (board.tx_pkt->gimbal_target_pkt.yaw_mec_tar != previous_yaw_target)) ? 1u : 0u,
        ((keyboard_active != 0u) && (keyboard_mech == 0u) && (uturn_mech == 0u)) ? 1u : 0u);
}

/* 当前控制链每1ms运行 */

void StartCtrlTask(void const *argument)
{
    (void)argument;

    for (;;)
    {
        Chassis_Input_Update();

        Board_Debug_Gimbal_Command();

        Chassis_Follow_UpdateMode();
        Chassis_Spin_UpdateMode();
        Chassis_Follow_Update(&chassis_input_cmd);
        Chassis_Spin_Update(&chassis_input_cmd);
        Chassis_Control_Update(&chassis_input_cmd);
        launch.work(&launch);

#if SUPERCAP_BRINGUP_ENABLE
        SuperCap_Tx();
#endif
        osDelay(1);
    }
}
