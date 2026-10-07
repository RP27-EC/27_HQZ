/* control_task.c - 控制任务 */

#include "control_task.h"
#include "cap.h"
#include "ui.h"
#include "priority_ui.h"
#include "infantry.h"
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

typedef struct
{
    volatile uint8_t rc_state;     // 遥控在线状态，枚举值
    volatile uint8_t s1;           // S1档位，1/2/3
    volatile uint8_t s2;           // S2档位，1/2/3
    volatile uint8_t b_value;      // B键按下，0/1
    volatile uint8_t button_event; // 升降切换事件，0/1
    volatile uint8_t is_hole;      // 下降目标请求，0/1
    volatile uint8_t exit_pending; // 等待顶部确认，0/1
    volatile int16_t thumbwheel;   // 拨轮原始量，±660
    volatile uint8_t turn_event;   // 掉头触发事件，0/1
} board_lift_debug_t;

volatile board_lift_debug_t board_lift_dbg;
volatile uint8_t board_hole_request;
volatile uint8_t board_hole_exit_pending;

static uint8_t board_turn_event;
static volatile uint8_t board_hole_down_tx_seen;
static uint32_t board_hole_exit_tick;
static uint32_t board_hole_exit_nonready_count;

static uint8_t Board_Lift_TopReady(void)
{
#if BOARD_LIFT_ENABLE
    if ((board.status == NULL) || (board.rx_meg == NULL) ||
        (board.status->state_data_valid == 0u) ||
        ((HAL_GetTick() - board.status->state_rx_time_ms) > BOARD_LIFT_STATE_TIMEOUT_MS))
    {
        return 0u;
    }
    return ((board.rx_meg->state_meg.height_motor_state != 0u) &&
            (board.rx_meg->state_meg.is_down == 2u)) ? 1u : 0u;
#else
    return 1u;
#endif
}

uint8_t Board_Lift_IsRestricted(void)
{
#if BOARD_LIFT_ENABLE
    return ((board_hole_request != 0u) || (board_hole_exit_pending != 0u)) ? 1u : 0u;
#else
    return 0u;
#endif
}

uint8_t Board_Lift_IsReady(void)
{
    return ((Board_Lift_IsRestricted() == 0u) &&
            (Board_Lift_TopReady() != 0u)) ? 1u : 0u;
}

uint8_t Board_Control_GetTurnEvent(void)
{
    return board_turn_event;
}

void Board_Control_NotifyLiftTx(uint8_t is_hole)
{
    if (is_hole != 0u)
    {
        board_hole_down_tx_seen = 1u;
    }
}

static void Board_Control_InputUpdate(void)
{
    static uint8_t wheel_armed = 0u;
    static uint8_t last_r = 0u;
    static uint8_t last_b = 0u;
    const rc_data_t *rc = rc_dev.info;
    uint8_t r_now;
    uint8_t b_now;
    uint8_t wheel_up = 0u;
    uint8_t wheel_down = 0u;
    uint8_t lift_event;
    uint32_t now = HAL_GetTick();

    board_turn_event = 0u;
    board_lift_dbg.rc_state = (uint8_t)rc_dev.work_state;
    board_lift_dbg.button_event = 0u;
    board_lift_dbg.turn_event = 0u;
    if ((rc_dev.work_state != DEV_ONLINE) || (rc == NULL))
    {
        wheel_armed = 0u;
        last_r = (rc != NULL && (rc->key_v & KEY_PRESSED_OFFSET_R) != 0u) ? 1u : 0u;
        last_b = (rc != NULL && rc->B.value != 0u) ? 1u : 0u;
        board_hole_request = 0u;
        board_hole_exit_pending = 0u;
        board_hole_down_tx_seen = 0u;
        board.tx_pkt->gimbal_target_pkt.is_hole = 0u;
        board_lift_dbg.is_hole = 0u;
        board_lift_dbg.exit_pending = 0u;
        board_lift_dbg.b_value = 0u;
        board_lift_dbg.thumbwheel = 0;
        return;
    }

    r_now = ((rc->key_v & KEY_PRESSED_OFFSET_R) != 0u) ? 1u : 0u;
    b_now = (rc->B.value != 0u) ? 1u : 0u;
    if ((rc->thumbwheel.value >= -BOARD_WHEEL_CENTER_RAW) &&
        (rc->thumbwheel.value <= BOARD_WHEEL_CENTER_RAW))
    {
        wheel_armed = 1u;
    }
    else if (wheel_armed != 0u)
    {
        if (rc->thumbwheel.value <= -BOARD_WHEEL_TRIGGER_RAW)
        {
            wheel_up = 1u;
            wheel_armed = 0u;
        }
        else if (rc->thumbwheel.value >= BOARD_WHEEL_TRIGGER_RAW)
        {
            wheel_down = 1u;
            wheel_armed = 0u;
        }
    }

    lift_event = (wheel_down != 0u ||
                  ((b_now != 0u) && (last_b == 0u) &&
                   (rc->s1.value == RC_SW_UP) && (rc->s2.value == RC_SW_MID))) ? 1u : 0u;
    board_turn_event = (wheel_up != 0u ||
                        ((r_now != 0u) && (last_r == 0u))) ? 1u : 0u;
    last_r = r_now;
    last_b = b_now;
#if BOARD_LIFT_ENABLE
    if (lift_event != 0u)
    {
        if (Board_Lift_IsRestricted() == 0u)
        {
            board_hole_down_tx_seen = 0u;
        }
        board_hole_request ^= 1u;
        board_hole_exit_pending = (board_hole_request == 0u) ? 1u : 0u;
        if (board_hole_exit_pending != 0u)
        {
            board_hole_exit_tick = now;
            board_hole_exit_nonready_count = board.status->lift_nonready_count;
        }
        board_turn_event = 0u;
    }
    // NOTE: 撤销确认后再恢复顶部
    if ((board_hole_exit_pending != 0u) &&
        ((now - board_hole_exit_tick) >= (2u * BOARD_COMM_D1D2_PERIOD_MS)) &&
        (Board_Lift_TopReady() != 0u) &&
        ((board_hole_down_tx_seen == 0u) ||
         (board.status->lift_nonready_count != board_hole_exit_nonready_count)) &&
        ((uint32_t)(now - board.status->state_rx_time_ms) < (now - board_hole_exit_tick)))
    {
        board_hole_exit_pending = 0u;
        board_hole_down_tx_seen = 0u;
    }
#endif
    board.tx_pkt->gimbal_target_pkt.is_hole = board_hole_request;
    board_lift_dbg.s1 = (uint8_t)rc->s1.value;
    board_lift_dbg.s2 = (uint8_t)rc->s2.value;
    board_lift_dbg.b_value = b_now;
    board_lift_dbg.button_event = lift_event;
    board_lift_dbg.is_hole = board_hole_request;
    board_lift_dbg.exit_pending = board_hole_exit_pending;
    board_lift_dbg.thumbwheel = rc->thumbwheel.value;
    board_lift_dbg.turn_event = board_turn_event;
}

#if BOARD_COMM_DEBUG
static uint8_t Board_Debug_Hole_Command(void)
{
    if (Board_Lift_IsRestricted() == 0u)
    {
        return 0u;
    }

    // NOTE: 回正先于升降下行
    Chassis_Input_ResetYawReference();
    board.tx_pkt->car_pkt.gimbal_mode = 0u;
    board.tx_pkt->gimbal_target_pkt.yaw_mec_tar = BOARD_MEC_YAW_FRONT_RAD;
    board.tx_pkt->gimbal_target_pkt.pitch_mec_tar = BOARD_HOLE_PITCH_TARGET_RAD;
    return 1u;
}

static void Board_Debug_Gimbal_Command(void)
{
    static uint8_t mec_mode_active = 0u;
    static float pitch_mec_target = 0.0f;
    const rc_data_t *rc_info = rc_dev.info;
    uint8_t keyboard_active;
    uint8_t keyboard_mech;
    uint8_t uturn_mech;

    if ((rc_dev.work_state != DEV_ONLINE) || (rc_info == NULL))
    {
        mec_mode_active = 0u;
        board.tx_pkt->car_pkt.car_state = 0u;
        board.tx_pkt->car_pkt.gimbal_mode = 0u;
        board.tx_pkt->gimbal_target_pkt.yaw_mec_tar = 0.0f;
        board.tx_pkt->gimbal_target_pkt.pitch_mec_tar = 0.0f;
        board.tx_pkt->gimbal_target_pkt.yaw_imu_tar = 0.0f;
        board.tx_pkt->gimbal_target_pkt.pitch_imu_tar = 0.0f;
        board.tx_pkt->gimbal_target_pkt.is_hole = 0u;
        return;
    }

    board.tx_pkt->car_pkt.car_state = 1u;
    keyboard_active = Chassis_Input_IsKeyboardMode();
    keyboard_mech = Chassis_Input_IsKeyboardMechMode();
    uturn_mech = (Chassis_Input_GetUturnState() == CHASSIS_UTURN_POSITION) ? 1u : 0u;
    if (Board_Debug_Hole_Command() != 0u)
    {
        mec_mode_active = 0u;
        return;
    }

    board.tx_pkt->gimbal_target_pkt.yaw_mec_tar = Chassis_Input_GetKeyboardYawTargetRad();
    if ((rc_info->s1.value == RC_SW_DOWN) || (keyboard_mech != 0u) || (uturn_mech != 0u))
    {
        board.tx_pkt->car_pkt.gimbal_mode = 0u;
        if (mec_mode_active == 0u)
        {
            pitch_mec_target = board.rx_meg->gimbal_meg.pitch_mec;
            mec_mode_active = 1u;
        }
        if (keyboard_active == 0u)
        {
            pitch_mec_target += (float)rc_info->ch1 / BOARD_RC_AXIS_MAX * BOARD_MEC_PITCH_STEP_RAD;
            pitch_mec_target = constrain(pitch_mec_target, BOARD_MEC_PITCH_MIN_RAD, BOARD_MEC_PITCH_MAX_RAD);
        }
        board.tx_pkt->gimbal_target_pkt.pitch_mec_tar = pitch_mec_target;
    }
    else
    {
        board.tx_pkt->car_pkt.gimbal_mode = 1u;
        mec_mode_active = 0u;
    }
}
#endif

uint8_t open_ui = 0; /* UI 首次发送延迟标志 */

/* 下板 1 kHz 控制任务，按调试阶段切换控制链路 */

void StartCtrlTask(void const *argument)
{
    (void)argument;

    for (;;)
    {
        Board_Control_InputUpdate();
#if CHASSIS_BRINGUP_ENABLE
        Chassis_Input_Update();
#endif

#if BOARD_COMM_DEBUG
        Board_Debug_Gimbal_Command();
#endif

    /* 第一阶段底盘调试链路 */
#if CHASSIS_BRINGUP_ENABLE
        Chassis_Follow_UpdateMode();
        Chassis_Spin_UpdateMode();
        Chassis_Follow_Update(&chassis_input_cmd);
        Chassis_Spin_Update(&chassis_input_cmd);
        Chassis_Control_Update(&chassis_input_cmd);
        launch.work(&launch);
#elif !BOARD_COMM_DEBUG
        infantry.work(&infantry);

#if BOARD_CAP_ENABLE
        cap.tx();
#endif

#if BOARD_UI_ENABLE
    /* 第一阶段 UI 首帧跳过后再持续刷新 */
        if (open_ui == 0)
        {
            open_ui = 1;
        }
        else
        {
            Ui_Info_Update();
            Ui_Send();
        }
#endif
#endif

#if SUPERCAP_BRINGUP_ENABLE
        SuperCap_Tx();
#endif
        osDelay(1);
    }
}

