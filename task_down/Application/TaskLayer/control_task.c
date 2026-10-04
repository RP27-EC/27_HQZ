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
    volatile uint8_t rc_state;
    volatile uint8_t s1;
    volatile uint8_t s2;
    volatile uint8_t b_value;
    volatile uint8_t button_event;
    volatile uint8_t is_hole;
    volatile uint8_t exit_pending;
    volatile uint8_t hole_rear_blocked; /* 反向时按 B 被拒绝 */
} board_lift_debug_t;

volatile board_lift_debug_t board_lift_dbg;
volatile uint8_t board_hole_request;
volatile uint8_t board_hole_exit_pending;

/* 云台机械模式前后方向状态：0 前，1 后。
 * 放在文件级是因为过洞命令也要判它（反向不允许进狗洞），而且 R 键现在在
 * 任何 S1 位置都要能切换，只有遥控离线才复位。 */
static uint8_t board_gimbal_yaw_rear = 0u;

#if BOARD_COMM_DEBUG
/* 生效中的"前后方向"：键鼠模式看掉头基准，遥控模式看上面那套状态。
 * 过洞的反向拦截必须用生效值，否则键鼠掉头后判断会失效。 */
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
    uint8_t button_event;

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
            /*
             * 狗洞只允许云台正对前方时进入。反向时忽略这次按键、不置位请求，
             * 上板升降因此不会进入对位/下压流程。
             */
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
        /*
         * 过洞强制云台回前方零位，掉头基准也要跟着回前方：否则跟随中心还停在
         * 180deg，云台被拉回前方时跟随环会用 180deg 误差把底盘转过去。
         */
        Chassis_Input_ResetYawReference();
        board.tx_pkt->gimbal_target_pkt.yaw_mec_tar = BOARD_MEC_YAW_FRONT_RAD;
        board.tx_pkt->gimbal_target_pkt.pitch_mec_tar =
            BOARD_HOLE_PITCH_TARGET_RAD;
        return 1u;
    }

    return 0u;
}

/* 调试模式下用遥控右摇杆生成云台机械角目标 */
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
        return;
    }

    board.tx_pkt->car_pkt.car_state = 1u;
    keyboard_active = Chassis_Input_IsKeyboardMode();
    keyboard_mech = Chassis_Input_IsKeyboardMechMode();
    uturn_mech = (Chassis_Input_GetUturnState() == CHASSIS_UTURN_POSITION) ? 1u : 0u;
    r_pressed = ((rc_info->key_v & KEY_PRESSED_OFFSET_R) != 0u) ? 1u : 0u;
#if BOARD_LIFT_ENABLE
    if (Board_Debug_Hole_Command(rc_info) != 0u)
    {
        /* 过洞期间云台被强制正前方，不响应 R；方向状态留到退出后再改 */
        mec_mode_active = 0u;
        last_r_pressed = r_pressed;
        return;
    }
#endif

    /*
     * R 键分工：
     *   键鼠模式：R 归 chassis_input.c 管（机械档翻掉头基准、跟随档起掉头动作），
     *             这里不再动 S1 下位那套前后零位预置，两套状态互不干扰。
     *   遥控模式：保持原行为，R 在任何 S1 位置都生效，切到 S1 下位时立刻用上。
     */
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

    /* 机械模式：S1 下位（遥控）或 键鼠 X 档（云台锁机械零位，看着跟底盘走） */
    if ((rc_info->s1.value == RC_SW_DOWN) || (keyboard_mech != 0u) || (uturn_mech != 0u))
    {
        board.tx_pkt->car_pkt.gimbal_mode = 0u;
        /* 进入机械模式时从当前角度起调，避免跳变 */
        if (mec_mode_active == 0u)
        {
            pitch_mec_target = board.rx_meg->gimbal_meg.pitch_mec;
            mec_mode_active = 1u;
        }

        /* 只有遥控才用右摇杆积分 Pitch。键鼠的 Pitch 走上板 D5 鼠标角速度通路，
         * 机械模式里 D2 的 Pitch 机械目标并不参与运算。 */
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
}
#endif

uint8_t open_ui = 0; /* UI 首次发送延迟标志 */

/* 下板 1 kHz 控制任务，按调试阶段切换控制链路 */

void StartCtrlTask(void const *argument)
{
    (void)argument;

    for (;;)
    {
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

