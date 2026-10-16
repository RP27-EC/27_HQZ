/* lift.h - 狗洞升降控制 */

#ifndef __LIFT_H
#define __LIFT_H

#include <stdint.h>

#include "PID.h"
#include "rm_motor.h"

typedef enum
{
    LIFT_WAIT = 0, // 等待使能，值0
    LIFT_HOMING_UP, // 找顶标定，值1
    LIFT_RETRACT_DOWN, // 找顶回退，值2
    LIFT_READY_UP, // 顶部保持，值3
    LIFT_ALIGN_DOWN, // 下降对齐，值4
    LIFT_MOVING_DOWN, // 下行运动，值5
    LIFT_MOVING_UP, // 上行运动，值6
    LIFT_READY_DOWN, // 底部保持，值7
    LIFT_FAULT, // 硬故障停机，值8
    LIFT_STALL_STOP, // 旧堵停状态，值9
    LIFT_PAUSED, // 可恢复停机，值10
} lift_state_e;

typedef struct
{
    lift_state_e state; // 升降阶段，0~10
    rm_motor_t *motor; // 升降驱动对象，非空

    int32_t top_zero; // 物理顶部，count
    int32_t top_target; // 顶部保持目标，count
    int32_t bottom_target; // 底部保持目标，count
    uint8_t cmd_seen; // 请求基准有效，0/1
    uint8_t last_is_hole; // 上拍下降请求，0/1
    uint8_t pending_is_hole; // 待处理下降请求，0/1
    uint8_t pending_valid; // 待处理请求有效，0/1
    uint8_t control_is_hole; // 有效下降请求，0/1
    uint8_t pitch_zero_hold; // 俯仰锁零请求，0/1
    uint8_t fault_code; // 最近保护原因，0~7
    uint8_t home_valid; // 行程标定有效，0/1

    uint32_t state_enter_tick; // 阶段起点，ms
    uint32_t last_tick; // 上次控制时刻，ms
    uint32_t homing_confirm_ms; // 找顶连续确认，ms
    uint32_t stable_confirm_ms; // 到位连续确认，ms
    uint32_t overcurrent_confirm_ms; // 运动过流持续，ms
    uint32_t stall_progress_tick; // 进展窗口起点，ms
    int32_t stall_progress_count; // 窗口起始位置，count
    int32_t home_start_count; // 找顶起始位置，count
    int32_t home_window_count; // 找顶窗口位置，count
    uint32_t home_window_tick; // 找顶观察起点，ms
    uint32_t init_tick; // 初始化时刻，ms
    uint8_t home_stationary_ok; // 找顶位置稳定，0/1
    uint8_t home_speed_ok; // 找顶速度低，0/1
    uint8_t home_current_ok; // 找顶电流达限，0/1

    pid_ctrl_t position_pid; // 位置环，rpm/count
    pid_ctrl_t speed_pid; // 速度环，raw/(rad/s)
    float speed_target_rpm; // 轨迹目标速度，rpm
    uint32_t overtravel_ms; // 严重越程持续，ms
    uint32_t hold_current_ms; // 保持高流持续，ms
    uint32_t fault_count; // 保护累计次数，次
    volatile uint8_t recover_request; // Watch恢复请求，0/1
} lift_t;

extern lift_t lift;

typedef struct
{
    volatile float home_speed_rpm; // 找顶速度，rpm
    volatile float home_output_limit_raw; // 找顶输出上限，raw
    volatile float home_current_raw; // 找顶反馈电流，raw
    volatile float home_confirm_ms; // 找顶确认时间，ms
    volatile float home_stall_speed_rpm; // 找顶低速门限，rpm
    volatile float home_speed_grace_ms; // 找顶启动宽限，ms
    volatile float retract_counts; // 顶部回退位移，count
    volatile float retract_speed_rpm; // 回退速度上限，rpm
    volatile float retract_output_limit_raw; // 回退输出上限，raw
    volatile float move_speed_rpm; // 运动速度上限，rpm
    volatile float pos_kp; // 保持位置增益，rpm/count
    volatile float pos_out_max_rpm; // 位置输出上限，rpm
    volatile float pos_min_speed_rpm; // 保留旧最低速度，rpm
    volatile float speed_kp; // 速度增益，raw/(rad/s)
    volatile float speed_direction_sign; // 速度反馈符号，±1
    volatile float speed_out_max_raw; // 运动输出上限，raw
    volatile float over_current_down_raw; // 下行高流门限，raw
    volatile float over_current_up_raw; // 上行高流门限，raw
    volatile float over_current_confirm_ms; // 兼容过流确认，ms
    volatile float down_over_current_confirm_ms; // 下行过流确认，ms
    volatile float up_over_current_confirm_ms; // 上行过流确认，ms
    volatile float stall_progress_counts; // 最小运动进展，count
} lift_tune_t;

extern lift_tune_t lift_tune;

typedef struct
{
    /* mode: 0停止 1向下 2向上 3清零 4卸力记录行程 */
    volatile uint8_t mode; // 手动模式，0~4
    volatile uint8_t stats_valid; // 行程记录有效，0/1
    volatile float output_raw; // 手动驱动量，raw
    volatile int32_t min_count; // 记录最小位置，count
    volatile int32_t max_count; // 记录最大位置，count
    volatile int32_t travel_counts; // 记录行程，count
    volatile uint32_t overcurrent_ms; // 手动过流持续，ms
} lift_debug_t;

extern volatile lift_debug_t lift_debug;

void Lift_Init(void);
void Lift_Work(void);
uint8_t Lift_IsPitchZeroHoldActive(void);
uint8_t Lift_MotorOnline(void);
uint8_t Lift_Get_Report_State(void);

#endif
