/* lift.h - 狗洞升降控制 */

#ifndef __LIFT_H
#define __LIFT_H

#include <stdint.h>

#include "PID.h"
#include "rm_motor.h"

typedef enum
{
    LIFT_WAIT = 0, // 等待使能，状态0
    LIFT_HOMING_UP, // 向上找顶，状态1
    LIFT_RETRACT_DOWN, // 下退卸载，状态2
    LIFT_READY_UP, // 上位就绪，状态3
    LIFT_ALIGN_DOWN, // 前向对齐，状态4
    LIFT_MOVING_DOWN, // 向下运动，状态5
    LIFT_MOVING_UP, // 向上运动，状态6
    LIFT_READY_DOWN, // 下位就绪，状态7
    LIFT_FAULT, // 故障卸力，状态8
    LIFT_STALL_STOP, // 下行堵转卸力，状态9
} lift_state_e;

typedef struct
{
    lift_state_e state; // 升降阶段，0~9
    rm_motor_t *motor; // 升降电机对象指针

    int32_t top_zero; // 找顶零点，配置位置单位
    int32_t top_target; // 上位目标，配置位置单位
    int32_t bottom_target; // 下位目标，配置位置单位
    uint8_t cmd_seen; // 已建立请求基准，0/1
    uint8_t last_is_hole; // 上次过洞请求，0/1
    uint8_t pending_is_hole; // 待生效过洞请求，0/1
    uint8_t pending_valid; // 待生效请求有效，0/1
    uint8_t control_is_hole; // 有效过洞请求，0/1
    uint8_t pitch_zero_hold; // Pitch锁零许可，0/1
    uint8_t fault_code; // 故障原因，0~5
    uint8_t home_valid; // 找顶零点有效，0/1

    uint32_t state_enter_tick; // 阶段进入时刻，ms
    uint32_t last_tick; // 上次控制时刻，ms
    uint32_t homing_confirm_ms; // 找顶连续确认，ms
    uint32_t stable_confirm_ms; // 到位连续确认，ms
    uint32_t overcurrent_confirm_ms; // 低速过流确认，ms
    uint32_t stall_progress_tick; // 位移观察窗起点，ms
    int32_t stall_progress_count; // 位移观察基准，count
    int32_t home_start_count; // 找顶初始编码器，count
    int32_t home_window_count; // 找顶观察基准，count
    uint32_t home_window_tick; // 找顶观察窗起点，ms
    uint32_t init_tick; // 初始化时刻，ms
    uint8_t home_stationary_ok; // 找顶位移静止判据，0/1
    uint8_t home_speed_ok; // 找顶低速判据，0/1
    uint8_t home_current_ok; // 找顶高流判据，0/1

    pid_ctrl_t position_pid; // 位置外环，输出rpm
    pid_ctrl_t speed_pid; // 速度内环，输入rad/s
} lift_t;

extern lift_t lift;

typedef struct
{
    volatile float home_speed_rpm; // 找顶速度幅值，rpm
    volatile float home_output_limit_raw; // 找顶电流限幅，原始量
    volatile float home_current_raw; // 找顶电流门限，原始量
    volatile float home_confirm_ms; // 找顶连续确认阈值，ms
    volatile float home_stall_speed_rpm; // 找顶低速门限，rpm
    volatile float home_speed_grace_ms; // 找顶启动宽限，ms
    volatile float retract_counts; // 顶部回退量，配置位置单位
    volatile float retract_speed_rpm; // 顶部回退速度，rpm
    volatile float retract_output_limit_raw; // 回退电流限幅，原始量
    volatile float move_speed_rpm; // 运动速度上限，rpm
    volatile float pos_kp; // 位置增益，rpm/位置单位
    volatile float pos_out_max_rpm; // 位置环输出限幅，rpm
    volatile float pos_min_speed_rpm; // 非零目标最低速度，rpm
    volatile float speed_kp; // 速度增益，原始量/(rad/s)
    volatile float speed_direction_sign; // 速度反馈方向，仅±1
    volatile float speed_out_max_raw; // 速度环电流限幅，原始量
    volatile float over_current_down_raw; // 下行过流门限，原始量
    volatile float over_current_up_raw; // 上行过流门限，原始量
    volatile float over_current_confirm_ms; // 兼容确认阈值，ms
    volatile float down_over_current_confirm_ms; // 下行过流确认，ms
    volatile float up_over_current_confirm_ms; // 上行过流确认，ms
    volatile float stall_progress_counts; // 最小编码器进展，count
} lift_tune_t;

extern lift_tune_t lift_tune;

typedef struct
{
    /* 0停1下2上3清零4卸力记录 */
    volatile uint8_t mode; // 调试模式，0~4
    volatile uint8_t stats_valid; // 行程统计有效，0/1
    volatile float output_raw; // 调试电流幅值，原始量
    volatile int32_t min_count; // 最小累计编码器，count
    volatile int32_t max_count; // 最大累计编码器，count
    volatile int32_t travel_counts; // 累计编码器极差，count
    volatile uint32_t overcurrent_ms; // 调试过流确认，ms
} lift_debug_t;

extern volatile lift_debug_t lift_debug;

void Lift_Init(void);
void Lift_Work(void);
uint8_t Lift_IsPitchZeroHoldActive(void);
uint8_t Lift_MotorOnline(void);
uint8_t Lift_Get_Report_State(void);

#endif
