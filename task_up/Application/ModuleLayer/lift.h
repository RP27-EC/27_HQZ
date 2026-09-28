/* lift.h - 狗洞升降控制 */

#ifndef __LIFT_H
#define __LIFT_H

#include <stdint.h>

#include "PID.h"
#include "rm_motor.h"

typedef enum
{
    LIFT_WAIT = 0,  // 等待上电找零
    LIFT_HOMING_UP,  // 上行找顶
    LIFT_RETRACT_DOWN,  // 到顶后回转
    LIFT_READY_UP,  // 顶部待命
    LIFT_ALIGN_DOWN,  // 下降前对齐云台
    LIFT_MOVING_DOWN,  // 下降中
    LIFT_MOVING_UP,  // 上升中
    LIFT_READY_DOWN,  // 底部待命
    LIFT_FAULT,  // 故障锁定
    LIFT_STALL_STOP,  // 堵转停机
} lift_state_e;

typedef struct
{
    lift_state_e state;
    rm_motor_t *motor;  // 绑定的升降电机

    int32_t top_zero;  // 顶部零点计数
    int32_t top_target;  // 顶部目标计数
    int32_t bottom_target;  // 底部目标计数
    uint8_t cmd_seen;  // 收到过狗洞指令
    uint8_t last_is_hole;  // 上拍狗洞状态
    uint8_t pending_is_hole;  // 待执行狗洞
    uint8_t pending_valid;  // 待执行有效
    uint8_t control_is_hole;  // 当前按狗洞控制
    uint8_t fault_code;  // 故障码
    uint8_t home_valid;  // 找零完成

    uint32_t state_enter_tick;  // 状态进入时刻
    uint32_t last_tick;  // 上拍时刻
    uint32_t homing_confirm_ms;  // 找顶条件确认计时
    uint32_t stable_confirm_ms;  // 到位稳定计时
    uint32_t overcurrent_confirm_ms;  // 过流确认计时
    uint32_t stall_progress_tick;  // 无进展计时起点
    int32_t stall_progress_count;  // 无进展基准计数
    int32_t home_start_count;  // 找零起始计数
    int32_t home_window_count;  // 静止窗口基准计数
    uint32_t home_window_tick;  // 静止窗口起点
    uint32_t init_tick;  // 初始化时刻
    uint8_t home_stationary_ok;  // 静止条件满足
    uint8_t home_speed_ok;  // 低速条件满足
    uint8_t home_current_ok;  // 电流条件满足

    pid_ctrl_t position_pid;  // 位置环
    pid_ctrl_t speed_pid;  // 速度环
} lift_t;

extern lift_t lift;

typedef struct
{
    volatile float home_speed_rpm;  // 找顶速度
    volatile float home_output_limit_raw;  // 找顶输出限幅
    volatile float home_current_raw;  // 找顶电流阈值
    volatile float home_confirm_ms;  // 找顶确认时长
    volatile float home_stall_speed_rpm;  // 顶部低速阈值
    volatile float home_speed_grace_ms;  // 启动宽限时长
    volatile float retract_counts;  // 到顶回转量
    volatile float retract_speed_rpm;  // 回转速度
    volatile float retract_output_limit_raw;  // 回转输出限幅
    volatile float move_speed_rpm;  // 运动速度
    volatile float pos_kp;  // 位置环 P
    volatile float pos_out_max_rpm;  // 位置环输出限速
    volatile float pos_min_speed_rpm;  // 最小爬行速度
    volatile float speed_kp;  // 速度环 P
    volatile float speed_direction_sign;  // 速度方向修正
    volatile float speed_out_max_raw;  // 速度环输出限幅
    volatile float over_current_down_raw;  // 下行堵转电流
    volatile float over_current_up_raw;  // 上行堵转电流
    volatile float over_current_confirm_ms;  // 堵转确认时长
    volatile float down_over_current_confirm_ms;  // 下行堵转确认
    volatile float up_over_current_confirm_ms;  // 上行堵转确认
    volatile float stall_progress_counts;  // 无进展判定阈值
} lift_tune_t;

extern lift_tune_t lift_tune;

typedef struct
{
    /* mode: 0停止 1向下 2向上 3清零 4卸力记录行程 */
    volatile uint8_t mode;
    volatile uint8_t stats_valid;  // 统计有效
    volatile float output_raw;  // 当前输出
    volatile int32_t min_count;  // 行程最小值
    volatile int32_t max_count;  // 行程最大值
    volatile int32_t travel_counts;  // 总行程
    volatile uint32_t overcurrent_ms;  // 过流累计时长
} lift_debug_t;

extern volatile lift_debug_t lift_debug;

void Lift_Init(void);
void Lift_Work(void);
uint8_t Lift_MotorOnline(void);
uint8_t Lift_Get_Report_State(void);

#endif
