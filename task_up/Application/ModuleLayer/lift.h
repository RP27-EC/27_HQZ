/* lift.h - 狗洞升降控制 */

#ifndef __LIFT_H
#define __LIFT_H

#include <stdint.h>

#include "PID.h"
#include "rm_motor.h"

typedef enum
{
    LIFT_WAIT = 0,
    LIFT_HOMING_UP,
    LIFT_RETRACT_DOWN,
    LIFT_READY_UP,
    LIFT_ALIGN_DOWN,
    LIFT_MOVING_DOWN,
    LIFT_MOVING_UP,
    LIFT_READY_DOWN,
    LIFT_FAULT,
    LIFT_STALL_STOP,
} lift_state_e;

typedef struct
{
    lift_state_e state;
    rm_motor_t *motor;

    int32_t top_zero;
    int32_t top_target;
    int32_t bottom_target;
    uint8_t cmd_seen;
    uint8_t last_is_hole;
    uint8_t pending_is_hole;
    uint8_t pending_valid;
    uint8_t control_is_hole;
    uint8_t pitch_zero_hold;
    uint8_t fault_code;
    uint8_t home_valid;

    uint32_t state_enter_tick;
    uint32_t last_tick;
    uint32_t homing_confirm_ms;
    uint32_t stable_confirm_ms;
    uint32_t overcurrent_confirm_ms;
    uint32_t stall_progress_tick;
    int32_t stall_progress_count;
    int32_t home_start_count;
    int32_t home_window_count;
    uint32_t home_window_tick;
    uint32_t init_tick;
    uint8_t home_stationary_ok;
    uint8_t home_speed_ok;
    uint8_t home_current_ok;

    pid_ctrl_t position_pid;
    pid_ctrl_t speed_pid;
} lift_t;

extern lift_t lift;

typedef struct
{
    volatile float home_speed_rpm;
    volatile float home_output_limit_raw;
    volatile float home_current_raw;
    volatile float home_confirm_ms;
    volatile float home_stall_speed_rpm;
    volatile float home_speed_grace_ms;
    volatile float retract_counts;
    volatile float retract_speed_rpm;
    volatile float retract_output_limit_raw;
    volatile float move_speed_rpm;
    volatile float pos_kp;
    volatile float pos_out_max_rpm;
    volatile float pos_min_speed_rpm;
    volatile float speed_kp;
    volatile float speed_direction_sign;
    volatile float speed_out_max_raw;
    volatile float over_current_down_raw;
    volatile float over_current_up_raw;
    volatile float over_current_confirm_ms;
    volatile float down_over_current_confirm_ms;
    volatile float up_over_current_confirm_ms;
    volatile float stall_progress_counts;
} lift_tune_t;

extern lift_tune_t lift_tune;

typedef struct
{
    /* mode: 0停止 1向下 2向上 3清零 4卸力记录行程 */
    volatile uint8_t mode;
    volatile uint8_t stats_valid;
    volatile float output_raw;
    volatile int32_t min_count;
    volatile int32_t max_count;
    volatile int32_t travel_counts;
    volatile uint32_t overcurrent_ms;
} lift_debug_t;

extern volatile lift_debug_t lift_debug;

void Lift_Init(void);
void Lift_Work(void);
uint8_t Lift_IsPitchZeroHoldActive(void);
uint8_t Lift_MotorOnline(void);
uint8_t Lift_Get_Report_State(void);

#endif
