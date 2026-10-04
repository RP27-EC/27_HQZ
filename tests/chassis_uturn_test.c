#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "chassis_config.h"
#include "board_comm_config.h"

#define __CHASSIS_CONTROL_H
typedef enum {
    CHASSIS_SRC_NONE = 0, CHASSIS_SRC_RC, CHASSIS_SRC_RC_FOLLOW,
    CHASSIS_SRC_SPIN, CHASSIS_SRC_KEYBOARD
} chassis_source_e;
typedef struct {
    float vx, vy, wz;
    uint8_t valid;
    chassis_source_e source;
} chassis_cmd_t;
#include "chassis_input.h"
#include "chassis_follow.h"

typedef struct { uint16_t value; } test_switch_t;
typedef struct {
    int16_t ch0, ch1, ch2, ch3;
    test_switch_t s1, s2, B, mouse_btn_l, mouse_btn_r;
    uint16_t key_v;
    int16_t mouse_x, mouse_y;
} rc_data_t;
enum { DEV_OFFLINE = 0, DEV_ONLINE = 1 };
static rc_data_t test_rc;
static struct { int work_state; rc_data_t *info; } rc_dev;
typedef struct {
    struct {
        uint8_t car_state, gimbal_mode, vision_mode, game_start, my_color;
        float v_x, v_y;
    } car_pkt;
    struct { uint8_t launch_state, shoot_mode, shoot_level; } shoot_pkt;
    struct {
        float yaw_mec_tar, pitch_mec_tar, yaw_imu_tar, pitch_imu_tar;
        uint8_t is_hole;
    } gimbal_target_pkt;
} test_tx_t;
typedef struct {
    struct { float yaw_mec, pitch_mec, yaw_imu, pitch_imu; } gimbal_meg;
    struct { uint8_t is_down; } state_meg;
} test_rx_t;
typedef struct {
    uint8_t gimbal_data_valid;
    uint32_t gimbal_rx_time_ms;
    uint8_t gimbal_d1_tx_ok, gimbal_d2_tx_ok;
} test_status_t;
typedef struct {
    test_tx_t *tx_pkt;
    test_rx_t *rx_meg;
    test_status_t *status;
} Board_t;
static test_tx_t test_tx;
static test_rx_t test_rx;
static test_status_t test_status;
static Board_t board = { &test_tx, &test_rx, &test_status };
static uint32_t test_tick;
static uint32_t HAL_GetTick(void) { return test_tick; }
static float constrain(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}
typedef int HAL_StatusTypeDef;
typedef struct { uint32_t ECR, CCCR; } test_can_registers_t;
typedef struct { test_can_registers_t *Instance; } FDCAN_HandleTypeDef;
typedef struct {
    uint32_t IdType, Identifier, DataLength, ErrorStateIndicator, BitRateSwitch;
    uint32_t FDFormat, TxFrameType, MessageMarker, TxEventFifoControl;
} FDCAN_TxHeaderTypeDef;
enum {
    FDCAN_STANDARD_ID = 0, FDCAN_DLC_BYTES_8 = 8, FDCAN_ESI_ACTIVE = 0,
    FDCAN_BRS_OFF = 0, FDCAN_CLASSIC_CAN = 0, FDCAN_DATA_FRAME = 0,
    FDCAN_NO_TX_EVENTS = 0, FDCAN_CCCR_INIT = 1
};
#define CLEAR_BIT(reg, bits) ((reg) &= ~(bits))
static test_can_registers_t test_can_registers;
static FDCAN_HandleTypeDef hfdcan2 = { &test_can_registers };
static uint8_t pkt_01[8], pkt_02[8];
static uint8_t pkt_05[8];
static uint8_t sent_d5[8];
enum { HAL_OK = 0, HAL_ERROR = 1 };
static uint32_t failed_id;
static int communication_enabled;
static uint16_t float_to_uint(float x, float lo, float hi, int bits) {
    return (uint16_t)((constrain(x, lo, hi) - lo) * ((1u << bits) - 1u) / (hi - lo));
}
static HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *can,
    FDCAN_TxHeaderTypeDef *header, uint8_t *data) {
    (void)can;
    if (header->Identifier == failed_id) return HAL_ERROR;
    if (header->Identifier == 0xD5u) memcpy(sent_d5, data, sizeof(sent_d5));
    return HAL_OK;
}
volatile float board_manual_yaw_rate_deg_s;
extern volatile uint8_t board_hole_request;
extern volatile uint8_t board_hole_exit_pending;
static uint8_t Chassis_Spin_IsSelected(void) { return 0u; }

#include "production.inc"

#define CHECK(condition) do { \
    if (!(condition)) { \
        printf("  FAIL line %d: %s\n", __LINE__, #condition); \
        return 0; \
    } \
} while (0)
#define PI 3.14159265358979323846f

static void step(uint16_t keys, float yaw_rad, uint32_t elapsed, int fresh) {
    test_tick += elapsed;
    test_rc.key_v = keys;
    test_rx.gimbal_meg.yaw_mec = yaw_rad;
    test_rx.gimbal_meg.yaw_imu = yaw_rad * (180.0f / PI);
    if (fresh) {
        test_status.gimbal_data_valid = 1u;
        test_status.gimbal_rx_time_ms = test_tick;
    }
    Chassis_Input_Update();
    Board_Debug_Gimbal_Command();
    if (communication_enabled) {
        Board_Tx_Pkt_01(&board);
        Board_Tx_Pkt_02(&board);
        Board_Tx_Pkt_05(&board);
    }
}

static void reset_fixture(void) {
    memset(&test_rc, 0, sizeof(test_rc));
    memset(&test_tx, 0, sizeof(test_tx));
    memset(&test_rx, 0, sizeof(test_rx));
    memset(&test_status, 0, sizeof(test_status));
    test_tick += 3000u;
    failed_id = 0u;
    communication_enabled = 1;
    rc_dev.info = &test_rc;
    rc_dev.work_state = DEV_OFFLINE;
    board_hole_request = 0u;
    board_hole_exit_pending = 0u;
    Board_Debug_Hole_Command(&test_rc);
    Chassis_Input_Init();
    Chassis_Follow_Init();
    Board_Debug_Gimbal_Command();
    rc_dev.work_state = DEV_ONLINE;
    test_rc.s1.value = RC_SW_UP;
    test_rc.s2.value = RC_SW_UP;
    step(KEY_PRESSED_OFFSET_F, 0.0f, 1u, 1);
    step(0u, 0.0f, 1u, 1);
}

static void start_turn(void) {
    step(KEY_PRESSED_OFFSET_R, 0.0f, 1u, 1);
    step(0u, 0.0f, 1u, 1);
    step(0u, 0.0f, 1u, 1);
}

static void settle(float yaw_rad) {
    unsigned i;
    for (i = 0; i < 105u; ++i) step(0u, yaw_rad, 1u, 1);
    step(0u, yaw_rad, 3u, 1);
}

static int mechanical_direct(void) {
    reset_fixture();
    step(KEY_PRESSED_OFFSET_X, 0.0f, 1u, 1);
    step(KEY_PRESSED_OFFSET_R, 0.0f, 1u, 1);
    CHECK(test_tx.car_pkt.gimbal_mode == 0u);
    CHECK(fabsf(test_tx.gimbal_target_pkt.yaw_mec_tar - PI) < 0.0001f);
    CHECK(Chassis_Input_IsUturnActive() == 0u);
    return 1;
}

static int prepare_then_mechanical(void) {
    reset_fixture();
    step(KEY_PRESSED_OFFSET_R, 0.0f, 1u, 1);
    CHECK(test_tx.car_pkt.gimbal_mode == 1u);
    CHECK(fabsf(test_tx.gimbal_target_pkt.yaw_mec_tar - PI) < 0.0001f);
    CHECK(sent_d5[2] == 0u && sent_d5[3] == 0u);
    step(0u, 0.0f, 1u, 1);
    CHECK(test_tx.car_pkt.gimbal_mode == 1u);
    step(0u, 0.0f, 1u, 1);
    CHECK(test_tx.car_pkt.gimbal_mode == 0u);
    CHECK(Chassis_Input_GetKeyboardChassisMode() == CHASSIS_KEY_MODE_FOLLOW);
    return 1;
}

static int stable_arrival_and_restore(void) {
    unsigned i;
    reset_fixture();
    start_turn();
    for (i = 0; i < 100u; ++i) {
        step(0u, PI - 0.003f, 1u, 1);
        CHECK(test_tx.car_pkt.gimbal_mode == 0u);
    }
    step(0u, PI - 0.003f, 1u, 1);
    CHECK(test_tx.car_pkt.gimbal_mode == 1u);
    CHECK(Chassis_Input_IsUturnActive() != 0u);
    CHECK(fabsf(Chassis_Input_GetYawReferenceRad() - PI) < 0.0001f);
    step(0u, PI - 0.003f, 1u, 1);
    CHECK(Chassis_Input_IsUturnActive() != 0u);
    step(0u, PI - 0.003f, 1u, 1);
    CHECK(Chassis_Input_IsUturnActive() == 0u);
    return 1;
}

static int wrapped_rear_arrival(void) {
    reset_fixture();
    start_turn();
    settle(-PI + 0.003f);
    CHECK(Chassis_Input_IsUturnActive() == 0u);
    CHECK(test_tx.car_pkt.gimbal_mode == 1u);
    CHECK(fabsf(Chassis_Input_GetYawReferenceRad() - PI) < 0.0001f);
    return 1;
}

static int unstable_samples_reset_dwell(void) {
    unsigned i;
    reset_fixture();
    start_turn();
    for (i = 0; i < 80u; ++i) step(0u, PI, 1u, 1);
    step(0u, PI - 0.02f, 1u, 1);
    for (i = 0; i < 80u; ++i) step(0u, PI, 1u, 1);
    CHECK(test_tx.car_pkt.gimbal_mode == 0u);
    settle(PI);
    CHECK(Chassis_Input_IsUturnActive() == 0u);
    return 1;
}

static int duplicate_samples_do_not_arrive(void) {
    unsigned i;
    reset_fixture();
    start_turn();
    step(0u, PI, 1u, 1);
    for (i = 0; i < 45u; ++i) step(0u, PI, 1u, 0);
    CHECK(test_tx.car_pkt.gimbal_mode == 0u);
    step(0u, PI, 10u, 0);
    CHECK(Chassis_Input_IsUturnActive() == 0u);
    step(0u, PI, 1u, 1);
    CHECK(Chassis_Input_IsUturnActive() == 0u);
    return 1;
}

static int timeout_does_not_shift_next_endpoint(void) {
    reset_fixture();
    start_turn();
    step(0u, 0.7f, 2500u, 1);
    CHECK(test_tx.car_pkt.gimbal_mode == 1u);
    CHECK(fabsf(Chassis_Input_GetYawReferenceRad() - 0.7f) < 0.0001f);
    CHECK(Chassis_Input_IsKeyboardYawRear() != 0u);
    step(0u, 0.7f, 1u, 1);
    step(0u, 0.7f, 2u, 1);
    step(KEY_PRESSED_OFFSET_R, 0.7f, 1u, 1);
    CHECK(fabsf(test_tx.gimbal_target_pkt.yaw_mec_tar) < 0.0001f);
    return 1;
}

static int repeated_r_ignored_and_mouse_pitch_preserved(void) {
    reset_fixture();
    test_rc.mouse_x = 20;
    test_rc.mouse_y = 5;
    start_turn();
    step(KEY_PRESSED_OFFSET_R, 0.3f, 1u, 1);
    CHECK(fabsf(test_tx.gimbal_target_pkt.yaw_mec_tar - PI) < 0.0001f);
    CHECK(sent_d5[2] == 0u && sent_d5[3] == 0u);
    CHECK((int16_t)(((uint16_t)sent_d5[4] << 8) | sent_d5[5]) == -150);
    return 1;
}

static int manual_modes_cancel(void) {
    reset_fixture();
    start_turn();
    step(KEY_PRESSED_OFFSET_X, 0.2f, 1u, 1);
    CHECK(Chassis_Input_IsUturnActive() == 0u);
    CHECK(test_tx.car_pkt.gimbal_mode == 0u);
    settle(PI);
    CHECK(test_tx.car_pkt.gimbal_mode == 0u);
    reset_fixture();
    start_turn();
    step(KEY_PRESSED_OFFSET_C, 0.2f, 1u, 1);
    CHECK(Chassis_Input_IsUturnActive() == 0u);
    reset_fixture();
    start_turn();
    step(KEY_PRESSED_OFFSET_F, 0.2f, 1u, 1);
    CHECK(Chassis_Input_IsUturnActive() == 0u);
    reset_fixture();
    start_turn();
    test_rc.s1.value = RC_SW_DOWN;
    step(0u, 0.2f, 1u, 1);
    CHECK(Chassis_Input_IsUturnActive() == 0u);
    return 1;
}

static int safety_and_hole_cancel(void) {
    reset_fixture();
    start_turn();
    Chassis_Input_ResetYawReference();
    CHECK(Chassis_Input_IsUturnActive() == 0u);
    CHECK(Chassis_Input_IsKeyboardYawRear() == 0u);
    step(0u, 0.0f, 1u, 1);
    board_hole_request = 1u;
    step(KEY_PRESSED_OFFSET_R, 0.0f, 1u, 1);
    CHECK(Chassis_Input_IsUturnActive() == 0u);
    CHECK(fabsf(test_tx.gimbal_target_pkt.yaw_mec_tar) < 0.0001f);
    board_hole_request = 0u;
    step(KEY_PRESSED_OFFSET_R, 0.0f, 1u, 1);
    CHECK(Chassis_Input_IsUturnActive() == 0u);
    reset_fixture();
    start_turn();
    rc_dev.work_state = DEV_OFFLINE;
    step(KEY_PRESSED_OFFSET_R, 0.2f, 1u, 1);
    CHECK(Chassis_Input_IsUturnActive() == 0u);
    CHECK(test_tx.car_pkt.car_state == 0u);
    return 1;
}

static int twenty_turns_keep_fixed_endpoints(void) {
    unsigned i;
    float yaw = 0.0f;
    reset_fixture();
    for (i = 0; i < 20u; ++i) {
        float expected = (i % 2u == 0u) ? PI : 0.0f;
        step(KEY_PRESSED_OFFSET_R, yaw, 1u, 1);
        CHECK(fabsf(test_tx.gimbal_target_pkt.yaw_mec_tar - expected) < 0.0001f);
        step(0u, yaw, 2u, 1);
        yaw = expected - 0.003f;
        settle(yaw);
        CHECK(Chassis_Input_IsUturnActive() == 0u);
        CHECK(fabsf(Chassis_Input_GetYawReferenceRad() - expected) < 0.0001f);
    }
    return 1;
}

static int frozen_follow_keeps_manual_rotation(void) {
    reset_fixture();
    start_turn();
    step(KEY_PRESSED_OFFSET_W | KEY_PRESSED_OFFSET_Q, 0.0f, 1u, 1);
    Chassis_Follow_UpdateMode();
    Chassis_Follow_Update(&chassis_input_cmd);
    CHECK(chassis_input_cmd.valid != 0u);
    CHECK(chassis_input_cmd.vx < -1.0f);
    CHECK(chassis_input_cmd.wz > 0.0f);
    CHECK(chassis_follow.wz_target == 0.0f);
    return 1;
}

static int no_transmission_does_not_complete_handoff(void) {
    reset_fixture();
    communication_enabled = 0;
    step(KEY_PRESSED_OFFSET_R, 0.0f, 1u, 1);
    step(0u, 0.0f, 10u, 1);
    CHECK(test_tx.car_pkt.gimbal_mode == 1u);
    communication_enabled = 1;
    step(0u, 0.0f, 1u, 1);
    step(0u, 0.0f, 1u, 1);
    CHECK(test_tx.car_pkt.gimbal_mode == 1u);
    step(0u, 0.0f, 1u, 1);
    CHECK(test_tx.car_pkt.gimbal_mode == 0u);
    communication_enabled = 0;
    settle(PI);
    CHECK(Chassis_Input_IsUturnActive() != 0u);
    communication_enabled = 1;
    step(0u, PI, 1u, 1);
    step(0u, PI, 1u, 1);
    CHECK(Chassis_Input_IsUturnActive() != 0u);
    step(0u, PI, 1u, 1);
    CHECK(Chassis_Input_IsUturnActive() == 0u);
    return 1;
}

static int stale_target_preemption_not_counted(void) {
    reset_fixture();
    test_tick++;
    test_status.gimbal_rx_time_ms = test_tick;
    test_rc.key_v = KEY_PRESSED_OFFSET_R;
    Chassis_Input_Update();
    Board_Tx_Pkt_01(&board);
    Board_Tx_Pkt_02(&board);
    Board_Tx_Pkt_05(&board);
    step(0u, 0.0f, 2u, 1);
    CHECK(test_tx.car_pkt.gimbal_mode == 1u);
    step(0u, 0.0f, 1u, 1);
    CHECK(test_tx.car_pkt.gimbal_mode == 1u);
    step(0u, 0.0f, 1u, 1);
    CHECK(test_tx.car_pkt.gimbal_mode == 0u);
    return 1;
}

static int failed_frame_not_counted(void) {
    uint32_t ids[] = { 0xD1u, 0xD2u, 0xD5u };
    unsigned i;
    for (i = 0; i < 3u; ++i) {
        reset_fixture();
        failed_id = ids[i];
        step(KEY_PRESSED_OFFSET_R, 0.0f, 1u, 1);
        step(0u, 0.0f, 5u, 1);
        CHECK(test_tx.car_pkt.gimbal_mode == 1u);
        failed_id = 0u;
        step(0u, 0.0f, 1u, 1);
        step(0u, 0.0f, 1u, 1);
        CHECK(test_tx.car_pkt.gimbal_mode == 1u);
        step(0u, 0.0f, 1u, 1);
        CHECK(test_tx.car_pkt.gimbal_mode == 0u);
    }
    return 1;
}

static int feedback_gap_resets_stability(void) {
    unsigned i;
    reset_fixture();
    start_turn();
    for (i = 0; i < 80u; ++i) step(0u, PI, 1u, 1);
    step(0u, PI, 70u, 1);
    CHECK(test_tx.car_pkt.gimbal_mode == 0u);
    settle(PI);
    CHECK(Chassis_Input_IsUturnActive() == 0u);
    return 1;
}

static int mechanical_entry_restores_fixed_follow_center(void) {
    reset_fixture();
    start_turn();
    step(0u, 0.7f, 2500u, 1);
    step(0u, 0.7f, 3u, 1);
    step(KEY_PRESSED_OFFSET_X, 0.7f, 1u, 1);
    CHECK(fabsf(Chassis_Input_GetYawReferenceRad() - PI) < 0.0001f);
    step(KEY_PRESSED_OFFSET_Z, PI, 1u, 1);
    CHECK(fabsf(Chassis_Input_GetYawReferenceRad() - PI) < 0.0001f);
    return 1;
}

static int held_r_and_tick_wrap(void) {
    unsigned i;
    reset_fixture();
    test_tick = UINT32_MAX - 5u;
    step(KEY_PRESSED_OFFSET_R, 0.0f, 1u, 1);
    step(KEY_PRESSED_OFFSET_R, 0.0f, 1u, 1);
    step(KEY_PRESSED_OFFSET_R, 0.0f, 1u, 1);
    for (i = 0; i < 105u; ++i) step(KEY_PRESSED_OFFSET_R, PI, 1u, 1);
    step(KEY_PRESSED_OFFSET_R, PI, 3u, 1);
    CHECK(Chassis_Input_IsUturnActive() == 0u);
    CHECK(Chassis_Input_GetUturnResult() == CHASSIS_UTURN_DONE);
    CHECK(Chassis_Input_IsKeyboardYawRear() != 0u);
    step(KEY_PRESSED_OFFSET_R, PI, 10u, 1);
    CHECK(Chassis_Input_IsUturnActive() == 0u);
    return 1;
}

static int can_driver_propagates_queue_failure(void) {
    uint8_t data[8] = { 0u };
    reset_fixture();
    failed_id = 0xD2u;
    CHECK(CAN_SendData(&hfdcan2, 0xD2u, data) == HAL_ERROR);
    failed_id = 0u;
    CHECK(CAN_SendData(&hfdcan2, 0xD2u, data) == HAL_OK);
    return 1;
}

int main(void) {
    struct { const char *name; int (*run)(void); } cases[] = {
        { "mechanical_direct", mechanical_direct },
        { "prepare_then_mechanical", prepare_then_mechanical },
        { "stable_arrival_and_restore", stable_arrival_and_restore },
        { "wrapped_rear_arrival", wrapped_rear_arrival },
        { "unstable_samples_reset_dwell", unstable_samples_reset_dwell },
        { "duplicate_samples_do_not_arrive", duplicate_samples_do_not_arrive },
        { "timeout_does_not_shift_next_endpoint", timeout_does_not_shift_next_endpoint },
        { "repeated_r_ignored_and_mouse_pitch_preserved", repeated_r_ignored_and_mouse_pitch_preserved },
        { "manual_modes_cancel", manual_modes_cancel },
        { "safety_and_hole_cancel", safety_and_hole_cancel },
        { "twenty_turns_keep_fixed_endpoints", twenty_turns_keep_fixed_endpoints },
        { "frozen_follow_keeps_manual_rotation", frozen_follow_keeps_manual_rotation },
        { "no_transmission_does_not_complete_handoff", no_transmission_does_not_complete_handoff },
        { "stale_target_preemption_not_counted", stale_target_preemption_not_counted },
        { "failed_frame_not_counted", failed_frame_not_counted },
        { "feedback_gap_resets_stability", feedback_gap_resets_stability },
        { "mechanical_entry_restores_fixed_follow_center", mechanical_entry_restores_fixed_follow_center },
        { "held_r_and_tick_wrap", held_r_and_tick_wrap },
        { "can_driver_propagates_queue_failure", can_driver_propagates_queue_failure }
    };
    unsigned i, passed = 0u;
    for (i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        int ok = cases[i].run();
        printf("%s %s\n", ok ? "PASS" : "FAIL", cases[i].name);
        passed += ok != 0;
    }
    printf("%u/%u passed\n", passed, (unsigned)(sizeof(cases) / sizeof(cases[0])));
    return passed == sizeof(cases) / sizeof(cases[0]) ? 0 : 1;
}
