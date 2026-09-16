#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "gimbal.h"

#define TEST_PI 3.14159265358979323846f
#define DEG_TO_RAD (TEST_PI / 180.0f)

static gimbal_input_t gimbal_inputs_online(void)
{
    gimbal_input_t inputs = {0};

    inputs.yaw_motor_online = true;
    inputs.pitch_motor_online = true;
    inputs.imu_online = true;
    inputs.imu_calibrated = true;
    inputs.rc_online = true;

    inputs.yaw_mec_angle = 0.0f;
    inputs.yaw_mec_speed = 0.0f;
    inputs.pitch_mec_angle = 0.0f;
    inputs.pitch_mec_speed = 0.0f;

    inputs.yaw_imu_angle = 0.0f;
    inputs.yaw_imu_speed = 0.0f;
    inputs.pitch_imu_angle = 0.0f;
    inputs.pitch_imu_speed = 0.0f;

    return inputs;
}

static void test_yaw_target_takes_shortest_path(void)
{
    gimbal_t gimbal;
    gimbal_input_t inputs = gimbal_inputs_online();
    gimbal_output_t output;

    Gimbal_Init(&gimbal);
    Gimbal_SetMode(&gimbal, G_MEC);

    inputs.yaw_mec_angle = 170.0f * DEG_TO_RAD;
    inputs.yaw_target = -170.0f * DEG_TO_RAD;

    Gimbal_Update(&gimbal, &inputs, 0.001f, &output);
    Gimbal_Update(&gimbal, &inputs, 0.001f, &output);

    assert(fabsf(gimbal.yaw_target - 190.0f * DEG_TO_RAD) < 0.001f);
}

static void test_pitch_target_is_clamped_to_mechanical_limits(void)
{
    gimbal_t gimbal;
    gimbal_input_t inputs = gimbal_inputs_online();
    gimbal_output_t output;

    Gimbal_Init(&gimbal);
    Gimbal_SetMode(&gimbal, G_MEC);

    inputs.pitch_target = 90.0f * DEG_TO_RAD;

    Gimbal_Update(&gimbal, &inputs, 0.001f, &output);
    Gimbal_Update(&gimbal, &inputs, 0.001f, &output);

    assert(gimbal.pitch_target <= 30.0f * DEG_TO_RAD + 0.0001f);
    assert(gimbal.pitch_target >= -10.0f * DEG_TO_RAD - 0.0001f);
}

static void test_mode_switch_outputs_zero_and_takes_current_feedback(void)
{
    gimbal_t gimbal;
    gimbal_input_t inputs = gimbal_inputs_online();
    gimbal_output_t output;

    Gimbal_Init(&gimbal);
    Gimbal_SetMode(&gimbal, G_MEC);
    Gimbal_Update(&gimbal, &inputs, 0.001f, &output);

    inputs.yaw_imu_angle = 0.25f;
    inputs.pitch_imu_angle = 0.10f;
    Gimbal_SetMode(&gimbal, G_GYRO);
    Gimbal_Update(&gimbal, &inputs, 0.001f, &output);

    assert(fabsf(output.yaw_torque) < 0.0001f);
    assert(fabsf(output.pitch_torque) < 0.0001f);
    assert(fabsf(gimbal.yaw_target - inputs.yaw_imu_angle) < 0.0001f);
    assert(fabsf(gimbal.pitch_target - inputs.pitch_imu_angle) < 0.0001f);
}

static void test_offline_motor_forces_zero_output(void)
{
    gimbal_t gimbal;
    gimbal_input_t inputs = gimbal_inputs_online();
    gimbal_output_t output;

    Gimbal_Init(&gimbal);
    Gimbal_SetMode(&gimbal, G_MEC);
    Gimbal_Update(&gimbal, &inputs, 0.001f, &output);

    inputs.pitch_motor_online = false;
    inputs.pitch_target = 10.0f * DEG_TO_RAD;
    Gimbal_Update(&gimbal, &inputs, 0.001f, &output);

    assert(gimbal.mode == G_SLEEP);
    assert(fabsf(output.yaw_torque) < 0.0001f);
    assert(fabsf(output.pitch_torque) < 0.0001f);
}

static void test_gyro_mode_requires_calibrated_imu(void)
{
    gimbal_t gimbal;
    gimbal_input_t inputs = gimbal_inputs_online();
    gimbal_output_t output;

    Gimbal_Init(&gimbal);
    Gimbal_SetMode(&gimbal, G_GYRO);

    inputs.imu_calibrated = false;
    Gimbal_Update(&gimbal, &inputs, 0.001f, &output);

    assert(gimbal.mode == G_SLEEP);
    assert(fabsf(output.yaw_torque) < 0.0001f);
    assert(fabsf(output.pitch_torque) < 0.0001f);
}

static void test_pid_history_is_not_reset_every_control_period(void)
{
    gimbal_t gimbal;
    gimbal_input_t inputs = gimbal_inputs_online();
    gimbal_output_t output;
    float first_integral;

    Gimbal_Init(&gimbal);
    Gimbal_SetMode(&gimbal, G_MEC);

    inputs.yaw_target = 10.0f * DEG_TO_RAD;

    Gimbal_Update(&gimbal, &inputs, 0.001f, &output);
    Gimbal_Update(&gimbal, &inputs, 0.001f, &output);
    first_integral = gimbal.yaw_rate_pid.integral;

    Gimbal_Update(&gimbal, &inputs, 0.001f, &output);
    assert(fabsf(gimbal.yaw_rate_pid.integral) > fabsf(first_integral));
}

static void test_sleep_protection_count_increments_once_per_fault(void)
{
    gimbal_t gimbal;
    gimbal_input_t inputs = gimbal_inputs_online();
    gimbal_output_t output;

    Gimbal_Init(&gimbal);
    Gimbal_SetMode(&gimbal, G_MEC);
    Gimbal_Update(&gimbal, &inputs, 0.001f, &output);

    inputs.yaw_motor_online = false;
    Gimbal_Update(&gimbal, &inputs, 0.001f, &output);
    Gimbal_Update(&gimbal, &inputs, 0.001f, &output);

    assert(gimbal.safety_trip_count == 1U);
}

int main(void)
{
    test_yaw_target_takes_shortest_path();
    test_pitch_target_is_clamped_to_mechanical_limits();
    test_mode_switch_outputs_zero_and_takes_current_feedback();
    test_offline_motor_forces_zero_output();
    test_gyro_mode_requires_calibrated_imu();
    test_pid_history_is_not_reset_every_control_period();
    test_sleep_protection_count_increments_once_per_fault();

    puts("gimbal tests passed");
    return 0;
}
