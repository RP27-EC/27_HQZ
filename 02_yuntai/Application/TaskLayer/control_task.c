/**
  ******************************************************************************
  * @file    control_task.c
  * @brief   陀螺仪任务
  ******************************************************************************
  */
#include "control_task.h"
#include "imu_sensor.h"
#include "gimbal.h"
#include "motor.h"
#include "rc_sensor.h"

#include <math.h>


volatile imu_debug_t imu_dbg;

#define CONTROL_DT_S              0.001f
#define RC_AXIS_MAX              660.0f
#define RC_AXIS_DEADBAND         20.0f
#define GIMBAL_YAW_RC_RANGE_RAD  (90.0f * GIMBAL_DEG_TO_RAD)
#define GIMBAL_PITCH_RC_RANGE_RAD (20.0f * GIMBAL_DEG_TO_RAD)
#define GIMBAL_YAW_TORQUE_SIGN   1.0f
#define GIMBAL_PITCH_TORQUE_SIGN 1.0f
#define GIMBAL_DEBUG_TARGETS_ENABLED 0
#define GIMBAL_DEBUG_YAW_TARGET_RAD  0.0f
#define GIMBAL_DEBUG_MODE             G_MEC
#define GIMBAL_DEBUG_PITCH_TARGET_RAD 0.0f

/**
  * @brief  把 imu_sensor 里的数据搬到 imu_dbg
  */
static void imu_debug_update(void)
{
	imu_info_t *info = imu_sensor.info;

	imu_dbg.acc_x = info->raw_info.acc_x;
	imu_dbg.acc_y = info->raw_info.acc_y;
	imu_dbg.acc_z = info->raw_info.acc_z;

	imu_dbg.gyro_x = info->raw_info.gyro_x;
	imu_dbg.gyro_y = info->raw_info.gyro_y;
	imu_dbg.gyro_z = info->raw_info.gyro_z;

	imu_dbg.yaw   = info->base_info.yaw;
	imu_dbg.pitch = info->base_info.pitch;
	imu_dbg.roll  = info->base_info.roll;

	imu_dbg.rate_yaw   = info->base_info.rate_yaw;
	imu_dbg.rate_pitch = info->base_info.rate_pitch;
	imu_dbg.rate_roll  = info->base_info.rate_roll;

	imu_dbg.ave_rate_yaw   = info->base_info.ave_rate_yaw;
	imu_dbg.ave_rate_pitch = info->base_info.ave_rate_pitch;
	imu_dbg.ave_rate_roll  = info->base_info.ave_rate_roll;

	imu_dbg.accx = info->base_info.accx;
	imu_dbg.accy = info->base_info.accy;
	imu_dbg.accz = info->base_info.accz;

	imu_dbg.temperature = info->base_info.temperature;

	imu_dbg.dev_state = (uint8_t)imu_sensor.work_state.dev_state;
	imu_dbg.cali_end  = imu_sensor.work_state.cali_end;
	imu_dbg.err_code  = (uint8_t)imu_sensor.work_state.err_code;

}

static float rc_axis_to_delta(int16_t value, float range_rad)
{
    float axis = (float)value;

    if (fabsf(axis) <= RC_AXIS_DEADBAND)
    {
        return 0.0f;
    }

    axis -= (axis > 0.0f) ? RC_AXIS_DEADBAND : -RC_AXIS_DEADBAND;
    axis /= (RC_AXIS_MAX - RC_AXIS_DEADBAND);

    if (axis > 1.0f)
    {
        axis = 1.0f;
    }
    else if (axis < -1.0f)
    {
        axis = -1.0f;
    }

    return axis * range_rad;
}

static gimbal_mode_e gimbal_mode_from_rc(void)
{
    if (rc_sensor.work_state != DEV_ONLINE)
    {
        return G_SLEEP;
    }

    if (rc_sensor_info.s2.value == RC_SW_DOWN)
    {
        return G_SLEEP;
    }

    if (rc_sensor_info.s1.value == RC_SW_UP)
    {
        return G_GYRO;
    }

    if (rc_sensor_info.s1.value == RC_SW_MID)
    {
        return G_MEC;
    }

    return G_SLEEP;
}

static void gimbal_request_mode(void)
{
    static gimbal_mode_e requested_mode = G_SLEEP;
    gimbal_mode_e new_mode;

#if GIMBAL_DEBUG_TARGETS_ENABLED
    if (requested_mode != GIMBAL_DEBUG_MODE)
    {
        requested_mode = GIMBAL_DEBUG_MODE;
        Gimbal_SetMode(&Gimbal, GIMBAL_DEBUG_MODE);
    }
    return;
#endif

    if (rc_sensor.work_state != DEV_ONLINE)
    {
        return;
    }

    new_mode = gimbal_mode_from_rc();
    if (new_mode == G_SLEEP)
    {
        if (requested_mode != G_SLEEP)
        {
            requested_mode = G_SLEEP;
            Gimbal_SetMode(&Gimbal, G_SLEEP);
        }
        return;
    }

    if (new_mode != requested_mode)
    {
        requested_mode = new_mode;
        Gimbal_SetMode(&Gimbal, new_mode);
    }
}

static bool dm_motor_is_control_ready(const Motor_DM_t *motor)
{
    if (motor->state->status != DEV_ONLINE)
    {
        return false;
    }

    return (motor->state->motor_state == Motor_Enable) ||
           (motor->state->motor_state == Motor_Unenable);
}

static gimbal_input_t gimbal_inputs_from_hardware(void)
{
    gimbal_input_t inputs = {0};
    imu_info_t *imu = imu_sensor.info;
    float yaw_delta = rc_axis_to_delta(RC_RIGH_CH_LR_VALUE, GIMBAL_YAW_RC_RANGE_RAD);
    float pitch_delta = rc_axis_to_delta(RC_RIGH_CH_UD_VALUE, GIMBAL_PITCH_RC_RANGE_RAD);
    float yaw_control_angle;
    float pitch_control_angle;

    inputs.yaw_motor_online = dm_motor_is_control_ready(&Yaw_Motor);
    inputs.pitch_motor_online = dm_motor_is_control_ready(&Pitch_Motor);
    inputs.imu_online = (imu_sensor.work_state.dev_state == DEV_ONLINE);
    inputs.imu_calibrated = (imu_sensor.work_state.cali_end != 0);
    inputs.rc_online = (rc_sensor.work_state == DEV_ONLINE);
    inputs.emergency_stop = inputs.rc_online && (rc_sensor_info.s2.value == RC_SW_DOWN);

    inputs.yaw_mec_angle = Yaw_Motor.rx_info->motor_angle_sum;
    inputs.yaw_mec_speed = Yaw_Motor.rx_info->speed;
    inputs.pitch_mec_angle = Pitch_Motor.rx_info->motor_angle_sum;
    inputs.pitch_mec_speed = Pitch_Motor.rx_info->speed;

    inputs.yaw_imu_angle = imu->base_info.yaw * GIMBAL_DEG_TO_RAD;
    inputs.yaw_imu_speed = imu->base_info.rate_yaw * GIMBAL_DEG_TO_RAD;
    inputs.pitch_imu_angle = imu->base_info.pitch * GIMBAL_DEG_TO_RAD;
    inputs.pitch_imu_speed = imu->base_info.rate_pitch * GIMBAL_DEG_TO_RAD;

    if (Gimbal.mode == G_GYRO)
    {
        yaw_control_angle = inputs.yaw_imu_angle;
        pitch_control_angle = inputs.pitch_imu_angle;
    }
    else
    {
        yaw_control_angle = inputs.yaw_mec_angle;
        pitch_control_angle = inputs.pitch_mec_angle;
    }

    inputs.yaw_target = yaw_control_angle + yaw_delta;
    inputs.pitch_target = pitch_control_angle + pitch_delta;

    return inputs;
}

static void gimbal_send_output(const gimbal_output_t *output)
{
    Yaw_Motor.tx_info->torque = output->yaw_torque * GIMBAL_YAW_TORQUE_SIGN;
    Pitch_Motor.tx_info->torque = output->pitch_torque * GIMBAL_PITCH_TORQUE_SIGN;

    Yaw_Motor.single_set_torque(&Yaw_Motor);
    Pitch_Motor.single_set_torque(&Pitch_Motor);
}

void StartControlTask(void const * argument)
{
	for(;;)
	{
		gimbal_input_t gimbal_inputs;
		gimbal_output_t gimbal_output = {0};

		if ((imu_sensor.work_state.err_code == IMU_NONE_ERR) || \
				(imu_sensor.work_state.err_code == IMU_DATA_CALI))
		{
			imu_sensor.update(&imu_sensor);
		}

		imu_debug_update();

		gimbal_request_mode();
		Gimbal_SetDebugTargets(&Gimbal,
		                     GIMBAL_DEBUG_TARGETS_ENABLED,
		                     GIMBAL_DEBUG_YAW_TARGET_RAD,
		                     GIMBAL_DEBUG_PITCH_TARGET_RAD);
		gimbal_inputs = gimbal_inputs_from_hardware();
		Gimbal_Update(&Gimbal, &gimbal_inputs, CONTROL_DT_S, &gimbal_output);
		gimbal_send_output(&gimbal_output);

		osDelay(1);	
	}
}
