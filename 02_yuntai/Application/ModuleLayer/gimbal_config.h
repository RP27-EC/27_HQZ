#ifndef __GIMBAL_CONFIG_H
#define __GIMBAL_CONFIG_H

#define GIMBAL_PI              3.14159265358979323846f
#define GIMBAL_TWO_PI          6.28318530717958647692f
#define GIMBAL_DEG_TO_RAD      (GIMBAL_PI / 180.0f)

/* Adjust after the real pitch assembly zero point is measured. */
#define GIMBAL_PITCH_MIDDLE_RAD  0.0f
#define GIMBAL_PITCH_MIN_RAD     (-10.0f * GIMBAL_DEG_TO_RAD)
#define GIMBAL_PITCH_MAX_RAD     (30.0f * GIMBAL_DEG_TO_RAD)

#define GIMBAL_YAW_RATE_LIMIT    6.0f
#define GIMBAL_PITCH_RATE_LIMIT  4.0f
#define GIMBAL_TORQUE_LIMIT      3.0f

/* Keep gravity compensation disabled until the real direction is verified. */
#define GIMBAL_GRAVITY_COMP_NM   0.0f
#define GIMBAL_GRAVITY_PHASE_RAD 0.0f

#define GIMBAL_YAW_MEC_ANGLE_KP    8.0f
#define GIMBAL_YAW_MEC_ANGLE_KI    0.0f
#define GIMBAL_YAW_MEC_ANGLE_KD    0.0f
#define GIMBAL_YAW_MEC_RATE_KP     0.8f
#define GIMBAL_YAW_MEC_RATE_KI     0.05f
#define GIMBAL_YAW_MEC_RATE_KD     0.0f

#define GIMBAL_YAW_GYRO_ANGLE_KP   8.0f
#define GIMBAL_YAW_GYRO_ANGLE_KI   0.0f
#define GIMBAL_YAW_GYRO_ANGLE_KD   0.0f
#define GIMBAL_YAW_GYRO_RATE_KP    0.9f
#define GIMBAL_YAW_GYRO_RATE_KI    0.05f
#define GIMBAL_YAW_GYRO_RATE_KD    0.0f

#define GIMBAL_PITCH_MEC_ANGLE_KP   10.0f
#define GIMBAL_PITCH_MEC_ANGLE_KI   0.0f
#define GIMBAL_PITCH_MEC_ANGLE_KD   0.0f
#define GIMBAL_PITCH_MEC_RATE_KP    1.0f
#define GIMBAL_PITCH_MEC_RATE_KI    0.05f
#define GIMBAL_PITCH_MEC_RATE_KD    0.0f

#define GIMBAL_PITCH_GYRO_ANGLE_KP  10.0f
#define GIMBAL_PITCH_GYRO_ANGLE_KI  0.0f
#define GIMBAL_PITCH_GYRO_ANGLE_KD  0.0f
#define GIMBAL_PITCH_GYRO_RATE_KP   1.0f
#define GIMBAL_PITCH_GYRO_RATE_KI   0.05f
#define GIMBAL_PITCH_GYRO_RATE_KD   0.0f

#endif
