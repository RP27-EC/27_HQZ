#ifndef __IMU_CALIBRATION_CONFIG_H
#define __IMU_CALIBRATION_CONFIG_H

#define IMU_CALI_SAMPLE_COUNT        2000u    // 连续静止样本数，次
#define IMU_CALI_GYRO_MAX_RAD_S      0.05f    // 静止角速度上限，rad/s
#define IMU_CALI_GRAVITY_M_S2        9.80665f // 重力参考，m/s^2
#define IMU_CALI_ACCEL_TOL_M_S2      0.5f     // 重力模长容差，m/s^2

#endif
