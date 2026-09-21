/* imu_xrobot.h - ATOM-IMU 串口解析 */

#ifndef IMU_XROBOT_H
#define IMU_XROBOT_H

#include <stdint.h>
/** @brief 三维向量 */
typedef struct __attribute__((packed)) {
    float x;
    float y;
    float z;
} Vector3;

/** @brief 四元数 */
typedef struct __attribute__((packed)) {
    float q0;
    float q1;
    float q2;
    float q3;
} Quaternion;

/** @brief 欧拉角 */
typedef struct __attribute__((packed)) {
    float rol;   // 横滚角 (度)
    float pit;   // 俯仰角 (度)
    float yaw;   // 偏航角 (度)
} EulerAngles;

/** @brief 完整 IMU 数据帧结构 */
typedef struct __attribute__((packed)) {
    uint8_t prefix;           // 帧头，固定 0xA5
    uint8_t time[5];          // 微秒时间戳 (40-bit)
    uint8_t sync[5];          // 同步时间戳 (40-bit)
    Quaternion quat_;         // 四元数
    Vector3 gyro_;            // 陀螺仪角速度 (rad/s)
    Vector3 accl_;            // 加速度 (m/s^2)
    EulerAngles eulr_;        // 欧拉角 (度)
    uint8_t crc8;             // CRC8 校验
} ImuUartData;
extern ImuUartData g_atom_imu;
void IMU_UART_Init(void);
uint8_t CRC8_Calc(uint8_t *data, uint16_t len);

#endif



