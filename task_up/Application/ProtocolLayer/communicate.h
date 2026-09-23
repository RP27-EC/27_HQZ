/* communicate.h - 上下板通信 */

#ifndef __COMMUNICATE_H
#define __COMMUNICATE_H

#include <stdint.h>

#include "rp_device_config.h"

#define ID_BOARD_TX1 0xC1
#define ID_BOARD_TX2 0xC2
#define ID_BOARD_RX1 0xD1
#define ID_BOARD_RX2 0xD2
#define ID_BOARD_RX3 0xD3
#define ID_BOARD_RX4 0xD4
#define ID_BOARD_RX5 0xD5

typedef struct
{
    uint8_t car_state;  // 整车状态: 0=下电 1=正常
    uint8_t gimbal_mode;  // 云台模式
    uint8_t vision_mode;  // 视觉模式
    uint8_t game_start;  // 比赛开始
    uint8_t my_color;  // 己方颜色
} Board_State_Pkt_t;

typedef struct
{
    float yaw_mec_tar;  // 云台机械角目标
    float yaw_imu_tar;  // 云台 IMU 角目标
    float pitch_mec_tar;  // 云台机械角目标
    float pitch_imu_tar;  // 云台 IMU 角目标
} Board_Gimbal_Target_Pkt_t;

typedef struct
{
    uint8_t valid;  // 指令有效标志
    uint8_t ctrl_source;  // 控制源: 0=遥控器 1=键鼠
    uint8_t button_bits;  // 按键位
    float yaw_rate_deg_s;  // 目标角速度(deg/s)
    float pitch_rate_deg_s;  // 目标角速度(deg/s)
} Board_Remote_Cmd_Pkt_t;

typedef struct
{
    uint8_t launch_state;  // 发射使能
    uint8_t shoot_mode;  // 射击模式: 0=单发 1=连发
    uint8_t shoot_level;  // 扳机电平
    uint8_t is_hole;  // 打符模式
} Board_Shoot_Pkt_t;

typedef struct
{
    Board_State_Pkt_t state_pkt;
    Board_Gimbal_Target_Pkt_t gimbal_target_pkt;
    Board_Shoot_Pkt_t shoot_pkt;
    Board_Remote_Cmd_Pkt_t remote_cmd_pkt;
} Board_Rx_Info_t;

typedef struct
{
    float yaw_mec;  // 云台机械角反馈
    float pitch_mec;  // 云台机械角反馈
    float yaw_imu;  // 云台 IMU 角反馈
    float pitch_imu;  // 云台 IMU 角反馈
} Board_Gimbal_Meg_t;

typedef struct
{
    uint8_t yaw_motor_state;  // Yaw 电机在线
    uint8_t pitch_motor_state;  // Pitch 电机在线
    uint8_t lift_motor_state;  // 升降电机在线
    uint8_t r_fric_state;  // 右摩擦轮在线
    uint8_t l_fric_state;  // 左摩擦轮在线
    uint8_t dial_motor_state;  // 拨盘在线
    uint8_t vision_state;  // 视觉在线
    uint8_t lift_state;  // 升降状态
} Board_State_Meg_t;

typedef struct
{
    Board_Gimbal_Meg_t gimbal_meg;
    Board_State_Meg_t state_meg;
} Board_Tx_Info_t;

typedef struct
{
    dev_work_state_t status;
    uint16_t offline_cnt_1;  // D1 包离线计数
    uint16_t offline_cnt_2;  // D2 包离线计数
    uint16_t offline_cnt_3;  // D3 包离线计数
    uint16_t offline_cnt_4;  // D4 包离线计数
    uint16_t offline_cnt_5;  // D5 包离线计数
    uint16_t offline_cnt_max;  // 离线判定阈值
} Board_HeartBeat_t;

extern Board_Rx_Info_t Board_Rx_Info;
extern Board_Tx_Info_t Board_Tx_Info;
extern Board_HeartBeat_t Board_HeartBeat;

void Board_Rx_01(uint8_t *rxbuf);
void Board_Rx_02(uint8_t *rxbuf);
void Board_Rx_03(uint8_t *rxbuf);
void Board_Rx_04(uint8_t *rxbuf);
void Board_Rx_05(uint8_t *rxbuf);
void Send_To_Down_Board(void);
void C_Board_Communicate_HeartBeat(void);

#endif

