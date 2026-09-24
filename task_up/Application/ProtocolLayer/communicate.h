/* communicate.h - 上下板通信 */

#ifndef __COMMUNICATE_H
#define __COMMUNICATE_H

#include <stdint.h>

#include "rp_device_config.h"

/* 上板发给下板的报文 ID */
#define ID_BOARD_TX1 0xC1  /* 云台姿态反馈 */
#define ID_BOARD_TX2 0xC2  /* 云台角度反馈 */
/* 下板发给上板的报文 ID */
#define ID_BOARD_RX1 0xD1  /* 整车状态与发射状态 */
#define ID_BOARD_RX2 0xD2  /* 云台目标角度 */
#define ID_BOARD_RX3 0xD3  /* 裁判系统射击信息 */
#define ID_BOARD_RX4 0xD4  /* 血量数据 */
#define ID_BOARD_RX5 0xD5  /* 遥控/键鼠控制量 */

/* 整车状态报文 */
typedef struct
{
    uint8_t car_state;   /* 底盘状态 */
    uint8_t gimbal_mode; /* 云台控制模式 */
    uint8_t vision_mode; /* 视觉模式 */
    uint8_t game_start;  /* 比赛开始标志 */
    uint8_t my_color;    /* 己方颜色 */
} Board_State_Pkt_t;

/* 云台目标报文，机械角单位 rad，IMU 角单位 deg */
typedef struct
{
    float yaw_mec_tar;   /* Yaw 机械目标角 */
    float yaw_imu_tar;   /* Yaw IMU 目标角 */
    float pitch_mec_tar; /* Pitch 机械目标角 */
    float pitch_imu_tar; /* Pitch IMU 目标角 */
} Board_Gimbal_Target_Pkt_t;

/* 遥控与键鼠控制报文 */
typedef struct
{
    uint8_t valid;           /* 控制数据有效 */
    uint8_t ctrl_source;     /* 0 = 遥控，1 = 键鼠 */
    uint8_t button_bits;     /* 鼠标左右键位 */
    float yaw_rate_deg_s;    /* Yaw 角速度，deg/s */
    float pitch_rate_deg_s;  /* Pitch 角速度，deg/s */
    uint8_t cmd_type;        /* 0 = 角速度，1 = 鼠标增量 */
    int16_t mouse_dx;        /* 鼠标 X 增量 */
    int16_t mouse_dy;        /* 鼠标 Y 增量 */
} Board_Remote_Cmd_Pkt_t;

/* 发射控制报文 */
typedef struct
{
    uint8_t launch_state; /* 发射机构使能 */
    uint8_t shoot_mode;   /* 0 = 单发，1 = 连发 */
    uint8_t shoot_level;  /* 发射触发电平 */
    uint8_t is_hole;      /* 弹仓剩余检测 */
} Board_Shoot_Pkt_t;

/* 上板接收缓存 */
typedef struct
{
    Board_State_Pkt_t state_pkt;               /* 整车状态 */
    Board_Gimbal_Target_Pkt_t gimbal_target_pkt; /* 云台目标 */
    Board_Shoot_Pkt_t shoot_pkt;               /* 发射控制 */
    Board_Remote_Cmd_Pkt_t remote_cmd_pkt;     /* 键鼠控制 */
} Board_Rx_Info_t;

/* 云台姿态反馈，机械角单位 rad，IMU 角单位 deg */
typedef struct
{
    float yaw_mec;   /* Yaw 机械角 */
    float pitch_mec; /* Pitch 机械角 */
    float yaw_imu;   /* Yaw IMU 角 */
    float pitch_imu; /* Pitch IMU 角 */
} Board_Gimbal_Meg_t;

/* 下板关心的上板在线状态 */
typedef struct
{
    uint8_t yaw_motor_state;   /* Yaw 电机在线 */
    uint8_t pitch_motor_state; /* Pitch 电机在线 */
    uint8_t lift_motor_state;  /* 抬升电机在线 */
    uint8_t r_fric_state;      /* 右摩擦轮在线 */
    uint8_t l_fric_state;      /* 左摩擦轮在线 */
    uint8_t dial_motor_state;  /* 拨盘在线 */
    uint8_t vision_state;      /* 视觉在线 */
    uint8_t lift_state;        /* 抬升机构状态 */
} Board_State_Meg_t;

/* 上板发送缓存 */
typedef struct
{
    Board_Gimbal_Meg_t gimbal_meg; /* 云台姿态 */
    Board_State_Meg_t state_meg;   /* 设备在线状态 */
} Board_Tx_Info_t;

/* 板间心跳，按报文分通道计数 */
typedef struct
{
    dev_work_state_t status;   /* 板间综合在线状态 */
    uint16_t offline_cnt_1;    /* D1 离线计数 */
    uint16_t offline_cnt_2;    /* D2 离线计数 */
    uint16_t offline_cnt_3;    /* D3 离线计数 */
    uint16_t offline_cnt_4;    /* D4 离线计数 */
    uint16_t offline_cnt_5;    /* D5 离线计数 */
    uint16_t offline_cnt_max;  /* 离线判定阈值 */
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

