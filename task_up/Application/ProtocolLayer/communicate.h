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
    uint8_t car_state;
    uint8_t gimbal_mode;
    uint8_t vision_mode;
    uint8_t game_start;
    uint8_t my_color;
} Board_State_Pkt_t;

typedef struct
{
    float yaw_mec_tar;
    float yaw_imu_tar;
    float pitch_mec_tar;
    float pitch_imu_tar;
} Board_Gimbal_Target_Pkt_t;

typedef struct
{
    uint8_t valid;
    uint8_t ctrl_source;
    uint8_t button_bits;
    float yaw_rate_deg_s;
    float pitch_rate_deg_s;
} Board_Remote_Cmd_Pkt_t;

typedef struct
{
    uint8_t launch_state;
    uint8_t shoot_mode;
    uint8_t shoot_level;
    uint8_t is_hole;
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
    float yaw_mec;
    float pitch_mec;
    float yaw_imu;
    float pitch_imu;
} Board_Gimbal_Meg_t;

typedef struct
{
    uint8_t yaw_motor_state;
    uint8_t pitch_motor_state;
    uint8_t lift_motor_state;
    uint8_t r_fric_state;
    uint8_t l_fric_state;
    uint8_t dial_motor_state;
    uint8_t vision_state;
    uint8_t lift_state;
} Board_State_Meg_t;

typedef struct
{
    Board_Gimbal_Meg_t gimbal_meg;
    Board_State_Meg_t state_meg;
} Board_Tx_Info_t;

typedef struct
{
    dev_work_state_t status;
    uint16_t offline_cnt_1;
    uint16_t offline_cnt_2;
    uint16_t offline_cnt_3;
    uint16_t offline_cnt_4;
    uint16_t offline_cnt_5;
    uint16_t offline_cnt_max;
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

