/* communicate.c - 上下板通信 */

#include "communicate.h"
#include "drv_can.h"
#include "gimbal.h"
#include "imu_sensor.h"
#include "motor.h"

#include <string.h>

extern CAN_HandleTypeDef hcan2;

Board_Rx_Info_t Board_Rx_Info;
Board_Tx_Info_t Board_Tx_Info;
Board_HeartBeat_t Board_HeartBeat =
{
    .status = DEV_OFFLINE,
    .offline_cnt_max = 100,
    .offline_cnt_1 = 100,
    .offline_cnt_2 = 100,
    .offline_cnt_3 = 100,
    .offline_cnt_4 = 100,
    .offline_cnt_5 = 100,
};

static uint8_t board_tx_buf1[8];
static uint8_t board_tx_buf2[8];

static uint16_t board_float_to_uint(float value, float min_value, float max_value)
{
    float span = max_value - min_value;
    float normalized;

    if (value < min_value)
    {
        value = min_value;
    }
    else if (value > max_value)
    {
        value = max_value;
    }

    normalized = (value - min_value) / span;
    return (uint16_t)(normalized * 65535.0f);
}

static float board_uint_to_float(uint16_t value, float min_value, float max_value)
{
    float span = max_value - min_value;
    return ((float)value * span / 65535.0f) + min_value;
}

static void Board_Rx_Pkt_01(uint8_t *rxbuf)
{
    Board_Rx_Info.state_pkt.car_state = rxbuf[0] & 0x03;
    Board_Rx_Info.state_pkt.gimbal_mode = (rxbuf[0] >> 2) & 0x01;
    Board_Rx_Info.state_pkt.vision_mode = (rxbuf[0] >> 3) & 0x07;
    Board_Rx_Info.state_pkt.game_start = (rxbuf[0] >> 6) & 0x01;
    Board_Rx_Info.state_pkt.my_color = (rxbuf[0] >> 7) & 0x01;

    Board_Rx_Info.shoot_pkt.launch_state = rxbuf[5] & 0x01;
    Board_Rx_Info.shoot_pkt.shoot_mode = (rxbuf[5] >> 1) & 0x01;
    Board_Rx_Info.shoot_pkt.shoot_level = (rxbuf[5] >> 2) & 0x01;
    Board_Rx_Info.shoot_pkt.is_hole = (rxbuf[5] >> 3) & 0x01;
}

static void Board_Rx_Pkt_02(uint8_t *rxbuf)
{
    uint16_t pitch_imu_raw = ((uint16_t)rxbuf[0] << 8) | rxbuf[1];
    uint16_t yaw_imu_raw = ((uint16_t)rxbuf[2] << 8) | rxbuf[3];
    uint16_t pitch_mec_raw = ((uint16_t)rxbuf[4] << 8) | rxbuf[5];
    uint16_t yaw_mec_raw = ((uint16_t)rxbuf[6] << 8) | rxbuf[7];

    Board_Rx_Info.gimbal_target_pkt.pitch_imu_tar =
        board_uint_to_float(pitch_imu_raw, -360.0f, 360.0f);
    Board_Rx_Info.gimbal_target_pkt.yaw_imu_tar =
        board_uint_to_float(yaw_imu_raw, -360.0f, 360.0f);
    Board_Rx_Info.gimbal_target_pkt.pitch_mec_tar =
        board_uint_to_float(pitch_mec_raw, -4.0f, 4.0f);
    Board_Rx_Info.gimbal_target_pkt.yaw_mec_tar =
        board_uint_to_float(yaw_mec_raw, -4.0f, 4.0f);
}

static void Board_Rx_Pkt_05(uint8_t *rxbuf)
{
    int16_t yaw_raw = (int16_t)(((uint16_t)rxbuf[2] << 8) | rxbuf[3]);
    int16_t pitch_raw = (int16_t)(((uint16_t)rxbuf[4] << 8) | rxbuf[5]);

    Board_Rx_Info.remote_cmd_pkt.valid = rxbuf[0] & 0x01u;
    Board_Rx_Info.remote_cmd_pkt.ctrl_source = (rxbuf[0] >> 1) & 0x01u;
    Board_Rx_Info.remote_cmd_pkt.button_bits = rxbuf[1];
    Board_Rx_Info.remote_cmd_pkt.cmd_type = (rxbuf[0] >> 2) & 0x01u;
    Board_Rx_Info.remote_cmd_pkt.mouse_dx = 0;
    Board_Rx_Info.remote_cmd_pkt.mouse_dy = 0;

    if ((Board_Rx_Info.remote_cmd_pkt.ctrl_source == 1u) &&
        (Board_Rx_Info.remote_cmd_pkt.cmd_type == 1u))
    {
        Board_Rx_Info.remote_cmd_pkt.mouse_dx = yaw_raw;
        Board_Rx_Info.remote_cmd_pkt.mouse_dy = pitch_raw;
        Board_Rx_Info.remote_cmd_pkt.yaw_rate_deg_s = 0.0f;
        Board_Rx_Info.remote_cmd_pkt.pitch_rate_deg_s = 0.0f;
    }
    else
    {
        Board_Rx_Info.remote_cmd_pkt.yaw_rate_deg_s = (float)yaw_raw * 0.1f;
        Board_Rx_Info.remote_cmd_pkt.pitch_rate_deg_s = (float)pitch_raw * 0.1f;
    }
}

static void Board_Tx_Update(void)
{
    Board_Tx_Info.gimbal_meg.yaw_mec = Gimbal.base_info.yaw_mec_angle * GIMBAL_DEG_TO_RAD;
    Board_Tx_Info.gimbal_meg.pitch_mec = Gimbal.base_info.pitch_mec_angle * GIMBAL_DEG_TO_RAD;
    Board_Tx_Info.gimbal_meg.yaw_imu = Gimbal.base_info.yaw_imu_angle;
    Board_Tx_Info.gimbal_meg.pitch_imu = Gimbal.base_info.pitch_imu_angle;

    Board_Tx_Info.state_meg.yaw_motor_state =
        (dm_motor[YAW].state->status == DEV_ONLINE) ? 1 : 0;
    Board_Tx_Info.state_meg.pitch_motor_state =
        (dm_motor[PITCH].state->status == DEV_ONLINE) ? 1 : 0;
    Board_Tx_Info.state_meg.lift_motor_state = 0;
    Board_Tx_Info.state_meg.r_fric_state =
        (rm_motor[SHOOT_FRIC_R].state->status == DEV_ONLINE) ? 1 : 0;
    Board_Tx_Info.state_meg.l_fric_state =
        (rm_motor[SHOOT_FRIC_L].state->status == DEV_ONLINE) ? 1 : 0;
    Board_Tx_Info.state_meg.dial_motor_state =
        (dail_motor.KT_motor_info.state_info.work_state == M_ONLINE) ? 1 : 0;
    Board_Tx_Info.state_meg.vision_state = 0;
    Board_Tx_Info.state_meg.lift_state = 0;
}

static void Board_Tx_Meg_01(uint8_t *txbuf)
{
    uint16_t zero = board_float_to_uint(0.0f, -360.0f, 360.0f);

    memset(txbuf, 0, 8);
    txbuf[0] = (Board_Tx_Info.state_meg.yaw_motor_state & 0x01) |
               ((Board_Tx_Info.state_meg.pitch_motor_state & 0x01) << 1) |
               ((Board_Tx_Info.state_meg.lift_motor_state & 0x01) << 2) |
               ((Board_Tx_Info.state_meg.r_fric_state & 0x01) << 3) |
               ((Board_Tx_Info.state_meg.l_fric_state & 0x01) << 4) |
               ((Board_Tx_Info.state_meg.dial_motor_state & 0x01) << 5) |
               ((Board_Tx_Info.state_meg.vision_state & 0x01) << 6);
    txbuf[1] = Board_Tx_Info.state_meg.lift_state;
    txbuf[2] = (uint8_t)(zero >> 8);
    txbuf[3] = (uint8_t)zero;
    txbuf[4] = (uint8_t)(zero >> 8);
    txbuf[5] = (uint8_t)zero;
    txbuf[6] = 0;

    CAN_SendData(&hcan2, ID_BOARD_TX1, txbuf);
}

static void Board_Tx_Meg_02(uint8_t *txbuf)
{
    uint16_t yaw_mec = board_float_to_uint(Board_Tx_Info.gimbal_meg.yaw_mec, -4.0f, 4.0f);
    uint16_t pitch_mec = board_float_to_uint(Board_Tx_Info.gimbal_meg.pitch_mec, -4.0f, 4.0f);
    uint16_t yaw_imu = board_float_to_uint(Board_Tx_Info.gimbal_meg.yaw_imu, -360.0f, 360.0f);
    uint16_t pitch_imu = board_float_to_uint(Board_Tx_Info.gimbal_meg.pitch_imu, -360.0f, 360.0f);

    txbuf[0] = (uint8_t)(yaw_mec >> 8);
    txbuf[1] = (uint8_t)yaw_mec;
    txbuf[2] = (uint8_t)(pitch_mec >> 8);
    txbuf[3] = (uint8_t)pitch_mec;
    txbuf[4] = (uint8_t)(yaw_imu >> 8);
    txbuf[5] = (uint8_t)yaw_imu;
    txbuf[6] = (uint8_t)(pitch_imu >> 8);
    txbuf[7] = (uint8_t)pitch_imu;

    CAN_SendData(&hcan2, ID_BOARD_TX2, txbuf);
}

void Board_Rx_01(uint8_t *rxbuf)
{
    Board_Rx_Pkt_01(rxbuf);
    Board_HeartBeat.offline_cnt_1 = 0;
}

void Board_Rx_02(uint8_t *rxbuf)
{
    Board_Rx_Pkt_02(rxbuf);
    Board_HeartBeat.offline_cnt_2 = 0;
}

void Board_Rx_03(uint8_t *rxbuf)
{
    (void)rxbuf;
    Board_HeartBeat.offline_cnt_3 = 0;
}

void Board_Rx_04(uint8_t *rxbuf)
{
    (void)rxbuf;
    Board_HeartBeat.offline_cnt_4 = 0;
}

void Board_Rx_05(uint8_t *rxbuf)
{
    Board_Rx_Pkt_05(rxbuf);
    Board_HeartBeat.offline_cnt_5 = 0;
}

void Send_To_Down_Board(void)
{
    Board_Tx_Update();
    Board_Tx_Meg_01(board_tx_buf1);
    Board_Tx_Meg_02(board_tx_buf2);
}

void C_Board_Communicate_HeartBeat(void)
{
    if (Board_HeartBeat.offline_cnt_1 < Board_HeartBeat.offline_cnt_max)
    {
        Board_HeartBeat.offline_cnt_1++;
    }
    if (Board_HeartBeat.offline_cnt_2 < Board_HeartBeat.offline_cnt_max)
    {
        Board_HeartBeat.offline_cnt_2++;
    }
    if (Board_HeartBeat.offline_cnt_3 < Board_HeartBeat.offline_cnt_max)
    {
        Board_HeartBeat.offline_cnt_3++;
    }
    if (Board_HeartBeat.offline_cnt_4 < Board_HeartBeat.offline_cnt_max)
    {
        Board_HeartBeat.offline_cnt_4++;
    }

    if (Board_HeartBeat.offline_cnt_5 < Board_HeartBeat.offline_cnt_max)
    {
        Board_HeartBeat.offline_cnt_5++;
    }

    if ((Board_HeartBeat.offline_cnt_1 >= Board_HeartBeat.offline_cnt_max) ||
        (Board_HeartBeat.offline_cnt_2 >= Board_HeartBeat.offline_cnt_max))
    {
        Board_HeartBeat.status = DEV_OFFLINE;
    }
    else
    {
        Board_HeartBeat.status = DEV_ONLINE;
    }
}



