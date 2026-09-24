/* communicate.c - 上下板通信 */

#include "communicate.h"
#include "drv_can.h"
#include "gimbal.h"
#include "imu_sensor.h"
#include "motor.h"

#include <string.h>

extern CAN_HandleTypeDef hcan2;

Board_Rx_Info_t Board_Rx_Info; /* 下板控制输入 */
Board_Tx_Info_t Board_Tx_Info; /* 上板反馈输出 */
Board_HeartBeat_t Board_HeartBeat = /* 板间链路状态 */
{
    .status = DEV_OFFLINE,
    .offline_cnt_max = 100,
    .offline_cnt_1 = 100,
    .offline_cnt_2 = 100,
    .offline_cnt_3 = 100,
    .offline_cnt_4 = 100,
    .offline_cnt_5 = 100,
};

static uint8_t board_tx_buf1[8]; /* C1 发送缓存 */
static uint8_t board_tx_buf2[8]; /* C2 发送缓存 */

/* 浮点按线性量程压入 16 位协议字段 */

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

/* 16 位协议字段还原为浮点 */
static float board_uint_to_float(uint16_t value, float min_value, float max_value)
{
    float span = max_value - min_value;
    return ((float)value * span / 65535.0f) + min_value;
}

/* 解析 D1：整车状态与发射控制 */
static void Board_Rx_Pkt_01(uint8_t *rxbuf)
{
    Board_Rx_Info.state_pkt.car_state = rxbuf[0] & 0x03;          /* 车辆状态 */
    Board_Rx_Info.state_pkt.gimbal_mode = (rxbuf[0] >> 2) & 0x01; /* 云台模式 */
    Board_Rx_Info.state_pkt.vision_mode = (rxbuf[0] >> 3) & 0x07; /* 视觉模式 */
    Board_Rx_Info.state_pkt.game_start = (rxbuf[0] >> 6) & 0x01;  /* 比赛开始 */
    Board_Rx_Info.state_pkt.my_color = (rxbuf[0] >> 7) & 0x01;    /* 己方颜色 */

    Board_Rx_Info.shoot_pkt.launch_state = rxbuf[5] & 0x01;       /* 发射许可 */
    Board_Rx_Info.shoot_pkt.shoot_mode = (rxbuf[5] >> 1) & 0x01;  /* 发射模式 */
    Board_Rx_Info.shoot_pkt.shoot_level = (rxbuf[5] >> 2) & 0x01; /* 触发电平 */
    Board_Rx_Info.shoot_pkt.is_hole = (rxbuf[5] >> 3) & 0x01;     /* 过洞标志 */
}

/* 解析 D2：云台目标角度 */
static void Board_Rx_Pkt_02(uint8_t *rxbuf)
{
    uint16_t pitch_imu_raw = ((uint16_t)rxbuf[0] << 8) | rxbuf[1]; /* Pitch IMU 原始值 */
    uint16_t yaw_imu_raw = ((uint16_t)rxbuf[2] << 8) | rxbuf[3];   /* Yaw IMU 原始值 */
    uint16_t pitch_mec_raw = ((uint16_t)rxbuf[4] << 8) | rxbuf[5]; /* Pitch 机械原始值 */
    uint16_t yaw_mec_raw = ((uint16_t)rxbuf[6] << 8) | rxbuf[7];   /* Yaw 机械原始值 */

    Board_Rx_Info.gimbal_target_pkt.pitch_imu_tar =
        board_uint_to_float(pitch_imu_raw, -360.0f, 360.0f);
    Board_Rx_Info.gimbal_target_pkt.yaw_imu_tar =
        board_uint_to_float(yaw_imu_raw, -360.0f, 360.0f);
    Board_Rx_Info.gimbal_target_pkt.pitch_mec_tar =
        board_uint_to_float(pitch_mec_raw, -4.0f, 4.0f);
    Board_Rx_Info.gimbal_target_pkt.yaw_mec_tar =
        board_uint_to_float(yaw_mec_raw, -4.0f, 4.0f);
}

/* 解析 D5：遥控角速度或鼠标增量 */
static void Board_Rx_Pkt_05(uint8_t *rxbuf)
{
    int16_t yaw_raw = (int16_t)(((uint16_t)rxbuf[2] << 8) | rxbuf[3]);   /* Yaw 控制原始值 */
    int16_t pitch_raw = (int16_t)(((uint16_t)rxbuf[4] << 8) | rxbuf[5]); /* Pitch 控制原始值 */

    Board_Rx_Info.remote_cmd_pkt.valid = rxbuf[0] & 0x01u;            /* 数据有效 */
    Board_Rx_Info.remote_cmd_pkt.ctrl_source = (rxbuf[0] >> 1) & 0x01u;/* 输入源 */
    Board_Rx_Info.remote_cmd_pkt.button_bits = rxbuf[1];              /* 鼠标键 */
    Board_Rx_Info.remote_cmd_pkt.cmd_type = (rxbuf[0] >> 2) & 0x01u;  /* 控制类型 */
    Board_Rx_Info.remote_cmd_pkt.mouse_dx = 0;
    Board_Rx_Info.remote_cmd_pkt.mouse_dy = 0;

    if ((Board_Rx_Info.remote_cmd_pkt.ctrl_source == 1u) &&
        (Board_Rx_Info.remote_cmd_pkt.cmd_type == 1u))
    {
        Board_Rx_Info.remote_cmd_pkt.mouse_dx = yaw_raw;   /* 鼠标 X 增量 */
        Board_Rx_Info.remote_cmd_pkt.mouse_dy = pitch_raw; /* 鼠标 Y 增量 */
        Board_Rx_Info.remote_cmd_pkt.yaw_rate_deg_s = 0.0f;
        Board_Rx_Info.remote_cmd_pkt.pitch_rate_deg_s = 0.0f;
    }
    else
    {
        Board_Rx_Info.remote_cmd_pkt.yaw_rate_deg_s = (float)yaw_raw * 0.1f; /* Yaw 角速度 */
        Board_Rx_Info.remote_cmd_pkt.pitch_rate_deg_s = (float)pitch_raw * 0.1f; /* Pitch 角速度 */
    }
}

/* 汇总云台姿态与上板设备在线状态 */
static void Board_Tx_Update(void)
{
    Board_Tx_Info.gimbal_meg.yaw_mec = Gimbal.base_info.yaw_mec_angle * GIMBAL_DEG_TO_RAD; /* Yaw 机械角 */
    Board_Tx_Info.gimbal_meg.pitch_mec = Gimbal.base_info.pitch_mec_angle * GIMBAL_DEG_TO_RAD; /* Pitch 机械角 */
    Board_Tx_Info.gimbal_meg.yaw_imu = Gimbal.base_info.yaw_imu_angle; /* Yaw IMU 角 */
    Board_Tx_Info.gimbal_meg.pitch_imu = Gimbal.base_info.pitch_imu_angle; /* Pitch IMU 角 */

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

/* 打包 C1：设备在线状态 */
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

/* 打包 C2：云台机械角与 IMU 角 */
static void Board_Tx_Meg_02(uint8_t *txbuf)
{
    uint16_t yaw_mec = board_float_to_uint(Board_Tx_Info.gimbal_meg.yaw_mec, -4.0f, 4.0f);       /* Yaw 机械角 */
    uint16_t pitch_mec = board_float_to_uint(Board_Tx_Info.gimbal_meg.pitch_mec, -4.0f, 4.0f);   /* Pitch 机械角 */
    uint16_t yaw_imu = board_float_to_uint(Board_Tx_Info.gimbal_meg.yaw_imu, -360.0f, 360.0f);   /* Yaw IMU 角 */
    uint16_t pitch_imu = board_float_to_uint(Board_Tx_Info.gimbal_meg.pitch_imu, -360.0f, 360.0f); /* Pitch IMU 角 */

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

/* 收到 D1，清零对应心跳计数 */
void Board_Rx_01(uint8_t *rxbuf)
{
    Board_Rx_Pkt_01(rxbuf);
    Board_HeartBeat.offline_cnt_1 = 0;
}

/* 收到 D2，清零对应心跳计数 */
void Board_Rx_02(uint8_t *rxbuf)
{
    Board_Rx_Pkt_02(rxbuf);
    Board_HeartBeat.offline_cnt_2 = 0;
}

/* D3 暂无数据字段，仅维持心跳 */
void Board_Rx_03(uint8_t *rxbuf)
{
    (void)rxbuf;
    Board_HeartBeat.offline_cnt_3 = 0;
}

/* D4 暂无数据字段，仅维持心跳 */
void Board_Rx_04(uint8_t *rxbuf)
{
    (void)rxbuf;
    Board_HeartBeat.offline_cnt_4 = 0;
}

/* 收到 D5，刷新遥控/键鼠控制量 */
void Board_Rx_05(uint8_t *rxbuf)
{
    Board_Rx_Pkt_05(rxbuf);
    Board_HeartBeat.offline_cnt_5 = 0;
}

/* 上板周期发送 C1/C2 到通信 CAN */
void Send_To_Down_Board(void)
{
    Board_Tx_Update();
    Board_Tx_Meg_01(board_tx_buf1);
    Board_Tx_Meg_02(board_tx_buf2);
}

/* 板间心跳：D1/D2 任一超时即判离线 */
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



