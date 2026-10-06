# CAN2 发送拥堵修复交付

日期：2026-10-06。以下为当前源码中的完整修改单元。

上板 C1/C2 各 5 ms 交错发送；下板 D3 到期优先入队，失败下一任务周期重试。电机控制周期不变，D1/D2/D5 容量足够时整组入队。

上下板均需人工编译与更新。未编译、未烧录，按用户规范跳过测试脚本、Mock 和自动测试。

## Board_Status_t

来源：`task_down/Application/ProtocolLayer/board_protocol.h`

```c
typedef struct{
    uint16_t offline_cnt_max; /* 离线阈值，任务周期数 */
    dev_work_state_t status; /* 链路状态，见枚举 */
    uint16_t offline_cnt; /* 离线计数，任务周期数 */
    volatile uint32_t gimbal_rx_time_ms; /* C2接收时刻，ms */
    volatile uint8_t gimbal_d1_tx_ok; /* 本组D1入队成功，0/1 */
    volatile uint8_t gimbal_d2_tx_ok; /* 本组D2入队成功，0/1 */
    volatile uint8_t gimbal_data_valid; /* 云台反馈有效，0/1 */
    volatile uint8_t heat_d3_tx_ok; /* 最近D3入队成功，0/1 */
    volatile uint32_t heat_d3_tx_ok_count; /* D3成功次数，uint32循环 */
    volatile uint32_t heat_d3_tx_fail_count; /* D3失败次数，uint32循环 */
    volatile uint32_t heat_d3_tx_tick; /* 最近D3成功时刻，ms */
    volatile uint32_t heat_d3_tx_gap_ms; /* 最近D3成功间隔，ms */
    volatile uint32_t heat_d3_tx_max_gap_ms; /* 最大D3成功间隔，ms */
    volatile uint32_t control_tx_defer_count; /* 整组延后次数，uint32循环 */
}Board_Status_t;
```

## Board_Init

来源：`task_down/Application/ProtocolLayer/board_protocol.c`

```c
void Board_Init(Board_t* board)
{
	board->status->offline_cnt = board->status->offline_cnt_max;
	board->status->status = DEV_OFFLINE;
	board->status->gimbal_rx_time_ms = 0u;
	board->status->gimbal_data_valid = 0u;
	board->status->gimbal_d1_tx_ok = 0u;
	board->status->gimbal_d2_tx_ok = 0u;
	board->status->heat_d3_tx_ok = 0u;
	board->status->heat_d3_tx_ok_count = 0u;
	board->status->heat_d3_tx_fail_count = 0u;
	board->status->heat_d3_tx_tick = 0u;
	board->status->heat_d3_tx_gap_ms = 0u;
	board->status->heat_d3_tx_max_gap_ms = 0u;
	board->status->control_tx_defer_count = 0u;
	
	board->tx_01 = Board_Tx_Pkt_01;
	board->tx_02 = Board_Tx_Pkt_02;
	board->tx_03 = Board_Tx_Pkt_03;
	board->tx_04 = Board_Tx_Pkt_04;
	board->tx_05 = Board_Tx_Pkt_05;
	
	board->rx_01 = Board_Rx_Meg_01;
	board->rx_02 = Board_Rx_Meg_02;
	
	board->heartbeat = Board_Heart_Beat;
}
```

## Board_Tx_Pkt_03

来源：`task_down/Application/ProtocolLayer/board_protocol.c`

```c
void Board_Tx_Pkt_03(Board_t* board)
{
    judge_heat_snapshot_t snapshot;
    uint32_t now;
    uint32_t gap;
    uint8_t flags = 0u;
    board->status->heat_d3_tx_ok = 0u;
    if (Judge_GetHeatSnapshot(&snapshot) == 0u)
    {
        return;
    }
    now = HAL_GetTick();
    if ((snapshot.limit_seen != 0u) && (snapshot.heat_limit != 0u) &&
        ((uint32_t)(now - snapshot.limit_tick) < BOARD_HEAT_LIMIT_TIMEOUT_MS))
    {
        flags |= 0x01u;
    }
    if ((snapshot.heat_seen != 0u) &&
        ((uint32_t)(now - snapshot.heat_tick) < BOARD_HEAT_VALUE_TIMEOUT_MS))
    {
        flags |= 0x02u;
    }
    pkt_03[0] = (uint8_t)(snapshot.heat_limit >> 8);
    pkt_03[1] = (uint8_t)snapshot.heat_limit;
    pkt_03[2] = (uint8_t)(snapshot.barrel_heat >> 8);
    pkt_03[3] = (uint8_t)snapshot.barrel_heat;
    pkt_03[4] = (uint8_t)(snapshot.cooling_rate >> 8);
    pkt_03[5] = (uint8_t)snapshot.cooling_rate;
    pkt_03[6] = snapshot.heat_seq;
    pkt_03[7] = flags;
    if (CAN_SendData(&hfdcan2, ID_PKT_03, pkt_03) != HAL_OK)
    {
        board->status->heat_d3_tx_fail_count++;
        return;
    }
    now = HAL_GetTick();
    if (board->status->heat_d3_tx_ok_count != 0u)
    {
        gap = now - board->status->heat_d3_tx_tick;
        board->status->heat_d3_tx_gap_ms = gap;
        if (gap > board->status->heat_d3_tx_max_gap_ms)
        {
            board->status->heat_d3_tx_max_gap_ms = gap;
        }
    }
    board->status->heat_d3_tx_tick = now;
    board->status->heat_d3_tx_ok_count++;
    board->status->heat_d3_tx_ok = 1u;
}
```

## StartConnectTask

来源：`task_down/Application/TaskLayer/connect_task.c`

```c
void StartConnectTask(void const *argument)
{
#if BOARD_COMM_TX_ENABLE && BOARD_COMM_D3_ENABLE
    uint32_t d3_tick = HAL_GetTick();
#endif
#if BOARD_COMM_TX_ENABLE && BOARD_COMM_D4_ENABLE
    uint32_t d4_tick = HAL_GetTick();
#endif
#if BOARD_COMM_TX_ENABLE
    uint32_t control_slots = 2u;
#if BOARD_COMM_D5_ENABLE
    control_slots++;
#endif
#endif

    (void)argument;

    for (;;)
    {
#if BOARD_COMM_TX_ENABLE
#if BOARD_COMM_D3_ENABLE
        if ((uint32_t)(HAL_GetTick() - d3_tick) >= BOARD_COMM_D3_PERIOD_MS)
        {
            board.tx_03(&board);
            if (board.status->heat_d3_tx_ok != 0u)
            {
                d3_tick = board.status->heat_d3_tx_tick;
            }
        }
#endif

        /* 整组容量不足时留待下周期 */
        board.status->gimbal_d1_tx_ok = 0u;
        board.status->gimbal_d2_tx_ok = 0u;
        if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) >= control_slots)
        {
            board.tx_01(&board);
            board.tx_02(&board);
#if BOARD_COMM_D5_ENABLE
            board.tx_05(&board);
#endif
        }
        else
        {
            board.status->control_tx_defer_count++;
        }
#if BOARD_COMM_D4_ENABLE
        if (((uint32_t)(HAL_GetTick() - d4_tick) >= BOARD_COMM_D4_PERIOD_MS) &&
            (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) > 0u))
        {
            d4_tick = HAL_GetTick();
            board.tx_04(&board);
        }
#endif
#endif

        osDelay(BOARD_COMM_D1D2_PERIOD_MS);
    }
}
```

## 板间反馈配置

来源：`task_up/Application/ConfigLayer/board_remote_config.h`

```c
/* 板间反馈降频，不改变控制周期 */
#define BOARD_FEEDBACK_PERIOD_MS 5u // 每类反馈周期，ms，至少2
```

## Board_Feedback_Debug_t

来源：`task_up/Application/ProtocolLayer/communicate.h`

```c
typedef struct
{
    uint32_t c1_tx_tick; /* 最近C1入队成功时刻，ms */
    uint32_t c2_tx_tick; /* 最近C2入队成功时刻，ms */
    uint32_t c1_ok_count; /* C1成功次数，uint32循环 */
    uint32_t c2_ok_count; /* C2成功次数，uint32循环 */
    uint32_t c1_fail_count; /* C1失败次数，uint32循环 */
    uint32_t c2_fail_count; /* C2失败次数，uint32循环 */
    uint32_t defer_count; /* 无邮箱延后次数，uint32循环 */
} Board_Feedback_Debug_t;

extern volatile Board_Feedback_Debug_t board_feedback_debug;
```

## 反馈调度对象

来源：`task_up/Application/ProtocolLayer/communicate.c`

```c
volatile Board_Feedback_Debug_t board_feedback_debug;
static uint8_t board_feedback_next; /* 下一反馈类型，0/1 */
```

## Board_Tx_Meg_01

来源：`task_up/Application/ProtocolLayer/communicate.c`

```c
static HAL_StatusTypeDef Board_Tx_Meg_01(uint8_t *txbuf)
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

    return CAN_SendData(&hcan2, ID_BOARD_TX1, txbuf);
}
```

## Board_Tx_Meg_02

来源：`task_up/Application/ProtocolLayer/communicate.c`

```c
static HAL_StatusTypeDef Board_Tx_Meg_02(uint8_t *txbuf)
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

    return CAN_SendData(&hcan2, ID_BOARD_TX2, txbuf);
}
```

## Send_To_Down_Board

来源：`task_up/Application/ProtocolLayer/communicate.c`

```c
void Send_To_Down_Board(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t c1_age = now - board_feedback_debug.c1_tx_tick;
    uint32_t c2_age = now - board_feedback_debug.c2_tx_tick;
    uint8_t selected = board_feedback_next;
    HAL_StatusTypeDef result;

    if (((selected == 0u) ? c1_age : c2_age) < BOARD_FEEDBACK_PERIOD_MS)
    {
        selected ^= 1u;
    }
    if (((selected == 0u) ? c1_age : c2_age) < BOARD_FEEDBACK_PERIOD_MS)
    {
        return;
    }
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0u)
    {
        board_feedback_debug.defer_count++;
        return;
    }
    board_feedback_next = selected ^ 1u;
    Board_Tx_Update();
    if (selected == 0u)
    {
        result = Board_Tx_Meg_01(board_tx_buf1);
        if (result == HAL_OK)
        {
            board_feedback_debug.c1_tx_tick = HAL_GetTick();
            board_feedback_debug.c1_ok_count++;
        }
        else
        {
            board_feedback_debug.c1_fail_count++;
        }
    }
    else
    {
        result = Board_Tx_Meg_02(board_tx_buf2);
        if (result == HAL_OK)
        {
            board_feedback_debug.c2_tx_tick = HAL_GetTick();
            board_feedback_debug.c2_ok_count++;
        }
        else
        {
            board_feedback_debug.c2_fail_count++;
        }
    }
}
```

## 人工观察

| 位置 | Watch | 含义与判据 |
|---|---|---|
| 下板 | `board.status->heat_d3_tx_ok_count` | 成功入队次数，应持续增长 |
| 下板 | `board.status->heat_d3_tx_fail_count` | 入队失败累计；观察增长速度 |
| 下板 | `board.status->heat_d3_tx_gap_ms` | 最近成功入队间隔，预期约 10 ms |
| 下板 | `board.status->heat_d3_tx_max_gap_ms` | 启动以来最大间隔，历史值不会自动下降 |
| 下板 | `board.status->control_tx_defer_count` | D1/D2/D5 因容量不足整组延后次数 |
| 上板 | `uwTick - Board_Rx_Info.heat_pkt.rx_tick` | D3 接收年龄，应持续小于 100 ms |
| 上板 | `Board_Rx_Info.heat_pkt.flags` | 3 表示裁判参数和热量均有效 |
| 上板 | `launcher_heat.source` | 裁判参数、热量有效且 D3 未超时时应为 1 |
| 上板 | `board_feedback_debug` | C1/C2 入队成功、失败与无邮箱延后统计 |

入队成功不代表上板收到，最终以接收年龄为准。CAN 错误码可能保留历史错误，不能只看它是否归零。人工同时确认跟随、升降和掉头反馈是否正常。
