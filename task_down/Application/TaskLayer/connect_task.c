/* connect_task.c - 连接任务 */

#include "connect_task.h"
#include "board_protocol.h"
#include "board_comm_config.h"
#include "main.h"

extern FDCAN_HandleTypeDef hfdcan2;

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

