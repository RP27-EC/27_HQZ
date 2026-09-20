/**
  ******************************************************************************
  * @file    connect_task.c
  * @brief   上下板通信任务
  ******************************************************************************
  */
#include "connect_task.h"
#include "board_protocol.h"
#include "board_comm_config.h"

void StartConnectTask(void const *argument)
{
#if BOARD_COMM_TX_ENABLE
    uint16_t d3d4_div = 0u;
#endif

    (void)argument;

    for (;;)
    {
#if BOARD_COMM_TX_ENABLE
        board.tx_01(&board);
        board.tx_02(&board);

#if BOARD_COMM_D5_ENABLE
        board.tx_05(&board);
#endif

#if BOARD_COMM_D3D4_ENABLE
        d3d4_div++;
        if (d3d4_div >= BOARD_COMM_D3D4_PERIOD_MS)
        {
            d3d4_div = 0u;
            board.tx_03(&board);
            board.tx_04(&board);
        }
#else
        (void)d3d4_div;
#endif
#endif

        osDelay(BOARD_COMM_D1D2_PERIOD_MS);
    }
}
