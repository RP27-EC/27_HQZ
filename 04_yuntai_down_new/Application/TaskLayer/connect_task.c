/**
  ******************************************************************************
  * @file    connect_task.c
  * @brief   Board-to-board transmit task.
  ******************************************************************************
  */
#include "connect_task.h"
#include "board_protocol.h"
#include "board_comm_config.h"

void StartConnectTask(void const *argument)
{
    (void)argument;

    for (;;)
    {
        board.tx_01(&board);
        board.tx_02(&board);

#if BOARD_COMM_D5_ENABLE
        board.tx_05(&board);
#endif

        osDelay(BOARD_COMM_D1D2_PERIOD_MS);
    }
}
