/* connect_task.c - 连接任务 */

#include "connect_task.h"
#include "board_protocol.h"
#include "board_comm_config.h"

/* 板间发送任务, 周期见 board_comm_config.h */
void StartConnectTask(void const *argument)
{
    (void)argument;

    for (;;)
    {
#if BOARD_COMM_TX_ENABLE
        board.tx_01(&board);
        board.tx_02(&board);

#if BOARD_COMM_D5_ENABLE
        board.tx_05(&board);
#endif
#endif

        osDelay(BOARD_COMM_D1D2_PERIOD_MS);
    }
}

