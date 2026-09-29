/* connect_task.c - 连接任务 */

#include "connect_task.h"
#include "board_protocol.h"
#include "board_comm_config.h"

/* 板间发送任务 */
void StartConnectTask(void const *argument)
{
    (void)argument;

    for (;;)
    {
#if BOARD_COMM_TX_ENABLE
        board.tx_01(&board);  /* 发 D1：底盘/云台/发射机构等 状态与控制 */
        board.tx_02(&board);  /* 发 D2：云台一些参数：pitch和yaw */

#if BOARD_COMM_D5_ENABLE
        board.tx_05(&board);  /* 发 D5：遥控/键鼠控制量 */
#endif
#endif

        osDelay(BOARD_COMM_D1D2_PERIOD_MS);  /* 发送周期 */
    }
}

