#ifndef __BOARD_COMM_CONFIG_H
#define __BOARD_COMM_CONFIG_H

/* Board-to-board bring-up switches. */
#define BOARD_COMM_DEBUG                1u
#define BOARD_COMM_TX_ENABLE            1u
#define BOARD_COMM_D1D2_PERIOD_MS       1u
#define BOARD_COMM_D3D4_PERIOD_MS       10u
#define BOARD_COMM_D3D4_ENABLE          0u
#define BOARD_COMM_D5_ENABLE            1u

/* Temporarily bypassed vehicle modules during bring-up. */
#define BOARD_CAP_ENABLE                0u
#define BOARD_JUDGE_ENABLE              0u
#define BOARD_UI_ENABLE                 0u

#endif
