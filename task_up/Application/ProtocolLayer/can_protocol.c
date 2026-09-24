/* can_protocol.c - CAN 报文分发 */

#include "can_protocol.h"
#include "communicate.h"
#include "motor.h"

/* CAN1：Pitch、摩擦轮、拨盘反馈 */
void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
    switch (rxId)
    {
    case ID_GIMB_P:
        dm_motor[PITCH].rx(&dm_motor[PITCH], rxBuf);
        break;

    case ID_FRIC_L:
        rm_motor[SHOOT_FRIC_L].rx(&rm_motor[SHOOT_FRIC_L], rxBuf);
        break;

    case ID_FRIC_R:
        rm_motor[SHOOT_FRIC_R].rx(&rm_motor[SHOOT_FRIC_R], rxBuf);
        break;

    case ID_DIAL:
        dail_motor.get_info(&dail_motor, rxBuf);
        break;

    default:
        break;
    }
}

/* CAN2：Yaw 与下板报文 */
void CAN2_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
    switch (rxId)
    {
    case ID_GIMB_Y:
        dm_motor[YAW].rx(&dm_motor[YAW], rxBuf);
        break;

    case ID_BOARD_RX1:
        Board_Rx_01(rxBuf);
        break;

    case ID_BOARD_RX2:
        Board_Rx_02(rxBuf);
        break;

    case ID_BOARD_RX3:
        Board_Rx_03(rxBuf);
        break;

    case ID_BOARD_RX4:
        Board_Rx_04(rxBuf);
        break;

    case ID_BOARD_RX5:
        Board_Rx_05(rxBuf);
        break;

    default:
        break;
    }
}

