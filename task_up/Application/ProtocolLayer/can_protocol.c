#include "can_protocol.h"
#include "communicate.h"
#include "motor.h"

void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
    switch (rxId)
    {
    case ID_GIMB_P:
        dm_motor[PITCH].rx(&dm_motor[PITCH], rxBuf);
        break;

    default:
        break;
    }
}

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
