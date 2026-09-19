#include "can_protocol.h"
#include "board_protocol.h"

void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
    (void)rxId;
    (void)rxBuf;
}

void CAN2_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
    switch (rxId)
    {
        case ID_MEG_01:
            board.rx_01(&board, rxBuf);
            break;

        case ID_MEG_02:
            board.rx_02(&board, rxBuf);
            break;

        default:
            break;
    }
}

void CAN3_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
    (void)rxId;
    (void)rxBuf;
}
