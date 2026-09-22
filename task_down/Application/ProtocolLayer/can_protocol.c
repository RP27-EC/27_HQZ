/* can_protocol.c - CAN 报文分发 */

#include "can_protocol.h"
#include "cap_protocol.h"
#include "board_protocol.h"
#include "judge.h"
#include "cap.h"
#include "board_comm_config.h"

/* CAN1 接收分发 */
void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	switch (rxId)
	{
		case ID_WHEEL_LF:
			wheel_motor[WHEEL_LF].rx(&wheel_motor[WHEEL_LF],rxBuf);
			break;
		
		case ID_WHEEL_LB:
			wheel_motor[WHEEL_LB].rx(&wheel_motor[WHEEL_LB],rxBuf);
			break;
		
		case ID_WHEEL_RF:
			wheel_motor[WHEEL_RF].rx(&wheel_motor[WHEEL_RF],rxBuf);
			break;
		
		case ID_WHEEL_RB:
			wheel_motor[WHEEL_RB].rx(&wheel_motor[WHEEL_RB],rxBuf);
			break;
		
#if BOARD_CAP_ENABLE
		case ID_SUPER_CAP_RX :
			cap.rx(&cap,rxBuf);
		  break;
#endif
		
		case ID_WIRELESS_CHARGE:
			memcpy(&wireless_rx_info,rxBuf,sizeof(wireless_rx_info_t));
		  wireless_rx_info.charging_power = int16_to_float(wireless_rx_info.charging_power, 32000, -32000, 150, 0);
			break;
		
		
		default:
			break;
	}
}

/* CAN2 接收分发 */
void CAN2_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	switch (rxId)
	{
		case ID_MEG_01:
			board.rx_01(&board,rxBuf);
			break;
		
		case ID_MEG_02:
			board.rx_02(&board,rxBuf);
		  break;	
		
		
		default:
			break;
	}
}


/* CAN3 接收分发 */
void CAN3_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	switch (rxId)
	{
		
		
		
		default:
			break;
	}
}


