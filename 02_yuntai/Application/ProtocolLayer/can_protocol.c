#include "can_protocol.h"
#include "motor.h"

/**
 *  @brief  CAN1 接收数据
 */
void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	switch (rxId)
	{
		case ID_GIMB_P:
			Pitch_Motor.rx(&Pitch_Motor, rxBuf);
			break;


		default:
			break;
	}
}
/**
 *  @brief  CAN2 接收数据
 */
void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
	
	switch (canId)
	{
		
		case ID_GIMB_Y:
			Yaw_Motor.rx(&Yaw_Motor, rxBuf);
			break;

		default:
			break;
	}
}
