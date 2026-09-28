/* can_protocol.h - CAN 报文分发 */

#ifndef __CAN_PROTOCOL_H
#define __CAN_PROTOCOL_H
#include "driver.h"
#include "device.h"

/* CAN 接收分发 */
void CAN1_rxDataHandler(uint32_t canId, uint8_t *rxBuf); /* CAN1 分发 */
void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf); /* CAN2 分发 */

#endif


