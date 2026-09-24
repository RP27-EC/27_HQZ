/* can_protocol.h - CAN 报文分发 */

#ifndef __CAN_PROTOCOL_H
#define __CAN_PROTOCOL_H
#include "driver.h"
#include "device.h"
#define SLAVE_TX_ID
#define SLAVE_RX_ID

/* CAN 接收分发 */
void CAN1_rxDataHandler(uint32_t canId, uint8_t *rxBuf); /* CAN1 分发 */
void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf); /* CAN2 分发 */
void CAN_SendAll(void);     /* 组帧发送全部 CAN 输出 */
void CAN_Send(void);        /* 发送当前 CAN 输出 */
void CAN_SendAllZero(void); /* 全零输出 */


#endif


