/* can_protocol.h - CAN 报文分发 */
#ifndef __CAN_PROTOCOL_H
#define __CAN_PROTOCOL_H
#include "driver.h"
#include "device.h"
/* 下主控CAN ID */
#define SLAVE_TX_ID 
#define SLAVE_RX_ID 

/*CAN1*/
#define CHASSIS_CAN_ID_LF RM3508_CAN_ID_201 /* 左前轮 */
#define CHASSIS_CAN_ID_RF RM3508_CAN_ID_202 /* 右前轮 */
#define CHASSIS_CAN_ID_LB RM3508_CAN_ID_203 /* 左后轮 */
#define CHASSIS_CAN_ID_RB RM3508_CAN_ID_204 /* 右后轮 */

/*CAN2*/
#define GIMBAL_CAN_ID_PITCH GM6020_CAN_ID_205 /* Pitch 轴 */
#define GIMBAL_CAN_ID_YAW GM6020_CAN_ID_206   /* Yaw 轴 */
#define FRIC_CAN_ID_LEFT RM3508_CAN_ID_201  /* 左摩擦轮 */
#define FRIC_CAN_ID_RIGHT RM3508_CAN_ID_202 /* 右摩擦轮 */
#define LAUNCH_CAN_ID_DIAL RM2006_CAN_ID_203 /* 拨盘 */
void CAN1_rxDataHandler(uint32_t canId, uint8_t *rxBuf); /* CAN1 分发 */
void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf); /* CAN2 分发 */
void CAN3_rxDataHandler(uint32_t canId, uint8_t *rxBuf); /* CAN3 分发 */
void cap_data_send(uint8_t can_num);                     /* 电容数据发送 */

#endif

