/* BRT_code.h - 外设扩展模块驱动 */

#ifndef __BRT_CODE_H
#define __BRT_CODE_H
#include "rp_config.h"
#include "drv_can.h"

typedef enum Code_BRT_Command
{
	Read_encoder = 0x01,
	Set_Id,  // SetID
	Set_Baud,
	Set_Code_Mode,  // Set代码模式
	Set_Receive_Time,  // SetReceive时间
	Set_Zero_Pole,  // Set零Pole
	Set_Dire,
	Read_sum_encoder,
	Read_Turn,
	Read_speed,  // Read速度
	Set_Sample_Time,  // SetSample时间
	Set_Mid_Pole,  // Set中Pole
	Set_Num_Pole,  // Set数量Pole
	BRT_Command_Num,
}Code_BRT_Command_e;


typedef struct Code_BRT_Born_Info_struct_t
{
		uint32_t stdId;  // stdID
	
    FDCAN_HandleTypeDef *hcan;
}Code_BRT_Born_Info_t;


typedef struct Code_BRT_Rx_Info_struct_t
{
    uint16_t angle_raw;//0~4096
	
    float angle;//0~2PI

		int32_t speed_raw;//-2147483648~2147483647
	
    float speed;//rpm
	
	  uint32_t sum_angle_raw;//0~2147483647
	
		int32_t sum_encoder;//-1073741823~1073741823
	
	  float sum_angle;
	
		uint32_t sum_turn_raw;//0~524287
	
	  int32_t sum_turn;//-262143~262143
}Code_BRT_Rx_Info_t;

typedef struct Code_BRT_Tx_Info_struct_t
{
	uint8_t command_flag[BRT_Command_Num];  // command标志
	
	uint8_t tx_buff[8];
	
	uint8_t new_id;
}Code_BRT_Tx_Info_t;

typedef struct Code_BRT_State_struct_t
{
    uint32_t offline_cnt;

    uint32_t offline_cnt_max;

    dev_work_state_t status;
}Code_BRT_State_t;


typedef struct Code_BRT_struct_t
{
	
    Code_BRT_Born_Info_t* born_info;
	
    Code_BRT_Rx_Info_t* rx_info;
	
    Code_BRT_State_t* state;
	
	  Code_BRT_Tx_Info_t* tx_info;
	
	  void (*rx)(struct Code_BRT_struct_t *code, uint8_t *rxBuf);
	
	  void (*init)(struct Code_BRT_struct_t *code);
	
	  void (*single_heart_beat)(struct Code_BRT_struct_t *code);
	
		void (*set_receive_time)(struct Code_BRT_struct_t *code, uint16_t Time);
	
		void (*receive_encoder)(struct Code_BRT_struct_t *code);
	
	  void (*receive_speed)(struct Code_BRT_struct_t *code);
	
	  void (*receive_sum_encoder)(struct Code_BRT_struct_t *code);
		
		void (*receive_sum_turn)(struct Code_BRT_struct_t *code);
	
	  void (*set_mode)(struct Code_BRT_struct_t *code, uint8_t Mode);
		
		void (*set_id)(struct Code_BRT_struct_t *code, uint8_t id);
		
		void (*set_baud)(struct Code_BRT_struct_t *code, uint8_t Baud);
		
		void (*set_zero)(struct Code_BRT_struct_t *code);
		
		void (*set_mid)(struct Code_BRT_struct_t *code);
		
		void (*set_encoder)(struct Code_BRT_struct_t *code,uint16_t encoder);
		
		void (*set_dire)(struct Code_BRT_struct_t *code, uint8_t dire);
		
		void (*set_sample_time)(struct Code_BRT_struct_t* code, uint16_t time);
}Code_BRT_t;

void BRT_Code_Init(Code_BRT_t* code);

#endif

