/* cap_protocol.h - 超电通信协议 */

#ifndef __CAP_PROTOCOL_H
#define __CAP_PROTOCOL_H

#include "main.h"

#define  ID_SUPER_CAP_TX        0x222
#define  ID_SUPER_CAP_RX        0x211
#define  ID_WIRELESS_CHARGE     0x212


typedef struct __attribute__((packed)) cap_rx_info_struct {
    
    int16_t now_chassis_power;  // now底盘功率
    int16_t now_cap_V;
    int16_t now_cap_I;
    
    struct __attribute__((packed)) bit_state_struct
    {
        uint8_t ability             : 1;
			  uint8_t is_in_pre_charge_mode : 1;
	
        uint8_t unuse               : 6;
    }bit_state;
    
} cap_rx_info_t;

typedef struct
{
		int16_t chassis_power;
    float cap_Ucr;
    float cap_I;
    
		uint8_t ability;
}cap_receive_data_t;

typedef struct __attribute__((packed))cap_transmit_data_struct {
    
    uint8_t  chassis_power_buffer;  // 底盘功率缓冲
    uint16_t chassis_power_limit ;  // 底盘功率limit
    int16_t  cap_power_out_limit ;  // cap功率输出limit
    uint16_t cap_power_in_limit  ;
    
    struct __attribute__((packed)) bit_control_struct
    {
        uint8_t cap_switch : 1;
        uint8_t turbo_mode : 1;
			  uint8_t pre_charge_mode_en : 1;
			
        uint8_t unuse      : 5;
    }bit_control;
    
}cap_transmit_data_t;


typedef struct __attribute__((packed)){
 int16_t charging_power;  // charging功率
 uint8_t is_charging;
 uint8_t reserved1;
 uint16_t reserved2;
 uint16_t reserved3; 
} wireless_rx_info_t;


typedef struct
{
	uint16_t offline_cnt_max;
	uint8_t status;
	uint16_t offline_cnt;
}cap_status_t;

typedef struct cap_struct_t
{
	uint8_t Y_O_N;
	uint8_t record_Y_O_N;
	cap_status_t* status;
	cap_receive_data_t* info;
	
	void (*heartbeat)(struct cap_struct_t *my_cap);
	void (*tx)(void);
	void (*rx)(struct cap_struct_t* my_cap, uint8_t *rxBuf);
	void (*init)(struct cap_struct_t *my_cap);
}cap_t;

extern cap_receive_data_t cap_receive_data;
extern cap_transmit_data_t cap_tx_info;
extern wireless_rx_info_t wireless_rx_info;

void cap_send_2E(void);
void cap_send_2F(void);
void cap_update(cap_t* my_cap, uint8_t *rxBuf);
int16_t float_to_int16(float a, float a_max, float a_min, int16_t b_max, int16_t b_min);
float int16_to_float(int16_t a, int16_t a_max, int16_t a_min, float b_max, float b_min);

#endif




