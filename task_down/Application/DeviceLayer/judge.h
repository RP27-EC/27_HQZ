/* judge.h - 裁判系统协议数据结构 */

#ifndef __JUDGE_H
#define __JUDGE_H
#include "stm32h7xx_hal.h"
#include "rp_config.h"
#include "judge_protocol.h"
#include <stdint.h>
#define JUDGE_OFFLINE_CNT_MAX 1000

 
#define ID_game_status               0x0001U
#define ID_game_result               0x0002U
#define ID_game_robot_HP             0x0003U
#define ID_event_data                0x0101U
#define ID_referee_warning           0x0104U
#define ID_dart_info                 0x0105U
#define ID_robot_status              0x0201U
#define ID_power_heat_data           0x0202U
#define ID_robot_pos                 0x0203U
#define ID_buff                      0x0204U
#define ID_hurt_data                 0x0206U
#define ID_shoot_data                0x0207U
#define ID_projectile_allowance      0x0208U
#define ID_rfid_status               0x0209U
#define ID_robot_interaction_data    0x0301U
#define ID_map_command               0x0303U
#define ID_map_robot_data            0x0305U
#define ID_map_data                  0x0307U
#define ID_custom_info               0x0308U
#define ID_set_video_channel         0x0F01U
#define ID_query_video_channel       0x0F02U
#define ID_radar_enemy_HP             0x0A02U
#define ID_radar_enemy_ammo           0x0A03U
#define ID_radar_enemy_team_status    0x0A04U
#define ID_radar_enemy_robot_status   0x0A05U

 
 
#define LEN_game_status              11U
#define LEN_game_result              1U
#define LEN_game_robot_HP            20U
#define LEN_event_data               4U
#define LEN_referee_warning          3U
#define LEN_dart_info                3U
#define LEN_robot_status             17U
#define LEN_power_heat_data          14U
#define LEN_robot_pos                12U
#define LEN_buff                     8U
#define LEN_hurt_data                1U
#define LEN_shoot_data               7U
#define LEN_projectile_allowance     8U
#define LEN_rfid_status              5U
#define LEN_robot_interaction_data   112U
#define LEN_map_command              12U
#define LEN_map_robot_data           48U
#define LEN_map_data                 105U
#define LEN_custom_info              34U
#define LEN_set_video_channel        1U
#define LEN_query_video_channel      1U
#define LEN_radar_enemy_HP            12U
#define LEN_radar_enemy_ammo          10U
#define LEN_radar_enemy_team_status   8U
#define LEN_radar_enemy_robot_status  41U

 
/* -------------------- 0x0001 -------------------- */
/* struct */
typedef struct __attribute__((packed))
{
    uint8_t  game_type     : 4;  
    uint8_t  game_progress : 4;  
    uint16_t stage_remain_time;  
    uint64_t SyncTimeStamp;      
} game_status_t;

/* -------------------- 0x0002 -------------------- */
/* struct */
typedef struct __attribute__((packed))
{
    uint8_t winner;  
} game_result_t;

/* -------------------- 0x0003 -------------------- */
/* struct */
typedef struct __attribute__((packed))
{
    uint16_t ally_1_robot_HP;
    uint16_t ally_2_robot_HP;
    uint16_t ally_3_robot_HP;
    uint16_t ally_4_robot_HP;

    int16_t damage_difference;     

    uint16_t ally_7_robot_HP;
    uint16_t ally_outpost_HP;
    uint16_t ally_base_HP;

    uint16_t enemy_outpost_HP;
    uint16_t enemy_base_HP;
} game_robot_HP_t;


/* -------------------- 0x0101 -------------------- */
/* struct */
typedef struct __attribute__((packed))
{
    uint32_t event_data;
} event_data_t;

/* -------------------- 0x0104 -------------------- */
/* struct */
typedef struct __attribute__((packed))
{
    uint8_t level;               
    uint8_t offending_robot_id;  
    uint8_t count;               
} referee_warning_t;

/* -------------------- 0x0105 -------------------- */
/* struct */
typedef struct __attribute__((packed))
{
    uint8_t  dart_remaining_time;  
    uint16_t dart_info;            
} dart_info_t;

/* -------------------- 0x0201 -------------------- */
/* struct */
typedef struct __attribute__((packed))
{
    uint8_t  robot_id;
    uint8_t  robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;

    float shooter_barrel_speed_limit;

    uint8_t power_management_gimbal_output  : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
    uint8_t reserved                        : 5;
} robot_status_t;


/* -------------------- 0x0202 -------------------- */
/* struct */
typedef struct __attribute__((packed))
{
    uint16_t reserved1;
    uint16_t reserved2;
    float    reserved3;
    uint16_t buffer_energy;              
    uint16_t shooter_17mm_1_barrel_heat;  
    uint16_t shooter_42mm_barrel_heat;    
} power_heat_data_t;

/* -------------------- 0x0203 -------------------- */
/* struct */
typedef struct __attribute__((packed))
{
    float x;      
    float y;      
    float angle;  
} robot_pos_t;

/* -------------------- 0x0204 -------------------- */
/* struct */
typedef struct __attribute__((packed))
{
    uint8_t  recovery_buff;       
    uint16_t cooling_buff;        
    uint8_t  defence_buff;        
    uint8_t  vulnerability_buff;  
    uint16_t attack_buff;         
    uint8_t  remaining_energy;    
} buff_t;

/* -------------------- 0x0206 -------------------- */
/* struct */
typedef struct __attribute__((packed))
{
    uint8_t armor_id : 4;             
    uint8_t HP_deduction_reason : 4;  
} hurt_data_t;

/* -------------------- 0x0207 -------------------- */
/* struct */
typedef struct __attribute__((packed))
{
    uint8_t bullet_type;        
    uint8_t shooter_number;     
    uint8_t launching_frequency;  
    float   initial_speed;      
} shoot_data_t;

/* -------------------- 0x0208 -------------------- */
/* struct */
typedef struct __attribute__((packed))
{
    uint16_t projectile_allowance_17mm;    
    uint16_t projectile_allowance_42mm;    
    uint16_t remaining_gold_coin;          
    uint16_t projectile_allowance_fortress;  
} projectile_allowance_t;

/* -------------------- 0x0209 -------------------- */
/* struct */
typedef struct __attribute__((packed))
{
    uint32_t rfid_status;    
    uint8_t  rfid_status_2;  
} rfid_status_t;


/* -------------------- 0x0301 -------------------- */
/* struct */
typedef struct __attribute__((packed))
{
    uint16_t data_cmd_id;  
    uint16_t sender_id;    
    uint16_t receiver_id;  
    uint8_t  user_data[112];  
} robot_interaction_data_t;

/* -------------------- 0x0303 -------------------- */
/* struct */
typedef struct __attribute__((packed))
{
    float    target_position_x;  
    float    target_position_y;  
    uint8_t  cmd_keyboard;       
    uint8_t  target_robot_id;    
    uint16_t cmd_source;         
} map_command_t;

/* -------------------- 0x0305 -------------------- */
/* struct */
typedef struct __attribute__((packed))
{
    uint16_t opponent_hero_position_x;
    uint16_t opponent_hero_position_y;
    uint16_t opponent_engineer_position_x;
    uint16_t opponent_engineer_position_y;
    uint16_t opponent_infantry_3_position_x;
    uint16_t opponent_infantry_3_position_y;
    uint16_t opponent_infantry_4_position_x;
    uint16_t opponent_infantry_4_position_y;
    uint16_t opponent_aerial_position_x;
    uint16_t opponent_aerial_position_y;
    uint16_t opponent_sentry_position_x;
    uint16_t opponent_sentry_position_y;

    uint16_t ally_hero_position_x;
    uint16_t ally_hero_position_y;
    uint16_t ally_engineer_position_x;
    uint16_t ally_engineer_position_y;
    uint16_t ally_infantry_3_position_x;
    uint16_t ally_infantry_3_position_y;
    uint16_t ally_infantry_4_position_x;
    uint16_t ally_infantry_4_position_y;
    uint16_t ally_aerial_position_x;
    uint16_t ally_aerial_position_y;
    uint16_t ally_sentry_position_x;
    uint16_t ally_sentry_position_y;
} map_robot_data_t;

/* -------------------- 0x0307 -------------------- */
/* struct */
typedef struct __attribute__((packed))
{
    uint8_t  intention;          
    uint16_t start_position_x;   
    uint16_t start_position_y;   
    int8_t   delta_x[49];        
    int8_t   delta_y[49];        
    uint16_t sender_id;          
} map_data_t;

/* -------------------- 0x0308 -------------------- */
/* struct */
typedef struct __attribute__((packed))
{
    uint16_t sender_id;     
    uint16_t receiver_id;   
    uint8_t  user_data[30];  
} custom_info_t;

/* -------------------- 0x0F01 -------------------- */
/* struct */
typedef struct __attribute__((packed))
{
    uint8_t channel;  
} set_video_channel_t;

/* -------------------- 0x0F02 -------------------- */
/* struct */
typedef struct __attribute__((packed))
{
    uint8_t query;  
} query_video_channel_t;


/* struct */
typedef struct __attribute__((packed))
{
    uint16_t enemy_hero_HP;        
    uint16_t enemy_engineer_HP;    
    uint16_t enemy_infantry_3_HP;  
    uint16_t enemy_infantry_4_HP;  
    uint16_t enemy_sentry_HP;      
		uint32_t update_timestamp;

} radar_enemy_HP_t;


/* struct */
typedef struct __attribute__((packed))
{
    uint16_t enemy_hero_ammo;        
    uint16_t enemy_infantry_3_ammo;  
    uint16_t enemy_infantry_4_ammo;  
    uint16_t enemy_aerial_ammo;      
    uint16_t enemy_sentry_ammo;      
	  uint32_t update_timestamp;

} radar_enemy_ammo_t;


/* struct */
typedef struct __attribute__((packed))
{
    uint16_t remaining_gold_coin;  
    uint16_t total_gold_coin;      
    uint32_t field_status;         
	  uint32_t update_timestamp;
} radar_enemy_team_status_t;


 
typedef struct __attribute__((packed))
{
    uint8_t  recovery_buff;        
    uint16_t cooling_buff;         
    uint8_t  defence_buff;         
    uint8_t  vulnerability_buff;   
    uint16_t attack_buff;          
} radar_enemy_robot_buff_t;


/* struct */
typedef struct __attribute__((packed))
{
    radar_enemy_robot_buff_t hero;        
    radar_enemy_robot_buff_t engineer;    
    radar_enemy_robot_buff_t infantry_3;  
    radar_enemy_robot_buff_t infantry_4;  
    radar_enemy_robot_buff_t sentry;      

    uint8_t sentry_mode;                  

    uint8_t hero_status;                  
    uint8_t engineer_status;              
    uint8_t infantry_3_status;            
    uint8_t infantry_4_status;            
    uint8_t sentry_status;                
		uint32_t update_timestamp;

} radar_enemy_robot_status_t;

typedef struct __attribute__((packed))
{
    radar_enemy_HP_t            radar_enemy_HP;
    radar_enemy_ammo_t          radar_enemy_ammo;
    radar_enemy_team_status_t   radar_enemy_team_status;
    radar_enemy_robot_status_t  radar_enemy_robot_status;
	  uint32_t current_timestamp;

} radar_information_status_t;


typedef struct{
	game_status_t              game_status;           
	game_result_t              game_result;          
  game_robot_HP_t            game_robot_HP;        
	event_data_t               event_data;           
	referee_warning_t          referee_warning;    
	dart_info_t                dart_info;            
	robot_status_t             robot_status;        
	power_heat_data_t          power_heat_data;       
	robot_pos_t                robot_pos;             
	buff_t                     buff;            
	hurt_data_t                hurt_data;           
	shoot_data_t               shoot_data;         
	projectile_allowance_t     projectile_allowance;
	rfid_status_t              rfid_status;         
	robot_interaction_data_t   robot_interaction_data;
	map_command_t              map_command;       
	map_robot_data_t           map_robot_data;      
	map_data_t                 map_data;           
	custom_info_t              custom_info;      
	set_video_channel_t        set_video_channel;   
	query_video_channel_t      query_video_channel;
	
	radar_information_status_t radar_information_status;

}Judge_Info_t;


typedef enum{
	J_HERO=0,
	J_ENGINEER,
	J_INFANTRY_3,
	J_INFANTRY_4,
	J_SENTRY,
	J_OUTPOST,
	J_BASE,
	J_ROBOT_CNT,
	
}Judge_Robot_Class_e;


typedef struct{
	uint8_t  robot_id;                        
  uint8_t  game_progress;
	
  uint16_t blood[J_ROBOT_CNT];

  uint16_t shooter_barrel_heat_limit;      
	uint16_t shooter_17mm_1_barrel_heat;     
	
  uint16_t chassis_power_limit;            
  uint16_t buffer_energy;                  
  
	uint8_t launching_frequency;             
  float   initial_speed;                   
	uint16_t projectile_allowance_17mm;      
	
	
  uint16_t enemy_blood[J_ROBOT_CNT];           
  uint16_t enemy_ammo[J_ROBOT_CNT];        
	uint16_t enemy_remaining_gold;
  uint8_t enemy_robot_status[J_ROBOT_CNT];
	
}Judge_Pkt_t;


typedef struct
{
	uint16_t offline_cnt_max;
	dev_work_state_t status;
	uint16_t offline_cnt;
}Judge_Status_t;

typedef struct Judge_Struct_t
{
	Judge_Info_t* info;
	Judge_Pkt_t* pkt;
	Judge_Status_t* status;
	
	void (*heartbeat)(struct Judge_Struct_t *judge);
	
	void (*rx)(uint16_t id, uint8_t *rxBuf);

	void (*init)(struct Judge_Struct_t *judge);
	
}Judge_t;


typedef struct
{
	float speed_now;
	uint16_t shoot_num;
	
	uint16_t lower_237;
	uint16_t speed_237;
	uint16_t speed_238;
	uint16_t speed_239;
	uint16_t speed_240;
	uint16_t speed_241;
	uint16_t speed_242;
	uint16_t speed_243;
	uint16_t speed_244;
	uint16_t speed_245;
	uint16_t speed_246;
	uint16_t speed_247;
	uint16_t speed_248;
	uint16_t speed_249;
	uint16_t speed_250;
	uint16_t higher_250;
	uint16_t num;
	float mean;
	float variance;
	
	uint32_t shooting_cmd_excute_tick;
	uint16_t shooting_cmd_excute_tick_buf[100];
	float shooting_cmd_excute_tick_mean;
	float shooting_cmd_excute_tick_variance;
	
	int16_t temperature_L;
	uint8_t shooting_flag;
	uint8_t shoot_mode;
	
}bullet_data_t;


extern Judge_t judge;

extern bullet_data_t  shoot_statistics;

void Judge_Init(Judge_t* judge);
void Judge_Heart_Beat(Judge_t* judge);
void Judge_Data_Update(uint16_t id, uint8_t *rxBuf);

void Shooting_Cmd_Excute_Tick_Calculating(uint8_t flag);
void Speed_Statistic(void);

#endif

