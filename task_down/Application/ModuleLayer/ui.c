#include "ui.h"
#include "priority_ui.h"
#include "infantry.h"
#include "arm_math.h"
#include "ui_protocol.h"
#include "chassis.h"
#include "judge.h"
#include "rp_math.h"
#include "gimbal.h"
#include "board_protocol.h"
#include "vision.h"
#include "launch.h"
#include "cap.h"


//UI_Dynamic_Info_t My_UI_Dynamic_Info;
#define LEFT_UP_X (Client_mid_position_x - 820)
#define LEFT_UP_Y (Client_mid_position_y + 320)
#define RIGHT_UP_X (Client_mid_position_x + 820)
#define RIGHT_UP_Y (Client_mid_position_y + 320)

#define CHAS_CIRCLE_X     (Client_mid_position_x)
#define CHAS_CIRCLE_Y     (Client_mid_position_y - 250)
#define CHAS_CIRCLE_R     (65)

#define BODY_CENTER_X     (Client_mid_position_x - 700)
#define BODY_CENTER_Y     (Client_mid_position_y + 50)
#define PITCH_LENGTH      100

#define DIAL_CENTER_X     (Client_mid_position_x)
#define DIAL_CENTER_Y     (Client_mid_position_y + 280)


#define ENEMY_BLUE_HERO_AMMO_X  (Client_mid_position_x + 220)
#define ENEMY_BLUE_HERO_AMMO_Y  (Client_mid_position_y + 360)
#define ENEMY_RED_HERO_AMMO_X   (Client_mid_position_x - 280)
#define ENEMY_RED_HERO_AMMO_Y   (Client_mid_position_y + 360)
#define ENEMY_AMMO_DISTANE   120


void rotate_point(__packed uint16_t *x, __packed uint16_t *y, uint16_t raw_x, uint16_t raw_y, float mid_x, float mid_y, float angle);
void My_Chas_Circle_Update(float angle);
static void Motor_Color_Update(uint8_t mmotor_state, uint8_t special_state, uint32_t index);
static void Gimbal_Line_Update(float angle,uint8_t height);
static void Robot_Status_Update(uint8_t robot_status,uint32_t index);
static void Robot_Status_Update(uint8_t robot_status,uint32_t index);


ui_info_t dynamic_ui_info [DYNAMIC_NUM] = 
{
	[MODE_CHAR] = {
		/*******不变配置*********/
    .ui_config.priority = HIGH_PRIORITY,
    .ui_config.ui_type = CHAR,     
    .ui_config.name = "d1",            
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    
    .ui_config.layer = 1,                
    .ui_config.color = WHITE,            
		 .ui_config.size = 30,               
    .ui_config.width = 3,                
    .ui_config.start_x = Client_mid_position_x + 580,             
    .ui_config.start_y = Client_mid_position_y + 170,             
    .ui_config.text = "SLEEP",          

	},
	
	
	[VISION_FRAME] = {
		/*******不变配置*********/
    .ui_config.priority = LOW_PRIORITY, 
    .ui_config.ui_type = RECTANGEL,        
    .ui_config.name = "d2",              
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    
    .ui_config.layer = 1,               
    .ui_config.color = WHITE,           
    .ui_config.width = 3,               
     .ui_config.start_x = Client_mid_position_x + 790 ,         
    .ui_config.start_y = Client_mid_position_y + 110 ,              
		.ui_config.end_x = Client_mid_position_x + 570 ,
		.ui_config.end_y = Client_mid_position_y + 50 ,
		
	},
	
	[VISION_NUM] = {
		/*******不变配置*********/
		.ui_config.priority = HIGH_PRIORITY,
    .ui_config.ui_type = INT,          
    .ui_config.name = "d3",             
    /*******可变配置*********/
		.ui_config.operate_type = MODIFY,  
    .ui_config.layer = 1,               
    .ui_config.color = CYAN_BLUE,            
    .ui_config.size = 30,               
    .ui_config.width = 2,               
    .ui_config.start_x = Client_mid_position_x + 760,             
    .ui_config.start_y = Client_mid_position_y + 97,             
    .ui_config.int_num = 0,
		
	},
	
	[BULLET_NUM] = {
		/*******不变配置*********/
    .ui_config.priority = MID_PRIORITY,
    .ui_config.ui_type = INT,           
    .ui_config.name = "d4",             
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,   
    .ui_config.layer = 1,               
    .ui_config.color = GREEN,          
    .ui_config.size = 50,              
    .ui_config.width = 4,              
    .ui_config.start_x = Client_mid_position_x - 260,           
    .ui_config.start_y = Client_mid_position_y + 160,             
    .ui_config.int_num = 0,            
	},
	
	[CHAS_HEAD_LINE] = {
		/*******不变配置*********/
    .ui_config.priority = HIGH_PRIORITY,
    .ui_config.ui_type = LINE,        
    .ui_config.name = "d5",             
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,   
    .ui_config.layer = 2,               
    .ui_config.color = PINK,           
    .ui_config.width = 8,               
    .ui_config.start_x = CHAS_CIRCLE_X,             
    .ui_config.start_y = CHAS_CIRCLE_Y,              
    .ui_config.end_x = CHAS_CIRCLE_X,               
    .ui_config.end_y = CHAS_CIRCLE_Y + CHAS_CIRCLE_R,               
  },
	
	[CHAS_SIDE_LINE] = {
		/*******不变配置*********/
    .ui_config.priority = HIGH_PRIORITY,
    .ui_config.ui_type = LINE,        
    .ui_config.name = "d6",             
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    
    .ui_config.layer = 2,                
    .ui_config.color = PINK,            
    .ui_config.width = 8,               
    .ui_config.start_x = CHAS_CIRCLE_X - CHAS_CIRCLE_R,             
    .ui_config.start_y = CHAS_CIRCLE_Y,              
    .ui_config.end_x = CHAS_CIRCLE_X + CHAS_CIRCLE_R,               
    .ui_config.end_y = CHAS_CIRCLE_Y,                
	},
	
	
	[CAP_LINE] = {
		 /*******不变配置*********/
    .ui_config.priority = HIGH_PRIORITY, 
    .ui_config.ui_type = LINE,         
    .ui_config.name = "d7",             
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    
    .ui_config.layer = 1,               
    .ui_config.color = GREEN,           
    .ui_config.width = 25,               
    .ui_config.start_x = Client_mid_position_x - 250,            
    .ui_config.start_y = Client_mid_position_y + 220,             
    .ui_config.end_x = Client_mid_position_x + 250,               
    .ui_config.end_y = Client_mid_position_y + 220,                
	},
	
	
	[AUTO_CATCH_FRAME] = {
		 /*******不变配置*********/
    .ui_config.priority = HIGH_PRIORITY, 
    .ui_config.ui_type = RECTANGEL,      
    .ui_config.name = "d8",              
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    
    .ui_config.layer = 1,                
    .ui_config.color = WHITE,            
    .ui_config.width = 3,                
     .ui_config.start_x = Client_mid_position_x - 280,             
    .ui_config.start_y = Client_mid_position_y + 170 ,             
		.ui_config.end_x = Client_mid_position_x + 280 ,
		.ui_config.end_y = Client_mid_position_y - 180 ,
	},
	
	[CAR_SPEED] = {
		/*******不变配置*********/
  .ui_config.priority = MID_PRIORITY,
  .ui_config.ui_type = FLOAT,
	.ui_config.name = "d9",
  /*********不变配置*********/
  .ui_config.operate_type = MODIFY, 
  .ui_config.layer = 1, 
  .ui_config.color = CYAN_BLUE, 
  .ui_config.size = 30, 
  .ui_config.width = 2,
  .ui_config.start_x = Client_mid_position_x - 60,
  .ui_config.start_y = Client_mid_position_y - 400, 
  .ui_config.float_num = 0, 
  .ui_config.decimal = 2, 
	},
	
	[R_FRIC_CIRCLE] = {
		 /*******不变配置*********/
    .ui_config.priority = MID_PRIORITY, 
    .ui_config.ui_type = CIRCLE,        
    .ui_config.name = "d10",              
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    
    .ui_config.layer = 1,                
    .ui_config.color = GREEN,            
    .ui_config.width = 8,                
    .ui_config.start_x = DIAL_CENTER_X + 35, // Client_mid_position_x + 750,              
    .ui_config.start_y = DIAL_CENTER_Y + 30,  //Client_mid_position_y + 100,              
		.ui_config.radius = 20,
	},
	
	[L_FRIC_CIRCLE] = {
		 /*******不变配置*********/
    .ui_config.priority = MID_PRIORITY,
    .ui_config.ui_type = CIRCLE,       
    .ui_config.name = "d11",           
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,  
    .ui_config.layer = 1,              
    .ui_config.color = GREEN,          
    .ui_config.width = 8,              
    .ui_config.start_x = DIAL_CENTER_X - 35,//Client_mid_position_x + 680,            
    .ui_config.start_y = DIAL_CENTER_Y + 30,//Client_mid_position_y + 100,            
		.ui_config.radius = 20,
	},
	
	[DIAL_CIRCLE] = {
		 /*******不变配置*********/
    .ui_config.priority = MID_PRIORITY,
    .ui_config.ui_type = CIRCLE,       
    .ui_config.name = "d12",           
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,  
    .ui_config.layer = 1,              
    .ui_config.color = GREEN,          
    .ui_config.width = 8,              
    .ui_config.start_x = DIAL_CENTER_X,//Client_mid_position_x + 715,           
    .ui_config.start_y = DIAL_CENTER_Y,//Client_mid_position_y + 70,            
		.ui_config.radius = 20,
	},
	
	
	[LF_CIRCLE] = {
		 /*******不变配置*********/
    .ui_config.priority = MID_PRIORITY, 
    .ui_config.ui_type = CIRCLE,        
    .ui_config.name = "d13",            
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,   
    .ui_config.layer = 1,               
    .ui_config.color = GREEN,           
    .ui_config.width = 8,               
    .ui_config.start_x = CHAS_CIRCLE_X - CHAS_CIRCLE_R - 30,    
    .ui_config.start_y = CHAS_CIRCLE_Y + 30,                    
		.ui_config.radius = 20,
	},
	
	[LB_CIRCLE] = {
		 /*******不变配置*********/
    .ui_config.priority = MID_PRIORITY, 
    .ui_config.ui_type = CIRCLE,        
    .ui_config.name = "d14",            
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,   
    .ui_config.layer = 1,               
    .ui_config.color = GREEN,           
    .ui_config.width = 8,               
    .ui_config.start_x = CHAS_CIRCLE_X - CHAS_CIRCLE_R - 30,          
    .ui_config.start_y = CHAS_CIRCLE_Y - 30,                          
		.ui_config.radius = 20,
	},
	
	[RF_CIRCLE] = {
		 /*******不变配置*********/
    .ui_config.priority = MID_PRIORITY,
    .ui_config.ui_type = CIRCLE,       
    .ui_config.name = "d15",           
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,  
    .ui_config.layer = 1,              
    .ui_config.color = GREEN,          
    .ui_config.width = 8,              
    .ui_config.start_x = CHAS_CIRCLE_X + CHAS_CIRCLE_R + 30,          
    .ui_config.start_y = CHAS_CIRCLE_Y + 30,                          
		.ui_config.radius = 20,
	},
	
	[RB_CIRCLE] = {
		 /*******不变配置*********/
    .ui_config.priority = MID_PRIORITY,
    .ui_config.ui_type = CIRCLE,       
    .ui_config.name = "d16",           
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,  
    .ui_config.layer = 1,              
    .ui_config.color = GREEN,          
    .ui_config.width = 8,              
    .ui_config.start_x = CHAS_CIRCLE_X + CHAS_CIRCLE_R + 30,             
    .ui_config.start_y = CHAS_CIRCLE_Y - 30,                             
		.ui_config.radius = 20,
	},
	
	[YAW_CIRCLE] = {
		 /*******不变配置*********/
    .ui_config.priority = MID_PRIORITY, 
    .ui_config.ui_type = CIRCLE,        
    .ui_config.name = "d17",            
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,   
    .ui_config.layer = 1,               
    .ui_config.color = GREEN,           
    .ui_config.width = 8,               
    .ui_config.start_x = BODY_CENTER_X, 
    .ui_config.start_y = BODY_CENTER_Y, 
		.ui_config.radius = 20,
	},
	
	
	[LEFT_CIRCLE] = {
		 /*******不变配置*********/
    .ui_config.priority = MID_PRIORITY, 
    .ui_config.ui_type = CIRCLE,        
    .ui_config.name = "d18",            
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,   
    .ui_config.layer = 1,               
    .ui_config.color = GREEN,           
    .ui_config.width = 8,               
    .ui_config.start_x = BODY_CENTER_X - 100,      
    .ui_config.start_y = BODY_CENTER_Y,            
		.ui_config.radius = 20,
	},
	
	[PITCH_CIRCLE] = {
		 /*******不变配置*********/
    .ui_config.priority = MID_PRIORITY, 
    .ui_config.ui_type = CIRCLE,        
    .ui_config.name = "d19",            
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,   
    .ui_config.layer = 1,               
    .ui_config.color = GREEN,           
    .ui_config.width = 8,               
    .ui_config.start_x = BODY_CENTER_X, 
    .ui_config.start_y = BODY_CENTER_Y + 120,             
		.ui_config.radius = 20,
	},
	
	
	[PITCH_LINE] = {
		 /*******不变配置*********/
    .ui_config.priority = HIGH_PRIORITY, 
    .ui_config.ui_type = LINE,         
    .ui_config.name = "d20",             
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    
    .ui_config.layer = 1,                
    .ui_config.color = WHITE,            
    .ui_config.width = 4,                
    .ui_config.start_x = BODY_CENTER_X,  
    .ui_config.start_y = BODY_CENTER_Y + 120,              
    .ui_config.end_x = BODY_CENTER_X + PITCH_LENGTH,       
    .ui_config.end_y = BODY_CENTER_Y + 120,                
	},
	
	[ENEMY_COIN_NUM] = {
		/*******不变配置*********/
    .ui_config.priority = MID_PRIORITY,
    .ui_config.ui_type = INT,           
    .ui_config.name = "d21",             
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,   
    .ui_config.layer = 1,               
    .ui_config.color = WHITE,          
    .ui_config.size = 20,              
    .ui_config.width = 4,              
    .ui_config.start_x = Client_mid_position_x - 170,           
    .ui_config.start_y = Client_mid_position_y + 370,             
    .ui_config.int_num = 0,            
	},
	
	[ENEMY_HERO_AMMO_NUM] = {
		/*******不变配置*********/
    .ui_config.priority = MID_PRIORITY,
    .ui_config.ui_type = INT,           
    .ui_config.name = "d22",             
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,   
    .ui_config.layer = 1,               
    .ui_config.color = WHITE,          
    .ui_config.size = 20,              
    .ui_config.width = 4,              
    .ui_config.start_x = ENEMY_BLUE_HERO_AMMO_X,           
    .ui_config.start_y = ENEMY_BLUE_HERO_AMMO_Y - 40,             
    .ui_config.int_num = 0,            
	},
	
	[ENEMY_3_AMMO_NUM] = {
		/*******不变配置*********/
    .ui_config.priority = MID_PRIORITY,
    .ui_config.ui_type = INT,           
    .ui_config.name = "d23",             
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,   
    .ui_config.layer = 1,               
    .ui_config.color = WHITE,          
    .ui_config.size = 20,              
    .ui_config.width = 4,              
    .ui_config.start_x = ENEMY_BLUE_HERO_AMMO_X + 2*ENEMY_AMMO_DISTANE,           
    .ui_config.start_y = ENEMY_BLUE_HERO_AMMO_Y - 40,             
    .ui_config.int_num = 0,            
	},
	
	
	[ENEMY_4_AMMO_NUM] = {
		/*******不变配置*********/
    .ui_config.priority = MID_PRIORITY,
    .ui_config.ui_type = INT,           
    .ui_config.name = "d24",             
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,   
    .ui_config.layer = 1,               
    .ui_config.color = WHITE,          
    .ui_config.size = 20,              
    .ui_config.width = 4,              
    .ui_config.start_x = ENEMY_BLUE_HERO_AMMO_X + 3*ENEMY_AMMO_DISTANE,           
    .ui_config.start_y = ENEMY_BLUE_HERO_AMMO_Y - 40,             
    .ui_config.int_num = 0,            
	},
	
	
	[ENEMY_7_AMMO_NUM] = {
		/*******不变配置*********/
    .ui_config.priority = MID_PRIORITY,
    .ui_config.ui_type = INT,           
    .ui_config.name = "d25",             
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,   
    .ui_config.layer = 1,               
    .ui_config.color = WHITE,          
    .ui_config.size = 20,              
    .ui_config.width = 4,              
    .ui_config.start_x = ENEMY_BLUE_HERO_AMMO_X + 5*ENEMY_AMMO_DISTANE,           
    .ui_config.start_y = ENEMY_BLUE_HERO_AMMO_Y - 40,             
    .ui_config.int_num = 0,            
	},
	
	[ENEMY_HERO_HP_NUM] = {
		/*******不变配置*********/
    .ui_config.priority = MID_PRIORITY,
    .ui_config.ui_type = INT,           
    .ui_config.name = "d26",             
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,   
    .ui_config.layer = 1,               
    .ui_config.color = WHITE,          
    .ui_config.size = 20,              
    .ui_config.width = 4,              
    .ui_config.start_x = ENEMY_BLUE_HERO_AMMO_X,           
    .ui_config.start_y = ENEMY_BLUE_HERO_AMMO_Y,             
    .ui_config.int_num = 0,            
	},
	
	[ENEMY_3_HP_NUM] = {
		/*******不变配置*********/
    .ui_config.priority = MID_PRIORITY,
    .ui_config.ui_type = INT,           
    .ui_config.name = "d27",             
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,   
    .ui_config.layer = 1,               
    .ui_config.color = WHITE,          
    .ui_config.size = 20,              
    .ui_config.width = 4,              
    .ui_config.start_x = ENEMY_BLUE_HERO_AMMO_X + 2*ENEMY_AMMO_DISTANE,           
    .ui_config.start_y = ENEMY_BLUE_HERO_AMMO_Y,             
    .ui_config.int_num = 0,            
	},
	
	[ENEMY_4_HP_NUM] = {
		/*******不变配置*********/
    .ui_config.priority = MID_PRIORITY,
    .ui_config.ui_type = INT,           
    .ui_config.name = "d28",             
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,   
    .ui_config.layer = 1,               
    .ui_config.color = WHITE,          
    .ui_config.size = 20,              
    .ui_config.width = 4,              
    .ui_config.start_x = ENEMY_BLUE_HERO_AMMO_X + 3*ENEMY_AMMO_DISTANE,           
    .ui_config.start_y = ENEMY_BLUE_HERO_AMMO_Y,             
    .ui_config.int_num = 0,            
	},
	
	[ENEMY_7_HP_NUM] = {
		/*******不变配置*********/
    .ui_config.priority = MID_PRIORITY,
    .ui_config.ui_type = INT,           
    .ui_config.name = "d29",             
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,   
    .ui_config.layer = 1,               
    .ui_config.color = WHITE,          
    .ui_config.size = 20,              
    .ui_config.width = 4,              
    .ui_config.start_x = ENEMY_BLUE_HERO_AMMO_X + 5*ENEMY_AMMO_DISTANE,           
    .ui_config.start_y = ENEMY_BLUE_HERO_AMMO_Y,             
    .ui_config.int_num = 0,            
	},
	
	
	
	
	[CHARGE_CHAR] = {
		/*******不变配置*********/
    .ui_config.priority = MID_PRIORITY,
    .ui_config.ui_type = CHAR,     
    .ui_config.name = "d30",            
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    
    .ui_config.layer = 1,                
    .ui_config.color = WHITE,            
		 .ui_config.size = 30,               
    .ui_config.width = 3,                
    .ui_config.start_x = Client_mid_position_x + 580,             
    .ui_config.start_y = Client_mid_position_y + 24,             
    .ui_config.text = "CHARGED",          

	},
};

ui_info_t const_ui_info [CONST_NUM] = 
{

	 [VISION_CHAR] = {
		 /*******不变配置*********/
    .ui_config.ui_type = CHAR,          
    .ui_config.name = "g1",             
    /*******可变配置*********/
    .ui_config.layer = 1,               
    .ui_config.color = WHITE,           
    .ui_config.size = 30,               
    .ui_config.width = 2,               
    .ui_config.start_x = Client_mid_position_x + 580,           
    .ui_config.start_y = Client_mid_position_y + 97,              
    .ui_config.text = "VISION",           
	 },
	 
	 [CHAS_CIRCLE] = {
		 /*******不变配置*********/
    .ui_config.ui_type = CIRCLE,        
    .ui_config.name = "g2",             
    /*******可变配置*********/
    .ui_config.layer = 0,               
    .ui_config.color = GREEN,           
    .ui_config.width = 4,               
    .ui_config.start_x = CHAS_CIRCLE_X ,
    .ui_config.start_y = CHAS_CIRCLE_Y, 
    .ui_config.radius = 65, 
	 },
	 
	 [CAP_FRAME] = {
		 /*******不变配置*********/
    .ui_config.ui_type = RECTANGEL,    
    .ui_config.name = "g3",            
    /*******可变配置*********/
 
    .ui_config.layer = 1,              
    .ui_config.color = WHITE,          
    .ui_config.width = 3,              
    .ui_config.start_x = Client_mid_position_x - 253,            
    .ui_config.start_y = Client_mid_position_y + 233 ,           
		.ui_config.end_x = Client_mid_position_x + 253 ,
		.ui_config.end_y = Client_mid_position_y + 205 ,
	 },
	 
	 [MOVE_L_LINE] = {
		  /*******不变配置*********/
    .ui_config.ui_type = LINE,        
    .ui_config.name = "g4",           
    /*******可变配置*********/
    .ui_config.layer = 1,             
    .ui_config.color = WHITE,         
    .ui_config.width = 1,             
    .ui_config.start_x = Client_mid_position_x - 334,            
    .ui_config.start_y = 0,             
    .ui_config.end_x = Client_mid_position_x - 82,               
    .ui_config.end_y = Client_mid_position_y - 150 ,             
		 
	 },

	 [MOVE_R_LINE] = {
		  /*******不变配置*********/
    .ui_config.ui_type = LINE,         
    .ui_config.name = "g5",            
    /*******可变配置*********/
    .ui_config.layer = 1,              
    .ui_config.color = WHITE,          
    .ui_config.width = 1,              
    .ui_config.start_x = Client_mid_position_x + 334,            
    .ui_config.start_y = 0,            
    .ui_config.end_x = Client_mid_position_x + 82,               
    .ui_config.end_y = Client_mid_position_y - 150 ,             
		 
	 },

	 
	 [CAP_DIVISION_1] = {
		 /*******不变配置*********/
    .ui_config.ui_type = LINE,         
    .ui_config.name = "g6",            
    /*******可变配置*********/
    .ui_config.layer = 0,              
    .ui_config.color = WHITE,          
    .ui_config.width = 2,              
    .ui_config.start_x = Client_mid_position_x - 250 + 70,        
    .ui_config.start_y = Client_mid_position_y + 243,             
    .ui_config.end_x = Client_mid_position_x - 250 + 70,          
    .ui_config.end_y = Client_mid_position_y + 193 ,              
	 },
//	 
	 [CAP_DIVISION_2] = {
		 /*******不变配置*********/
    .ui_config.ui_type = LINE,        
    .ui_config.name = "g7",           
    /*******可变配置*********/
    .ui_config.layer = 0,             
    .ui_config.color = WHITE,         
    .ui_config.width = 2,             
    .ui_config.start_x = Client_mid_position_x - 250 + 195,        
    .ui_config.start_y = Client_mid_position_y + 243,              
    .ui_config.end_x = Client_mid_position_x - 250 + 195,          
    .ui_config.end_y = Client_mid_position_y + 193 ,               
	 },
	
	 
	 [BODY_LINE] = {
		 /*******不变配置*********/
    .ui_config.ui_type = LINE,        
    .ui_config.name = "g8",           
    /*******可变配置*********/
    .ui_config.layer = 0,             
    .ui_config.color = WHITE,         
    .ui_config.width = 4,             
    .ui_config.start_x = BODY_CENTER_X - 100,      
    .ui_config.start_y = BODY_CENTER_Y,            
    .ui_config.end_x = BODY_CENTER_X + 100,        
    .ui_config.end_y = BODY_CENTER_Y ,             
	 },
	
	 
	  [LEFT_LINE] = {
		 /*******不变配置*********/
    .ui_config.ui_type = LINE,       
    .ui_config.name = "g9",          
    /*******可变配置*********/
    .ui_config.layer = 0,            
    .ui_config.color = WHITE,        
    .ui_config.width = 4,            
    .ui_config.start_x = BODY_CENTER_X,           
    .ui_config.start_y = BODY_CENTER_Y,           
    .ui_config.end_x = BODY_CENTER_X,             
    .ui_config.end_y = BODY_CENTER_Y + 120 ,      
	 },
	 
};





void My_Ui_Init(void)
{
  Init_Ui_List(dynamic_ui_info, sizeof(dynamic_ui_info)/sizeof(ui_info_t),const_ui_info, sizeof(const_ui_info)/sizeof(ui_info_t));
}

void Ui_Info_Update(void)
{
	client_info_update();
	
	//整车模式更新
	static Infantry_Mode_e  last_mode = I_SLEEP;
	
	if(last_mode != infantry.mode)
	{
		if(infantry.mode == I_SLEEP)
		{
			strcpy(dynamic_ui_info[MODE_CHAR].ui_config.text, "SLEEP");
		}
		else if(infantry.mode == I_INIT)
		{
			strcpy(dynamic_ui_info[MODE_CHAR].ui_config.text, "INIT");
		}
		else if(infantry.mode == I_IMU)
		{
			strcpy(dynamic_ui_info[MODE_CHAR].ui_config.text, "IMU");
		}
		else if(infantry.mode == I_MEC)
		{
			strcpy(dynamic_ui_info[MODE_CHAR].ui_config.text, "MEC");
		}
		else if(infantry.mode == I_TURN)
		{
			strcpy(dynamic_ui_info[MODE_CHAR].ui_config.text, "TOP");
		}
		else if(infantry.mode == I_HOLE)
		{
			strcpy(dynamic_ui_info[MODE_CHAR].ui_config.text, "HOLE");
		}
		
		Enqueue_Ui_For_Sending(&dynamic_ui_info[MODE_CHAR]);

	}
	
	last_mode = infantry.mode;
	
	
	//发射机构状态更新
	static uint8_t last_r_fric_state = 0;
	static uint8_t last_l_fric_state = 0;
	static uint8_t last_dial_state = 0;
	static uint8_t last_launch_state = 0;
	
	if(last_r_fric_state != launch.heart.r_fric_heart || last_launch_state != launch.state)
	{
		Motor_Color_Update(launch.heart.r_fric_heart, launch.state, R_FRIC_CIRCLE);
	}
	if(last_l_fric_state != launch.heart.l_fric_heart || last_launch_state != launch.state)
	{
		Motor_Color_Update(launch.heart.l_fric_heart, launch.state, L_FRIC_CIRCLE);
	}
	if(last_dial_state != launch.heart.dial_heart || last_launch_state != launch.state)
	{
		Motor_Color_Update(launch.heart.dial_heart, launch.state, DIAL_CIRCLE);
	}
  
	last_r_fric_state = launch.heart.r_fric_heart;
  last_l_fric_state = launch.heart.l_fric_heart;
  last_dial_state = launch.heart.dial_heart ;
  last_launch_state = launch.state;


  //底盘轮组状态更新
	static uint8_t last_lf_state = 0;
	static uint8_t last_lb_state = 0;
	static uint8_t last_rf_state = 0;
	static uint8_t last_rb_state = 0;
	
	static uint8_t  last_burst_flag = 0;
	
	if(last_lf_state != chassis.state.wheel_heart[WHEEL_LF] || last_burst_flag != chassis.burst_flag)
	{
		Motor_Color_Update(chassis.state.wheel_heart[WHEEL_LF], chassis.burst_flag, LF_CIRCLE);
	}
	if(last_lb_state != chassis.state.wheel_heart[WHEEL_LB]|| last_burst_flag != chassis.burst_flag)
	{
		Motor_Color_Update(chassis.state.wheel_heart[WHEEL_LB], chassis.burst_flag, LB_CIRCLE);
	}
	if(last_rf_state != chassis.state.wheel_heart[WHEEL_RF]|| last_burst_flag != chassis.burst_flag)
	{
		 Motor_Color_Update(chassis.state.wheel_heart[WHEEL_RF], chassis.burst_flag, RF_CIRCLE);
	}
	if(last_rb_state != chassis.state.wheel_heart[WHEEL_RB]|| last_burst_flag != chassis.burst_flag)
	{
		 Motor_Color_Update(chassis.state.wheel_heart[WHEEL_RB], chassis.burst_flag, RB_CIRCLE);
	}
	
	last_lf_state = chassis.state.wheel_heart[WHEEL_LF];
	last_lb_state = chassis.state.wheel_heart[WHEEL_LB];
  last_rf_state = chassis.state.wheel_heart[WHEEL_RF];
	last_rb_state = chassis.state.wheel_heart[WHEEL_RB];
	
	last_burst_flag = chassis.burst_flag;
	
	
	//云台电机状态更新
	static uint8_t last_yaw_state = 0;
	static uint8_t last_pitch_state = 0;
	static uint8_t last_left_state = 0;
	
	if(last_yaw_state != gimbal.state.yaw_heart)
	{
		Motor_Color_Update(gimbal.state.yaw_heart, 0, YAW_CIRCLE);
	}
	if(last_pitch_state != gimbal.state.pitch_heart)
	{
		Motor_Color_Update(gimbal.state.pitch_heart, 0, PITCH_CIRCLE);
	}
	if(last_left_state != gimbal.state.left_heart)
	{
		Motor_Color_Update(gimbal.state.left_heart, 0, LEFT_CIRCLE);
	}
	
	last_yaw_state = gimbal.state.yaw_heart;
  last_pitch_state = gimbal.state.pitch_heart;
	last_left_state = gimbal.state.left_heart;
	
	
	//云台状态更新
	Gimbal_Line_Update(board.rx_meg->gimbal_meg.pitch_mec,board.rx_meg->state_meg.is_down);
	
	//剩余发弹量更新
	static int16_t bullet_remain = 0;
	
	if(bullet_remain != judge.pkt->projectile_allowance_17mm)
	{
		dynamic_ui_info[BULLET_NUM].ui_config.int_num = (int16_t)judge.pkt->projectile_allowance_17mm;
		Enqueue_Ui_For_Sending(&dynamic_ui_info[BULLET_NUM]);
	}
	
	bullet_remain = judge.pkt->projectile_allowance_17mm;
	
	
	//视觉数字更新
	static uint8_t vision_last_mode = 0;
	
	if(vision_last_mode != vision.mode)
	{
		if(vision.mode == 1)
		{
			dynamic_ui_info[VISION_NUM].ui_config.int_num = 1;
		}
		else if(vision.mode == 2)
		{
			dynamic_ui_info[VISION_NUM].ui_config.int_num = 2;
		}
		else if(vision.mode == 3)
		{
			dynamic_ui_info[VISION_NUM].ui_config.int_num = 3;
		}
		else if(vision.mode == 4)
		{
			dynamic_ui_info[VISION_NUM].ui_config.int_num = 4;
		}
		else if(vision.mode == 5)
		{
			dynamic_ui_info[VISION_NUM].ui_config.int_num = 5;
		}
		else
		{
			dynamic_ui_info[VISION_NUM].ui_config.int_num = 0;
		}
		
		Enqueue_Ui_For_Sending(&dynamic_ui_info[VISION_NUM]);
	}
	vision_last_mode = vision.mode;
	
	
	
	//视觉框更新
	static uint8_t vision_last_state = 0;
	
	if(vision_last_state != vision.info.vision_heart)
	{
		if(board.rx_meg->state_meg.vision_state == 1)
		{
			dynamic_ui_info[VISION_FRAME].ui_config.color = GREEN;
		}
		else
		{
			dynamic_ui_info[VISION_FRAME].ui_config.color = BLACK;
		}
		Enqueue_Ui_For_Sending(&dynamic_ui_info[VISION_FRAME]);
	}
	vision_last_state = vision.info.vision_heart;
	
	

	//超电条更新,24.f:500
	float cap_V = 0;
	static uint16_t cap_line = 0,cap_line_last = 0;
	static uint8_t last_cap_ability = 1;
	
	cap_line = (uint16_t)(((cap.info->cap_Ucr * cap.info->cap_Ucr) / (24.f * 24.f)) * 500);
	
	if(cap_line_last != cap_line || last_cap_ability != cap.info->ability)
	{
		dynamic_ui_info[CAP_LINE].ui_config.end_x = (Client_mid_position_x -250) + cap_line;
//		dynamic_ui_info[CAP_LINE].ui_config.end_y = Client_mid_position_y + 320;
		
		if(cap.info->ability == 0)
		{
			dynamic_ui_info[CAP_LINE].ui_config.color = FUCHSIA;
		}
		else{
	  	if(((cap.info->cap_Ucr * cap.info->cap_Ucr) / (24.f * 24.f)) <= 0.14)
		  {
			  dynamic_ui_info[CAP_LINE].ui_config.color = FUCHSIA;
		  }			
		  else if(((cap.info->cap_Ucr * cap.info->cap_Ucr) / (24.f * 24.f)) <= 0.39)
		  {
			  dynamic_ui_info[CAP_LINE].ui_config.color = ORANGE;
		  }
		  else
		  {
			  dynamic_ui_info[CAP_LINE].ui_config.color = GREEN;
		  }
		
		}
		
		Enqueue_Ui_For_Sending(&dynamic_ui_info[CAP_LINE]);
	}
	cap_line_last = cap_line;
	last_cap_ability = cap.info->ability;
	
	
	//底盘方位角更新
	static float chas_angle_err_last = 0.f,test_chas_angle = 0.f;
	
	test_chas_angle = gimbal.info.yaw_mec - YAW_MEC_ZERO_ANGLE;
	if(fabs(test_chas_angle) > PI)
	{
		test_chas_angle -= sgn(test_chas_angle) * 2 * PI;
	}
	
	if(chas_angle_err_last != test_chas_angle)
	{
		My_Chas_Circle_Update(test_chas_angle);
		
	}
	
	//自瞄框更新
	static uint8_t last_vision_state = 0,now_vision_state = 0;
	static uint8_t last_vision_heart = 0;
	
	if((vision.mode == 1 || vision.mode == 5) && vision.info.is_find_target == 1)
	{
		now_vision_state = 1;
	}
	else if(vision.mode == 4 && vision.info.is_find_target == 1)
	{
		now_vision_state = 2;
	}
	else if((vision.mode == 2 || vision.mode == 3) && vision.info.is_find_target == 1)
	{
		now_vision_state = 3;
	}
	else
	{
		now_vision_state = 0;
	}
	
	if(last_vision_state != now_vision_state || last_vision_heart != vision.info.vision_heart)
	{
		if(vision.info.vision_heart == 0)
		{
			dynamic_ui_info[AUTO_CATCH_FRAME].ui_config.color = BLACK;
		}
		else if(now_vision_state == 1)
		{
			dynamic_ui_info[AUTO_CATCH_FRAME].ui_config.color = GREEN;
		}
		else if(now_vision_state == 2)
		{
			dynamic_ui_info[AUTO_CATCH_FRAME].ui_config.color = ORANGE;
		}
		else if(now_vision_state == 3)
		{
			dynamic_ui_info[AUTO_CATCH_FRAME].ui_config.color = PINK;
		}
		else if(now_vision_state == 0)
		{
			dynamic_ui_info[AUTO_CATCH_FRAME].ui_config.color = WHITE;
		}
		Enqueue_Ui_For_Sending(&dynamic_ui_info[AUTO_CATCH_FRAME]);
	}
	
	last_vision_state = now_vision_state;
	last_vision_heart = vision.info.vision_heart;
	
	
	//车体速度更新
	static float speed_last = 0.f,speed_now = 0.f;
	
//	speed_now = -board.tx_pkt->car_pkt.v_x;
	
	if(infantry.flag.turn_flag == true)
	{
		speed_now = chassis.measure.cycle_speed;
	}
	else if(abs(chassis.measure.front_speed) >= abs(chassis.measure.left_speed))
	{
		speed_now = chassis.measure.front_speed;
	}
	else{
	  speed_now = chassis.measure.left_speed;
	}
	if(speed_last != speed_now)
	{
		dynamic_ui_info[CAR_SPEED].ui_config.float_num = speed_now;
		Enqueue_Ui_For_Sending(&dynamic_ui_info[CAR_SPEED]);
	}
	speed_last = speed_now;
	
	
	
	if(judge.pkt->robot_id <= 10)
	{
		dynamic_ui_info[ENEMY_COIN_NUM].ui_config.start_x = Client_mid_position_x + 50;
		
		dynamic_ui_info[ENEMY_HERO_AMMO_NUM].ui_config.start_x = ENEMY_BLUE_HERO_AMMO_X;
		dynamic_ui_info[ENEMY_3_AMMO_NUM].ui_config.start_x = ENEMY_BLUE_HERO_AMMO_X + 2*ENEMY_AMMO_DISTANE;
		dynamic_ui_info[ENEMY_4_AMMO_NUM].ui_config.start_x = ENEMY_BLUE_HERO_AMMO_X + 3*ENEMY_AMMO_DISTANE;
		dynamic_ui_info[ENEMY_7_AMMO_NUM].ui_config.start_x = ENEMY_BLUE_HERO_AMMO_X + 5*ENEMY_AMMO_DISTANE;
		
		dynamic_ui_info[ENEMY_HERO_HP_NUM].ui_config.start_x = ENEMY_BLUE_HERO_AMMO_X;
		dynamic_ui_info[ENEMY_3_HP_NUM].ui_config.start_x = ENEMY_BLUE_HERO_AMMO_X + 2*ENEMY_AMMO_DISTANE;
	  dynamic_ui_info[ENEMY_4_HP_NUM].ui_config.start_x = ENEMY_BLUE_HERO_AMMO_X + 3*ENEMY_AMMO_DISTANE;
		dynamic_ui_info[ENEMY_7_HP_NUM].ui_config.start_x = ENEMY_BLUE_HERO_AMMO_X + 5*ENEMY_AMMO_DISTANE;


	}
	else{
		dynamic_ui_info[ENEMY_COIN_NUM].ui_config.start_x = Client_mid_position_x - 170;
		
	  dynamic_ui_info[ENEMY_HERO_AMMO_NUM].ui_config.start_x = ENEMY_RED_HERO_AMMO_X;
		dynamic_ui_info[ENEMY_3_AMMO_NUM].ui_config.start_x = ENEMY_RED_HERO_AMMO_X - 2*ENEMY_AMMO_DISTANE;
		dynamic_ui_info[ENEMY_4_AMMO_NUM].ui_config.start_x = ENEMY_RED_HERO_AMMO_X - 3*ENEMY_AMMO_DISTANE;
		dynamic_ui_info[ENEMY_7_AMMO_NUM].ui_config.start_x = ENEMY_RED_HERO_AMMO_X - 5*ENEMY_AMMO_DISTANE;

		
		dynamic_ui_info[ENEMY_HERO_HP_NUM].ui_config.start_x = ENEMY_RED_HERO_AMMO_X;
		dynamic_ui_info[ENEMY_3_HP_NUM].ui_config.start_x = ENEMY_RED_HERO_AMMO_X - 2*ENEMY_AMMO_DISTANE;
	  dynamic_ui_info[ENEMY_4_HP_NUM].ui_config.start_x = ENEMY_RED_HERO_AMMO_X - 3*ENEMY_AMMO_DISTANE;
		dynamic_ui_info[ENEMY_7_HP_NUM].ui_config.start_x = ENEMY_RED_HERO_AMMO_X - 5*ENEMY_AMMO_DISTANE;

		
	}
	
	
	static uint32_t last_enemy_coin_timestamp = 0;
	if(last_enemy_coin_timestamp != judge.info->radar_information_status.radar_enemy_team_status.update_timestamp)
	{
		dynamic_ui_info[ENEMY_COIN_NUM].ui_config.int_num = judge.info->radar_information_status.radar_enemy_team_status.remaining_gold_coin;
		Enqueue_Ui_For_Sending(&dynamic_ui_info[ENEMY_COIN_NUM]);
		
	}
	last_enemy_coin_timestamp = judge.info->radar_information_status.radar_enemy_team_status.update_timestamp;
	
	static uint32_t last_enemy_ammo_timestamp = 0;
	if(last_enemy_ammo_timestamp != judge.info->radar_information_status.radar_enemy_ammo.update_timestamp)
	{
		dynamic_ui_info[ENEMY_HERO_AMMO_NUM].ui_config.int_num = judge.info->radar_information_status.radar_enemy_ammo.enemy_hero_ammo;
		Enqueue_Ui_For_Sending(&dynamic_ui_info[ENEMY_HERO_AMMO_NUM]);
		
		dynamic_ui_info[ENEMY_3_AMMO_NUM].ui_config.int_num = judge.info->radar_information_status.radar_enemy_ammo.enemy_infantry_3_ammo;
		Enqueue_Ui_For_Sending(&dynamic_ui_info[ENEMY_3_AMMO_NUM]);
		
		dynamic_ui_info[ENEMY_4_AMMO_NUM].ui_config.int_num = judge.info->radar_information_status.radar_enemy_ammo.enemy_infantry_4_ammo;
		Enqueue_Ui_For_Sending(&dynamic_ui_info[ENEMY_4_AMMO_NUM]);
		
		dynamic_ui_info[ENEMY_7_AMMO_NUM].ui_config.int_num = judge.info->radar_information_status.radar_enemy_ammo.enemy_sentry_ammo;
		Enqueue_Ui_For_Sending(&dynamic_ui_info[ENEMY_7_AMMO_NUM]);
		
	}
  last_enemy_ammo_timestamp = judge.info->radar_information_status.radar_enemy_ammo.update_timestamp;

	
	static uint32_t last_enemy_hp_timestamp = 0;
	static uint32_t last_enemy_status_timestamp = 0;
	
	if(last_enemy_hp_timestamp != judge.info->radar_information_status.radar_enemy_HP.update_timestamp || last_enemy_status_timestamp != judge.info->radar_information_status.radar_enemy_robot_status.update_timestamp) 
	{
		dynamic_ui_info[ENEMY_HERO_HP_NUM].ui_config.int_num = judge.info->radar_information_status.radar_enemy_HP.enemy_hero_HP;
		Robot_Status_Update(judge.info->radar_information_status.radar_enemy_robot_status.hero_status,ENEMY_HERO_HP_NUM);
		
		dynamic_ui_info[ENEMY_3_HP_NUM].ui_config.int_num = judge.info->radar_information_status.radar_enemy_HP.enemy_infantry_3_HP;
		Robot_Status_Update(judge.info->radar_information_status.radar_enemy_robot_status.infantry_3_status,ENEMY_3_HP_NUM);
		
		dynamic_ui_info[ENEMY_4_HP_NUM].ui_config.int_num = judge.info->radar_information_status.radar_enemy_HP.enemy_infantry_4_HP;
		Robot_Status_Update(judge.info->radar_information_status.radar_enemy_robot_status.infantry_4_status,ENEMY_4_HP_NUM);
		
		dynamic_ui_info[ENEMY_7_HP_NUM].ui_config.int_num = judge.info->radar_information_status.radar_enemy_HP.enemy_sentry_HP;
		Robot_Status_Update(judge.info->radar_information_status.radar_enemy_robot_status.sentry_status,ENEMY_7_HP_NUM);
		
		
		
	}
	last_enemy_hp_timestamp = judge.info->radar_information_status.radar_enemy_HP.update_timestamp;
	last_enemy_status_timestamp = judge.info->radar_information_status.radar_enemy_robot_status.update_timestamp;
	
	
	static uint8_t last_charge = 0;
	if(last_charge != wireless_rx_info.is_charging)
	{
		if(wireless_rx_info.is_charging == 1)
		{
			strcpy(dynamic_ui_info[CHARGE_CHAR].ui_config.text, "CHARGING");
		}
		else{
		  strcpy(dynamic_ui_info[CHARGE_CHAR].ui_config.text, "CHARGED");
		}
		
		Enqueue_Ui_For_Sending(&dynamic_ui_info[CHARGE_CHAR]);
	}
	last_charge = wireless_rx_info.is_charging;
}


/**
 * @brief 把某点绕某点旋转一定角度
 * 
 * @param x 存储旋转后x的地址
 * @param y 存储旋转后y的地址
 * @param raw_x 旋转前x的值
 * @param raw_y 旋转后y的值
 * @param mid_x 旋转原点x
 * @param mid_y 旋转原点y
 * @param angle 旋转的角度rad
 */
void rotate_point(__packed uint16_t *x, __packed uint16_t *y, uint16_t raw_x, uint16_t raw_y, float mid_x, float mid_y, float angle) 
{
  float s = sin(angle);
  float c = cos(angle);
 // 平移到原点
  float origin_x = raw_x - mid_x;
  float origin_y = raw_y - mid_y;
  // 旋转
  float new_x = origin_x * c - origin_y * s;
  float new_y = origin_x * s + origin_y * c;
  // 平移回去并更新原始坐标
  *x = new_x + mid_x;
  *y = new_y + mid_y;
}

/*底盘方位角更新*/
void My_Chas_Circle_Update(float angle)
{
	rotate_point(&dynamic_ui_info[CHAS_HEAD_LINE].ui_config.end_x,&dynamic_ui_info[CHAS_HEAD_LINE].ui_config.end_y,
	              CHAS_CIRCLE_X,CHAS_CIRCLE_Y + CHAS_CIRCLE_R,
	              CHAS_CIRCLE_X,CHAS_CIRCLE_Y,
	              angle);
	
	rotate_point(&dynamic_ui_info[CHAS_SIDE_LINE].ui_config.start_x,&dynamic_ui_info[CHAS_SIDE_LINE].ui_config.start_y,
	              CHAS_CIRCLE_X - CHAS_CIRCLE_R,CHAS_CIRCLE_Y,
	              CHAS_CIRCLE_X,CHAS_CIRCLE_Y,
	              angle);
	
	rotate_point(&dynamic_ui_info[CHAS_SIDE_LINE].ui_config.end_x,&dynamic_ui_info[CHAS_SIDE_LINE].ui_config.end_y,
	              CHAS_CIRCLE_X + CHAS_CIRCLE_R,CHAS_CIRCLE_Y,
	              CHAS_CIRCLE_X,CHAS_CIRCLE_Y,
	              angle);
	
	Enqueue_Ui_For_Sending(&dynamic_ui_info[CHAS_HEAD_LINE]);
	Enqueue_Ui_For_Sending(&dynamic_ui_info[CHAS_SIDE_LINE]);
}


//发射机构状态更新
static void Motor_Color_Update(uint8_t motor_state, uint8_t special_state, uint32_t index)
{
	if(motor_state == 0)       //电机不在线
  {
    dynamic_ui_info[index].ui_config.color = BLACK;
  }
  else                 
  {
    if(special_state == 1)   
    {
      dynamic_ui_info[index].ui_config.color = FUCHSIA;
    }
    else
    {
      dynamic_ui_info[index].ui_config.color = GREEN;
    }
  }
		
  Enqueue_Ui_For_Sending(&dynamic_ui_info[index]);
	
}



static void Gimbal_Line_Update(float angle,uint8_t height)
{
	static float last_angle = 0;
	static uint8_t last_height = 2;
	
	if(last_angle != angle || last_height != height)
	{
		dynamic_ui_info[PITCH_CIRCLE].ui_config.start_y = BODY_CENTER_Y +40 + height * 40;
		dynamic_ui_info[PITCH_LINE].ui_config.start_y = BODY_CENTER_Y + 40 + height * 40;
		dynamic_ui_info[PITCH_LINE].ui_config.end_y = BODY_CENTER_Y + 40 + height * 40;
		
		
		rotate_point(&dynamic_ui_info[PITCH_LINE].ui_config.end_x,&dynamic_ui_info[PITCH_LINE].ui_config.end_y,
	              BODY_CENTER_X + PITCH_LENGTH, BODY_CENTER_Y + 40 + height * 40,
	              BODY_CENTER_X, BODY_CENTER_Y + 40 + height * 40,   
	              angle);
		
		
		Enqueue_Ui_For_Sending(&dynamic_ui_info[PITCH_CIRCLE]);
		Enqueue_Ui_For_Sending(&dynamic_ui_info[PITCH_LINE]);
	}
	
	last_angle = angle;
	last_height = height;
}


static void Robot_Status_Update(uint8_t robot_status,uint32_t index)
{
	if(robot_status == 1)
	{
		dynamic_ui_info[index].ui_config.color = BLACK;
	}
	else if(robot_status == 2 || robot_status == 3)
	{
	  dynamic_ui_info[index].ui_config.color = GREEN;
	}
	else{
    dynamic_ui_info[index].ui_config.color = WHITE;

	}
	
	 Enqueue_Ui_For_Sending(&dynamic_ui_info[index]);

}


static void Radar_Enemy_Status_Update(int16_t coin,int16_t* robot_ammo,uint8_t* robot_status)
{
	
}
