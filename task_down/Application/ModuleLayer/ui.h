#ifndef __MY_UI_H
#define __MY_UI_H

#include "stm32h7xx_hal.h"

typedef enum{
	MODE_CHAR,
	VISION_FRAME,//????buff
	VISION_NUM,//buff???
	BULLET_NUM,
	CHAS_HEAD_LINE,//?????
	CHAS_SIDE_LINE,//??????
	CAP_LINE,//??????
	AUTO_CATCH_FRAME,//?????
	CAR_SPEED,//???????
	R_FRIC_CIRCLE,
	L_FRIC_CIRCLE,
	DIAL_CIRCLE,
	LF_CIRCLE,
	LB_CIRCLE,
	RF_CIRCLE,
	RB_CIRCLE,
	YAW_CIRCLE,
	PITCH_CIRCLE,
	LEFT_CIRCLE,
	PITCH_LINE,
	
	ENEMY_COIN_NUM,
	ENEMY_HERO_AMMO_NUM,
	ENEMY_HERO_HP_NUM,
	ENEMY_3_AMMO_NUM,
	ENEMY_3_HP_NUM,
	ENEMY_4_AMMO_NUM,
	ENEMY_4_HP_NUM,
	ENEMY_7_AMMO_NUM,
	ENEMY_7_HP_NUM,
	

	CHARGE_CHAR,
	
	DYNAMIC_NUM,
}dynamic_ui_cnt_e;

typedef enum{
	VISION_CHAR,
	CHAS_CIRCLE,//???????
	CAP_FRAME,//?????
	MOVE_L_LINE,//???§Ô???
	MOVE_R_LINE,//???§Ô???
	CAP_DIVISION_1,//????????
	CAP_DIVISION_2,
	LEFT_LINE,
  BODY_LINE,
	
	CONST_NUM,
}const_ui_cnt_e;


void My_Ui_Init(void);
void Ui_Info_Update(void);
#endif
