#ifndef __VISION_H
#define __VISION_H

#include "stdint.h"

typedef enum{
	V_NORMAL,
	V_AUTO,
	V_S_BUFF,
	V_B_BUFF,
  V_OUTPOST,
	V_HERO,
	VISION_CNT,

}Vision_Mode_e;

typedef struct{
	float  vision_yaw_tar;
	float  vision_pitch_tar;
	
	uint8_t  is_find_target;

	uint8_t  vision_heart;

}Vision_Info_t;


typedef struct Vision_Struct_t{
	Vision_Mode_e         mode;
	Vision_Info_t         info;
	
	void (*init)(struct Vision_Struct_t* vision);
	void (*work)(struct Vision_Struct_t* vision);
	void (*heart_beat)(struct Vision_Struct_t* vision);
	
}Vision_t;


extern Vision_t vision;



#endif 
