#ifndef __FLAG_HANDLE_H
#define __FLAG_HANDLE_H

/* Includes ------------------------------------------------------------------*/
#include "pid.h"
#include "Chassis.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/* Servo functions */
void Jump_Target_Process(Chassis_t* My_Chassis);
void Rescue_Check(void);
void Rescue_Process(void);
void Knee_Strike_React(Chassis_t* My_Chassis);
//void Knee_Strike_Check(Chassis_t* My_Chassis);
void Knee_Strike_Target_Leg(Chassis_t* My_Chassis);
#endif
