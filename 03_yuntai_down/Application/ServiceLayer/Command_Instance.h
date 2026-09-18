#ifndef __COMMAND_Instance_H
#define __COMMAND_Instance_H

#include "command.h"
#include "Balance.h"

enum 
{
   JUMP,
   KNEE_STRIKE,
	 U_TURN,
	 R_TURN45,
	 L_TURN45,
	 FLY,
	 RESERVE_FLY,
	 LOB,
	
  COMMAND_LIST,
};

void Cmd_Init(void);
void Cmd_Heartbeat(void);
void Command_Update(void);
extern command_t command[COMMAND_LIST];
#endif
