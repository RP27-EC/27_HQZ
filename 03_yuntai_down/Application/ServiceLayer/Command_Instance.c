
#include "Command_Instance.h"
#include "Board_protocol.h"
#include "Balance.h"

command_t command[COMMAND_LIST] = 
{
  [JUMP] = {
    .cmd_type = RISE_TRIGER_C,
    .run_time_max = OUT_TIME_OFF,  
	.init = Cmd_Class_Init,
	},
  [KNEE_STRIKE] = {
    .cmd_type = RISE_TRIGER_C,
    .run_time_max = OUT_TIME_OFF,  
	.init = Cmd_Class_Init,
	},
	
	[U_TURN] = {
    .cmd_type = RISE_TRIGER_C,
    .run_time_max = OUT_TIME_OFF,  
	.init = Cmd_Class_Init,
	},
	[R_TURN45] = {
    .cmd_type = RISE_TRIGER_C,
    .run_time_max = OUT_TIME_OFF,  
	.init = Cmd_Class_Init,
	},
	
	[L_TURN45] = {
    .cmd_type = RISE_TRIGER_C,
    .run_time_max = OUT_TIME_OFF,  
	.init = Cmd_Class_Init,
	},
	
	[FLY] = {
    .cmd_type = RISE_TRIGER_C,
    .run_time_max = OUT_TIME_OFF,  
	.init = Cmd_Class_Init,
	},
	[RESERVE_FLY] = {
    .cmd_type = RISE_TRIGER_C,
    .run_time_max = OUT_TIME_OFF,  
	.init = Cmd_Class_Init,
	},
  [LOB] = {
    .cmd_type = RISE_TRIGER_C,
    .run_time_max = OUT_TIME_OFF,  
	.init = Cmd_Class_Init,
	},
	

};

/**
 * @brief 命令初始化，调用一次
 */
void Cmd_Init(void)
{
	for(uint8_t i = 0; i < COMMAND_LIST; i++)
	{
		command[i].init(&command[i]);
	}
}
/**
 * @brief 命令心跳 循环调用
 */
void Cmd_Heartbeat(void)
{
	for(uint8_t i = 0; i < COMMAND_LIST; i++)
	{
		command[i].heartbeat(&command[i]);
	}
}
/**
 * @brief 命令更新 循环调用
 */
void Command_Update(void)
{
	static uint32_t RC_ONLINE_TICK;
	rc_sensor_info_t*  rc_info=Balance.rc->sensor->info;
	
	if(Balance.rc->sensor->work_state==DEV_ONLINE)
	{
		RC_ONLINE_TICK++;
	}
	else
	{
		RC_ONLINE_TICK=0;
	}
	static uint8_t last_rc_info_s1;
	static uint8_t last_rc_info_s2;
	static uint8_t last_rc_info_wheel[4];
	static uint8_t fly_step;
	
	
	if(RC_ONLINE_TICK>=200)//屏蔽开控命令
	{
		/*命令更新填这里*/
		if(Balance.ctrl == RC_CTRL)
		{
//			command[JUMP].update(&command[JUMP],rc_info->s1 == RC_SW_UP && rc_info->s2 ==  RC_SW_UP 
//		                       && ((rc_info->thumbwheel.step[0] != last_rc_info_wheel[0])||(rc_info->thumbwheel.step[1] != last_rc_info_wheel[1])));
//		
//		  command[KNEE_STRIKE].update(&command[KNEE_STRIKE],rc_info->s1 == RC_SW_UP && rc_info->s2 ==  RC_SW_UP 
//		                              && ((rc_info->thumbwheel.step[2] != last_rc_info_wheel[2])||(rc_info->thumbwheel.step[3] != last_rc_info_wheel[3])));
//		
//			command[U_TURN].update(&command[U_TURN],rc_info->s1 == RC_SW_DOWN && rc_info->s2 ==  RC_SW_UP 
//		                              && ((rc_info->thumbwheel.step[0] != last_rc_info_wheel[0])||(rc_info->thumbwheel.step[1] != last_rc_info_wheel[1])));
			
			
			command[JUMP].update(&command[JUMP],rc_info->s1 == RC_SW_DOWN && rc_info->s2 ==  RC_SW_UP 
		                       && ((rc_info->thumbwheel.step[0] != last_rc_info_wheel[0])||(rc_info->thumbwheel.step[1] != last_rc_info_wheel[1])));
		
		  command[KNEE_STRIKE].update(&command[KNEE_STRIKE],rc_info->s1 == RC_SW_DOWN && rc_info->s2 ==  RC_SW_UP 
		                              && ((rc_info->thumbwheel.step[2] != last_rc_info_wheel[2])||(rc_info->thumbwheel.step[3] != last_rc_info_wheel[3])));
		
			command[U_TURN].update(&command[U_TURN],rc_info->s1 == RC_SW_DOWN && rc_info->s2 ==  RC_SW_MID 
		                              && ((rc_info->thumbwheel.step[0] != last_rc_info_wheel[0])||(rc_info->thumbwheel.step[1] != last_rc_info_wheel[1])));
			
//		  command[FLY].update(&command[FLY],rc_info->s1 == RC_SW_DOWN && rc_info->s2 ==  RC_SW_MID 
//		                              && ((rc_info->thumbwheel.step[0] != last_rc_info_wheel[0])||(rc_info->thumbwheel.step[1] != last_rc_info_wheel[1])));
//		
//		  command[RESERVE_FLY].update(&command[RESERVE_FLY],rc_info->s1 == RC_SW_DOWN && rc_info->s2 ==  RC_SW_MID 
//		                              && ((rc_info->thumbwheel.step[2] != last_rc_info_wheel[2])||(rc_info->thumbwheel.step[3] != last_rc_info_wheel[3])));
		}
		
		else if(Balance.ctrl == KEY_CTRL)
		{
			command[JUMP].update(&command[JUMP],rc_info->Q.status == release_to_press);
		
		  command[KNEE_STRIKE].update(&command[KNEE_STRIKE],rc_info->C.status == release_to_press);
			
			command[U_TURN].update(&command[U_TURN],rc_info->R.status == release_to_press );//&& (D_Board_Tx_Pkt.vision_mode == 0 || (D_Board_Tx_Pkt.vision_mode != 0 && D_Board_Rx_Info.vision_state == 0))
			
//			command[L_TURN45].update(&command[L_TURN45],rc_info->Q.status == release_to_press);
//			
//			command[R_TURN45].update(&command[R_TURN45],rc_info->E.status == release_to_press);
		
//			if(rc_info->F.status == release_to_press)
//			{
//				fly_step ++;
//			}
//			
		  command[FLY].update(&command[FLY], rc_info->G.status == release_to_press);
//		
//		  command[RESERVE_FLY].update(&command[RESERVE_FLY],rc_info->F.status == release_to_press && fly_step % 3 == 1);
//			
//			command[LOB].update(&command[LOB],(rc_info->Shift.status == release_to_press || rc_info->Shift.status == short_press || rc_info->Shift.status == long_press)
//		                                    && rc_info->Z.status == release_to_press);
			
		}

	}
	
	
	last_rc_info_s1 = rc_info->s1;
	last_rc_info_s2 = rc_info->s2;
	for(uint8_t i = 0; i < 4;i++)
	{
		last_rc_info_wheel[i] = rc_info->thumbwheel.step[i];
	}
	
	
}



