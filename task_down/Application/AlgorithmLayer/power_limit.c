/* power_limit.c - 功率限制 */

#include "Power_Limit.h"
#include "judge.h"



//void Chassis_Motor_Power_Limit(int16_t *data)
//{
//	float buffer = judge.pkt->buffer_energy;
//	float heat_rate, Limit_k, CHAS_LimitOutput, CHAS_TotalOutput;
//	
//	float OUT_MAX = 0.f;
//	
//	OUT_MAX = CHAS_SP_MAX_OUT * 4.f;
//	

//	
//	Limit_k = buffer / 60.f;
//	
//	if(buffer < 25.f)

//	else

//	
//	if(buffer < 60.f)
//		CHAS_LimitOutput = Limit_k * OUT_MAX;
//	else 
//		CHAS_LimitOutput = OUT_MAX;    
//	
//	CHAS_TotalOutput = abs(data[0]) + abs(data[1]) + abs(data[2]) + abs(data[3]) ;
//	
//	heat_rate = CHAS_LimitOutput / CHAS_TotalOutput;
//	
//  if(CHAS_TotalOutput >= CHAS_LimitOutput)
//  {
//		for(char i = 0 ; i < 4 ; i++)
//		{	
//			data[i] = (int16_t)(data[i] * heat_rate);	
//		}
//	}
//}



//static void Chassis_Power_Limit(Chassis_t * chassis)
//{
//	  static float last_buffer = 0;
//		float limit_output_speed[4];
//	
//		float buffer = (float)judge.pkt->buffer_energy;




//		

//		for(uint8_t i = 0;i<4;i++)
//		{
//			limit_output_speed[i] = chassis->wheel->motor[i]->rx_info->speed;
//		}
//		
//		float OUT_MAX = 0;
//	

//		
//		if(buffer > 60.f)
//		{

//		}
//		

//		
//		if(buffer < 25.f)
//		{

//		}
//		else
//		{

//		}
//			
//		if(buffer < 60.f)
//		{

//		}
//		else 
//		{

//		}
//			
//		CHAS_TotalOutput = fabs(limit_output_speed[0]) + fabs(limit_output_speed[1]) + fabs(limit_output_speed[2]) + fabs(limit_output_speed[3]) ;
//		
//		if(CHAS_TotalOutput >= CHAS_LimitOutput)
//		{

//		}
//		else{
//		  heat_rate = 1.f;
//		}
//		
//		for(uint8_t i = 0 ; i < 4 ; i++) 
//		{	
//			chassis->out.wheel_powerd_out[i] = (float)(chassis->out.wheel_initial_out[i] * heat_rate);	
//		}
//		
//		if(buffer <= 0 && last_buffer > 0)
//		{
//			power_fail ++;
//		}
//		
//		last_buffer = buffer;
//		
//}

