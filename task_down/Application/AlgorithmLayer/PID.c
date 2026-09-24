/* PID.c - 鍗曠幆 PID */

#include "pid.h"
#include "rp_math.h"
/* 鍗曠幆 PID 璁＄畻 */
void single_pid_ctrl(pid_ctrl_t *pid)
{

	//pid->err = pid->target-pid->measure;
	  pid->integral += pid->err; /* 累加误差 */
    pid->integral = constrain(pid->integral, -pid->integral_max, +pid->integral_max); /* 积分限幅 */

    pid->pout = pid->kp * pid->err; /* 比例项 */
    pid->iout = pid->ki * pid->integral; /* 积分项 */
	  pid->dout = pid->kd * (pid->err - pid->last_err); /* 微分项 */
	  pid->last_dout=pid->dout;

    pid->out = pid->pout + pid->iout + pid->dout; /* 总输出 */
    pid->out = constrain(pid->out, -pid->out_max, pid->out_max); /* 输出限幅 */

    pid->last_err = pid->err; /* 保存微分历史 */
}


/* all_pid_calc */
float  all_pid_calc (pid_ctrl_t *out,pid_ctrl_t *inn,float target,float mea_out,float mea_in,float inner_kp,uint8_t err_cal_mode)
{
	if(inn == NULL)return 0;
	
	 else if(out == NULL&&inn!=NULL)  // 杈撳嚭
	{
		inn->target=target;
		inn->measure=mea_in;
		inn->err=inn->target-mea_in;
		single_pid_ctrl(inn);
		return inn->out;
	}
	
	else if(out != NULL&&inn!=NULL)
	{
		
		out->target=target;
		out->measure=mea_out;
		out->err=out->target-out->measure;  // 璇樊
		switch(err_cal_mode)
		{
			
			case 0:			
				break;
			
			case 1:
				out->err = motor_half_cycle(out->err, 8191);
				break;		
			
			case 2:
				out->err = motor_half_cycle(out->err, 8191);
				out->err = motor_half_cycle(out->err, 4095);
				break;
			
			case 3:
				out->err = motor_half_cycle(out->err, 360);
				break;
			
			case 4:
				out->err = motor_half_cycle(out->err, 65535);
				break;
			
			case 5:
				out->err = motor_half_cycle(out->err, 191);
				break;
			default:
				break;
		}
		
		single_pid_ctrl(out);  // 杈撳嚭
		inn->target=out->out;  // 鐩爣
		inn->measure=mea_in*inner_kp;
		inn->err=inn->target+inn->measure;  // 璇樊
		single_pid_ctrl(inn);
		return inn->out;  // 杈撳嚭
	}
	else
	{
		return 0;
	}
}


/* 鍓嶉+PID 璁＄畻 */
float feedforward_pid_calc(float K_ff,pid_ctrl_t *out,pid_ctrl_t *inn,float target,float mea_out,float mea_in,float inner_kp,uint8_t err_cal_mode)
{
	float target_now;
	static float target_last;
	target_now=out->target;
	float output= (target_now-target_last)*K_ff +all_pid_calc (out,inn,target,mea_out,mea_in,inner_kp,err_cal_mode);
	target_last=target_now;
	return output;
}






