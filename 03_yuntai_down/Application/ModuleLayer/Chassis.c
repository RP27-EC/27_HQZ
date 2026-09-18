#include "chassis.h"
#include "gimbal.h"
#include "infantry.h"
#include "board_protocol.h"
#include "rc_sensor.h"
#include "judge.h"
#include "cap.h"
#include "config_chassis.h"
#include <stdint.h>
static void Chassis_Init(Chassis_t* chassis);
static void Chassis_Status_Update(Chassis_t* chassis);
static void Chassis_Target_Update(Chassis_t* chassis);
static void Chassis_Inverse_Calculate(Chassis_t* chassis);
static void Chassis_Positive_Calculate(Chassis_t* chassis);
static void Chassis_Offline_Update(Chassis_t* chassis);
static void Chassis_Offline_Process(Chassis_t* chassis);
static void Chassis_Feedforward_Calculate(Chassis_t* chassis);
static void Chassis_Pid_Calculate(Chassis_t* chassis);
static void Chassis_Power_Limit(Chassis_t * chassis);
static void New_Chassis_Power_Limit(Chassis_t *chassis);
static void Chassis_Cmd_Transmit(Chassis_t* chassis);
static void Chassis_Work(Chassis_t* chassis);
static int8_t random_step_calculate(uint8_t cmd, uint32_t seed);

Chassis_t  chassis = {
	.wheel = &wheel_group,
	.pid_mode = SPEED_MODE,
	.slip = {
		.slip_flag = false,
		.is_allot = true,
	  .wheel_speed_max_difference = 9000.f,
	  .slip_low_out = 0,
	},
	.key = {
	  .w_s_now = 0,
		.a_d_now = 0,
		
	  .w_s_last = 0,
	  .a_d_last = 0,
	},
	
	.power_coefficient = {
		{1.4268163740611692,0.0004488821106870444,8.492604098272384e-05,1.7818822187359053e-06,1.3769792187274362e-07,3.5482352783775733e-07},
			{1.316759451137222,-0.0003859926122313794,-0.0001505750547614686,1.561834267598007e-06,1.6344718643622346e-07,4.450747190731389e-07},
				{1.3732605217163856,-0.0005512718723215252,0.00016888118401289773,1.6267287913987835e-06,1.535613047156326e-07,3.9997112666134343e-07},
					{1.3732605217163856,-0.0005512718723215252,0.00016888118401289773,1.6267287913987835e-06,1.535613047156326e-07,3.9997112666134343e-07},
					//{1.2153953585472708,0.0005208459563644549,-0.0011565571647848352,1.2295486958182895e-06,1.5197943464022477e-07,8.38544692326543e-07},
								
	},
	
	.burst_flag = false,
	
	.init = Chassis_Init, 
};



static void Chassis_Init(Chassis_t* chassis)
{
	chassis->work = Chassis_Work;
	chassis->heart_beat = Chassis_Offline_Update;
}

/**
 * @brief 底盘模式状态更新
 * @note   狗洞要特殊处理
 */
static void Chassis_Status_Update(Chassis_t* chassis)
{
	switch (infantry.mode)
	{
	  case I_SLEEP:
			chassis->mode = C_SLEEP;
			break;
		
		case I_INIT:
			chassis->mode = C_INIT;
			break;
		
		case I_MEC:
			chassis->mode = C_BOSS;
			break;
		
		case I_HOLE:
		  if(infantry.flag.hole_flag == true && infantry.flag.chassis_reset.value == true)
			{
				chassis->mode = C_SLAVE;
			}
			else if(infantry.flag.hole_flag == true && infantry.flag.chassis_reset.value == false)
			{
				chassis->mode = C_BOSS;
			}
			else if(infantry.flag.hole_flag == false && board.rx_meg->state_meg.is_down == 2)
			{
				chassis->mode = C_SLAVE;
			}
			else if(infantry.flag.hole_flag == false && board.rx_meg->state_meg.is_down != 2)
			{
				chassis->mode = C_BOSS;
			}
			break;
			
		case I_IMU:
		case I_TURN:
			chassis->mode = C_SLAVE;
			break;
		
		default:
			break;
	
	}
	
	if(infantry.flag.chassis_off == true)           //底盘掉电就底盘睡眠，头能动
	{
		chassis->mode = C_SLEEP;
		
		infantry.flag.turn_flag = false;			//退出小陀螺
	}
	
}

/**
 * @brief  键鼠W,S,A,D输入
 * @note   未验证
 */
static void Chassis_Key_Input(Chassis_t* chassis)
{
	Chassis_Key_Info_t* key = &chassis->key;
	rc_sensor_info_t* rc = rc_sensor.info;

	
	if((rc->W.status == press_to_release && rc->S.status == release) || (rc->W.status == release && rc->S.status == press_to_release) || (rc->W.status == press_to_release && rc->S.status == press_to_release))
	{
		key->w_s_now = chassis->measure.front_speed / FRONT_MAX_SPEED * KEY_W_CNT_MAX;
	}
	else if(rc->W.status == release && rc->S.status == release)
	{
		key->w_s_now -=3;
		if(key->w_s_now <= 0)
		{
			key->w_s_now = 0;
		}
	}
	else{
	  key->w_s_now += sgn(rc->W.cnt - rc->S.cnt) * 3; 
	  key->w_s_now = step_limit_filter(key->w_s_now,key->w_s_last,5);
		key->w_s_now = constrain(key->w_s_now,-KEY_W_CNT_MAX,KEY_W_CNT_MAX);
	}
	
	
	if((rc->A.status == press_to_release && rc->D.status == release) || (rc->A.status == release && rc->D.status == press_to_release) || (rc->A.status == press_to_release && rc->D.status == press_to_release))
	{
		key->a_d_now = chassis->measure.left_speed / LEFT_MAX_SPEED * KEY_A_CNT_MAX;
	}
	else if(rc->A.status == release && rc->D.status == release)
	{
		key->a_d_now -=3;
		if(key->a_d_now <= 0)
		{
			key->a_d_now = 0;
		}
	}
	else{
	  key->a_d_now += sgn(rc->A.cnt - rc->D.cnt) * 3; 
	  key->a_d_now = step_limit_filter(key->a_d_now,key->a_d_last,5);
		key->a_d_now = constrain(key->a_d_now,-KEY_A_CNT_MAX,KEY_A_CNT_MAX);
	}

  key->w_s_last	= key->w_s_now;
	key->a_d_last = key->a_d_now;
	
}
float front_speed,left_speed,cycle_speed;

static void Chassis_Target_Update(Chassis_t* chassis)
{
	#ifdef test_cycle
	float yaw_angle_err_rad = -gimbal.info.yaw_mec_err_raw;
	#else
	float yaw_angle_err_rad = gimbal.info.yaw_mec_err_act;
	#endif
	 
	
	
	static float straight_yaw = 0;
	
	static bool last_turn = false;
	
	float last_front_cnt,last_left_cnt,now_front_cnt,now_left_cnt;
	
//	now_front_cnt = step_limit_filter(rc_sensor.info->W.cnt - rc_sensor.info->S.cnt, last_front_cnt, 400);
//	now_left_cnt = step_limit_filter(rc_sensor.info->A.cnt - rc_sensor.info->D.cnt, last_left_cnt, 400);
//	
	
	now_front_cnt = rc_sensor.info->W.cnt - rc_sensor.info->S.cnt;
	now_left_cnt = rc_sensor.info->A.cnt - rc_sensor.info->D.cnt;
	
 
	if(infantry.ctrl == RC_CTRL)
	{
		front_speed = -rc_sensor.info->ch3/660.f * FRONT_MAX_SPEED;
    left_speed = rc_sensor.info->ch2/660.f * LEFT_MAX_SPEED;
	  cycle_speed = rc_sensor.info->ch0/660.f * CYCLE_MAX_SPEED;
	
	}
	else{
	  front_speed = -1*(float)now_front_cnt/ KEY_W_CNT_MAX* FRONT_MAX_SPEED;
	  left_speed = -(float)now_left_cnt/ KEY_A_CNT_MAX* LEFT_MAX_SPEED;
	  cycle_speed = rc_sensor.info->mouse_vx;
		cycle_speed = constrain(cycle_speed,-CYCLE_MAX_SPEED,CYCLE_MAX_SPEED);
	}
	
	last_front_cnt = now_front_cnt;
	last_left_cnt = now_left_cnt;
	
	
	switch (chassis->mode)
	{
		//睡眠初始化底盘不动
	  case C_SLEEP:
		case C_INIT:
      chassis->target.front_speed = 0;
		  chassis->target.left_speed = 0;
	    chassis->target.cycle_speed = 0;
		
		  straight_yaw = imu_sensor.info->base_info.yaw;
	
	    break;
		
		case C_BOSS:	
		 if (abs(gimbal.info.yaw_mec_err_raw) > PI/2)   //掉头反着开
     {
       front_speed *= -1.f;
       left_speed *= -1.f;
     }
		
			chassis->target.front_speed = front_speed;
			chassis->target.left_speed = left_speed;
			chassis->target.cycle_speed = cycle_speed;
		//限制yaw的速度，防止追随不及时卡位
			if(board.rx_meg->state_meg.is_down != 2 && board.rx_meg->state_meg.is_down != 0)
			{
				chassis->target.cycle_speed = constrain(chassis->target.cycle_speed,-5.f,5.f);
			}
		
			//走偏矫正
//		  if(abs(chassis->target.front_speed) >=10 && abs(chassis->target.cycle_speed) <= 0.1)
//			{
//				chassis->target.cycle_speed = -1*motor_half_cycle(straight_yaw - imu_sensor.info->base_info.yaw,360.f);
//				chassis->target.cycle_speed = constrain(chassis->target.cycle_speed,-20.f,20.f);
//			}
//			else{
//				straight_yaw = imu_sensor.info->base_info.yaw;
//			}
		
	
			break;
		
		case C_SLAVE:
			if(infantry.flag.turn_flag == true)
			{
				#if TURN_MODE == 1
				  if(last_turn == false && infantry.flag.turn_flag == true)
	        {
		        start_time = HAL_GetTick();
	        }
			    chassis->target.cycle_speed = TURN_CYCLE_SPEED + 10.f/2 *(1 - arm_cos_f32(PI/1000*(HAL_GetTick() - start_time)));
				#elif TURN_MODE == 2
				  chassis->target.cycle_speed = TURN_CYCLE_SPEED + random_step_calculate(1,0); 
				#else
				  chassis->target.cycle_speed = -1*TURN_CYCLE_SPEED;
				  random_step_calculate(0, HAL_GetTick()); 
				#endif
	      	
				#if GIMBAL_SWITCH == 0
				  if (abs(yaw_angle_err_rad) > PI/2)   //掉头反着开
          {
            front_speed *= -1.f;
            left_speed *= -1.f;
          }
				#else
				#endif
				
			}		
			else{
				chassis->target.cycle_speed = -1*300.f*yaw_angle_err_rad * yaw_angle_err_rad*sgn(yaw_angle_err_rad);
				chassis->target.cycle_speed = constrain(chassis->target.cycle_speed,-CYCLE_MAX_SPEED,CYCLE_MAX_SPEED);
				
			}
			
      chassis->target.front_speed = front_speed * cos(gimbal.info.yaw_mec_err_raw) + left_speed * sin(gimbal.info.yaw_mec_err_raw);
		  chassis->target.left_speed = left_speed * cos(gimbal.info.yaw_mec_err_raw) - front_speed * sin(gimbal.info.yaw_mec_err_raw);
     
			straight_yaw = imu_sensor.info->base_info.yaw;
		
			break;
		
		
		default:
			break;
	}

	last_turn = infantry.flag.turn_flag;
	
}

/**
 * @brief  底盘运动学逆解算，车速算轮速
 * @note   速度是弧度，当前策略是尽量保旋转速度而削减平动速度
 */
static void Chassis_Inverse_Calculate(Chassis_t* chassis)
{
	float front = chassis->target.front_speed;
	float left = chassis->target.left_speed;
	float cycle = chassis->target.cycle_speed;

  float trans = abs(front)+abs(left);
  float rotate = abs(cycle);
  float total = trans + rotate;


  if(total > CHASSIS_MAX_SPEED)
  {
    float rotate_limit = CHASSIS_MAX_SPEED * 0.6f;

    if(rotate > rotate_limit)
    {
      cycle *= (float)rotate_limit / rotate;
    }

    float remain = CHASSIS_MAX_SPEED - abs(cycle);

    float k = (float)remain / trans;

    if(k < 1)
    {
      front *= k;
      left *= k;
    }
  }
	 
	chassis->target.motor_speed[WHEEL_LF]  = - front + left + cycle; 
	chassis->target.motor_speed[WHEEL_LB]  = - front - left + cycle;
	chassis->target.motor_speed[WHEEL_RB]  =   front - left + cycle; 
	chassis->target.motor_speed[WHEEL_RF]  =   front + left + cycle;
	
	chassis->target.front_speed = front;
	chassis->target.left_speed = left;
	chassis->target.cycle_speed =cycle;
	
	
}

/**
 * @brief  底盘运动学正解算，轮速算车速
 * @note   速度是弧度，便于求实际整车速度
 */
static void Chassis_Positive_Calculate(Chassis_t* chassis)
{
	float speed_rf = chassis->wheel->motor[WHEEL_RF]->rx_info->speed;
	float speed_rb = chassis->wheel->motor[WHEEL_RB]->rx_info->speed;
	float speed_lf = chassis->wheel->motor[WHEEL_LF]->rx_info->speed;
	float speed_lb = chassis->wheel->motor[WHEEL_LB]->rx_info->speed;
	
	float yaw_angle_err_rad = gimbal.info.yaw_mec_err_act;
	
	chassis->measure.front_speed = (- speed_lf - speed_lb + speed_rb + speed_rf)/4.f;
	chassis->measure.left_speed = (speed_lf - speed_lb - speed_rb + speed_rf)/4.f;
	chassis->measure.cycle_speed = (speed_lf + speed_lb + speed_rb + speed_rf)/4.f;
	
 //视觉所需车速度，x向前y向左
  board.tx_pkt->car_pkt.v_x = chassis->measure.front_speed * cos(yaw_angle_err_rad) 
            + chassis->measure.left_speed  * sin(yaw_angle_err_rad);

  board.tx_pkt->car_pkt.v_y  = chassis->measure.left_speed  * cos(yaw_angle_err_rad) 
            - chassis->measure.front_speed * sin(yaw_angle_err_rad);
	
}


/**
 * @brief  底盘掉线失联检查
 * @note   四个轮子都掉电才算失联
 */
static void Chassis_Offline_Update(Chassis_t* chassis)
{
	uint8_t offline_cnt = 0;
	static uint8_t offline_id[WHEEL_CNT] = {0,0,0,0};
	
	for(uint8_t i = 0;i<WHEEL_CNT;i++)
	{
		if(chassis->wheel->motor[i]->state->status == DEV_OFFLINE)
		{
			offline_id[i] = 1;
		}
		else{
		  offline_id[i] = 0;
		}
		
		offline_cnt += offline_id[i];
		
		chassis->state.wheel_heart[i] = 1 - offline_id[i];
	}
	
  if(offline_cnt == 4)
	{
		infantry.flag.chassis_off = true;
	}
	else{
	  infantry.flag.chassis_off = false;
	}

}


/**
 * @brief  底盘失联处理
 * @note   全部睡觉
 */
static void Chassis_Offline_Process(Chassis_t* chassis)
{
	for(uint8_t i = 0;i< WHEEL_CNT;i++)
	{
		chassis->out.wheel_end_out[i] = 0;
	}
	
}

/**
 * @brief  底盘前馈计算
 * @note   主要是斜坡的重力前馈，不想加，除非做到抱起来不转，且需要加死区
 */
static void Chassis_Feedforward_Calculate(Chassis_t* chassis)
{
	float direct = 1.f;
	
	if(abs(gimbal.info.yaw_mec_err_raw) <= PI/2)
	{
		direct = 1.f;
	}
	else{
	  direct = -1.f;
	}
		
	
	float car_x_f,car_y_f;
	car_x_f = CHASSIS_WEIGHT * GRAVITATIONAL_CONSTANT * sin(-imu_sensor.info->base_info.pitch);
	car_y_f = CHASSIS_WEIGHT * GRAVITATIONAL_CONSTANT * sin(imu_sensor.info->base_info.roll);
	
  chassis->out.wheel_feed_out[WHEEL_LF] = direct * (- car_x_f + car_y_f) / 4 * WHEEL_RADIUS;
	chassis->out.wheel_feed_out[WHEEL_LB] = direct * (- car_x_f - car_y_f) / 4 * WHEEL_RADIUS;
	chassis->out.wheel_feed_out[WHEEL_RF] = direct * (  car_x_f + car_y_f) / 4 * WHEEL_RADIUS;
	chassis->out.wheel_feed_out[WHEEL_RB] = direct * (  car_x_f - car_y_f) / 4 * WHEEL_RADIUS;
	
	chassis->out.wheel_feed_out[WHEEL_LF] = constrain(chassis->out.wheel_feed_out[WHEEL_LF],-1.f,1.f);
	chassis->out.wheel_feed_out[WHEEL_LB] = constrain(chassis->out.wheel_feed_out[WHEEL_LB],-1.f,1.f);
	chassis->out.wheel_feed_out[WHEEL_RF] = constrain(chassis->out.wheel_feed_out[WHEEL_RF],-1.f,1.f);
	chassis->out.wheel_feed_out[WHEEL_RB] = constrain(chassis->out.wheel_feed_out[WHEEL_RB],-1.f,1.f);
	
}




static void Chassis_Pid_Calculate(Chassis_t* chassis)
{
	static uint8_t wheel_sleep[WHEEL_CNT] = {1,1,1,1};
	if(chassis->mode == C_SLEEP)
	{
		for(uint8_t i = 0;i<WHEEL_CNT;i++)
		{
			if(wheel_sleep[i] == 0 && abs(chassis->wheel->motor[i]->rx_info->speed) >= 0.01)
			{
				chassis->wheel->motor[i]->ctrl->speed_ctrl->target = chassis->target.motor_speed[i];
			  chassis->wheel->motor[i]->ctrl->speed_ctrl->measure = chassis->wheel->motor[i]->rx_info->speed;
			  chassis->wheel->motor[i]->ctrl->speed_ctrl->err = chassis->wheel->motor[i]->ctrl->speed_ctrl->target - chassis->wheel->motor[i]->ctrl->speed_ctrl->measure;
			
			  single_pid_ctrl(chassis->wheel->motor[i]->ctrl->speed_ctrl);
			
			  chassis->out.wheel_initial_out[i] = chassis->wheel->motor[i]->ctrl->speed_ctrl->out;
			}
			else{
			  wheel_sleep[i] = 1;
				chassis->out.wheel_initial_out[i] = 0;
			}
		}
		
		Chassis_Offline_Process(chassis);
	}
	else if(chassis->pid_mode == SPEED_MODE)
	{
	  for(uint8_t i = 0;i < 4;i++)
		{
			chassis->wheel->motor[i]->ctrl->speed_ctrl->target = chassis->target.motor_speed[i];
			chassis->wheel->motor[i]->ctrl->speed_ctrl->measure = chassis->wheel->motor[i]->rx_info->speed;
			chassis->wheel->motor[i]->ctrl->speed_ctrl->err = chassis->wheel->motor[i]->ctrl->speed_ctrl->target - chassis->wheel->motor[i]->ctrl->speed_ctrl->measure;
			
			single_pid_ctrl(chassis->wheel->motor[i]->ctrl->speed_ctrl);
			
			chassis->out.wheel_initial_out[i] = chassis->wheel->motor[i]->ctrl->speed_ctrl->out;
			
			wheel_sleep[i] = 0;
		}
	
	}
  else if(chassis->pid_mode == POSITION_MODE)          //后面会处理掉这个多余的东西
	{
		for(uint8_t i = 0;i < 4;i++)
		{
			chassis->wheel->motor[i]->ctrl->angle_ctrl_outer->target = chassis->target.motor_position[i];
			chassis->wheel->motor[i]->ctrl->angle_ctrl_outer->measure = chassis->wheel->motor[i]->rx_info->motor_angle_sum;
			chassis->wheel->motor[i]->ctrl->angle_ctrl_outer->err = chassis->wheel->motor[i]->ctrl->angle_ctrl_outer->target - chassis->wheel->motor[i]->ctrl->angle_ctrl_outer->measure;
			
			single_pid_ctrl(chassis->wheel->motor[i]->ctrl->angle_ctrl_outer);
			
			chassis->wheel->motor[i]->ctrl->angle_ctrl_inner->target = chassis->wheel->motor[i]->ctrl->angle_ctrl_outer->out;
			chassis->wheel->motor[i]->ctrl->angle_ctrl_inner->measure = chassis->wheel->motor[i]->rx_info->speed;
			chassis->wheel->motor[i]->ctrl->angle_ctrl_inner->err = chassis->wheel->motor[i]->ctrl->angle_ctrl_inner->target - chassis->wheel->motor[i]->ctrl->angle_ctrl_inner->measure;
			
			single_pid_ctrl(chassis->wheel->motor[i]->ctrl->angle_ctrl_inner);
			
			chassis->out.wheel_initial_out[i] = chassis->wheel->motor[i]->ctrl->angle_ctrl_inner->out;
			
			wheel_sleep[i] = 0;
		}
	}

}



/**
  * @name    Chassis_Power_Limit
  * @brief   底盘功率限制(经典祖传算法)
  * @note    被我改了
  * @author  WRX
**/
uint32_t  power_fail = 0;
static void Chassis_Power_Limit(Chassis_t * chassis)
{
	  static float last_buffer = 0;
		float limit_output_speed[4];
	
		float buffer = (float)judge.pkt->buffer_energy;
		float heat_rate;//输出电流缩放比例
		float Limit_k; //轮组速度和缩放比例
		float CHAS_LimitOutput;//缩放后轮组最大速度之和
		float CHAS_TotalOutput;//轮组电流之和
		
		//获取理想的底盘输出
		for(uint8_t i = 0;i<WHEEL_CNT;i++)
		{
			limit_output_speed[i] = chassis->wheel->motor[i]->rx_info->speed;
		}
		
		float OUT_MAX = 0;
	
		OUT_MAX = CHASSIS_MAX_SPEED * 4;//最大速度之和
		
		if(buffer > 60.f)
		{
			buffer = 60.f;//防止飞坡之后缓冲250J变为正增益系数
		}
		
		Limit_k = buffer / 60.f;  //最大为1，飞坡后底盘一直最大速度运行
		
		if(buffer < 25.f)
		{
			Limit_k = Limit_k * Limit_k ;//缓冲没多小就更慢一点
		}
		else
		{
			Limit_k = Limit_k;// 缓冲能量还有比较多就限制一点
		}
			
		if(buffer < 60.f)
		{
			CHAS_LimitOutput = Limit_k * OUT_MAX; //只要缓冲能量没满才限制
		}
		else 
		{
			CHAS_LimitOutput = OUT_MAX;    //缓冲能量满的就全速前进
		}
			
		CHAS_TotalOutput = abs(limit_output_speed[0]) + abs(limit_output_speed[1]) + abs(limit_output_speed[2]) + abs(limit_output_speed[3]) ;
		
		if(CHAS_TotalOutput >= CHAS_LimitOutput)
		{
			heat_rate = CHAS_LimitOutput / CHAS_TotalOutput;//电流缩放比例 = 利用现在剩余缓冲能量算出的速度和限制比例 * 轮组最大速度和 / 解算出的理想轮组速度和
		}
		else{
		  heat_rate = 1.f;
		}
		
		for(uint8_t i = 0 ; i < 4 ; i++) 
		{	
			chassis->out.wheel_powerd_out[i] = (float)(chassis->out.wheel_initial_out[i] * heat_rate);	
		}
		
		if(buffer <= 0 && last_buffer > 0)
		{
			power_fail ++;
		}
		
		last_buffer = buffer;
		
}


/**
  * @brief   将功率用电流和转速表达，二阶泰勒展开
  * @param   电流  转速
  * @result  功率
**/
float result = 0;
float tx_result = 0;
static float Calculate_Predicted_Power(float* coefficient,float i, float w) {
    // 系数
	
//	k0 + k1 * I + k2 * ω + k3 * I * ω + k4 * I**2 + k5 * ω**2
	
	
	float k0 = coefficient[0];
  float k1 = coefficient[1];
  float k2 = coefficient[2];
	float k3 = coefficient[3];
	float k4 = coefficient[4];
	float k5 = coefficient[5];

    // 计算多项式曲面值
//    result = k0 + k1 * i + k2 * w + k3 * i * w + k4 * i*i + k5 * w*w;

//    return result;  
	 return k0 + k1 * i + k2 * w + k3 * i * w + k4 * i*i + k5 * w*w;  
}
/**
  * @Name    Calculate_Current_Out
  * @brief   解算出电流输出
  * @param   目标功率  转速  原始电流
  * @result  电流
**/
uint32_t error_test;
static float Calculate_Current_Out(float* coefficient,float target_power, float w, int16_t raw_current)
{
    if (target_power < 0)
    {
        return raw_current;
    }

    // 功率模型系数：P = k0 + k1*I + k2*ω + k3*I*ω + k4*I^2 + k5*ω^2
   float k0 = coefficient[0];
   float k1 = coefficient[1];
   float k2 = coefficient[2];
 	 float k3 = coefficient[3];
	 float k4 = coefficient[4];
	 float k5 = coefficient[5];

    // 一元二次方程：a*I^2 + b*I + c = 0
    // 其中：
    // a = k4
    // b = k1 + k3 * w
    // c = k0 + k2 * w + k5 * w^2 - target_power
    
    float a = k4;
    float b = k1 + k3 * w;
    float c_term = k0 + k2 * w + k5 * w * w - target_power;

    // 计算判别式
    float discriminant = b * b - 4.0f * a * c_term;

    // 判别式小于0，无实数解
    if (discriminant < 0)
    {
        return raw_current;  // 或返回0，根据需求处理
    }

    float sqrt_disc = sqrtf(discriminant);

    // 两个解
    float i_1 = (-b + sqrt_disc) / (2.0f * a);  // 正根（通常对应电动/加速）
    float i_2 = (-b - sqrt_disc) / (2.0f * a);  // 负根（通常对应发电/减速）

    // 通过原始电流正负判断用算出来的正电流还是负电流
    if (raw_current > 0)
    {
        // 检测解是否正确
        if (fabsf(Calculate_Predicted_Power(coefficient,i_1, w) - target_power) > 1.0f)
        {
            error_test++;
        }
        return i_1;
    }
    else if (raw_current < 0)
    {
        if (fabsf(Calculate_Predicted_Power(coefficient,i_2, w) - target_power) > 1.0f)
        {
            error_test++;
        }
        return i_2;
    }
    else
    {
        return 0.0f;
    }
}


static int16_t Torque_To_Current(float torque)
{
  float current_rad = torque / _3508_TORQUE_CONSTANT;
	current_rad = constrain(current_rad, -_3508_MAX_CURRENT*0.9f, _3508_MAX_CURRENT*0.9f);
	
	int16_t current_encoder = (int16_t)((current_rad / _3508_MAX_CURRENT) * 16384.f);
	return current_encoder;
}



static float Current_To_Torque(int16_t current_encoder)
{
	float current_rad = ((float)current_encoder / 16384.f) * _3508_MAX_CURRENT;
	current_rad = constrain(current_rad, -_3508_MAX_CURRENT*0.9f, _3508_MAX_CURRENT*0.9f);
	float torque = current_rad * _3508_TORQUE_CONSTANT;
	
	return torque;
}


/**
  * @Name    New_Chassis_Power_Limit
  * @brief   给底盘电机输出进行功率限制赋值,由于pid计算出的扭矩与功率计所需电流单位不一致，函数中存在转化
             在原新功率算法基础上改动过
  * @param   chassis
**/
float limit=60;
uint8_t buf[5];
float power[4];
float rate = 0;
float fit = 0;
float k_cap_top=10.f;
float k_cap_ord =7.f;
float k_cap = 0.f;
/*计算预测功率*/
		int16_t limit_output_current[4];
static void New_Chassis_Power_Limit(Chassis_t *chassis)
{
//		if (judge.pkt->buffer_energy < 30)
//		{
//			Chassis_Power_Limit(chassis);
//			return;
//		}
	
	//计算超功率次数
	static float last_buffer = 0;
	
	if(judge.pkt->buffer_energy<=0 && last_buffer >0)
	{
		power_fail ++;
	}

	last_buffer = judge.pkt->buffer_energy;

		for(uint8_t i =0;i<WHEEL_CNT;i++)
    {
			limit_output_current[i] = Torque_To_Current(chassis->out.wheel_initial_out[i]);
		}
		
		int16_t motor_speed[4];

		for(uint8_t i =0;i<WHEEL_CNT;i++)
    {
			motor_speed[i] = chassis->wheel->motor[i]->rx_info->encoder_speed;
		}
		
		float power_fit = 0;
		float temp_power[4];
	
		for(uint8_t i = 0; i < 4; i++)
		{
			//根据目标电流 实际转速 求预期功率
		  temp_power[i] = Calculate_Predicted_Power(chassis->power_coefficient[i], limit_output_current[i], motor_speed[i]);
		  
			//放电不算
			if(temp_power[i] > 0)
			{
				power_fit += temp_power[i];
			}
		
			power[i] = temp_power[i];
		}
		
		fit = power_fit;
		/*计算最大输出功率*/
		float max_power = judge.pkt->chassis_power_limit;
		
		float front_speed_err = abs(chassis->target.front_speed - chassis->measure.front_speed);
		float left_speed_err = abs(chassis->target.left_speed - chassis->measure.left_speed);
		float cycle_speed_err = abs(chassis->target.cycle_speed - chassis->measure.cycle_speed);
    float total_speed_err = front_speed_err + left_speed_err + cycle_speed_err;
		
		
		//超电在线，开超电，超电能放电，超电电量充裕，操作手用超电
		if(cap.status->status == DEV_ONLINE && cap_tx_info.bit_control.cap_switch == 1 && \
			cap.info->ability == 1 && cap.info->cap_Ucr > 13.f && infantry.flag.cap_use_flag == true)
		{
//			if(infantry.flag.turn_flag == true)
//			{
//				max_power += (cap.info->cap_Ucr - 13.f) *k_cap_top;
//			}
//			else{
//				max_power += (cap.info->cap_Ucr - 13.f) *k_cap_ord;

//			}
	
			if(total_speed_err >= 10.f)
			{
				k_cap = 2.f;
			}
			else{
			  k_cap = 0.f;
				
			}
			
			max_power += total_speed_err *k_cap;
			max_power = constrain(max_power ,judge.pkt->chassis_power_limit ,160.f);
			

			//爆发
			chassis->burst_flag = true;
		} 
		else
			{
			//防实际小误差使得缓冲慢慢掉
		  max_power *= ((judge.pkt->buffer_energy) * ((1 - 0.75) / (60 - 30)) + 0.5);
			
			chassis->burst_flag = false;
		}
		
		float power_rate = 0;
		if(power_fit == 0)
		{
			power_rate = 0;
		}
		else{
		  power_rate = max_power / power_fit;//折算率
		}
		
		rate = power_rate;
		
		/*计算输出电流*/
		//预测功率大于最大功率才限制
		if (power_fit > max(max_power,judge.pkt->chassis_power_limit))
		{
			//通过折算后的功率、电机现在的转速、pid算出的电流来得到折算后的电流
			
			chassis->out.wheel_powerd_out[WHEEL_RF] = Current_To_Torque(Calculate_Current_Out(chassis->power_coefficient[WHEEL_RF],temp_power[WHEEL_RF] * power_rate, motor_speed[WHEEL_RF],limit_output_current[WHEEL_RF]));
			chassis->out.wheel_powerd_out[WHEEL_RB] = Current_To_Torque(Calculate_Current_Out(chassis->power_coefficient[WHEEL_RB],temp_power[WHEEL_RB] * power_rate, motor_speed[WHEEL_RB],limit_output_current[WHEEL_RB]));
			chassis->out.wheel_powerd_out[WHEEL_LF] = Current_To_Torque(Calculate_Current_Out(chassis->power_coefficient[WHEEL_LF],temp_power[WHEEL_LF] * power_rate, motor_speed[WHEEL_LF],limit_output_current[WHEEL_LF]));
			chassis->out.wheel_powerd_out[WHEEL_LB] = Current_To_Torque(Calculate_Current_Out(chassis->power_coefficient[WHEEL_LB],temp_power[WHEEL_LB] * power_rate, motor_speed[WHEEL_LB],limit_output_current[WHEEL_LB]));
			
		}
		else{
			chassis->out.wheel_powerd_out[WHEEL_RF] = Current_To_Torque(limit_output_current[WHEEL_RF]);
			chassis->out.wheel_powerd_out[WHEEL_RB] = Current_To_Torque(limit_output_current[WHEEL_RB]);
			chassis->out.wheel_powerd_out[WHEEL_LF] = Current_To_Torque(limit_output_current[WHEEL_LF]);
		  chassis->out.wheel_powerd_out[WHEEL_LB] = Current_To_Torque(limit_output_current[WHEEL_LB]);
		}
	
}


/**
  * @brief   中值滤波 + 趋势预测的功率估算
  * @note    更适合缓冲能量跳变的情况，未验证
  */
float Power_Estimate_Advanced(void)
{
    uint16_t buffer = judge.pkt->buffer_energy;
    float power_limit = judge.pkt->chassis_power_limit;
    
    // 环形缓冲区保存最近 5 个采样（500ms）
    static uint16_t buffer_ring[5] = {0};
    static uint8_t idx = 0;
    static float power_out = 0;
    
    const float dt = 0.1f;
    
    // 存入新数据
    buffer_ring[idx] = buffer;
    idx = (idx + 1) % 5;
    
    // 计算中值（去噪）
    uint16_t sorted[5];
    memcpy(sorted, buffer_ring, sizeof(sorted));
    // 简单冒泡排序取中值
    for (uint8_t i = 0; i < 4; i++) {
        for (uint8_t j = 0; j < 4-i; j++) {
            if (sorted[j] > sorted[j+1]) {
                uint16_t tmp = sorted[j];
                sorted[j] = sorted[j+1];
                sorted[j+1] = tmp;
            }
        }
    }
    uint16_t buffer_median = sorted[2];  // 中值
    
    // 计算趋势（最近 200ms 的变化）
    uint8_t idx_now = (idx + 5 - 1) % 5;      // 最新
    uint8_t idx_old = (idx + 5 - 3) % 5;      // 200ms 前
    int16_t trend = (int16_t)buffer_ring[idx_now] - (int16_t)buffer_ring[idx_old];
    
    // 预测未来 100ms 的缓冲
    float predicted_delta = (float)trend / 2.0f;  // 200ms → 100ms
    
    // 反推功率
    float power_raw;
    if (buffer_median == 0) {
        power_raw = power_limit;
    } else if (predicted_delta < 0) {
        // 趋势减少，功率在上升
        power_raw = power_limit + fabsf(predicted_delta) / dt;
    } else if (predicted_delta > 0) {
        // 趋势增加，功率在下降
        power_raw = power_limit - 2.0f;
    } else {
        power_raw = power_limit + 2.0f;
    }
    
    // 输出滤波
    const float alpha = 0.25f;
    power_out = alpha * power_raw + (1.0f - alpha) * power_out;
    
    // 限幅
    if (power_out < 0) power_out = 0;
    if (power_out > power_limit * 2.5f) power_out = power_limit * 2.5f;
    
    return power_out;
}


///**
//  * @Name    Caluculate_All_Predicted_Power
//  * @brief   计算出所有的预测功率通过指针传出到全局变量，方便debug
//  * @param   底盘结构体  电机的功率  电机总功率  预测总功率和裁判系统收到的功率的差值
//**/
//static void Caluculate_All_Predicted_Power(Chassis_t *chassis,float *each_power,float *power_all,float *power_error)
//{
//	int16_t limit_output_current[4];
//	float motor_speed[4];
//	float power_fit = 0;
//	
//	limit_output_current[WHEEL_RF] = chassis->out.wheel_end_out[WHEEL_RF];
//	limit_output_current[WHEEL_RB] = chassis->out.wheel_end_out[WHEEL_RB];
//	limit_output_current[WHEEL_LF] = chassis->out.wheel_end_out[WHEEL_LF];
//	limit_output_current[WHEEL_LB] = chassis->out.wheel_end_out[WHEEL_LB];

//	motor_speed[WHEEL_RF] = chassis->wheel->motor[WHEEL_RF]->rx_info->speed;
//	motor_speed[WHEEL_RB] = chassis->wheel->motor[WHEEL_RB]->rx_info->speed;
//	motor_speed[WHEEL_LF] = chassis->wheel->motor[WHEEL_LF]->rx_info->speed;
//	motor_speed[WHEEL_LB] = chassis->wheel->motor[WHEEL_LB]->rx_info->speed;

//	for(uint8_t i = 0; i < 4; i++)
//	{
//		each_power[i] = Calculate_Predicted_Power(chassis->power_coefficient[i],limit_output_current[i], motor_speed[i]);
//		if(each_power[i] > 0)
//		{
//			power_fit += each_power[i];
//		}
//	}
//	*power_all = power_fit;
//	*power_error = power_fit - Power_Estimate_Advanced();
//}
	


int test_count;
/**
 * @brief  底盘报文发送
 * @note   关底盘和关功率在此最后处理
 */
static void Chassis_Cmd_Transmit(Chassis_t* chassis)
{
	static int count = 0,last_chassis_state = 0;
	

	#if CHASSIS_SWITCH == 0
	  chassis->wheel->motor[WHEEL_RF]->tx_info->torque = 0;
	  chassis->wheel->motor[WHEEL_RB]->tx_info->torque = 0;
	  chassis->wheel->motor[WHEEL_LF]->tx_info->torque = 0;
	  chassis->wheel->motor[WHEEL_LB]->tx_info->torque = 0;
	
	#else
  	#if POWER_LIMIT_SWITCH == 0
	    chassis->out.wheel_end_out[WHEEL_RF] = chassis->out.wheel_initial_out[WHEEL_RF];
	    chassis->out.wheel_end_out[WHEEL_RB] = chassis->out.wheel_initial_out[WHEEL_RB];
	    chassis->out.wheel_end_out[WHEEL_LF] = chassis->out.wheel_initial_out[WHEEL_LF];
	    chassis->out.wheel_end_out[WHEEL_LB] = chassis->out.wheel_initial_out[WHEEL_LB];
	  #else
	    chassis->out.wheel_end_out[WHEEL_RF] = chassis->out.wheel_powerd_out[WHEEL_RF];
	    chassis->out.wheel_end_out[WHEEL_RB] = chassis->out.wheel_powerd_out[WHEEL_RB];
	    chassis->out.wheel_end_out[WHEEL_LF] = chassis->out.wheel_powerd_out[WHEEL_LF];
	    chassis->out.wheel_end_out[WHEEL_LB] = chassis->out.wheel_powerd_out[WHEEL_LB];
	
	  #endif
		
	    chassis->wheel->motor[WHEEL_RF]->tx_info->torque = chassis->out.wheel_end_out[WHEEL_RF];
	    chassis->wheel->motor[WHEEL_RB]->tx_info->torque = chassis->out.wheel_end_out[WHEEL_RB];
	    chassis->wheel->motor[WHEEL_LF]->tx_info->torque = chassis->out.wheel_end_out[WHEEL_LF];
	    chassis->wheel->motor[WHEEL_LB]->tx_info->torque = chassis->out.wheel_end_out[WHEEL_LB];
	#endif
	
	int triggle = (chassis->wheel->motor[WHEEL_RF]->state->status == DEV_ONLINE &&
				   chassis->wheel->motor[WHEEL_RB]->state->status == DEV_ONLINE &&
				   chassis->wheel->motor[WHEEL_LF]->state->status == DEV_ONLINE &&
				   chassis->wheel->motor[WHEEL_LB]->state->status == DEV_ONLINE );

	//底盘全掉阵亡重启时不控
	if (count >= 5000 || triggle)
	{
		count = 0;
	}
	else if((!infantry.flag.chassis_off && last_chassis_state == 1) || 
		count != 0) 
	{
		count++;
	  chassis->wheel->motor[WHEEL_RF]->tx_info->torque = 0;
	  chassis->wheel->motor[WHEEL_RB]->tx_info->torque = 0;
	  chassis->wheel->motor[WHEEL_LF]->tx_info->torque = 0;
	  chassis->wheel->motor[WHEEL_LB]->tx_info->torque = 0;
	}
	
	
	chassis->wheel->group_set_torque(chassis->wheel);
	
	last_chassis_state = infantry.flag.chassis_off;
	
	test_count = count;
	
}

float each_power[4];
float power_all;
float power_error;
/**
 * @brief  底盘工作函数
 * @note   
 */
static void Chassis_Work(Chassis_t* chassis)
{
	Chassis_Offline_Update(chassis);
  Chassis_Status_Update(chassis);
	Chassis_Positive_Calculate(chassis);
	Chassis_Target_Update(chassis);
	Chassis_Inverse_Calculate(chassis);
	Chassis_Feedforward_Calculate(chassis);
	Chassis_Pid_Calculate(chassis);
//	Chassis_Power_Limit(chassis);
	New_Chassis_Power_Limit(chassis);
	result= 
	 Calculate_Predicted_Power(chassis->power_coefficient[0],(float)chassis->wheel->motor[0]->rx_info->torque_current_raw, (float)chassis->wheel->motor[0]->rx_info->encoder_speed)
	+Calculate_Predicted_Power(chassis->power_coefficient[1],(float)chassis->wheel->motor[1]->rx_info->torque_current_raw, (float)chassis->wheel->motor[1]->rx_info->encoder_speed)
	+Calculate_Predicted_Power(chassis->power_coefficient[2],(float)chassis->wheel->motor[2]->rx_info->torque_current_raw, (float)chassis->wheel->motor[2]->rx_info->encoder_speed)
	+Calculate_Predicted_Power(chassis->power_coefficient[3],(float)chassis->wheel->motor[3]->rx_info->torque_current_raw, (float)chassis->wheel->motor[3]->rx_info->encoder_speed);
	
	tx_result= 
	 Calculate_Predicted_Power(chassis->power_coefficient[0],(float)chassis->wheel->motor[0]->tx_info->torque_current_raw, (float)chassis->wheel->motor[0]->rx_info->encoder_speed)
	+Calculate_Predicted_Power(chassis->power_coefficient[1],(float)chassis->wheel->motor[1]->tx_info->torque_current_raw, (float)chassis->wheel->motor[1]->rx_info->encoder_speed)
	+Calculate_Predicted_Power(chassis->power_coefficient[2],(float)chassis->wheel->motor[2]->tx_info->torque_current_raw, (float)chassis->wheel->motor[2]->rx_info->encoder_speed)
	+Calculate_Predicted_Power(chassis->power_coefficient[3],(float)chassis->wheel->motor[3]->tx_info->torque_current_raw, (float)chassis->wheel->motor[3]->rx_info->encoder_speed);
	

	Chassis_Cmd_Transmit(chassis);
//	Caluculate_All_Predicted_Power(chassis,each_power,&power_all,&power_error);
	
}



#define BASE_MIN  TURN_CYCLE_SPEED
#define BASE_MAX  TURN_CYCLE_SPEED + 10
static const int8_t T[7][6] = {
    {2, -2, -1},
    {3, -3, -1, 0},
    {4, -3, -2, 0, 1},
    {4, -2, -1, 1, 2},
    {4, -1, 0, 2, 3},
    {3, 0, 1, 3},
    {2, 1, 2},
};
static uint8_t  g_base = 40;
static int8_t   g_step = 1;
static uint32_t g_seed;

// cmd: 0=初始化, 1=计算步长
static int8_t random_step_calculate(uint8_t cmd, uint32_t seed) {
    if (cmd == 0) {
        g_seed = seed;
        g_base = 40;
        g_step = 1;
        return 0;
    }

    uint8_t pos = g_base - BASE_MIN;
    const int8_t *t = T[g_step + 3];
    uint8_t n = t[0];

    int8_t  cands[4];
    uint8_t w[4], m = 0;

    for (uint8_t i = 0; i < n; i++) {
        int8_t d = t[i + 1];
        int8_t np = (int8_t)pos + d;
        if (np < 0 || np > 10) continue;
        cands[m] = d;
        w[m] = 10;
        if (g_step != 0 && (d * g_step) > 0) w[m] += 4;
        if (d == 0) w[m] = 3;
        m++;
    }

    if (m == 0) {
        int8_t d1, d2, d3;
        if (pos < 5) { d1 = 1; d2 = 2; d3 = 3; }
        else         { d1 = -3; d2 = -2; d3 = -1; }

        int8_t np1 = (int8_t)pos + d1;
        if (np1 >= 0 && np1 <= 10) { cands[m] = d1; w[m] = 10; m++; }

        int8_t np2 = (int8_t)pos + d2;
        if (np2 >= 0 && np2 <= 10) { cands[m] = d2; w[m] = 10; m++; }

        int8_t np3 = (int8_t)pos + d3;
        if (np3 >= 0 && np3 <= 10) { cands[m] = d3; w[m] = 10; m++; }
    }

    uint16_t total = 0, r, acc = 0;
    for (uint8_t i = 0; i < m; i++) total += w[i];

 
    g_seed = g_seed * 1103515245 + 12345;
    r = (uint8_t)(g_seed >> 16) % total;

    for (uint8_t i = 0; i < m; i++) {
        acc += w[i];
        if (r < acc) { g_step = cands[i]; g_base += cands[i]; return cands[i]; }
    }

    g_step = cands[0]; g_base += cands[0]; return cands[0];
}








