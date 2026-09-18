#include "Flag_Handle.h"
void Gimbal_Rescue_Process(void);
/*标志位处理相关函数均包含在此文件*/
/**
  * @brief  跳跃腿长目标值处理
  * @param  None
  * @retval None
  */
void Jump_Target_Process(Chassis_t* My_Chassis)
{
  static uint8_t step = 0;
  static float time = 0;
  static double org_tar = 0.f;
  
	Four_Bar_Link_t* Link_Var = My_Chassis->Link;
	
	Leg_Unit_t* my_r_leg = My_Chassis->Leg->r_leg;
  Leg_Unit_t* my_l_leg = My_Chassis->Leg->l_leg;
  if(Balance.command->chassis->Jump_Flag == true)
  {
		time += TIME_STEP/2;
		Balance.command->chassis->LQR_Model = true;
		if(time <=0.10660f)
    {
			
			my_r_leg->off_ground = false;
			my_l_leg->off_ground = false;
      Chassis.target->leg_length = 0.29220033285405910827847719701822f - 0.1122003328540591010620275369547f*arm_cos_f32(17.716039049245708270063914824277f*time);
      Link_Var[R_Link].link_pid->length_cal->kp = 800;
			Link_Var[L_Link].link_pid->length_cal->kp = 800;
			Link_Var[R_Link].link_pid->length_cal->kd = 0;
			Link_Var[L_Link].link_pid->length_cal->kd = 0;
      Link_Var[R_Link].link_pid->length_cal->out_max = 1000;
			Link_Var[L_Link].link_pid->length_cal->out_max = 1000;
			Link_Var[R_Link].info->force->Fbl = Link_Var[R_Link].info->force->F_gravity;
			Link_Var[L_Link].info->force->Fbl = Link_Var[L_Link].info->force->F_gravity;
    }
//    else if(time > 0.10660f && time <=0.21885f)
//    {
//			my_r_leg->off_ground = true;
//			my_l_leg->off_ground = true;
//      Chassis.target->leg_length = - 16523.353458312026937452421709355f*powf(time, 5) + 10126.854924719774418939029791616f*powf(time, 4) - 1794.5557732111474201769709062301f*powf(time, 3)- 5.5021487204249162061842071125284f*powf(time, 2) + 25.050936172883349655894309537668f*time - 1.1870406801356612201482064487589f;
//			Link_Var[R_Link].link_pid->length_cal->kp = 0;
//			Link_Var[L_Link].link_pid->length_cal->kp = 0;
//			Link_Var[R_Link].link_pid->length_cal->kd = 0;
//			Link_Var[L_Link].link_pid->length_cal->kd = 0;
//      Link_Var[R_Link].link_pid->length_cal->out_max = 1000;
//			Link_Var[L_Link].link_pid->length_cal->out_max = 1000;
//    }
//    else if(time > 0.21885f && time <= 0.32655f)
//    {
//			my_r_leg->off_ground = true;
//			my_l_leg->off_ground = true;
//			Chassis.target->leg_length = 11272.31063565249247427413465823f*powf(time, 5) - 10968.390383521972821296431269912f*powf(time, 4) + 3156.7282453122346839522150871742f*powf(time, 3) - 5.5021487204249162061842071125284f*powf(time, 2) - 120.71766411742731972362805528538f*time + 13.252929492697393403349057018181f;
//			Link_Var[R_Link].link_pid->length_cal->kp = 0;
//			Link_Var[L_Link].link_pid->length_cal->kp = 0;
//			Link_Var[R_Link].link_pid->length_cal->kd = 0;
//			Link_Var[L_Link].link_pid->length_cal->kd = 0;
//      Link_Var[R_Link].link_pid->length_cal->out_max = 500;
//			Link_Var[R_Link].link_pid->length_cal->out_max = 500;
//    }
//		else if(time > 0.32655f && time <= 0.47292f)
//    {
//			my_r_leg->off_ground = true;
//			my_l_leg->off_ground = true;
//			Chassis.target->leg_length = 0.30303553119974879770381903654197f - 0.12303553119974881824294499210737f*arm_cos_f32(10.731022528880526323291633161716f*time - 5.0749637538083392706048123688471f);
//			Link_Var[R_Link].link_pid->length_cal->kp = 0;
//			Link_Var[L_Link].link_pid->length_cal->kp = 0;
//			Link_Var[R_Link].link_pid->length_cal->kd = 0;
//			Link_Var[L_Link].link_pid->length_cal->kd = 0;
//      Link_Var[R_Link].link_pid->length_cal->out_max = 500;
//			Link_Var[R_Link].link_pid->length_cal->out_max = 500;
//    }
		else
		{
			my_r_leg->off_ground = false;
			my_l_leg->off_ground = false;
			Chassis.target->leg_length = TAR_LEG_LENGTH_INITIAL;
			Balance.command->chassis->Jump_Flag = false;
			Balance.command->chassis->LQR_Model = true;
			Link_Var[R_Link].link_pid->length_cal->kp = 950;
			Link_Var[L_Link].link_pid->length_cal->kp = 950;
			Link_Var[R_Link].link_pid->length_cal->kd = 70000;
			Link_Var[L_Link].link_pid->length_cal->kd = 70000;
      Link_Var[R_Link].link_pid->length_cal->out_max = 200;
			Link_Var[R_Link].link_pid->length_cal->out_max = 200;
		}
  }
	else
	{
		time = 0;
	}
  
}


/**
  * @brief  底盘自救判断
  * @param  None
  * @retval None
  */
void Rescue_Check(void)
{
	static uint16_t cnt = 0;
	
	float R_phi0 = Chassis.Link[R_Link].info->angle->vir_phi0_ + Chassis.Posture->info->pitch/Degree_to_rad;
	float L_phi0 = Chassis.Link[L_Link].info->angle->vir_phi0_ + Chassis.Posture->info->pitch/Degree_to_rad;
	
	if(Balance.command->chassis->Rescue_Check == false)
	{
		if(Balance.command->chassis->Rescue_Flag == false)
		{
			if((R_phi0 >= 90.f && R_phi0 < 190.f) ||
							(R_phi0 <= -65.f && R_phi0 > -190.f))
			{
				Balance.command->chassis->Rescue_step = 4;
				Chassis.Leg->r_leg->rescue_flag = true;
			}
			else
			{
				Chassis.Leg->r_leg->rescue_flag = false;
			}

			if((L_phi0 >= 90.f && L_phi0 < 190.f) ||
							(L_phi0 <= -65.f && L_phi0 > -190.f))
			{
				Balance.command->chassis->Rescue_step = 4;
				Chassis.Leg->l_leg->rescue_flag = true;
			}
			else
			{
				Chassis.Leg->l_leg->rescue_flag = false;
			}
		}
	}
	else
	{
		if(R_phi0 >= 60.f && R_phi0 < 190.f)
		{
			Balance.command->chassis->Rescue_step = 4;
			Chassis.Leg->r_leg->rescue_flag = true;
		}
		else if(R_phi0 < -10.f && Chassis.Link[R_Link].info->length->l0 >= 0.20f)
		{
			Balance.command->chassis->Rescue_step = 4;
			Chassis.Leg->r_leg->rescue_flag = true;
		}
		else
		{
			Chassis.Leg->r_leg->rescue_flag = false;
		}
		
		if(L_phi0 >= 60.f && L_phi0 < 190.f)
		{
			Balance.command->chassis->Rescue_step = 4;
			Chassis.Leg->l_leg->rescue_flag = true;
		}
		else if(L_phi0 < -10.f && Chassis.Link[L_Link].info->length->l0 >= 0.20f)
		{
			Balance.command->chassis->Rescue_step = 4;
			Chassis.Leg->l_leg->rescue_flag = true;
		}
		else
		{
			Chassis.Leg->l_leg->rescue_flag = false;
		}
		
		Balance.command->chassis->Rescue_Check = false;
	}
	
	if(Chassis.Posture->info->pitch > 1.570796326794f ||
		Chassis.Posture->info->pitch < -1.570796326794f)
	{
		Balance.command->Balance_Init_Flag = false;
		Balance.command->chassis->Rescue_Flag = true;
		if(abs(Chassis.Posture->info->roll) > 0.2f)
		{
			if(Balance.command->chassis->Rescue_step != 1)
			Balance.command->chassis->Rescue_step = 0;
		}
		else
		{
			if(Balance.command->chassis->Rescue_step != 3)
			Balance.command->chassis->Rescue_step = 2;
		}
	}
	else
	{
		if(Chassis.Leg->r_leg->rescue_flag == true ||
		 Chassis.Leg->l_leg->rescue_flag == true)
		{
			Balance.command->Balance_Init_Flag = false;
			Balance.command->chassis->Rescue_Flag = true;
			Balance.last_mode = Sleep_Mode;
			
			if(Balance.command->chassis->Rescue_step <= 3)
			Balance.command->chassis->Rescue_step = 4;
			
			if(Balance.command->chassis->Rescue_step == 5)
			{
				cnt ++;
				if(cnt > 500)
				{
					
					Balance.command->chassis->Rescue_Flag = false;
				}
			}
			else
			{
				cnt = 0;
			}
			
		}
		else
		{
			cnt = 0;
			Balance.command->chassis->Rescue_Flag = false;
		}
	}
}

pid_ctrl_t Resuce_Pitch_Ctrl = 
{
		.kp = 20.f,//
    .ki = 0.f,
    .kd = 500.f,
		.a = 0.5f,
    .integral_max = 2.f,
    .out_max = 200.f,//
};

pid_ctrl_t Resuce_Roll_Ctrl = 
{
		.kp = 45.f,//
    .ki = 0.f,
    .kd = 500.f,
		.a = 0.5f,
    .integral_max = 2.f,
    .out_max = 200.f,//
};
uint8_t test_Flag = 0;
/**
  * @brief  自救处理
  * @param  None
  * @retval None
  */
void Rescue_Process(void)
{
	Four_Bar_Link_t* R_Link_Var = &Chassis.Link[R_Link];
	Four_Bar_Link_t* L_Link_Var = &Chassis.Link[L_Link];
	static uint8_t cnt = 0;
	float R_phi0 = Chassis.Link[R_Link].info->angle->vir_phi0_ + Chassis.Posture->info->pitch/Degree_to_rad;
	float L_phi0 = Chassis.Link[L_Link].info->angle->vir_phi0_ + Chassis.Posture->info->pitch/Degree_to_rad;
	Chassis.target->leg_length = TAR_LEG_LENGTH_INITIAL;//腿长目标值改为初始值
	
	Chassis.target->yaw = Chassis.Leg->info->phi;//偏航角目标值等于测量值
	
	Chassis.target->roll = 0;
	
	Chassis.target->distance = Chassis.Leg->info->s;
	
	Chassis.target->velocity = 0;
	
	Chassis.target->yaw_v = 0;
	if(Balance.command->chassis->Rescue_step == 0)
	{
		if(Chassis.Posture->info->roll > 0)
		{
			R_Link_Var->link_pid->phi0_speed_cal->target = 3.f;
			R_Link_Var->link_pid->phi0_speed_cal->measure = R_Link_Var->info->angle->vir_phi0_d1;
			pid_err_cal(R_Link_Var->link_pid->phi0_speed_cal);
			single_pid_ctrl(R_Link_Var->link_pid->phi0_speed_cal);//
			Chassis.Leg->T_info->T_mat_storage[Sd_R] = R_Link_Var->link_pid->phi0_speed_cal->out;
			Chassis.Link[R_Link].info->force->Fbl = R_Link_Var->info->force->F;
			Chassis.Leg->T_info->T_mat_storage[Tw_R] = 0;
			
			Chassis.Leg->T_info->T_mat_storage[Sd_L] = 0;
			Chassis.Link[L_Link].info->force->Fbl = 0;
			Chassis.Leg->T_info->T_mat_storage[Tw_L] = 0;
			
			if(R_Link_Var->link_pid->phi0_speed_cal->err >= 2.9f)
			{
				if(Chassis.Posture->info->pitch > 0)
				{
					if(R_Link_Var->info->angle->vir_phi0_ < 150.f && R_Link_Var->info->angle->vir_phi0_ > 80.f)
					{
						cnt ++;
					}
					else
					{
						cnt = 0;
					}
				}
				else
				{
					if((R_Link_Var->info->angle->vir_phi0_ < 90.f && R_Link_Var->info->angle->vir_phi0_ >= 0.f) ||
						(R_Link_Var->info->angle->vir_phi0_ < 0.f && R_Link_Var->info->angle->vir_phi0_ > -10.f))
					{
						cnt ++;
					}
					else
					{
						cnt = 0;
					}
				}
			}
			else
			{
				cnt = 0;
			}
		}
		else
		{
			L_Link_Var->link_pid->phi0_speed_cal->target = 3.f;
			L_Link_Var->link_pid->phi0_speed_cal->measure = L_Link_Var->info->angle->vir_phi0_d1;
			pid_err_cal(L_Link_Var->link_pid->phi0_speed_cal);
			single_pid_ctrl(L_Link_Var->link_pid->phi0_speed_cal);//
			Chassis.Leg->T_info->T_mat_storage[Sd_L] = L_Link_Var->link_pid->phi0_speed_cal->out;
			Chassis.Link[L_Link].info->force->Fbl = L_Link_Var->info->force->F;
			Chassis.Leg->T_info->T_mat_storage[Tw_L] = 0;
			
			Chassis.Leg->T_info->T_mat_storage[Sd_R] = 0;
			Chassis.Link[R_Link].info->force->Fbl = 0;
			Chassis.Leg->T_info->T_mat_storage[Tw_R] = 0;
			
			if(L_Link_Var->link_pid->phi0_speed_cal->err >= 2.9f)
			{
				if(Chassis.Posture->info->pitch > 0)
				{
					if(L_Link_Var->info->angle->vir_phi0_ < 150.f && L_Link_Var->info->angle->vir_phi0_ > 80.f)
					{
						cnt ++;
					}
					else
					{
						cnt = 0;
					}
				}
				else
				{
					if((L_Link_Var->info->angle->vir_phi0_ < 90.f && L_Link_Var->info->angle->vir_phi0_ >= 0.f) ||
						(L_Link_Var->info->angle->vir_phi0_ < 0.f && L_Link_Var->info->angle->vir_phi0_ > -10.f))
					{
						cnt ++;
					}
					else
					{
						cnt = 0;
					}
				}
				
			}
			else
			{
				cnt = 0;
			}
		}
		
		if(cnt > 50)
		{
			Balance.command->chassis->Rescue_step = 1;
			cnt = 0;
		}
	}
	else if(Balance.command->chassis->Rescue_step == 1)
	{
		Resuce_Roll_Ctrl.target = 0;
		Resuce_Roll_Ctrl.measure = -abs(Chassis.Posture->info->roll);
		pid_err_cal(&Resuce_Roll_Ctrl);
		single_pid_ctrl(&Resuce_Roll_Ctrl);
		if(Chassis.Posture->info->roll > 0.15)
		{
			Chassis.Leg->T_info->T_mat_storage[Sd_L] = 0;
			Chassis.Link[L_Link].info->force->Fbl = 0;
			Chassis.Leg->T_info->T_mat_storage[Tw_L] = 0;
			
			Chassis.Leg->T_info->T_mat_storage[Sd_R] = Resuce_Roll_Ctrl.out;
			Chassis.Link[R_Link].info->force->Fbl = 0;
			Chassis.Leg->T_info->T_mat_storage[Tw_R] = 0;
		}
		else if(Chassis.Posture->info->roll < -0.15)
		{
			Chassis.Leg->T_info->T_mat_storage[Sd_L] = Resuce_Roll_Ctrl.out;
			Chassis.Link[L_Link].info->force->Fbl = 0;
			Chassis.Leg->T_info->T_mat_storage[Tw_L] = 0;
			
			Chassis.Leg->T_info->T_mat_storage[Sd_R] = 0;
			Chassis.Link[R_Link].info->force->Fbl = 0;
			Chassis.Leg->T_info->T_mat_storage[Tw_R] = 0;
		}
	}
	/*自救第一步，腿先着地*/
	else if(Balance.command->chassis->Rescue_step == 2)
	{
		if(Balance.command->gimbal->Rescue_Flag == false)
		{
			R_Link_Var->link_pid->phi0_speed_cal->target = 3.f*one(Chassis.Posture->info->pitch);
			R_Link_Var->link_pid->phi0_speed_cal->measure = R_Link_Var->info->angle->vir_phi0_d1;
			pid_err_cal(R_Link_Var->link_pid->phi0_speed_cal);
			single_pid_ctrl(R_Link_Var->link_pid->phi0_speed_cal);//
			Chassis.Leg->T_info->T_mat_storage[Sd_R] = R_Link_Var->link_pid->phi0_speed_cal->out-R_Link_Var->info->force->G_torque;
			Chassis.Link[R_Link].info->force->Fbl = R_Link_Var->info->force->F;
			Chassis.Leg->T_info->T_mat_storage[Tw_R] = 0;
			
			L_Link_Var->link_pid->phi0_speed_cal->target = 3.f*one(Chassis.Posture->info->pitch);
			L_Link_Var->link_pid->phi0_speed_cal->measure = L_Link_Var->info->angle->vir_phi0_d1;
			pid_err_cal(L_Link_Var->link_pid->phi0_speed_cal);
			single_pid_ctrl(L_Link_Var->link_pid->phi0_speed_cal);//
			Chassis.Leg->T_info->T_mat_storage[Sd_L] = L_Link_Var->link_pid->phi0_speed_cal->out-L_Link_Var->info->force->G_torque;
			Chassis.Link[L_Link].info->force->Fbl = L_Link_Var->info->force->F;
			Chassis.Leg->T_info->T_mat_storage[Tw_L] = 0;
		}
		else
		{
			Gimbal_Rescue_Process();
		}
		
		if(abs(R_Link_Var->link_pid->phi0_speed_cal->err) >= 2.9f &&
			abs(L_Link_Var->link_pid->phi0_speed_cal->err) >= 2.9f &&
		R_Link_Var->info->length->l0 > 0.30f && L_Link_Var->info->length->l0 > 0.30f)
		{
			if(Chassis.Posture->info->pitch > 0)
			{
				if(((R_Link_Var->info->angle->vir_phi0_ > 120.f && R_Link_Var->info->angle->vir_phi0_ < 190.f) ||
					(R_Link_Var->info->angle->vir_phi0_ < -90.f && R_Link_Var->info->angle->vir_phi0_ > -190.f)) &&
					((L_Link_Var->info->angle->vir_phi0_ > 120.f && L_Link_Var->info->angle->vir_phi0_ < 190.f) ||
					(L_Link_Var->info->angle->vir_phi0_ < -90.f && L_Link_Var->info->angle->vir_phi0_ > -190.f)))
				{
					cnt ++;
				}
				else
				{
					cnt = 0;
				}
			}
			else
			{
				if(((R_Link_Var->info->angle->vir_phi0_ > 100.f && R_Link_Var->info->angle->vir_phi0_ < 190.f) ||
					(R_Link_Var->info->angle->vir_phi0_ < -150.f && R_Link_Var->info->angle->vir_phi0_ > -190.f)) &&
					((L_Link_Var->info->angle->vir_phi0_ > 100.f && L_Link_Var->info->angle->vir_phi0_ < 190.f) ||
					(L_Link_Var->info->angle->vir_phi0_ < -150.f && L_Link_Var->info->angle->vir_phi0_ > -190.f)))
				{
					cnt ++;
				}
				else
				{
					cnt = 0;
				}
			}

			if(cnt >= 50)
			{
				Balance.command->chassis->Rescue_step = 3;
			}
		}
		else
		{
			cnt = 0;
		}
	}
	/*自救第二步，髋关节发力回正机体*/
	else if(Balance.command->chassis->Rescue_step == 3)
	{
		Resuce_Pitch_Ctrl.target = 0;
		Resuce_Pitch_Ctrl.measure = -Chassis.Posture->info->pitch;
		pid_err_cal(&Resuce_Pitch_Ctrl);
		single_pid_ctrl(&Resuce_Pitch_Ctrl);

		Chassis.Leg->T_info->T_mat_storage[Sd_R] = Resuce_Pitch_Ctrl.out;
		Chassis.Leg->T_info->T_mat_storage[Sd_L] = Resuce_Pitch_Ctrl.out;
		Chassis.Link[R_Link].info->force->Fbl = R_Link_Var->info->force->F;
		Chassis.Link[L_Link].info->force->Fbl = L_Link_Var->info->force->F;
		Chassis.Leg->T_info->T_mat_storage[Tw_R] = 0;
		Chassis.Leg->T_info->T_mat_storage[Tw_L] = 0;
	}
	/*自救第三步，腿转到与地面接触*/
	else if(Balance.command->chassis->Rescue_step == 4)
	{
		if(Chassis.Leg->r_leg->rescue_flag == true)
		{
			if(R_phi0 < 60.f && R_phi0 > -10.f)
			{
				R_Link_Var->link_pid->phi0_speed_cal->target = 0.f;
			}
			else
			{
				R_Link_Var->link_pid->phi0_speed_cal->target = -6.f;
			}
			
			R_Link_Var->link_pid->phi0_speed_cal->measure = R_Link_Var->info->angle->vir_phi0_d1;
			pid_err_cal(R_Link_Var->link_pid->phi0_speed_cal);
			single_pid_ctrl(R_Link_Var->link_pid->phi0_speed_cal);
			Chassis.Leg->T_info->T_mat_storage[Sd_R] = R_Link_Var->link_pid->phi0_speed_cal->out;
			Chassis.Link[R_Link].info->force->Fbl = R_Link_Var->info->force->F*0.6f;
			Chassis.Leg->T_info->T_mat_storage[Tw_R] = 0;
		}
		else
		{
			Chassis.Leg->T_info->T_mat_storage[Sd_R] = 0;
			Chassis.Link[R_Link].info->force->Fbl = 0;
			Chassis.Leg->T_info->T_mat_storage[Tw_R] = 0;
		}
		
		if(Chassis.Leg->l_leg->rescue_flag == true)
		{
			if(L_phi0 < 60.f && L_phi0 > -10.f)
			{
				L_Link_Var->link_pid->phi0_speed_cal->target = 0.f;
			}
			else
			{
				L_Link_Var->link_pid->phi0_speed_cal->target = -6.f;
			}
			L_Link_Var->link_pid->phi0_speed_cal->measure = L_Link_Var->info->angle->vir_phi0_d1;
			pid_err_cal(L_Link_Var->link_pid->phi0_speed_cal);
			single_pid_ctrl(L_Link_Var->link_pid->phi0_speed_cal);
			Chassis.Leg->T_info->T_mat_storage[Sd_L] = L_Link_Var->link_pid->phi0_speed_cal->out;
			Chassis.Link[L_Link].info->force->Fbl = L_Link_Var->info->force->F*0.6f;
			Chassis.Leg->T_info->T_mat_storage[Tw_L] = 0;
		}
		else
		{
			Chassis.Leg->T_info->T_mat_storage[Sd_L] = 0;
			Chassis.Link[L_Link].info->force->Fbl = 0;
			Chassis.Leg->T_info->T_mat_storage[Tw_L] = 0;
		}
		
		if((((R_Link_Var->info->angle->vir_phi0_ <= 95.f && R_Link_Var->info->angle->vir_phi0_ >= 0))
			 || (Chassis.Leg->r_leg->rescue_flag == false)|| 
				 (R_Link_Var->info->length->l0 <= 0.20f && R_Link_Var->link_pid->phi0_speed_cal->err < -5.f
			 && R_Link_Var->info->angle->vir_phi0_  > -90.f && R_Link_Var->info->angle->vir_phi0_ < 0.f))&&
			((((L_Link_Var->info->angle->vir_phi0_ <= 95.f && L_Link_Var->info->angle->vir_phi0_ >= 0)))
			 || (Chassis.Leg->l_leg->rescue_flag == false) || 
				 (L_Link_Var->info->length->l0 <= 0.20f && L_Link_Var->link_pid->phi0_speed_cal->err < -5.f
			    && L_Link_Var->info->angle->vir_phi0_ > -90.f && L_Link_Var->info->angle->vir_phi0_ < 0.f)) )
		{
			cnt ++;
			if(cnt >= 10)
			{
				Balance.command->chassis->Rescue_step = 5;
			}
		}
		else
		{
			cnt = 0;
		}
	}
	/*自救第四步，收腿回正车身*/
	else if(Balance.command->chassis->Rescue_step == 5)
	{
		if(Chassis.Leg->l_leg->rescue_flag == true)
		{
			Chassis.Leg->T_info->T_mat_storage[Tw_L] = 0.f;
			Chassis.Leg->T_info->T_mat_storage[Sd_L] *= 0.f;
			Chassis.Link[L_Link].info->force->Fbl = L_Link_Var->info->force->F*0.6f;
		}
		else
		{
			L_Link_Var->link_pid->phi0_speed_cal->target = 0.f;
			L_Link_Var->link_pid->phi0_speed_cal->measure = L_Link_Var->info->angle->vir_phi0_d1;
			pid_err_cal(L_Link_Var->link_pid->phi0_speed_cal);
			single_pid_ctrl(L_Link_Var->link_pid->phi0_speed_cal);
			Chassis.Leg->T_info->T_mat_storage[Sd_L] = 0;
			Chassis.Leg->T_info->T_mat_storage[Tw_L] = 0;
			Chassis.Link[L_Link].info->force->Fbl = L_Link_Var->link_pid->phi0_speed_cal->out;
		}
		
		if(Chassis.Leg->r_leg->rescue_flag == true)
		{
			Chassis.Link[R_Link].info->force->Fbl = R_Link_Var->info->force->F*0.6f;
			Chassis.Leg->T_info->T_mat_storage[Tw_R] = 0.f;
			Chassis.Leg->T_info->T_mat_storage[Sd_R] *= 0.f;
		}
		else
		{
			R_Link_Var->link_pid->phi0_speed_cal->target = 0.f;
			R_Link_Var->link_pid->phi0_speed_cal->measure = R_Link_Var->info->angle->vir_phi0_d1;
			pid_err_cal(R_Link_Var->link_pid->phi0_speed_cal);
			single_pid_ctrl(R_Link_Var->link_pid->phi0_speed_cal);
			Chassis.Leg->T_info->T_mat_storage[Sd_R] = 0;
			Chassis.Link[R_Link].info->force->Fbl = R_Link_Var->link_pid->phi0_speed_cal->out;
			Chassis.Leg->T_info->T_mat_storage[Tw_R] = 0;
		}
		
		
	}
//	
//倒数第二步
		
		
//最后一步（[Sd_R]不需要置0）
////		Chassis.Leg->T_info->T_mat_storage[Sd_R] = 0;
//		Chassis.Link[R_Link].info->force->Fbl = 0;
//		Chassis.Leg->T_info->T_mat_storage[Tw_R] = 1;
//		Chassis.Link[R_Link].info->force->Fbl = (Link_Var->info->length->l0 - 0.36f)*200.f + Link_Var->info->force->G_support*3.f;
//		Chassis.Leg->T_info->T_mat_storage[Sd_R] = 0;
//		Chassis.Leg->T_info->T_mat_storage[Tw_R] = 0;
}


/**
  * @brief  云台自救处理
  * @param  None
  * @retval None
  */
void Gimbal_Rescue_Process(void)
{
	Four_Bar_Link_t* R_Link_Var = &Chassis.Link[R_Link];
	Four_Bar_Link_t* L_Link_Var = &Chassis.Link[L_Link];
	
	static uint8_t cnt = 0;
	
	if(Balance.command->gimbal->Rescue_Step == 0)
	{
		R_Link_Var->link_pid->phi0_speed_cal->target = -3.f;
		R_Link_Var->link_pid->phi0_speed_cal->measure = R_Link_Var->info->angle->vir_phi0_d1;
		pid_err_cal(R_Link_Var->link_pid->phi0_speed_cal);
		single_pid_ctrl(R_Link_Var->link_pid->phi0_speed_cal);//
		Chassis.Leg->T_info->T_mat_storage[Sd_R] = R_Link_Var->link_pid->phi0_speed_cal->out-R_Link_Var->info->force->G_torque;
		Chassis.Link[R_Link].info->force->Fbl = 0;
		Chassis.Leg->T_info->T_mat_storage[Tw_R] = 0;
		
		L_Link_Var->link_pid->phi0_speed_cal->target = -3.f;
		L_Link_Var->link_pid->phi0_speed_cal->measure = L_Link_Var->info->angle->vir_phi0_d1;
		pid_err_cal(L_Link_Var->link_pid->phi0_speed_cal);
		single_pid_ctrl(L_Link_Var->link_pid->phi0_speed_cal);//
		Chassis.Leg->T_info->T_mat_storage[Sd_L] = L_Link_Var->link_pid->phi0_speed_cal->out-L_Link_Var->info->force->G_torque;
		Chassis.Link[L_Link].info->force->Fbl = 0;
		Chassis.Leg->T_info->T_mat_storage[Tw_L] = 0;
		
		if(((R_Link_Var->link_pid->phi0_speed_cal->err <= -3.0f && 
			(R_Link_Var->info->angle->vir_phi0_ <= -40.f && R_Link_Var->info->angle->vir_phi0_ >= -90.f)))&&
			(((L_Link_Var->link_pid->phi0_speed_cal->err <= -3.0f && 
		(L_Link_Var->info->angle->vir_phi0_ <= -40.f && L_Link_Var->info->angle->vir_phi0_ >=  -90.f)))))
		{
			cnt ++;
			if(cnt >= 10)
			{
				Balance.command->gimbal->Rescue_Step = 1;
			}
		}
		else
		{
			Balance.command->gimbal->Rescue_Step = 0;
			cnt = 0;
		}
	}
	else if(Balance.command->gimbal->Rescue_Step == 1)
	{
		R_Link_Var->link_pid->phi0_speed_cal->target = -3.f;
		R_Link_Var->link_pid->phi0_speed_cal->measure = R_Link_Var->info->angle->vir_phi0_d1;
		pid_err_cal(R_Link_Var->link_pid->phi0_speed_cal);
		single_pid_ctrl(R_Link_Var->link_pid->phi0_speed_cal);//
		Chassis.Leg->T_info->T_mat_storage[Sd_R] = R_Link_Var->link_pid->phi0_speed_cal->out-R_Link_Var->info->force->G_torque;
		Chassis.Link[R_Link].info->force->Fbl = R_Link_Var->info->force->F;
		Chassis.Leg->T_info->T_mat_storage[Tw_R] = 0;
		
		L_Link_Var->link_pid->phi0_speed_cal->target = -3.f;
		L_Link_Var->link_pid->phi0_speed_cal->measure = L_Link_Var->info->angle->vir_phi0_d1;
		pid_err_cal(L_Link_Var->link_pid->phi0_speed_cal);
		single_pid_ctrl(L_Link_Var->link_pid->phi0_speed_cal);//
		Chassis.Leg->T_info->T_mat_storage[Sd_L] = L_Link_Var->link_pid->phi0_speed_cal->out-L_Link_Var->info->force->G_torque;
		Chassis.Link[L_Link].info->force->Fbl = L_Link_Var->info->force->F;
		Chassis.Leg->T_info->T_mat_storage[Tw_L] = 0;
	}
}

/**
  * @brief  撞膝上台阶，状态检测
  * @param  None
  * @retval None
  */
void Knee_Strike_Check(Chassis_t* My_Chassis)
{
	Four_Bar_Link_t* My_L_Link = &My_Chassis->Link[L_Link];
	Four_Bar_Link_t* My_R_Link = &My_Chassis->Link[R_Link];
	Straight_Leg_t* My_Leg = My_Chassis->Leg;
	static uint8_t cnt = 0;
	static uint16_t over_cnt = 0;
	static uint8_t flag = 0;
	if(Balance.command->chassis->Knee_Strike_Flag == true)
	{
		flag = 1;
		switch(Balance.command->chassis->Knee_Strike_step)
		{
			case 0:
				over_cnt ++;
			if(over_cnt > 5000)
			{
				Balance.command->chassis->Knee_Strike_Flag = false;
			}
			
			if(My_Leg->info->thetab > 0.25f)
			{
				cnt ++;
				if(cnt > 5)
				{
					Chassis.target->distance += 0.10f;
				Balance.command->chassis->Knee_Strike_step ++;
					Balance.last_mode = Sleep_Mode;
					cnt = 0;
				}
			}
			else
			{
				cnt = 0;
			}
			break;
			case 1:
			over_cnt = 0;
			if(((My_L_Link->info->length->l0+My_R_Link->info->length->l0)*0.5f) < 0.16f)
			{
				cnt ++;
				if(cnt > 5)
				{
				Balance.command->chassis->Knee_Strike_step = 2;
					cnt = 0;
				}
			}
			else
			{
				cnt = 0;
			}
			break;
			case 2:
			over_cnt ++;
			if((My_L_Link->info->angle->vir_phi0 < 0.15f && My_R_Link->info->angle->vir_phi0 < 0.15f) || over_cnt > 1500)
			{
				cnt ++;
				
				if(cnt > 5 || over_cnt > 1000)
				{
				Balance.command->chassis->Knee_Strike_step = 0;
					Balance.command->chassis->Knee_Strike_Flag = false;
					cnt = 0;
				}
			}
			else
			{
				cnt = 0;
			}
			break;
		}
	}
	else
	{
		if(flag == 1)
		{
			if(My_Chassis->target->leg_length > TAR_LEG_LENGTH_INITIAL)
			{
				My_Chassis->target->leg_length -=   10*MAX_LIFT_SPEED * TIME_STEP;
			}
			else if(My_Chassis->target->leg_length < TAR_LEG_LENGTH_INITIAL)
			{
				My_Chassis->target->leg_length +=   10*MAX_LIFT_SPEED * TIME_STEP;
			}
			else
			{
				flag = 0;
			}
			cnt = 0;
			over_cnt = 0;
		}
	}
}


/**
  * @brief  撞膝上台阶，腿长目标值处理
  * @param  None
  * @retval None
  */
void Knee_Strike_Target_Leg(Chassis_t* My_Chassis)
{
	switch(Balance.command->chassis->Knee_Strike_step)
	{
		case 0:
		if(My_Chassis->target->leg_length <= 0.32f)
		{
			My_Chassis->target->leg_length +=  6.f * MAX_LIFT_SPEED * TIME_STEP;
		}
		
		break;
		case 1:
		My_Chassis->target->leg_length = 0.15f;
		break;
		case 2:
		My_Chassis->target->leg_length = 0.15f;
		break;
	}

}

pid_ctrl_t My_Link_Phi2_Pid[Link_Num] = 
{
	[R_Link] = 
	{
		.kp = 20.f,//
    .ki = 0.f,
    .kd = 700.f,
		.a = 0.5f,
    .integral_max = 2.f,
    .out_max = 200.f,//
	},
	[L_Link] = 
	{
		.kp = 20.f,
    .ki = 0.f,
    .kd = 700.f,
		.a = 0.5f,
    .integral_max = 2.f,
    .out_max = 200.f,//
	},
};
/**
  * @brief  撞膝上台阶，力矩处理
  * @param  None
  * @retval None
  */
void Knee_Strike_React(Chassis_t* My_Chassis)
{
	Four_Bar_Link_t* My_L_Link = &My_Chassis->Link[L_Link];
	Four_Bar_Link_t* My_R_Link = &My_Chassis->Link[R_Link];
	Straight_Leg_t* My_Leg = My_Chassis->Leg;
	switch(Balance.command->chassis->Knee_Strike_step)
	{
		case 0:

		break;
		case 1:
		/*左腿*/
		My_Link_Phi2_Pid[L_Link].measure = My_L_Link->info->angle->phi2 + My_Chassis->Posture->info->pitch;
		My_Link_Phi2_Pid[L_Link].target = 1.3f;
		pid_err_cal(&My_Link_Phi2_Pid[L_Link]);
		single_pid_ctrl(&My_Link_Phi2_Pid[L_Link]);
		My_L_Link->info->F_Sd_Output_Torque = -My_Link_Phi2_Pid[L_Link].out;
		My_L_Link->info->B_Sd_Output_Torque = -7.f;
		
		/*右腿*/
		My_Link_Phi2_Pid[R_Link].measure = My_R_Link->info->angle->phi2 + My_Chassis->Posture->info->pitch;
		My_Link_Phi2_Pid[R_Link].target = 1.3f;
		pid_err_cal(&My_Link_Phi2_Pid[R_Link]);
		single_pid_ctrl(&My_Link_Phi2_Pid[R_Link]);
		My_R_Link->info->F_Sd_Output_Torque = My_Link_Phi2_Pid[R_Link].out;
		My_R_Link->info->B_Sd_Output_Torque = 7.f;
		break;
		case 2:
		
		My_L_Link->info->force->Fbl = My_L_Link->info->force->F;
		My_Leg->T_info->T_mat_storage[Sd_L] = -12*(My_L_Link->info->angle->vir_phi0);
		My_R_Link->info->force->Fbl = My_R_Link->info->force->F;
		My_Leg->T_info->T_mat_storage[Sd_R] = -12*(My_R_Link->info->angle->vir_phi0);
		My_L_Link->torque_cal(My_L_Link, My_L_Link->info->force->Fbl, My_Leg->T_info->T_mat_storage[Sd_L], L_Link);
		My_R_Link->torque_cal(My_R_Link, My_R_Link->info->force->Fbl, My_Leg->T_info->T_mat_storage[Sd_R], R_Link);
		break;
	}
}
