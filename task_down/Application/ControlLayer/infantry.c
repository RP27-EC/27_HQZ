/* infantry.c - 步兵整车控制 */

#include "infantry.h"
#include "board_protocol.h"
#include "rc_protocol.h"
#include "chassis.h"
#include "gimbal.h"
#include "launch.h" 
#include "vision.h"
#include "ui.h"
#include "cap.h"
#include "judge.h"

static void Infantry_Init(Infantry_t* infantry);
static void Rc_Status_Update(Infantry_t* infantry);
static void Key_Status_Update(Infantry_t* infantry);
static void Infantry_Flag_Clean(Infantry_t* infantry);
static void Infantry_Flag_Update(Infantry_t* infantry);
Signal_Form_e Spec_Flag_Update(Flag_Class_t* flag,uint8_t heartbeat,bool is_cnt);
static void Infantry_Status_Update(Infantry_t* infantry);
static void Infantry_Work(Infantry_t* infantry);
static void Infantry_Offline_Update(Infantry_t* infantry);


Infantry_t  infantry = {
	.ctrl = RC_CTRL,//默认遥控器控制
	.last_ctrl = RC_CTRL,//上次控制源
	.mode = I_SLEEP,//默认休眠模式
	.last_mode = I_SLEEP,//上次模式
	.flag = {
		.mec_flag = true,//机械标志位
	  .imu_flag = false,//陀螺仪标志位
    .turn_flag = false,//小陀螺标志位
	  .hole_flag = false,//狗洞标志位
	  .vision_flag = 0,//自瞄标志位
	  .broken_flag = false,//故障标志位
		.cap_use_flag = true,//超电使用标志位
		
		.chassis_off = false,//底盘掉电标志位
		.gimbal_off = false,//云台掉电标志位
		
		// 掉头与复位动作的计数器
		.U_turn_flag= {
			.value = false,
			.tick_max = 800,	
		},
		
	  .L_turn_flag = {
			.value = false,
			.tick_max = 800,			
		},
		
	  .R_turn_flag = {
			.value = false,
			.tick_max = 800,	
		},
			
		.chassis_reset = {
			.value = false,
			.tick_max = 1500,	
		
		},
		.car_reset = false,	
	
	},
	
	.init = Infantry_Init,


};


static void Infantry_Init(Infantry_t* infantry)
{
	//工作函数和心跳函数指针
	infantry->work = Infantry_Work;
	infantry->heart_beat = Infantry_Offline_Update;

}

static uint8_t last_thumbwheel_step[4];


/* 步兵主循环 */
static void Infantry_Work(Infantry_t* infantry)
{
	//更新整车模式状态
	Infantry_Status_Update(infantry);
	
	//控制任务
	chassis.work(&chassis);//底盘解算
	gimbal.work(&gimbal);//云台控制
	launch.work(&launch);//发射
	vision.work(&vision);//视觉部分
}

/* 遥控器状态刷新 */
static void Rc_Status_Update(Infantry_t* infantry)
{
	rc_data_t*  rc_info = rc_dev.info;
	// 左拨杆最上 + 右拨杆最下，切入键鼠控制
	if(rc_info->s1.value == RC_SW_UP && rc_info->s2.value == RC_SW_DOWN)                //左上右下进键鼠
	{
	  infantry->ctrl = KEY_CTRL;
	}
	else{
	  infantry->ctrl = RC_CTRL;
	}
	
	// 左上 + 右中：波轮向上拨一次，切换进入/退出狗洞模式
	if(rc_info->s1.value == RC_SW_UP && rc_info->s2.value == RC_SW_MID)
	{
		if(WHEEL_UP_TO_ONCE)
		{
			infantry->flag.hole_flag = !infantry->flag.hole_flag;    
					
			if(infantry->flag.hole_flag == true)     //这里只进狗洞模式，退狗洞模式时模式位暂时不切，等完全抬头再切
			{
				infantry->mode = I_HOLE;
				infantry->flag.chassis_reset.value = true;
			
			}
			else{
//				infantry->mode = I_MEC;
					
			}
	  }
	}
	else if(rc_info->s1.value == RC_SW_DOWN && rc_info->s2.value == RC_SW_DOWN)
	{
		//滚轮上滚切换超电预充模式
		if(WHEEL_UP_TO_ONCE)
		{
			cap_tx_info.bit_control.pre_charge_mode_en = !cap_tx_info.bit_control.pre_charge_mode_en;    //预充模式
		}
		//滚轮下滚软件复位
		else if(WHEEL_DOWN_TO_ONCE)
		{
			infantry->flag.car_reset = true;                 //软件复位
		}
	}
	
	
	//过洞是最高优先级，进过洞后不能切换其他模式，不能发射，不开视觉，除非退出狗洞
	if(infantry->mode != I_HOLE)
	{
		switch (rc_info->s1.value)
	  {
		  case  RC_SW_UP:
			
		    if(rc_info->s2.value == RC_SW_UP)
			  {
					//左上右上，滚轮上滚切换小陀螺模式
				  if(WHEEL_UP_TO_ONCE)
				  {
					  infantry->flag.turn_flag = !infantry->flag.turn_flag;
					
					  if(infantry->flag.turn_flag == true)
					  {
						  infantry->mode = I_TURN;//开启小陀螺
						  //infantry->mode = I_IMU;
						
    				}
    				else{
    					infantry->mode = I_IMU;//关闭小陀螺
    				
    				}
    			}
    			//左上右上，滚轮下滚切换机械模式
    			else if(WHEEL_DOWN_TO_ONCE)
    			{
    				infantry->flag.mec_flag = !infantry->flag.mec_flag;
    				
    				if(infantry->flag.mec_flag == true)
    				{
    					infantry->mode = I_MEC;
							cap_tx_info.bit_control.pre_charge_mode_en = 1;
    					
    				}
    				else{
              infantry->mode = I_IMU;
							cap_tx_info.bit_control.pre_charge_mode_en = 0;

    				}						
    			}
    		}
				
    		else if(rc_info->s2.value == RC_SW_MID)
    		{
					//左上右中，滚轮下滚掉头
    			if(WHEEL_DOWN_TO_ONCE)
    			{
    				if(infantry->flag.chassis_reset.value == false && infantry->flag.vision_flag == 0)   //底盘复位，狗洞模式下不得掉头
    				{
    					if(infantry->flag.U_turn_flag.value == false)
    				  {
    				    infantry->flag.U_turn_flag.value = true;
    			  	}
    				}
    		
    			}
    //			  else if(WHEEL_UP_TO_ONCE)
    //				{
    //					infantry->flag.hole_flag = !infantry->flag.hole_flag;    
    //					
    //					if(infantry->flag.hole_flag == true)     //这里只进狗洞模式，退狗洞模式时模式位暂时不切，等完全抬头再切
    //					{
    //						infantry->mode = I_HOLE;
    //						infantry->flag.chassis_reset.value = true;
    //			
    //					}
    //					else{
    ////						infantry->mode = I_MEC;
    //					
    //					}
    //				}
    			
    		}
    	
    		break;
    	
    	case  RC_SW_MID:
				//左中，滚轮上滚开摩擦轮
    		if(WHEEL_UP_TO_ONCE)
    		{
    			launch.state = 1 - launch.state;
    
    		}
    		
    		break;
    	
    	case  RC_SW_DOWN:
    		if(rc_info->s2.value == RC_SW_UP)
    		{
					//左下右上，滚轮上滚切换自瞄
    			if(WHEEL_UP_TO_ONCE)
    			{
    				if(infantry->flag.vision_flag != 1)
    				{
    				  infantry->flag.vision_flag = 1;
    					
    				}
    				else{
    					infantry->flag.vision_flag = 0;
    				
    				}
    			  
    			}
					//左下右上，滚轮下滚切换前哨
    			else if(WHEEL_DOWN_TO_ONCE)
    			{
    				if(infantry->flag.vision_flag != 4)
    				{
    				  infantry->flag.vision_flag = 4;
    				}
    				else{
    					infantry->flag.vision_flag = 0;
    				
    				}
    			}
    		}
    		else if(rc_info->s2.value == RC_SW_MID)
    		{
					//左下右中，滚轮上滚切换小符
    			if(WHEEL_UP_TO_ONCE)
    			{
    				if(infantry->flag.vision_flag != 2)
    				{
    				  infantry->flag.vision_flag = 2;
    				}
    				else{
    					infantry->flag.vision_flag = 0;
    				
    				}
    			}
					//左下右中，滚轮下滚切换大符
    			else if(WHEEL_DOWN_TO_ONCE)
    			{
    				if(infantry->flag.vision_flag != 3)
    				{
    				  infantry->flag.vision_flag = 3;
    				}
    				else{
    					infantry->flag.vision_flag = 0;
    				
    				}
    			}
    		}
//    		else if(rc_info->s2.value == RC_SW_DOWN)
//    		{
//					//左下右下，滚轮上滚切换预充模式
//    			if(WHEEL_UP_TO_ONCE)
//    			{
//    				cap_tx_info.bit_control.pre_charge_mode_en = !cap_tx_info.bit_control.pre_charge_mode_en;    //预充模式
//    			}
//					//左下右下，滚轮下滚软件复位
//    			else if(WHEEL_DOWN_TO_ONCE)
//    			{
//    				infantry->flag.car_reset = true;                 //软件复位
//    			}
//    		}
    		
    		break;
    	
    	default:
    		break;
    	
    }
    
		//一旦发射关闭则上锁
    if(launch.state == L_LOCK)
    {
    	launch.shoot_lock = 1;
    }
    else{
      if(launch.shoot_lock == 0)
      {
    	  if(rc_info->s1.status == mid_R || rc_info->s1.status == up_R || rc_info->s1.status == down_R)           //只要左拨杆是变回中间以及从中间离开都上锁，防止右拨杆在下面导致一瞬间连发
    	  {
    	    launch.shoot_lock = 1;
    	  }
      }
      if(launch.shoot_lock == 1)
      {
    	  if(rc_info->s1.value == RC_SW_MID && rc_info->s2.value == RC_SW_MID)        //只有左右拨杆都在中间才能接触发射锁
    	  {
    		  launch.shoot_lock = 0;
    	  }
      }
    			
    }
   // 遥控器射击动作处理（仅在左拨杆居中时有效）
    if(rc_info->s1.value == RC_SW_MID)
    {
    	if(rc_info->s2.value == RC_SW_UP)//右杠最上：单发
      {
    	  launch.mode = SINGLE_SHOT;
    	  launch.shoot_level = !launch.shoot_lock;
    		
    		shoot_statistics.shoot_mode = 0;
    	  shoot_statistics.shooting_flag=0;
    	  if(launch.state == L_UNLOCK && launch.shoot_lock == 0)
    	  {
    		  if(rc_info->s2.status == up_R)                            //单发跳变开始计时拨弹延迟
    		  {
    			  Shooting_Cmd_Excute_Tick_Calculating(0);
    		  }
      	}
      }
      else if(rc_info->s2.value == RC_SW_MID)
      {
    	  launch.mode = SINGLE_SHOT;
    	  launch.shoot_level = 0;
    		
    		shoot_statistics.shoot_mode = 0;
        shoot_statistics.shooting_flag = 0;
      }
      else if(rc_info->s2.value == RC_SW_DOWN)
      {
    	  launch.mode = REPEAT_SHOT;
    	  launch.shoot_level = !launch.shoot_lock;
    		
    		if(launch.state == L_UNLOCK && launch.shoot_lock == 0)
    	  {
    		  shoot_statistics.shoot_mode = 1;
    		  if(shoot_statistics.shooting_flag == 0 && rc_info->s2.status == keep_R)         //连发模式下shooting_flag为0时是第一次，此时计时，后面shooting_flag变1后不会再进入这里，计时在串口中断才开始
    		  {
    			  Shooting_Cmd_Excute_Tick_Calculating(0);
    		    shoot_statistics.shooting_flag = 1;
    	  	}
    	
    	  }
      }
    }
    else{
      launch.mode = SINGLE_SHOT;
    	 launch.shoot_level = 0;
    	
    	shoot_statistics.shoot_mode = 0;
      shoot_statistics.shooting_flag = 0;
    }
	}
	else{
		launch.state = L_LOCK;
    
    launch.shoot_lock = 1;
		launch.mode = SINGLE_SHOT;
    launch.shoot_level = 0;
		
		shoot_statistics.shoot_mode = 0;
    shoot_statistics.shooting_flag = 0;
	
		infantry->flag.vision_flag = 0;
	}
	
	
	infantry->flag.cap_use_flag = true;
	
	
	
	
	#if GIMBAL_SWITCH == 0
	  if(infantry->mode == I_HOLE)
		{
			infantry->mode = I_IMU;
		}
		
		infantry->flag.vision_flag = 0;
//		launch.state = L_LOCK;
		
	#else
	#endif
	

	last_thumbwheel_step[0] = rc_info->thumbwheel.step[0];
	last_thumbwheel_step[1] = rc_info->thumbwheel.step[1];
	last_thumbwheel_step[2] = rc_info->thumbwheel.step[2];
	last_thumbwheel_step[3] = rc_info->thumbwheel.step[3];

}



/* 按键状态刷新 */
static void Key_Status_Update(Infantry_t* infantry)
{
  rc_data_t*  rc_info = rc_dev.info;
	
	if(rc_info->s1.value == RC_SW_UP && rc_info->s2.value == RC_SW_DOWN)
	{
	  infantry->ctrl = KEY_CTRL;
		
	}
	else{
	  infantry->ctrl = RC_CTRL;
	}
	
	if(infantry->ctrl == KEY_CTRL && infantry->last_ctrl == RC_CTRL)    //进键鼠后如果是机械就自动变陀螺仪
	{
	  infantry->flag.cap_use_flag = false;

		cap_tx_info.bit_control.pre_charge_mode_en = 0;        //关预充模式
		
		if(infantry->mode == I_MEC)
		{
			infantry->mode = I_IMU;
		}
	}
	
	//只进过洞，最高优先级，按键V
	if(rc_info->V.status == release_to_press)                      
	{
		infantry->flag.hole_flag = true;
		infantry->flag.chassis_reset.value = true;    //底盘对齐云台               //底盘先复位
	  infantry->mode = I_HOLE;
	
	}
	
	if(infantry->mode != I_HOLE)       //其他模式切换必须不在过洞模式下
	{
		//小陀螺点击shift开启
//		if(rc_info->Shift.status == release_to_press)               
//	  {
//		  infantry->mode = I_TURN;

//	  }
		
		
		//小陀螺长按shift开启
		if(rc_info->Shift.status == short_press || rc_info->Shift.status == long_press)               
	  {
		  infantry->mode = I_TURN;

	  }
		else if(rc_info->Shift.status == press_to_release)
		{
			infantry->mode = I_IMU;
			infantry->flag.chassis_reset.value = true;   
		}
		
		

	
		//机械模式点击G开启
	  if(rc_info->G.status == release_to_press)
	  { 
			infantry->mode = I_MEC;
			cap_tx_info.bit_control.pre_charge_mode_en = 1;
						
	  }
	
		//按键 R：触发 180 度掉头（无复位且无自瞄时触发）
	  if(infantry->flag.chassis_reset.value == false && infantry->flag.vision_flag == 0)
	  {
	    if(infantry->flag.R_turn_flag.value == false && infantry->flag.L_turn_flag.value == false)
	    {
        if(rc_info->R.status == release_to_press)
	      {
		      if(infantry->flag.U_turn_flag.value == false)
		      {
			      infantry->flag.U_turn_flag.value = true;
		      }
	      }
	    }
	
//	    if(infantry->flag.U_turn_flag.value == false && infantry->flag.L_turn_flag.value == false)
//	    {
//        if(rc_info->E.status == release_to_press)
//	      {
//		      if(infantry->flag.R_turn_flag.value == false)
//		      {
//			      infantry->flag.R_turn_flag.value = true;
//		      }
//	      }
//	    }
//		
//	    if(infantry->flag.U_turn_flag.value == false && infantry->flag.R_turn_flag.value == false)
//	    {
//        if(rc_info->Q.status == release_to_press)
//	      {
//		      if(infantry->flag.L_turn_flag.value == false)
//		      {
//			      infantry->flag.L_turn_flag.value = true;
//		      }
//	      }
//	    }
	
	  }
	
	  //视觉2，3，4，5只能同时进一个，进去后屏蔽1
	  if(rc_info->Z.status == release_to_press)
	  {
		  if (judge.info ->game_status.stage_remain_time < (60 * 7 - 60 * 3)) // 根据比赛剩余时间自动判断大小符
        {
            infantry->flag.vision_flag = 3;//大符
        }
        else
        {
            infantry->flag.vision_flag = 2;//小符
        }
		 	
	  }
	  else if(rc_info->C.status == release_to_press)
	  {
			infantry->flag.vision_flag = 4;
	  }
	
	  if(infantry->flag.vision_flag <= 1)
	  {
		  if(rc_info->mouse_btn_r.status == short_press)
		  {
			  infantry->flag.vision_flag = 1;
		  }
		  if(rc_info->mouse_btn_r.cnt == 0)
		  {
		    infantry->flag.vision_flag = 0;
		  }
	  }
	
	
		//鼠标左键不按默认单发
	  if(rc_info->mouse_btn_l.cnt == 0)
	  {
		  launch.mode = SINGLE_SHOT;
		  launch.shoot_level = 0;
	  }
		//长按连发
	  else if(rc_info->mouse_btn_l.cnt >=150)
	  {
		  launch.mode = REPEAT_SHOT;
		  launch.shoot_level = 1;
	  }
	  else{
			//摩擦轮关闭时，单击左键开启
	    launch.shoot_level = 1;
		  if(launch.state != 1)
			  launch.state =1;
	  }
	
		//摩擦轮点击B切换开关
	  if(rc_info->B.status == release_to_press)
	  {
		  launch.state = 1 - launch.state;
	  }
	}
	else{
		//进过洞时锁定发射机构，视觉，不允许切换其他模式
		launch.state = L_LOCK;
		launch.mode = SINGLE_SHOT;
		launch.shoot_level = 0;
		
		infantry->flag.vision_flag = 0;
	}
	
	
	if(rc_info->F.status == release_to_press)
	{
		infantry->flag.cap_use_flag = !infantry->flag.cap_use_flag;

	}
	
	
	if(rc_info->Ctrl.status == release_to_press)         //一键取消所有特殊模式，如果是退出狗洞先抬头，完整退出才变陀螺仪
	{
		if(infantry->mode == I_HOLE)
		{
			infantry->flag.hole_flag = false;
		}
		else{
		  infantry->mode = I_IMU;
			
		if(infantry->flag.vision_flag >= 2)
		{
			infantry->flag.vision_flag = 0;
		}
		
		infantry->flag.chassis_reset.value = true;            //除狗洞模式外其余需要底盘复位
	  cap_tx_info.bit_control.pre_charge_mode_en = 0;

//	    infantry->flag.car_reast = true;
		}
		
	}
	
}

/* 复位步兵标志 */
static void Infantry_Flag_Clean(Infantry_t* infantry)
{
  infantry->flag.mec_flag = true;           //机械标志位不除，默认睡眠掉电，便于初始化
	infantry->flag.imu_flag = false;
  infantry->flag.turn_flag = false;
	infantry->flag.hole_flag = false;
	infantry->flag.vision_flag = 0;
	infantry->flag.broken_flag = false;
	
  infantry->flag.U_turn_flag.value = false;
	infantry->flag.L_turn_flag.value = false;
	infantry->flag.R_turn_flag.value = false;

	infantry->flag.chassis_reset.value = false;
	infantry->flag.car_reset = false;

}


static void Infantry_Flag_Update(Infantry_t* infantry)
{
	//根据主mode互斥更新子标志位
	if(infantry->mode == I_SLEEP || infantry->mode == I_INIT || infantry->mode == I_MEC)
	{
		infantry->flag.mec_flag = true;
		infantry->flag.imu_flag = false;
		infantry->flag.turn_flag = false;
		infantry->flag.hole_flag = false;
		
	}
	else if(infantry->mode == I_IMU)
	{
		infantry->flag.imu_flag = true;
		infantry->flag.mec_flag = false;
		infantry->flag.turn_flag = false;
		infantry->flag.hole_flag = false;
		
	}
	else if(infantry->mode == I_TURN)
	{
		infantry->flag.turn_flag = true;
		infantry->flag.mec_flag = false;
		infantry->flag.imu_flag = false;
		infantry->flag.hole_flag = false;
		
	}
	else if(infantry->mode == I_HOLE)
	{
//		infantry->flag.hole_flag = true;
		infantry->flag.mec_flag = false;
		infantry->flag.imu_flag = false;
		infantry->flag.turn_flag = false;
		
	}
	//狗洞模式下屏蔽视觉
	if(infantry->flag.hole_flag == true)
	{
		infantry->flag.vision_flag = 0;
	}

	//硬件损坏变机械模式运行
  if(infantry->flag.broken_flag == true)
	{
		if(infantry->mode != I_SLEEP && infantry->mode != I_INIT)
		{
			infantry->mode = I_MEC;
		}
		
		infantry->flag.mec_flag = true;
		infantry->flag.imu_flag = false;
		infantry->flag.turn_flag = false;
	  infantry->flag.hole_flag = false;
	}
	// 开启视觉时，底盘切为 IMU 模式进行辅助瞄准锁定
	if(infantry->flag.vision_flag != 0)
	{
		if(infantry->flag.mec_flag == true)
		{
			infantry->flag.mec_flag = false;
			infantry->flag.imu_flag = true;
			
			infantry->mode = I_IMU;
			
		}
	}
	// 周期性推进特殊限时动作的超时计数
	Spec_Flag_Update(&infantry->flag.U_turn_flag,(infantry->mode > I_INIT),true);
	Spec_Flag_Update(&infantry->flag.R_turn_flag,(infantry->mode > I_INIT),true);
	Spec_Flag_Update(&infantry->flag.L_turn_flag,(infantry->mode > I_INIT),true);
	Spec_Flag_Update(&infantry->flag.chassis_reset,(infantry->mode > I_INIT),true);
	
}


Signal_Form_e Spec_Flag_Update(Flag_Class_t* flag,uint8_t heartbeat,bool is_cnt)
{
	//未准备好，清空计数复位为flase
	if(heartbeat == 0)
	{
		flag->tick = 0;
			
		flag->value = false;
	}
	else if(flag->value == true)
	{
		//计数器自增，超过最大值则清零
		if(is_cnt == true)
		{
			flag->tick ++;
		
		  if(flag->tick >= flag->tick_max)
		  {
			  flag->tick = 0;
			
			  flag->value = false;
		  }
		}
		
	}
	else{
		flag->tick = 0;
	}
	//判定信号的物理波形
	if(flag->value == false && flag->last_value == false)
	{
		flag->form = LOWING;
	}
	else if(flag->value == true && flag->last_value == false)
	{
		flag->form = RISING;
	}
	else if(flag->value == true && flag->last_value == true)
	{
		flag->form = HIGHING;
	}
	else if(flag->value == false && flag->last_value == true)
	{
		flag->form = FALLING;
	}
	
	flag->last_value = flag->value;
	
	return flag->form;
}

/* 步兵状态刷新 */
static void Infantry_Status_Update(Infantry_t* infantry)
{
	static bool last_c_off = false;
	static bool last_g_off = false;
	
	rc_data_t*  rc_info = rc_dev.info;
	//安全保护
	if(rc_dev.work_state == DEV_OFFLINE)
	{
		board.tx_pkt->car_pkt.car_state = 0;

		infantry->mode = I_SLEEP;
		
		launch.state = L_LOCK;
		launch.shoot_lock = 1;
		infantry->flag.vision_flag = 0;
		
		cap_tx_info.bit_control.pre_charge_mode_en = 0;        //关预充模式
		
		Infantry_Flag_Clean(infantry);                         //清标志位
		
		last_thumbwheel_step[0] = rc_info->thumbwheel.step[0];
		last_thumbwheel_step[1] = rc_info->thumbwheel.step[1];
		last_thumbwheel_step[2] = rc_info->thumbwheel.step[2];
		last_thumbwheel_step[3] = rc_info->thumbwheel.step[3];
		
	}
	
	else{
		
		if(infantry->ctrl == RC_CTRL)
		{
      board.tx_pkt->car_pkt.car_state = 1;
		}
	  else if(infantry->ctrl == KEY_CTRL)
		{
      board.tx_pkt->car_pkt.car_state = 2;
	  }
	
	  if(infantry->mode == I_SLEEP)
	  {  
			//开控进初始化
		  infantry->mode = I_INIT;
		 
		  launch.state = L_LOCK;
			launch.shoot_lock = 1;
		  infantry->flag.vision_flag = 0;
			
			Infantry_Flag_Clean(infantry);
		  cap_tx_info.bit_control.pre_charge_mode_en = 0;        
	  }
	  else if(infantry->mode == I_INIT)
	  {
		  launch.state = L_LOCK;
			launch.shoot_lock = 1;
		  infantry->flag.vision_flag = 0;
		
//			if(infantry->flag.gimbal_off == true)
//	  	{
//			  infantry->mode = I_MEC;
//			
//	  	}
//			else if(infantry->flag.broken_flag == true)
//		  {
//			 	infantry->mode = I_MEC;
//		  }
//		  else 
			if(gimbal.gimbal_reset_flag == true)
			{  
			  infantry->mode = I_IMU;
			  //infantry->mode = I_MEC;
		  }
			
			//初始化时不接受滚轮改变
			 last_thumbwheel_step[0] = rc_info->thumbwheel.step[0];
			 last_thumbwheel_step[1] = rc_info->thumbwheel.step[1];
			 last_thumbwheel_step[2] = rc_info->thumbwheel.step[2];
			 last_thumbwheel_step[3] = rc_info->thumbwheel.step[3];
	  } 
	  else{
			//防滚轮跳变
		  if(infantry->last_mode == I_INIT)
		  {
			   last_thumbwheel_step[0] = rc_info->thumbwheel.step[0];
			   last_thumbwheel_step[1] = rc_info->thumbwheel.step[1];
			   last_thumbwheel_step[2] = rc_info->thumbwheel.step[2];
			   last_thumbwheel_step[3] = rc_info->thumbwheel.step[3];
		  }
			
		  if(infantry->ctrl == RC_CTRL)
		  {
			  Rc_Status_Update(infantry);
		  }
	    else if(infantry->ctrl == KEY_CTRL)
		  {
			  Key_Status_Update(infantry);
		  }
		  
			
			
	    if(infantry->flag.gimbal_off == true && last_g_off == false)
	  	{
			  infantry->mode = I_MEC;
	  	}
  	}
	}
	
	Infantry_Flag_Update(infantry);
	
	
	last_c_off = infantry->flag.chassis_off;
	last_g_off = infantry->flag.gimbal_off;
	
	infantry->last_ctrl = infantry->ctrl;
	infantry->last_mode = infantry->mode;
}


static void Infantry_Offline_Update(Infantry_t* infantry)
{
	chassis.heart_beat(&chassis);
	gimbal.heart_beat(&gimbal);
	launch.heart_beat(&launch);
	vision.heart_beat(&vision);
}





