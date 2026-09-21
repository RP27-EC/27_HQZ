/* motor.c - 电机对象管理 */

#include "motor.h"
//

drv_can_t rm_motor_driver[] = {
	[GIMB_P] = {
		.can_id = DRV_CAN2,
		.rx_id = ID_GIMB_P, 
	}
};
drv_can_t ht_motor_drive={
		.rx_id = 0x0B,
		.tx_id =0x09,
		.can_id = DRV_CAN1,
};




motor_pid_t GIMB_P_mec = {
	.speed.kp = 0,
	.speed.ki = 0,
	.speed.kd = 0,
	.speed.integral_max = 3000,
	.speed.out_max = 28000,
	.angle.kp = 0, // 0.45
	.angle.ki = 0,
	.angle.kd = 0,
	.angle.integral_max = 0,
	.angle.out_max = 500,
};

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
/*HT_start*/
#if 0 /* Legacy HT motor: not used by the current gimbal board. */
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

ht_cfg_t L_Wheel_Born_Info = 
{	
	.stdId = 0x009,
	.hcan = &hcan1,
	.order_correction = 0,
};
ht_rx_t L_Wheel_Rx_Info_t;
ht_tx_t L_Wheel_Tx_Info_t;
ht_state_t L_Wheel_State_t;
ht_motor_t L_Wheel = 
{
	.born_info = &L_Wheel_Born_Info,
	
	.rx_info = &L_Wheel_Rx_Info_t,
	
	.tx_info = &L_Wheel_Tx_Info_t,
	
	.state = &L_Wheel_State_t,
	
	.single_init = &ht_motor_init,
};
/*HT_end*/
#endif

/*DM_start*/
dm_cfg_t Yaw_Born_Info =
{
    .stdId = 0x02,
    .hcan = &hcan2,

};

dm_rx_t Yaw_Rx_Info_t;

dm_tx_t Yaw_Tx_Info_t;

dm_state_t Yaw_State_t;

dm_motor_t Yaw_Motor = 
{
	.born_info = &Yaw_Born_Info,
	
	.rx_info = &Yaw_Rx_Info_t,
	
	.tx_info = &Yaw_Tx_Info_t,
	
	.state = &Yaw_State_t,
	
	.single_init = &dm_motor_init,
};

dm_cfg_t Pitch_Born_Info =
{
    .stdId = 0x01,
    .hcan = &hcan1,
};

dm_rx_t Pitch_Rx_Info_t;
dm_tx_t Pitch_Tx_Info_t;
dm_state_t Pitch_State_t;

dm_motor_t Pitch_Motor =
{
    .born_info = &Pitch_Born_Info,
    .rx_info = &Pitch_Rx_Info_t,
    .tx_info = &Pitch_Tx_Info_t,
    .state = &Pitch_State_t,
    .single_init = &dm_motor_init,
};

dm_motor_t dm_motor[] =
{
    [YAW] =
    {
        .born_info = &Yaw_Born_Info,
        .rx_info = &Yaw_Rx_Info_t,
        .tx_info = &Yaw_Tx_Info_t,
        .state = &Yaw_State_t,
        .single_init = &dm_motor_init,
    },
    [PITCH] =
    {
        .born_info = &Pitch_Born_Info,
        .rx_info = &Pitch_Rx_Info_t,
        .tx_info = &Pitch_Tx_Info_t,
        .state = &Pitch_State_t,
        .single_init = &dm_motor_init,
    },
};

dm_group_t DM_Group =
{
    .motor[YAW] = &dm_motor[YAW],
    .motor[PITCH] = &dm_motor[PITCH],
    .motor[2] = NULL,
    .motor[3] = NULL,
    .group_init = dm_group_init,
};
/*DM_end*/

/*RM START*/
#if 0 /* Legacy RM motor: not used by the current gimbal board. */
rm_cfg_t R_Fric_Born =
{
	.rxId = 0,
	
	.hcan = &hcan1,
	
	.type = _6020_Single,
	
	.stdId = 0x1FE,
};

rm_tx_t R_Fric_Tx;

rm_state_t R_Fric_State;

rm_rx_t R_Fric_Rx;

pid_ctrl_t R_Fric_Speed_Ctrl = 
{
	.kp = 10.f,//
	.ki = 0.2f,
	.kd = 0.f,
	.integral_max = 6000.f,
	.out_max = 8000.f,//
};

rm_ctrl_t R_Fric_Ctrl = 
{
	.speed_ctrl = &R_Fric_Speed_Ctrl,
};

rm_motor_t R_Fric = 
{
	.born_info = &R_Fric_Born,
	
	.rx_info = &R_Fric_Rx,
	
	.tx_info = &R_Fric_Tx,

  .state = &R_Fric_State,
	
	.single_init = RM_Motor_Init,
	
	.ctrl = &R_Fric_Ctrl,
};

rm_group_t RM_Group =
{
	.motor[0] = &R_Fric,
	
	.motor[1] = NULL,
	
	.motor[2] = NULL,
	
	.motor[3] = NULL,
	
	.stdId=0x1FE,
	
	.hcan=&hcan1,
	
	.group_init = rm_group_init,
};

/*RM END*/
#endif
#if 0 /* Legacy KT motor: not used by the current gimbal board. */
KT_motor_t kt_motor[] = {
	[0] = {
		.KT_motor_info = {
			.tx_info = {
				.angle_single_Control = 0,
				.angle_single_Control_maxSpeed = 0,
				.angle_single_Control_spinDirection = 0,
				.angle_add_Control = 0,
				.angle_add_Control_maxSpeed = 0,
				.angle_sum_Control = 0,
				.angle_sum_Control_maxSpeed = 0,
				.iqControl = 0,
				.speedControl = 0,
			},
			.id = {
				.tx_id = ID_GIMB_Y,
				.rx_id = 0x88,
				.drive_type = M_CAN1,
				.motor_type = KT9015,
			},
		},
		.init = KT_motor_class_init,
	},
};
#endif
#if 0 /* Legacy RM motor initialization disabled. */
void rm_motor_list_init()
{
	
	R_Fric.single_init(&R_Fric);
	RM_Group.group_init(&RM_Group);
}
#endif

#if 0 /* Legacy KT motor initialization disabled. */
void kt_motor_list_init()
{
	kt_motor[0].init(&kt_motor[0]);
	
	
}
#endif
void dm_motor_list_init()
{
    DM_Group.group_init(&DM_Group);
}

void dm_motor_list_heart_beat()
{
    DM_Group.group_heartbeat(&DM_Group);
}

#if 0 /* Legacy HT motor initialization disabled. */
void ht_motor_list_init()
{
	L_Wheel.single_init(&L_Wheel);
	
}
#endif

#if 0 /* Legacy RM motor heartbeat disabled. */
void rm_motor_list_heart_beat()
{
	RM_Group.group_heartbeat(&RM_Group);
}
#endif




