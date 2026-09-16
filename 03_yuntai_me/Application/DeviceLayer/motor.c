
/* Includes ------------------------------------------------------------------*/
#include "motor.h"


/* Private variables ---------------------------------------------------------*/
// ���ӵ��ʱ��Ҫ��ʼ���Ĳ�����3508����6020��CAN1����CAN2���Լ�����Ľ���ID��
// CAN���������������CAN1����CAN2�����������±��ڵ��Motor_SendData�жϡ�
//

drv_can_t rm_motor_driver[] = {
	[GIMB_P] = {
		.can_id = DRV_CAN2,
		.rx_id = ID_GIMB_P,  //0x204+���id��2
	}
};
//����ID�ͽ���ID�����Ե�������,���Ļ������ĵĵ�һ���ֽ������Ƿ��͸�����ID�����Ļ�������ID���Ե�������
drv_can_t ht_motor_drive={
		.rx_id = 0x0B,
		.tx_id =0x09,
		.can_id = DRV_CAN1,
};



/*PID�ṹ�嶨��------------------------------------------------*/
// ע�ⶨ����֮����Ҫ��rm_motor_list_init��rm_motor_pid_init��ʼ��

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
/*HT_start*/
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

Motor_HT_Born_Info_t L_Wheel_Born_Info = 
{	
	.stdId = 0x009,//������Ʊ���ID
	.hcan = &hcan1,//ʹ�õ�Can����
	.order_correction = 0,//����ܽǶȵ�������Ϊ˳ʱ��
};
Motor_HT_Rx_Info_t L_Wheel_Rx_Info_t;
Motor_HT_Tx_Info_t L_Wheel_Tx_Info_t;
Motor_HT_State_t L_Wheel_State_t;
Motor_HT_t L_Wheel = 
{
	.born_info = &L_Wheel_Born_Info,
	
	.rx_info = &L_Wheel_Rx_Info_t,
	
	.tx_info = &L_Wheel_Tx_Info_t,
	
	.state = &L_Wheel_State_t,
	
	.single_init = &HT_Single_Motor_Init,
};
/*HT_end*/

/*DM_start*/
Motor_DM_Born_Info_t Yaw_Born_Info =
{
    .stdId = 0x02,
    .hcan = &hcan2,

};

Motor_DM_Rx_Info_t Yaw_Rx_Info_t;

Motor_DM_Tx_Info_t Yaw_Tx_Info_t;

Motor_DM_State_t Yaw_State_t;

Motor_DM_t Yaw_Motor = 
{
	.born_info = &Yaw_Born_Info,
	
	.rx_info = &Yaw_Rx_Info_t,
	
	.tx_info = &Yaw_Tx_Info_t,
	
	.state = &Yaw_State_t,
	
	.single_init = &DM_Single_Motor_Init,
};

Motor_DM_Born_Info_t Pitch_Born_Info =
{
    .stdId = 0x01,
    .hcan = &hcan1,
};

Motor_DM_Rx_Info_t Pitch_Rx_Info_t;
Motor_DM_Tx_Info_t Pitch_Tx_Info_t;
Motor_DM_State_t Pitch_State_t;

Motor_DM_t Pitch_Motor =
{
    .born_info = &Pitch_Born_Info,
    .rx_info = &Pitch_Rx_Info_t,
    .tx_info = &Pitch_Tx_Info_t,
    .state = &Pitch_State_t,
    .single_init = &DM_Single_Motor_Init,
};

Motor_DM_t dm_motor[] =
{
    [YAW] =
    {
        .born_info = &Yaw_Born_Info,
        .rx_info = &Yaw_Rx_Info_t,
        .tx_info = &Yaw_Tx_Info_t,
        .state = &Yaw_State_t,
        .single_init = &DM_Single_Motor_Init,
    },
    [PITCH] =
    {
        .born_info = &Pitch_Born_Info,
        .rx_info = &Pitch_Rx_Info_t,
        .tx_info = &Pitch_Tx_Info_t,
        .state = &Pitch_State_t,
        .single_init = &DM_Single_Motor_Init,
    },
};

Motor_DM_Group_t DM_Group =
{
    .motor[YAW] = &dm_motor[YAW],
    .motor[PITCH] = &dm_motor[PITCH],
    .motor[2] = NULL,
    .motor[3] = NULL,
    .group_init = Group_Motor_Init,
};
/*DM_end*/

/*RM START*/
Motor_RM_Born_Info_t R_Fric_Born = 
{
	.rxId = 0,
	
	.hcan = &hcan1,
	
	.type = _6020_Single,
	
	.stdId = 0x1FE,
};

Motor_RM_Tx_Info_t R_Fric_Tx;

Motor_RM_State_t R_Fric_State;

Motor_RM_Rx_Info_t R_Fric_Rx;

pid_ctrl_t R_Fric_Speed_Ctrl = 
{
	.kp = 10.f,//
	.ki = 0.2f,
	.kd = 0.f,
	.integral_max = 6000.f,
	.out_max = 8000.f,//
};

Motor_RM_Ctrl_Info_t R_Fric_Ctrl = 
{
	.speed_ctrl = &R_Fric_Speed_Ctrl,
};

Motor_RM_t R_Fric = 
{
	.born_info = &R_Fric_Born,
	
	.rx_info = &R_Fric_Rx,
	
	.tx_info = &R_Fric_Tx,

  .state = &R_Fric_State,
	
	.single_init = RM_Motor_Init,
	
	.ctrl = &R_Fric_Ctrl,
};

Motor_RM_Group_t RM_Group =
{
	.motor[0] = &R_Fric,
	
	.motor[1] = NULL,
	
	.motor[2] = NULL,
	
	.motor[3] = NULL,
	
	.stdId=0x1FE,
	
	.hcan=&hcan1,
	
	.group_init = RM_Group_Motor_Init,
};

/*RM END*/
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

/* Exported functions --------------------------------------------------------*/
void rm_motor_list_init()
{
	/*�����Ϣ��ʼ��*/
	R_Fric.single_init(&R_Fric);
	RM_Group.group_init(&RM_Group);
}

void kt_motor_list_init()
{
	kt_motor[0].init(&kt_motor[0]);
	
	
}
void dm_motor_list_init()
{
    DM_Group.group_init(&DM_Group);
}

void dm_motor_list_heart_beat()
{
    DM_Group.group_heartbeat(&DM_Group);
}

void ht_motor_list_init()
{
	L_Wheel.single_init(&L_Wheel);
	
}

void rm_motor_list_heart_beat()
{
	RM_Group.group_heartbeat(&RM_Group);
}


