#include "motor.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;


pid_ctrl_t wheel_speed_pid[WHEEL_CNT] = {
	[WHEEL_LF] = {
		.kp = 1,
	  .ki = 0,
	  .kd = 0,
	  .integral_max = 0,
	  .out_max = 5.4,
	
	},
	[WHEEL_LB] = {
		.kp = 1,
	  .ki = 0,
	  .kd = 0,
	  .integral_max = 0,
	  .out_max = 5.4,
	
	},
	[WHEEL_RF] = {
		.kp = 1,
	  .ki = 0,
	  .kd = 0,
	  .integral_max = 0,
	  .out_max = 5.4,
	
	},
	[WHEEL_RB] = {
		.kp = 1,
	  .ki = 0,
	  .kd = 0,
	  .integral_max = 0,
	  .out_max = 5.4,
	
	},

};

pid_ctrl_t wheel_angle_inn_pid[WHEEL_CNT] = {
	[WHEEL_LF] = {
		.kp = 0,
	  .ki = 0,
	  .kd = 0,
	  .integral_max = 0,
	  .out_max = 0,
	
	},
	[WHEEL_LB] = {
		.kp = 0,
	  .ki = 0,
	  .kd = 0,
	  .integral_max = 0,
	  .out_max = 0,
	
	},
	[WHEEL_RF] = {
		.kp = 0,
	  .ki = 0,
	  .kd = 0,
	  .integral_max = 0,
	  .out_max = 0,
	
	},
	[WHEEL_RB] = {
		.kp = 0,
	  .ki = 0,
	  .kd = 0,
	  .integral_max = 0,
	  .out_max = 0,
	
	},

};

pid_ctrl_t wheel_angle_out_pid[WHEEL_CNT] = {
	[WHEEL_LF] = {
		.kp = 0,
	  .ki = 0,
	  .kd = 0,
	  .integral_max = 0,
	  .out_max = 3.8,
	
	},
	[WHEEL_LB] = {
		.kp = 0,
	  .ki = 0,
	  .kd = 0,
	  .integral_max = 0,
	  .out_max = 3.8,
	
	},
	[WHEEL_RF] = {
		.kp = 0,
	  .ki = 0,
	  .kd = 0,
	  .integral_max = 0,
	  .out_max = 3.8,
	
	},
	[WHEEL_RB] = {
		.kp = 0,
	  .ki = 0,
	  .kd = 0,
	  .integral_max = 0,
	  .out_max = 3.8,
	
	},

};


rm_cfg_t wheel_born_info[WHEEL_CNT] = {
	[WHEEL_LF] = {
		.order_correction = 0,
		.rxId = 0,
		.stdId = 0x200,
	  .type = _3508_Reduction,
	  .hcan = &hfdcan1
	
	},
	[WHEEL_LB] = {
		.order_correction = 0,
		.rxId = 1,
		.stdId = 0x200,
	  .type = _3508_Reduction,
	  .hcan = &hfdcan1,
	
	},
	[WHEEL_RF] = {
		.order_correction = 0,
		.rxId = 2,
		.stdId = 0x200,
	  .type = _3508_Reduction,
	  .hcan = &hfdcan1,
	
	},
	[WHEEL_RB] = {
		.order_correction = 0,
		.rxId = 3,
		.stdId = 0x200,
	  .type = _3508_Reduction,
	  .hcan = &hfdcan1,
	
	},

};


rm_rx_t   wheel_rx_info[WHEEL_CNT];
rm_tx_t   wheel_tx_info[WHEEL_CNT];
rm_state_t     wheel_state[WHEEL_CNT];
rm_ctrl_t wheel_ctrl_info[WHEEL_CNT] = {
	[WHEEL_LF] = {
	  .angle_ctrl_inner = &wheel_angle_inn_pid[WHEEL_LF],
		.angle_ctrl_outer = &wheel_angle_out_pid[WHEEL_LF],
	  .speed_ctrl = &wheel_speed_pid[WHEEL_LF],
	},
	[WHEEL_LB] = {
	  .angle_ctrl_inner = &wheel_angle_inn_pid[WHEEL_LB],
		.angle_ctrl_outer = &wheel_angle_out_pid[WHEEL_LB],
	  .speed_ctrl = &wheel_speed_pid[WHEEL_LB],
	},
	[WHEEL_RF] = {
	  .angle_ctrl_inner = &wheel_angle_inn_pid[WHEEL_RF],
		.angle_ctrl_outer = &wheel_angle_out_pid[WHEEL_RF],
	  .speed_ctrl = &wheel_speed_pid[WHEEL_RF],
	},
	[WHEEL_RB] = {
	  .angle_ctrl_inner = &wheel_angle_inn_pid[WHEEL_RB],
		.angle_ctrl_outer = &wheel_angle_out_pid[WHEEL_RB],
	  .speed_ctrl = &wheel_speed_pid[WHEEL_RB],
	},

};


rm_motor_t wheel_motor[WHEEL_CNT] = {
	[WHEEL_LF] = {
	 .born_info = &wheel_born_info[WHEEL_LF],
	 .rx_info = &wheel_rx_info[WHEEL_LF],
	 .tx_info = &wheel_tx_info[WHEEL_LF],
	 .state = &wheel_state[WHEEL_LF],
	 .ctrl = &wheel_ctrl_info[WHEEL_LF],
	 .single_init = rm_motor_init,	
	},
	[WHEEL_LB] = {
	 .born_info = &wheel_born_info[WHEEL_LB],
	 .rx_info = &wheel_rx_info[WHEEL_LB],
	 .tx_info = &wheel_tx_info[WHEEL_LB],
	 .state = &wheel_state[WHEEL_LB],
	 .ctrl = &wheel_ctrl_info[WHEEL_LB],
	 .single_init = rm_motor_init,	
	},
	[WHEEL_RF] = {
	 .born_info = &wheel_born_info[WHEEL_RF],
	 .rx_info = &wheel_rx_info[WHEEL_RF],
	 .tx_info = &wheel_tx_info[WHEEL_RF],
	 .state = &wheel_state[WHEEL_RF],
	 .ctrl = &wheel_ctrl_info[WHEEL_RF],
	 .single_init = rm_motor_init,	
	},
	[WHEEL_RB] = {
	 .born_info = &wheel_born_info[WHEEL_RB],
	 .rx_info = &wheel_rx_info[WHEEL_RB],
	 .tx_info = &wheel_tx_info[WHEEL_RB],
	 .state = &wheel_state[WHEEL_RB],
	 .ctrl = &wheel_ctrl_info[WHEEL_RB],
	 .single_init = rm_motor_init,	
	},
	

};


rm_group_t wheel_group = {
	.motor[WHEEL_LF] = &wheel_motor[WHEEL_LF],
	.motor[WHEEL_LB] = &wheel_motor[WHEEL_LB],
	.motor[WHEEL_RF] = &wheel_motor[WHEEL_RF],
	.motor[WHEEL_RB] = &wheel_motor[WHEEL_RB],
	.stdId = 0x200,
  .hcan = &hfdcan1,
  .group_init = rm_group_init,


};
void rm_motor_list_init()
{
	/*电机信息初始化*/
	wheel_group.group_init(&wheel_group);
}

void kt_motor_list_init()
{
	
	
	
}
void dm_motor_list_init()
{
	
}

void ht_motor_list_init()
{
	
	
}

void rm_motor_list_heart_beat()
{
  wheel_group.group_heartbeat(&wheel_group);
}




