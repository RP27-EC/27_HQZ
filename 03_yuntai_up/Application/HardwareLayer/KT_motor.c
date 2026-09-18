#include "KT_motor.h"

/* Exported variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/* Private functions ---------------------------------------------------------*/
void KT_motor_class_heartbeat(KT_motor_t *motor);
void kt_motor_class_pid_init(KT_motor_t *motor);


void get_kt_motor_info(KT_motor_t *motor, uint8_t *rxBuf);  
void tx_kt_motor_W_command(KT_motor_t *motor, uint8_t command);
void tx_kt_motor_R_command(KT_motor_t *motor, uint8_t command);


//璁剧疆PID鍙傛暟
void write_kt_motor_pid_param(KT_motor_t *motor, uint8_t* buff);

//鍐欏姞閫熷害鍙傛暟
void write_kt_motor_accel_param(KT_motor_t *motor, int32_t accel);

//鐢垫満闆剁偣
void write_kt_motor_encoderOffset_param(KT_motor_t *motor, uint16_t encoderOffset);

//杈撳嚭鍔熺巼鎺у埗
void write_kt_motor_powerControl_param(KT_motor_t *motor, int16_t powerControl);

//杞煩闂幆鎺у埗
void write_kt_motor_iqControl_param(KT_motor_t *motor, int16_t iqControl);

//閫熷害闂幆鎺у埗
void write_kt_motor_speedControl_param(KT_motor_t *motor, int32_t speedControl);

//澶氬湀闂幆瑙掑害鎺у埗
void write_kt_motor_angle_sum_Control_param(KT_motor_t    *motor, 
																						int32_t       angle_sum_Control,
	                                          uint16_t      angle_sum_Control_maxSpeed);

//鍗曞湀闂幆瑙掑害鎺у埗
void write_kt_motor_angle_single_Control_param(KT_motor_t   *motor, 
																	 uint16_t     angle_single_Control,
																	 uint8_t   	  angle_single_Control_spinDirection,
																	 uint16_t			angle_single_Control_maxSpeed);

//澧為噺寮忚搴︽帶鍒�
void write_kt_motor_angle_add_Control_param(KT_motor_t   *motor, 
																						int32_t      angle_add_Control,
																			      uint16_t     angle_add_Control_maxSpeed);

/* Exported functions --------------------------------------------------------*/


/**
 *	@brief	鐢垫満鍒濆鍖栵紝璐熻矗涓€浜涘弬鏁拌祴鍊笺€佸嚱鏁版寚閽堣祴鍊笺€佹竻闆跺彂閫佹暟缁勩€佹竻闆跺彂閫佺殑缁撴瀯浣撳弬鏁�
 */
void KT_motor_class_init(KT_motor_t *motor)
{
	
	if(motor == NULL)
		return;
	
	memset ( (uint8_t*)motor->tx_buff, 0, 8 );
	memset ( &motor->KT_motor_info.tx_info, 0, 8);
	
	motor->KT_motor_info.state_info.init_flag         = M_INIT;
	motor->KT_motor_info.state_info.offline_cnt_max   = OFFLINE_LINE_CNT_MAX;
	motor->KT_motor_info.state_info.selfprotect_cnt_max   = SELFPROTECT_CNT_MAX;	
	motor->KT_motor_info.state_info.offline_cnt      	 = 	0;
	motor->KT_motor_info.state_info.selfprotect_cnt       = 0;
	motor->KT_motor_info.state_info.work_state        = M_OFFLINE;	
	motor->KT_motor_info.state_info.selfprotect_flag  = M_PROTECT_OFF;
	

	motor->heartbeat  = KT_motor_class_heartbeat;
	
	motor->get_info = get_kt_motor_info;
	motor->tx_W_cmd = tx_kt_motor_W_command;
	motor->tx_R_cmd = tx_kt_motor_R_command;
	
	motor->W_pid                  = write_kt_motor_pid_param;
	motor->W_accel                = write_kt_motor_accel_param;
	motor->W_encoderOffset        = write_kt_motor_encoderOffset_param;
	motor->W_powerControl         = write_kt_motor_powerControl_param;
	motor->W_iqControl            = write_kt_motor_iqControl_param;
	motor->W_speedControl         = write_kt_motor_speedControl_param;
	motor->W_angle_sum_Control    = write_kt_motor_angle_sum_Control_param;
	motor->W_angle_single_Control = write_kt_motor_angle_single_Control_param;
	motor->W_angle_add_Control    = write_kt_motor_angle_add_Control_param;
	
}

/**
 *	@brief	鐢垫満蹇冭烦锛屽鏋滃彂鐢熷け鑱旓紝涓嬩竴娆℃敹鍒版暟鎹椂锛宱ffline_cnt浼氬湪鐢垫満鏇存柊涓繘琛屾竻闆�
 */
void KT_motor_class_heartbeat(KT_motor_t *motor)
{	
	static int16_t current_last;
	if(motor == NULL)	
		return;
			
	KT_motor_state_info_t *state_info = &motor->KT_motor_info.state_info;

	if(state_info->init_flag == M_DEINIT)
	{
		state_info->work_state = M_INIT_ERR;
		return;
	}
		
	state_info->offline_cnt++;
	//鍙戣繃鏉ョ殑鐢垫祦涓€鐩寸浉鍚屽垽鏂负杩涘叆鐢垫祦淇濇姢
	if(motor->KT_motor_info.rx_info.current==current_last)
	{
		state_info->selfprotect_cnt++;
	}
	current_last=motor->KT_motor_info.rx_info.current;
	
	if(state_info->offline_cnt > state_info->offline_cnt_max) 
	{
		state_info->offline_cnt = state_info->offline_cnt_max;
		state_info->work_state = M_OFFLINE;
	}
	else 
	{
		if(state_info->work_state == M_OFFLINE)
			state_info->work_state = M_ONLINE;
	}
	
	if(state_info->selfprotect_cnt > state_info->selfprotect_cnt_max) 
	{
		state_info->selfprotect_cnt = state_info->selfprotect_cnt_max;
		state_info->selfprotect_flag = M_PROTECT_ON;
	//	motor->tx_W_cmd(motor,MOTOR_RUN_ID);
	}
	else 
	{
		if(state_info->selfprotect_flag == M_PROTECT_ON)
		{
			state_info->selfprotect_flag = M_PROTECT_OFF;
			state_info->selfprotect_cnt =0;
		}
			
	}
}


/**
 *	@brief	鍒濆鍖栫數鏈虹殑PID鍙戦€併€佹帴鏀剁粨鏋勪綋
 */
void kt_motor_class_pid_init(KT_motor_t *motor)
{
	if(motor == NULL)	
		return;

	KT_motor_pid_t *pid = &motor->KT_motor_info.pid_info;
	
	pid->init_flag     = M_INIT;
	
	pid->rx.angleKp    = 0;
	pid->rx.angleKi    = 0;
	pid->rx.speedKp    = 0;
	pid->rx.speedKi    = 0;
	pid->rx.iqKp       = 0;
	pid->rx.iqKi       = 0;

	
	pid->tx.angleKp    = 10;
	pid->tx.angleKi    = 1;
	pid->tx.speedKp    = 10;
	pid->tx.speedKi    = 1;
	pid->tx.iqKp       = 1;
	pid->tx.iqKi       = 1;

}	


/*--------------------------澶氱數鏈哄懡浠や笉闇€瑕佸啀鏁扮粍涓～鍏ュ懡浠�-------------------------*/


/** 
 *	@brief 澶氱數鏈烘帶鍒讹紝闇€瑕佸湪澶栭儴鎶婃渶澶�4涓數鏈虹殑鎵煩鐢垫祦鎸夌収ID鍙风殑椤哄簭缁勬垚涓€涓暟缁�
					 濡傛灉浼犲叆鐨勭數鏈烘暟灏忎簬4锛屼細鑷姩妫€鏌ヤ紶鍏ョ殑鏁扮粍鍚庨儴鍒嗘槸鍚︽槸0锛屽鏋滀笉鏄�0锛屼細璁剧疆涓�0
 */
void kt_motor_multi_control(int16_t* iqControl, char kt_motor_num, motor_drive_e drive_type)
{
	//鍒ゆ柇鐢垫祦
	for(int i = 0; i < kt_motor_num; i ++)
	{
		if( within_or_not(iqControl[i], -KT_TX_IQ_CONTROL_MAX, KT_TX_IQ_CONTROL_MAX) == Flase )
			return;
	}
	
	for(int i = kt_motor_num; i < 4 ; i ++)
	{
		if( iqControl[i] != 0 )
			iqControl[i] = 0;
	}
	
	uint8_t tx_buff[8] = {0};
	
	tx_buff[0] = (uint8_t) iqControl[0];
	tx_buff[1] = (uint8_t) (iqControl[0] >> 8);
	tx_buff[2] = (uint8_t) iqControl[1];
	tx_buff[3] = (uint8_t) (iqControl[1] >> 8);
	tx_buff[4] = (uint8_t) iqControl[2];
	tx_buff[5] = (uint8_t) (iqControl[2] >> 8);
	tx_buff[6] = (uint8_t) iqControl[3];
	tx_buff[7] = (uint8_t) (iqControl[3] >> 8);
	
	if(drive_type == M_CAN1)
	{
		CAN_SendData(&hcan1, KT_MULTI_TX_ID, tx_buff);
	}
	else if(drive_type == M_CAN2)
	{
		CAN_SendData(&hcan2, KT_MULTI_TX_ID, tx_buff);
	}
	else
		return;
	
}


/*---------------------------浠ヤ笅鍑芥暟閮芥槸閽堝鍗曠數鏈烘帶鍒�----------------------------*/
/** 
 *	@brief 缁欑數鏈哄彂閫佹湁鍏�  鍐欏弬鏁版垨鑰呮兂瑕佺殑鎺у埗妯″紡鐨勫懡浠わ紝鍐呴儴宸茬粡鏈塩an鐨勫彂閫佸嚱鏁�
 */
void tx_kt_motor_W_command(KT_motor_t *motor, uint8_t command)
{
	
	if( motor == NULL )
		return;
	
	KT_motor_pid_t       *pid      = &motor->KT_motor_info.pid_info;
	
	KT_motor_tx_info_t   *tx_info  = &motor->KT_motor_info.tx_info;
	
	memset( (uint8_t*)motor->tx_buff, 0, 8 );
	
	switch(command)
	{
		case PID_TX_RAM_ID:  //鍐橮ID鍙傛暟鍒癛AM
			motor->tx_buff[0] = PID_TX_RAM_ID;
			motor->tx_buff[1] = 0x00;
		  motor->tx_buff[2] = pid->tx.angleKp;
			motor->tx_buff[3] = pid->tx.angleKi;
			motor->tx_buff[4] = pid->tx.speedKp;
			motor->tx_buff[5] = pid->tx.speedKi;
			motor->tx_buff[6] = pid->tx.iqKp;
			motor->tx_buff[7] = pid->tx.iqKi;
		break;
		
		case PID_TX_ROM_ID:  //鍐橮ID鍙傛暟鍒癛OM
			motor->tx_buff[0] = PID_TX_RAM_ID;
			motor->tx_buff[1] = 0x00;
		  motor->tx_buff[2] = pid->tx.angleKp;
			motor->tx_buff[3] = pid->tx.angleKi;
			motor->tx_buff[4] = pid->tx.speedKp;
			motor->tx_buff[5] = pid->tx.speedKi;
			motor->tx_buff[6] = pid->tx.iqKp;
			motor->tx_buff[7] = pid->tx.iqKi;
		break;
		
		case ACCEL_TX_ID:  //鍐欏姞閫熷害鍒癛AM
			motor->tx_buff[0] = ACCEL_TX_ID;
			motor->tx_buff[1] = 0x00;
		  motor->tx_buff[2] = 0x00;
			motor->tx_buff[3] = 0x00;
			motor->tx_buff[4] = (uint8_t) tx_info->accel;
			motor->tx_buff[5] = (uint8_t) (tx_info->accel >> 8);
			motor->tx_buff[6] = (uint8_t) (tx_info->accel >> 16);
			motor->tx_buff[7] = (uint8_t) (tx_info->accel >> 24);
		break;
		
		case ZERO_ENCODER_TX_ID:  //鍐欑紪鐮佸櫒鍊煎埌ROM浣滀负鐢垫満闆剁偣
			motor->tx_buff[0] = ZERO_ENCODER_TX_ID;
			motor->tx_buff[1] = 0x00;
		  motor->tx_buff[2] = 0x00;
			motor->tx_buff[3] = 0x00;
			motor->tx_buff[4] = 0x00;
			motor->tx_buff[5] = 0x00;
			motor->tx_buff[6] = (uint8_t) tx_info->encoderOffset;
			motor->tx_buff[7] = (uint8_t) (tx_info->encoderOffset >> 8);
		break;
		
		case ZERO_POSNOW_TX_ID:  //鍐欏綋鍓嶄綅缃€煎埌ROM浣滀负鐢垫満闆剁偣锛屽噺灏戜娇鐢�
			motor->tx_buff[0] = ZERO_POSNOW_TX_ID;
			motor->tx_buff[1] = 0x00;
		  motor->tx_buff[2] = 0x00;
			motor->tx_buff[3] = 0x00;
			motor->tx_buff[4] = 0x00;
			motor->tx_buff[5] = 0x00;
			motor->tx_buff[6] = (uint8_t) tx_info->encoderOffset;
			motor->tx_buff[7] = (uint8_t) (tx_info->encoderOffset >> 8);
		break;
		
		case MOTOR_CLOSE_ID:     //鐢垫満鍏抽棴鍛戒护锛屾竻闄よ繍琛岀姸鎬佸拰涔嬪墠鏀跺埌鐨勬寚浠�
			motor->tx_buff[0] = MOTOR_CLOSE_ID;
		break;
		
		case MOTOR_STOP_ID:     //鐢垫満鍋滄鍛戒护锛屾竻闄よ繍琛岀姸鎬佸拰涔嬪墠鏀跺埌鐨勬寚浠�
			motor->tx_buff[0] = MOTOR_STOP_ID;
		break;
		
		case MOTOR_RUN_ID:      //鐢垫満杩愯锛屼粠鍋滄涓仮澶�
			motor->tx_buff[0] = MOTOR_RUN_ID;
		break;
		
		case TORQUE_OPEN_LOOP_ID:  //寮€鐜浆鐭╂帶鍒讹紝鎺у埗杈撳嚭鍔熺巼
			motor->tx_buff[0] = TORQUE_OPEN_LOOP_ID;
			motor->tx_buff[1] = 0x00;
		  motor->tx_buff[2] = 0x00;
			motor->tx_buff[3] = 0x00;
			motor->tx_buff[4] = (uint8_t) tx_info->powerControl;
			motor->tx_buff[5] = (uint8_t) (tx_info->powerControl >> 8);
			motor->tx_buff[6] = 0x00;
			motor->tx_buff[7] = 0x00;
		break;
		
		case TORQUE_CLOSE_LOOP_ID:  //闂幆杞煩鎺у埗锛屾帶鍒舵壄鐭╃數娴�
			motor->tx_buff[0] = TORQUE_CLOSE_LOOP_ID;
			motor->tx_buff[1] = 0x00;
		  motor->tx_buff[2] = 0x00;
			motor->tx_buff[3] = 0x00;
			motor->tx_buff[4] = *(uint8_t*) (&tx_info->iqControl);
			motor->tx_buff[5] = *((uint8_t*)(&tx_info->iqControl)+1);
			motor->tx_buff[6] = 0x00;
			motor->tx_buff[7] = 0x00;
		break;
		
		case SPEED_CLOSE_LOOP_ID:    //閫熷害闂幆鎺у埗
			motor->tx_buff[0] = SPEED_CLOSE_LOOP_ID;
			motor->tx_buff[1] = 0x00;
		  motor->tx_buff[2] = 0x00;
			motor->tx_buff[3] = 0x00;
			motor->tx_buff[4] = (uint8_t) tx_info->speedControl;
			motor->tx_buff[5] = (uint8_t) (tx_info->speedControl >> 8);
			motor->tx_buff[6] = (uint8_t) (tx_info->speedControl >> 16);
			motor->tx_buff[7] = (uint8_t) (tx_info->speedControl >> 24);
		break;
		
		case POSI_CLOSE_LOOP_ID1:    //瑙掑害鎬诲拰闂幆锛岄€熷害涓嶉檺鍒�
			motor->tx_buff[0] = POSI_CLOSE_LOOP_ID1;
			motor->tx_buff[1] = 0x00;
		  motor->tx_buff[2] = 0x00;
			motor->tx_buff[3] = 0x00;
			motor->tx_buff[4] = (uint8_t) tx_info->angle_sum_Control;
			motor->tx_buff[5] = (uint8_t) (tx_info->angle_sum_Control >> 8);
			motor->tx_buff[6] = (uint8_t) (tx_info->angle_sum_Control >> 16);
			motor->tx_buff[7] = (uint8_t) (tx_info->angle_sum_Control >> 24);
		break;
		
		case POSI_CLOSE_LOOP_ID2:    //瑙掑害鎬诲拰闂幆锛岄€熷害闄愬埗
			motor->tx_buff[0] = POSI_CLOSE_LOOP_ID2;
			motor->tx_buff[1] = 0x00;
		  motor->tx_buff[2] = (uint8_t) tx_info->angle_sum_Control_maxSpeed;
			motor->tx_buff[3] = (uint8_t) (tx_info->angle_sum_Control_maxSpeed >> 8);
			motor->tx_buff[4] = (uint8_t) tx_info->angle_sum_Control;
			motor->tx_buff[5] = (uint8_t) (tx_info->angle_sum_Control >> 8);
			motor->tx_buff[6] = (uint8_t) (tx_info->angle_sum_Control >> 16);
			motor->tx_buff[7] = (uint8_t) (tx_info->angle_sum_Control >> 24);
		break;
		
		case POSI_CLOSE_LOOP_ID3:    //鍗曞湀瑙掑害锛屾湁鏂瑰悜
			motor->tx_buff[0] = POSI_CLOSE_LOOP_ID3;
			motor->tx_buff[1] = (uint8_t) tx_info->angle_single_Control_spinDirection;
		  motor->tx_buff[2] = 0x00;
			motor->tx_buff[3] = 0x00;
			motor->tx_buff[4] = (uint8_t) tx_info->angle_single_Control;
			motor->tx_buff[5] = (uint8_t) (tx_info->angle_single_Control >> 8);
			motor->tx_buff[6] = 0x00;
			motor->tx_buff[7] = 0x00;
		break;
		
		case POSI_CLOSE_LOOP_ID4:    //鍗曞湀瑙掑害锛屾湁鏂瑰悜锛屾湁鏈€楂樿浆閫�
			motor->tx_buff[0] = POSI_CLOSE_LOOP_ID4;
			motor->tx_buff[1] = (uint8_t) tx_info->angle_single_Control_spinDirection;
		  motor->tx_buff[2] = (uint8_t) tx_info->angle_single_Control_maxSpeed;
			motor->tx_buff[3] = (uint8_t) (tx_info->angle_single_Control_maxSpeed >> 8);
			motor->tx_buff[4] = (uint8_t) tx_info->angle_single_Control;
			motor->tx_buff[5] = (uint8_t) (tx_info->angle_single_Control >> 8);
			motor->tx_buff[6] = 0x00;
			motor->tx_buff[7] = 0x00;
		break;
		
		case POSI_CLOSE_LOOP_ID5:    //瑙掑害澧為噺锛屾棤閫熷害闄愬埗
			motor->tx_buff[0] = POSI_CLOSE_LOOP_ID5;
			motor->tx_buff[1] = 0x00;
		  motor->tx_buff[2] = 0x00;
			motor->tx_buff[3] = 0x00;
			motor->tx_buff[4] = (uint8_t) tx_info->angle_add_Control;
			motor->tx_buff[5] = (uint8_t) (tx_info->angle_add_Control >> 8);
			motor->tx_buff[6] = (uint8_t) (tx_info->angle_add_Control >> 16);
			motor->tx_buff[7] = (uint8_t) (tx_info->angle_add_Control >> 24);
		break;
				
		case POSI_CLOSE_LOOP_ID6:    //瑙掑害澧為噺锛屾湁閫熷害闄愬埗
			motor->tx_buff[0] = POSI_CLOSE_LOOP_ID6;
			motor->tx_buff[1] = 0x00;
		  motor->tx_buff[2] = (uint8_t) tx_info->angle_add_Control_maxSpeed;
			motor->tx_buff[3] = (uint8_t) (tx_info->angle_add_Control_maxSpeed >> 8);
			motor->tx_buff[4] = (uint8_t) tx_info->angle_add_Control;
			motor->tx_buff[5] = (uint8_t) (tx_info->angle_add_Control >> 8);
			motor->tx_buff[6] = (uint8_t) (tx_info->angle_add_Control >> 16);
			motor->tx_buff[7] = (uint8_t) (tx_info->angle_add_Control >> 24);
		break;
		
		
		default:
			break;
	}
	
	if(motor->KT_motor_info.id.drive_type == M_CAN1)
	{
		CAN_SendData(&hcan1, motor->KT_motor_info.id.tx_id, motor->tx_buff);
	}
	else if(motor->KT_motor_info.id.drive_type == M_CAN2)
	{
		CAN_SendData(&hcan2, motor->KT_motor_info.id.tx_id, motor->tx_buff);
	}
	else
		return;
}
	



/** 
 *	@brief 缁欑數鏈哄彂閫佷富鍔ㄨ鍙栨煇浜涘弬鏁扮殑鍛戒护锛屽唴閮ㄥ凡缁忔湁can鐨勫彂閫佸嚱鏁�
 */
void tx_kt_motor_R_command(KT_motor_t *motor, uint8_t command)
{
	if( motor == NULL )
		return;
	
	memset( (uint8_t*)motor->tx_buff, 0, 8 );
	
	switch(command)
	{
		case PID_RX_ID:   	//璇诲彇鍙戦€丳ID缁撴瀯浣撳弬鏁�
			motor->tx_buff[0] = PID_RX_ID;
		break;
		
		case ACCEL_RX_ID:   //璇诲彇鍙戦€佺殑缁撴瀯浣撲腑鐨勫姞閫熷害鍙傛暟
			motor->tx_buff[0] = ACCEL_RX_ID;
		break;
		
		case ENCODER_RX_ID:   //璇诲彇鍙戦€佺粨鏋勪綋涓殑缂栫爜鍣ㄦ暟鎹�
			motor->tx_buff[0] = ENCODER_RX_ID;
		break;
		
		case MOTOR_ANGLE_ID:  //璇诲彇鐢垫満澶氬湀缁濆瑙掑害
		 motor->tx_buff[0] = MOTOR_ANGLE_ID;
		break;
		
		case CIRCLE_ANGLE_ID:  //璇诲彇鐢垫満鍗曞湀瑙掑害
			motor->tx_buff[0] = CIRCLE_ANGLE_ID;
		break;
		
		case STATE1_ID:        //璇诲彇鐢垫満鐘舵€�1鍜岄敊璇爣蹇椾綅
			motor->tx_buff[0] = STATE1_ID;
		break;
		
		case STATE2_ID:        //璇诲彇鐢垫満鐘舵€�2
			motor->tx_buff[0] = STATE2_ID;
		break;
		
		case STATE3_ID:        //璇诲彇鐢垫満鐘舵€�3
			motor->tx_buff[0] = STATE3_ID;
		break;
		
		default:
			break;
	}
	
	if(motor->KT_motor_info.id.drive_type == M_CAN1)
	{
		CAN_SendData(&hcan1, motor->KT_motor_info.id.tx_id, motor->tx_buff);
	}
	else if(motor->KT_motor_info.id.drive_type == M_CAN2)
	{
		CAN_SendData(&hcan2, motor->KT_motor_info.id.tx_id, motor->tx_buff);
	}
	else
		return;
}



/**
 *	@brief	鎺ユ敹鐢垫満鍙戞潵鐨勪俊鎭苟鑷姩鏇存柊锛岄渶瑕佹敞鎰忥紝澶ч儴鍒嗗彂閫佺粰鐢垫満鐨勬寚浠わ紝鐢垫満涔熶細杩斿洖涓€浜涙暟鎹�
						濡傛灉浼犲叆绌烘寚閽堬紝璁や负鐢垫満鏁版嵁鍑洪敊锛屽苟杩斿洖
						鏈€鍚庨潰浼氭牴鎹帴鏀剁粨鏋勪綋鐨別rrorState鍒ゆ柇鏄惁瑕佽嚜鎴戜繚鎶�
 *  @return
 */
void get_kt_motor_info(KT_motor_t *motor, uint8_t *rxBuf)
{
	if( motor == NULL || rxBuf == NULL )
	{
		motor->KT_motor_info.state_info.work_state = M_DATA_ERR;
		return;
	}	
	
	uint8_t ID = rxBuf[0];
	
	KT_motor_pid_rx_info_t *pid_rx_info = &motor->KT_motor_info.pid_info.rx;
	KT_motor_rx_info_t     *rx_info     = &motor->KT_motor_info.rx_info;
	KT_motor_state_info_t  *state_info  = &motor->KT_motor_info.state_info;
	
	state_info->offline_cnt = 0;
	state_info->work_state = M_ONLINE;
	
	switch (ID)
	{
		case PID_RX_ID:                      //涓诲姩璇诲彇PID
			pid_rx_info->angleKp = rxBuf[2];
			pid_rx_info->angleKi = rxBuf[3];
			pid_rx_info->speedKp = rxBuf[4];
			pid_rx_info->speedKi = rxBuf[5];
			pid_rx_info->iqKp	   = rxBuf[6];
			pid_rx_info->iqKi	   = rxBuf[7];
		break;
		
		case PID_TX_RAM_ID:                  //鍙戦€丳ID鍙傛暟鍒癛AM鏃朵細杩斿洖
			pid_rx_info->angleKp = rxBuf[2];
			pid_rx_info->angleKi = rxBuf[3];
			pid_rx_info->speedKp = rxBuf[4];
			pid_rx_info->speedKi = rxBuf[5];
			pid_rx_info->iqKp	   = rxBuf[6];
			pid_rx_info->iqKi	   = rxBuf[7];
		break;
		
		case PID_TX_ROM_ID:                  //鍙戦€丳ID鍙傛暟鍒癛OM鏃朵細杩斿洖
			pid_rx_info->angleKp = rxBuf[2];
			pid_rx_info->angleKi = rxBuf[3];
			pid_rx_info->speedKp = rxBuf[4];
			pid_rx_info->speedKi = rxBuf[5];
			pid_rx_info->iqKp	   = rxBuf[6];
			pid_rx_info->iqKi	   = rxBuf[7];
		break;
		
		case ACCEL_RX_ID:                    //涓诲姩璇诲彇鍔犻€熷害
			rx_info->accel  = (int32_t)rxBuf[7];
			rx_info->accel <<= 8;
			rx_info->accel |= (int32_t)rxBuf[6];
			rx_info->accel <<= 8;
			rx_info->accel |= (int32_t)rxBuf[5];
			rx_info->accel <<= 8;
			rx_info->accel |= (int32_t)rxBuf[4];
			rx_info->accel <<= 8;
		break;
		
		case ACCEL_TX_ID:                    //鍙戦€佸姞閫熷害鍙傛暟鍒癛AM浼氳繑鍥�
			rx_info->accel  = (int32_t)rxBuf[7];
			rx_info->accel <<= 8;
			rx_info->accel |= (int32_t)rxBuf[6];
			rx_info->accel <<= 8;
			rx_info->accel |= (int32_t)rxBuf[5];
			rx_info->accel <<= 8;
			rx_info->accel |= (int32_t)rxBuf[4];
			rx_info->accel <<= 8;
		break;	
		
		case ENCODER_RX_ID:                   //涓诲姩璇诲彇缂栫爜鍣�
			rx_info->encoder = (uint16_t)rxBuf[3];
			rx_info->encoder <<= 8;
			rx_info->encoder |= (uint16_t)rxBuf[2];
			rx_info->encoderRaw = (uint16_t)rxBuf[5];
			rx_info->encoderRaw <<= 8;
			rx_info->encoderRaw |= (uint16_t)rxBuf[4];
			rx_info->encoderOffset = (uint16_t)rxBuf[7];
			rx_info->encoderOffset <<= 8;
			rx_info->encoderOffset |= (uint16_t)rxBuf[6];
		break;
		
		case ZERO_ENCODER_TX_ID:              //鍐欏叆缂栫爜鍣ㄥ€煎埌ROM浣滀负鐢垫満闆剁偣浼氳繑鍥�
			rx_info->encoderOffset = (uint16_t)rxBuf[7];
			rx_info->encoderOffset <<= 8;
			rx_info->encoderOffset |= (uint16_t)rxBuf[6];
		break;
		
		case ZERO_POSNOW_TX_ID:              //鍐欏叆缂栫爜鍣ㄥ€煎埌RAM浣滀负鐢垫満闆剁偣浼氳繑鍥�
			rx_info->encoderOffset = (uint16_t)rxBuf[7];
			rx_info->encoderOffset <<= 8;
			rx_info->encoderOffset |= (uint16_t)rxBuf[6];
		break;
		
		
		case MOTOR_ANGLE_ID:                  //涓诲姩璇诲彇鐢垫満澶氬湀缁濆瑙掑害锛屾鍊奸『鏃堕拡绱瑙掑害
			rx_info->motorAngle  = (int64_t)rxBuf[7];
			rx_info->motorAngle <<= 8;
			rx_info->motorAngle |= (int64_t)rxBuf[6];
			rx_info->motorAngle <<= 8;
			rx_info->motorAngle |= (int64_t)rxBuf[5];
			rx_info->motorAngle <<= 8;
			rx_info->motorAngle |= (int64_t)rxBuf[4];
			rx_info->motorAngle <<= 8;
			rx_info->motorAngle |= (int64_t)rxBuf[3];
			rx_info->motorAngle <<= 8;
			rx_info->motorAngle |= (int64_t)rxBuf[2];
			rx_info->motorAngle <<= 8;
			rx_info->motorAngle |= (int64_t)rxBuf[1];
		break;
		
		case CIRCLE_ANGLE_ID:                 //涓诲姩璇诲彇鐢垫満鍗曞湀瑙掑害
			rx_info->circleAngle  = (uint32_t)rxBuf[7];
			rx_info->circleAngle <<= 8;
			rx_info->circleAngle |= (uint32_t)rxBuf[6];
			rx_info->circleAngle <<= 8;
			rx_info->circleAngle |= (uint32_t)rxBuf[5];
			rx_info->circleAngle <<= 8;
			rx_info->circleAngle |= (uint32_t)rxBuf[4];
		break;
		
		case STATE1_ID:                       //涓诲姩璇诲彇鐢垫満鐘舵€�1鍜岄敊璇爣蹇椾綅
			rx_info->temperature = (int8_t)rxBuf[1]; 
			rx_info->voltage = (uint16_t)rxBuf[4];
			rx_info->voltage <<= 8;
			rx_info->voltage |= (uint16_t)rxBuf[3];
			rx_info->errorState = rxBuf[7];
		break;
		
		case STATE2_ID:                       //涓诲姩璇诲彇鐢垫満鐘舵€�2
			rx_info->temperature = (int8_t)rxBuf[1];
			rx_info->current = (int16_t)rxBuf[3];
			rx_info->current <<= 8;
			rx_info->current |= (int16_t)rxBuf[2];
			rx_info->speed = (int16_t)rxBuf[5];
			rx_info->speed <<= 8;
			rx_info->speed |= (int16_t)rxBuf[4];
			rx_info->encoder = (uint16_t)rxBuf[7];
			rx_info->encoder <<= 8;
			rx_info->encoder |= (uint16_t)rxBuf[6];
		break;
		
		case STATE3_ID:                        //涓诲姩璇诲彇鐢垫満鐘舵€�3
			rx_info->temperature = (int8_t)rxBuf[1];
			rx_info->current_A = (int16_t)rxBuf[3];
			rx_info->current_A <<= 8;
			rx_info->current_A |= (int16_t)rxBuf[2];
			rx_info->current_B = (int16_t)rxBuf[5];
			rx_info->current_B <<= 8;
			rx_info->current_B |= (int16_t)rxBuf[4];
			rx_info->current_C = (int16_t)rxBuf[7];
			rx_info->current_C <<= 8;
			rx_info->current_C |= (int16_t)rxBuf[6];
		break;
		
		case TORQUE_OPEN_LOOP_ID:              //鎵煩寮€鐜帶鍒朵細鑷姩杩斿洖
			rx_info->temperature = (int8_t)rxBuf[1]; 
			rx_info->powerControl = (int16_t)rxBuf[3];
			rx_info->powerControl <<= 8;	
			rx_info->powerControl |= (int16_t)rxBuf[2];
			rx_info->speed = (int16_t)rxBuf[5];
			rx_info->speed <<= 8;
			rx_info->speed |= (int16_t)rxBuf[4];
			rx_info->encoder = (uint16_t)rxBuf[7];
			rx_info->encoder <<= 8;
			rx_info->encoder |= (uint16_t)rxBuf[6];
		break;
		
		case  TORQUE_CLOSE_LOOP_ID:	//鎵煩闂幆鎺у埗銆侀€熷害闂幆銆佹墍鏈変綅缃棴鐜兘浼氳繑鍥�
		case  SPEED_CLOSE_LOOP_ID :
		case  POSI_CLOSE_LOOP_ID1 :
		case  POSI_CLOSE_LOOP_ID2 :
		case  POSI_CLOSE_LOOP_ID3 :
		case  POSI_CLOSE_LOOP_ID4 :	
		case  POSI_CLOSE_LOOP_ID5 :
		case  POSI_CLOSE_LOOP_ID6 :
			rx_info->temperature = (int8_t)rxBuf[1]; 
			rx_info->current = (int16_t)rxBuf[3];
			rx_info->current <<= 8;	
			rx_info->current |= (int16_t)rxBuf[2];
			rx_info->speed = (int16_t)rxBuf[5];
			rx_info->speed <<= 8;
			rx_info->speed |= (int16_t)rxBuf[4];
			rx_info->encoder = (uint16_t)rxBuf[7];
			rx_info->encoder <<= 8;
			rx_info->encoder |= (uint16_t)rxBuf[6];
		break;
		
		
		default:
			break;
	}

	
}

/**
 *	@brief鍐欑數鏈虹殑鍙戦€丳ID缁撴瀯浣撳弬鏁帮紝鏁村瀷鏁扮粍缁撴瀯{angleKp锛宎ngleKi锛宻peedKp锛宻peedKi锛宨qKp锛宨qKi锛�0锛�0}
 */
void write_kt_motor_pid_param(KT_motor_t *motor, uint8_t* buff)
{
	if(motor == NULL || buff == NULL)
		return;
	
	KT_motor_pid_t *pid = &motor->KT_motor_info.pid_info;
	
	pid->tx.angleKp    = buff[0];
	pid->tx.angleKi    = buff[1];
	pid->tx.speedKp    = buff[2];
	pid->tx.speedKi    = buff[3];
	pid->tx.iqKp       = buff[4];
	pid->tx.iqKi       = buff[5];
}


/**
 *	@brief鍐欑數鏈虹殑鍙戦€佺粨鏋勪綋鐨勫姞閫熷害鍙傛暟
 */

void write_kt_motor_accel_param(KT_motor_t *motor, int32_t accel)
{
	if(motor == NULL)
		return;
	
	motor->KT_motor_info.tx_info.accel = accel;
}


/** 
 *	@brief 鍐欑數鏈虹殑鍙戦€佺粨鏋勪綋鐨勭數鏈洪浂鐐瑰弬鏁�
 */

void write_kt_motor_encoderOffset_param(KT_motor_t *motor, uint16_t encoderOffset)
{
	if(motor == NULL)
		return;
	
	//鏃犵鍙锋暟鎹皟鐢╳ithin_or_not浼氭姤璀﹀憡
	if( encoderOffset > KT_TX_ENCODER_OFFSET_MAX )
	  return;
	
	motor->KT_motor_info.tx_info.encoderOffset = encoderOffset;
}

/** 
 *	@brief 鍐欑數鏈虹殑鍙戦€佺粨鏋勪綋鐨勮緭鍑哄姛鐜囨帶鍒剁洰鏍囧弬鏁�
 */


void write_kt_motor_powerControl_param(KT_motor_t *motor, int16_t powerControl)
{
	if(motor == NULL)
		return;
	
	if( within_or_not(powerControl, -KT_TX_POWER_CONTROL_MAX, KT_TX_POWER_CONTROL_MAX) == Flase )
	  return;
	
	motor->KT_motor_info.tx_info.powerControl = powerControl;
}

/** 
 *	@brief 鍐欑數鏈虹殑鍙戦€佺粨鏋勪綋鐨勬壄鐭╃數娴佹帶鍒剁洰鏍囧弬鏁�
 */


void write_kt_motor_iqControl_param(KT_motor_t *motor, int16_t iqControl)
{
	if(motor == NULL)
		return;
	
	iqControl = constrain(iqControl, -KT_TX_IQ_CONTROL_MAX, KT_TX_IQ_CONTROL_MAX);
	
	motor->KT_motor_info.tx_info.iqControl = iqControl;
}


/** 
 *	@brief 鍐欑數鏈虹殑鍙戦€佺粨鏋勪綋鐨勯€熷害鎺у埗鐩爣鍙傛暟
 */

void write_kt_motor_speedControl_param(KT_motor_t *motor, int32_t speedControl)
{
	if(motor == NULL)
		return;
		
	motor->KT_motor_info.tx_info.speedControl = speedControl;
}

/** 
 *	@brief 鍐欑數鏈虹殑鍙戦€佺粨鏋勪綋鐨勫鍦堣搴︽帶鍒舵秹鍙婂埌鐨勫弬鏁�
 */


void write_kt_motor_angle_sum_Control_param(KT_motor_t    *motor, 
																						int32_t       angle_sum_Control,
	                                          uint16_t      angle_sum_Control_maxSpeed)
{
	if(motor == NULL)
		return;
		
	motor->KT_motor_info.tx_info.angle_sum_Control = angle_sum_Control;
	
	motor->KT_motor_info.tx_info.angle_sum_Control_maxSpeed = angle_sum_Control_maxSpeed;
}


/** 
 *	@brief 鍐欑數鏈虹殑鍙戦€佺粨鏋勪綋鐨勫崟鍦堣搴︽帶鍒舵秹鍙婂埌鐨勫弬鏁�
 */

void write_kt_motor_angle_single_Control_param(KT_motor_t   *motor, 
											 uint16_t     angle_single_Control,
											 uint8_t   	  angle_single_Control_spinDirection,
										     uint16_t			angle_single_Control_maxSpeed)
{
//	if(motor == NULL)
//		return;
//	
//	//鏃犵鍙疯皟鐢╳ithin_or_not浼氭姤璀﹀憡
//	if( angle_single_Control > KT_TX_ANGLE_SIGNLE_MAX )
//		return;
//	
//	if( angle_single_Control_spinDirection != CLOCK_WISE ||
//	   	angle_single_Control_spinDirection != N_CLOCK_WISE )
//		return;
	
	
	motor->KT_motor_info.tx_info.angle_single_Control = angle_single_Control;
	
	motor->KT_motor_info.tx_info.angle_single_Control_spinDirection = angle_single_Control_spinDirection;
	
	motor->KT_motor_info.tx_info.angle_single_Control_maxSpeed = angle_single_Control_maxSpeed;
}

/** 
 *	@brief 鍐欑數鏈虹殑鍙戦€佺粨鏋勪綋鐨勮搴﹀閲忔帶鍒舵秹鍙婂埌鐨勫弬鏁�
 */

void write_kt_motor_angle_add_Control_param(KT_motor_t   *motor, 
																						int32_t      angle_add_Control,
																			      uint16_t     angle_add_Control_maxSpeed)
{
	if(motor == NULL)
		return;
	
	motor->KT_motor_info.tx_info.angle_add_Control = angle_add_Control;
	
	motor->KT_motor_info.tx_info.angle_add_Control_maxSpeed = angle_add_Control_maxSpeed;
	
}


