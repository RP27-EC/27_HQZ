#include "motor.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

Motor_DM_Born_Info_t Pitch_Born_Info = {
    .stdId = 0x01,
    .hcan = &hcan1,
    .order_correction = 0,
};

Motor_DM_Rx_Info_t Pitch_Rx_Info;
Motor_DM_Tx_Info_t Pitch_Tx_Info;
Motor_DM_State_t Pitch_State;

Motor_DM_t Pitch_Motor = {
    .born_info = &Pitch_Born_Info,
    .rx_info = &Pitch_Rx_Info,
    .tx_info = &Pitch_Tx_Info,
    .state = &Pitch_State,
    .single_init = &DM_Single_Motor_Init,
};

Motor_DM_Born_Info_t Yaw_Born_Info = {
    .stdId = 0x02,
    .hcan = &hcan2,
    .order_correction = 0,
};

Motor_DM_Rx_Info_t Yaw_Rx_Info;
Motor_DM_Tx_Info_t Yaw_Tx_Info;
Motor_DM_State_t Yaw_State;

Motor_DM_t Yaw_Motor = {
    .born_info = &Yaw_Born_Info,
    .rx_info = &Yaw_Rx_Info,
    .tx_info = &Yaw_Tx_Info,
    .state = &Yaw_State,
    .single_init = &DM_Single_Motor_Init,
};

void dm_motor_list_init(void)
{
    Pitch_Motor.single_init(&Pitch_Motor);
    Yaw_Motor.single_init(&Yaw_Motor);
}

void dm_motor_list_heart_beat(void)
{
    Pitch_Motor.single_heart_beat(&Pitch_Motor);
    Yaw_Motor.single_heart_beat(&Yaw_Motor);
}
