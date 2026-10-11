/* motor.c - 电机对象管理 */

#include "motor.h"
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/* Yaw DM 电机配置 */
dm_cfg_t Yaw_Born_Info =
{
    .stdId = 0x02, // 控制帧标准ID，0~0x7FF
    .hcan = &hcan2, // 绑定CAN外设指针

};

dm_rx_t Yaw_Rx_Info_t;

dm_tx_t Yaw_Tx_Info_t;

dm_state_t Yaw_State_t;


/* Pitch DM 电机配置 */
dm_cfg_t Pitch_Born_Info =
{
    .stdId = 0x01, // 控制帧标准ID，0~0x7FF
    .hcan = &hcan1, // 绑定CAN外设指针
};

dm_rx_t Pitch_Rx_Info_t;
dm_tx_t Pitch_Tx_Info_t;
dm_state_t Pitch_State_t;


/* 云台 DM 电机索引表 */
dm_motor_t dm_motor[] =
{
    [YAW] =
    {
        .born_info = &Yaw_Born_Info, // 电机通信配置指针
        .rx_info = &Yaw_Rx_Info_t, // 反馈对象指针
        .tx_info = &Yaw_Tx_Info_t, // 命令对象指针
        .state = &Yaw_State_t, // 在线状态对象指针
        .single_init = &dm_motor_init, // 单电机初始化回调
    },
    [PITCH] =
    {
        .born_info = &Pitch_Born_Info, // 电机通信配置指针
        .rx_info = &Pitch_Rx_Info_t, // 反馈对象指针
        .tx_info = &Pitch_Tx_Info_t, // 命令对象指针
        .state = &Pitch_State_t, // 在线状态对象指针
        .single_init = &dm_motor_init, // 单电机初始化回调
    },
};

/* 云台两轴 DM 电机组 */
dm_group_t DM_Group =
{
    .motor[YAW] = &dm_motor[YAW], // 电机组槽位，0~3
    .motor[PITCH] = &dm_motor[PITCH], // 电机组槽位，0~3
    .motor[2] = NULL, // 电机组槽位，0~3
    .motor[3] = NULL, // 电机组槽位，0~3
    .group_init = dm_group_init, // 电机组初始化回调
};

/* 左摩擦轮 CAN 配置 */
rm_cfg_t L_Fric_Born =
{
    .rxId = 0, // 组内槽位，0~3
    .stdId = 0x200, // 控制帧标准ID，0~0x7FF
    .type = _3508_Single, // 电机型号枚举
    .hcan = &hcan1, // 绑定CAN外设指针
};

/* 右摩擦轮 CAN 配置 */
rm_cfg_t R_Fric_Born =
{
    .rxId = 1, // 组内槽位，0~3
    .stdId = 0x200, // 控制帧标准ID，0~0x7FF
    .type = _3508_Single, // 电机型号枚举
    .hcan = &hcan1, // 绑定CAN外设指针
};

rm_tx_t L_Fric_Tx;
rm_tx_t R_Fric_Tx;
rm_rx_t L_Fric_Rx;
rm_rx_t R_Fric_Rx;
rm_state_t L_Fric_State;
rm_state_t R_Fric_State;

pid_ctrl_t L_Fric_Speed_Ctrl =
{
    .kp = 0.0f, // 速度比例增益，输出/rpm
    .ki = 0.0f, // 速度积分增益，输出/累加rpm
    .kd = 0.0f, // 速度微分增益，输出/rpm差
    .integral_max = 0.0f, // 累加误差上限，rpm
    .out_max = 0.0f, // 速度环输出限幅，原始量
};

pid_ctrl_t R_Fric_Speed_Ctrl =
{
    .kp = 0.0f, // 速度比例增益，输出/rpm
    .ki = 0.0f, // 速度积分增益，输出/累加rpm
    .kd = 0.0f, // 速度微分增益，输出/rpm差
    .integral_max = 0.0f, // 累加误差上限，rpm
    .out_max = 0.0f, // 速度环输出限幅，原始量
};

rm_ctrl_t L_Fric_Ctrl =
{
    .speed_ctrl = &L_Fric_Speed_Ctrl, // 速度PID对象指针
};

rm_ctrl_t R_Fric_Ctrl =
{
    .speed_ctrl = &R_Fric_Speed_Ctrl, // 速度PID对象指针
};

/* 升降 RM2006，占用 0x200 组第 4 路 */
rm_cfg_t Lift_Born =
{
    .rxId = 3, // 组内槽位，0~3
    .stdId = 0x200, // 控制帧标准ID，0~0x7FF
    .type = _2006_Single, // 电机型号枚举
    .hcan = &hcan1, // 绑定CAN外设指针
};

rm_tx_t Lift_Tx;
rm_rx_t Lift_Rx;
rm_state_t Lift_State;

pid_ctrl_t Lift_Speed_Ctrl =
{
    .kp = 0.0f, // 速度比例增益，输出/rpm
    .ki = 0.0f, // 速度积分增益，输出/累加rpm
    .kd = 0.0f, // 速度微分增益，输出/rpm差
    .integral_max = 0.0f, // 累加误差上限，rpm
    .out_max = 0.0f, // 速度环输出限幅，原始量
};

rm_ctrl_t Lift_Ctrl =
{
    .speed_ctrl = &Lift_Speed_Ctrl, // 速度PID对象指针
};

rm_motor_t lift_motor =
{
    .born_info = &Lift_Born, // 电机通信配置指针
    .rx_info = &Lift_Rx, // 反馈对象指针
    .tx_info = &Lift_Tx, // 命令对象指针
    .state = &Lift_State, // 在线状态对象指针
    .ctrl = &Lift_Ctrl, // 电机控制对象指针
    .single_init = rm_motor_init, // 单电机初始化回调
};

/* 摩擦轮索引表 */
rm_motor_t rm_motor[SHOOT_FRIC_NUM] =
{
    [SHOOT_FRIC_L] =
    {
        .born_info = &L_Fric_Born, // 电机通信配置指针
        .rx_info = &L_Fric_Rx, // 反馈对象指针
        .tx_info = &L_Fric_Tx, // 命令对象指针
        .state = &L_Fric_State, // 在线状态对象指针
        .ctrl = &L_Fric_Ctrl, // 电机控制对象指针
        .single_init = rm_motor_init, // 单电机初始化回调
    },
    [SHOOT_FRIC_R] =
    {
        .born_info = &R_Fric_Born, // 电机通信配置指针
        .rx_info = &R_Fric_Rx, // 反馈对象指针
        .tx_info = &R_Fric_Tx, // 命令对象指针
        .state = &R_Fric_State, // 在线状态对象指针
        .ctrl = &R_Fric_Ctrl, // 电机控制对象指针
        .single_init = rm_motor_init, // 单电机初始化回调
    },
};

/* 摩擦轮电机组，统一收发 0x200 组帧 */
rm_group_t RM_Group =
{
    .motor[SHOOT_FRIC_L] = &rm_motor[SHOOT_FRIC_L], // 电机组槽位，0~3
    .motor[SHOOT_FRIC_R] = &rm_motor[SHOOT_FRIC_R], // 电机组槽位，0~3
    .motor[3] = &lift_motor, // 电机组槽位，0~3
    .stdId = 0x200, // 控制帧标准ID，0~0x7FF
    .hcan = &hcan1, // 绑定CAN外设指针
    .group_init = rm_group_init, // 电机组初始化回调
};

/* 拨盘 KT4005 电机对象 */
KT_motor_t dail_motor =
{
    .KT_motor_info =
    {
        .id =
        {
            .tx_id = ID_DIAL, // 拨盘命令ID，0x141
            .rx_id = ID_DIAL, // 拨盘反馈ID，0x141
            .drive_type = M_CAN1, // 拨盘总线，CAN1
            .motor_type = KT4005, // 拨盘型号，KT4005
        },
    },
    .init = KT_motor_class_init, // 拨盘初始化回调
};

/* 初始化摩擦轮电机组 */
void rm_motor_list_init(void)
{
    RM_Group.group_init(&RM_Group);
}

/* 初始化拨盘 KT 电机 */
void kt_motor_list_init(void)
{
    KT_motor_class_init(&dail_motor);
}
/* 初始化云台 DM 电机组 */
void dm_motor_list_init()
{
    DM_Group.group_init(&DM_Group);
}

/* 云台电机组心跳 */
void dm_motor_list_heart_beat()
{
    DM_Group.group_heartbeat(&DM_Group);
}


/* 摩擦轮电机组心跳 */
void rm_motor_list_heart_beat(void)
{
    RM_Group.group_heartbeat(&RM_Group);
}

/* 拨盘 KT 电机心跳 */
void kt_motor_list_heart_beat(void)
{
    if (dail_motor.heartbeat != NULL)
    {
        dail_motor.heartbeat(&dail_motor);
    }
}
