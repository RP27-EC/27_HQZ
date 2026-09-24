/* board_protocol.h - 板间通信协议 */

#ifndef __BOARD_PROTOCOL_H
#define __BOARD_PROTOCOL_H

#include "stdint.h"
#include "rp_device_config.h"

#define  BOARD_OFFLINE_CNT_MAX    50  /* 离线判定周期数 */

#define BOARD_RC_AXIS_DEADBAND          20.0f  /* 摇杆死区 */
#define BOARD_RC_AXIS_MAX               660.0f /* 摇杆幅值 */
#define BOARD_D5_YAW_RATE_MAX_DEG_S     300.0f /* Yaw 最大角速度 */
#define BOARD_D5_PITCH_RATE_MAX_DEG_S   150.0f /* Pitch 最大角速度 */
#define BOARD_D5_RATE_LSB_DEG_S         0.1f   /* 角速度量化步长 */

#define BOARD_D5_CTRL_RC                0u  /* 遥控输入 */
#define BOARD_D5_CTRL_KEYBOARD          1u  /* 键鼠输入 */
#define BOARD_D5_CMD_RC_RATE            0u  /* 角速度控制量 */
#define BOARD_D5_CMD_MOUSE_DELTA        1u  /* 鼠标增量控制量 */

/* 下板发送报文 */
#define  ID_PKT_01     0xD1  /* 整车状态与发射 */
#define  ID_PKT_02     0xD2  /* 云台目标角度 */
#define  ID_PKT_03     0xD3  /* 射击热量信息 */
#define  ID_PKT_04     0xD4  /* 血量数据 */
#define  ID_PKT_05     0xD5  /* 遥控/键鼠控制 */
/* 上板发送报文 */
#define  ID_MEG_01     0xC1  /* 设备在线状态 */
#define  ID_MEG_02     0xC2  /* 云台姿态反馈 */
#define  ID_MEG_03     0xC3
#define  ID_MEG_04     0xC4
#define  ID_MEG_05     0xC5

/* 整车状态与速度指令 */
typedef struct{
  uint8_t car_state;    /* 车辆状态 */
  uint8_t gimbal_mode;  /* 云台模式 */
  uint8_t vision_mode;  /* 视觉模式 */
  uint8_t game_start;   /* 比赛开始标志 */
  uint8_t my_color;     /* 己方颜色 */
  float v_x;            /* 底盘纵向速度 */
  float v_y;            /* 底盘横向速度 */
}Board_Car_Pkt_t;


/* 射击与热量信息 */
typedef struct{
  float shoot_speed;          /* 弹速 */
  float shoot_freq;           /* 射频 */
  int16_t shoot_heat_err;     /* 热量余量 */
  uint16_t allowance_max;     /* 最大发弹量 */
}Board_Judge_Shoot_Pkt_t;


/* 云台目标，机械角单位 rad，IMU 角单位 deg */
typedef struct{
	float yaw_mec_tar;   /* Yaw 机械目标角 */
	float yaw_imu_tar;   /* Yaw IMU 目标角 */
	float pitch_mec_tar; /* Pitch 机械目标角 */
	float pitch_imu_tar; /* Pitch IMU 目标角 */
	uint8_t is_hole;     /* 弹仓剩余检测 */
}Board_Gimbal_Target_Pkt_t;

/* 发射控制 */
typedef struct{
	uint8_t launch_state; /* 发射机构使能 */
  uint8_t shoot_mode;   /* 0 = 单发，1 = 连发 */
	uint8_t shoot_level;  /* 发射触发电平 */
}Board_Shoot_Pkt_t;


/* 血量原始数据 */
typedef struct{
	uint8_t blood[8]; /* 血量字段透传 */
}Board_Blood_Pkt_t;


/* 下板发送缓存 */
typedef struct{
  Board_Car_Pkt_t car_pkt;                     /* 整车状态 */
  Board_Judge_Shoot_Pkt_t judge_shoot_pkt;     /* 射击热量 */
  Board_Gimbal_Target_Pkt_t gimbal_target_pkt; /* 云台目标 */
  Board_Shoot_Pkt_t shoot_pkt;                 /* 发射控制 */
  Board_Blood_Pkt_t blood_pkt;                 /* 血量数据 */
}Board_Tx_Pkt_t;



/* 云台姿态反馈 */
typedef struct{
	float yaw_mec;   /* Yaw 机械角，rad */
	float yaw_imu;   /* Yaw IMU 角，deg */
	float pitch_mec; /* Pitch 机械角，rad */
	float pitch_imu; /* Pitch IMU 角，deg */
}Board_Gimbal_Meg_t;

/* 视觉目标反馈 */
typedef struct{
	float vision_yaw_tar;   /* 视觉 Yaw 目标 */
	float vision_pitch_tar; /* 视觉 Pitch 目标 */
	uint8_t is_find_target; /* 1 = 已识别目标 */
}Board_Vision_Meg_t;


/* 上板设备在线状态 */
typedef struct{
  uint8_t yaw_motor_state;    /* Yaw 电机在线 */
	uint8_t pitch_motor_state;  /* Pitch 电机在线 */
	uint8_t height_motor_state; /* 高度电机在线 */
	uint8_t r_fric_state;       /* 右摩擦轮在线 */
	uint8_t l_fric_state;       /* 左摩擦轮在线 */
	uint8_t dial_motor_state;   /* 拨盘在线 */
	uint8_t vision_state;       /* 视觉在线 */
  uint8_t is_down;            /* 下板状态标志 */
}Board_State_Meg_t;


/* 上板接收缓存 */
typedef struct{
	Board_Gimbal_Meg_t gimbal_meg; /* 云台姿态 */
	Board_Vision_Meg_t vision_meg; /* 视觉目标 */
	Board_State_Meg_t state_meg;   /* 设备状态 */
}Board_Rx_Meg_t;


/* 板间链路状态 */
typedef struct{
	uint16_t offline_cnt_max; /* 离线判定阈值 */
	dev_work_state_t status;  /* 链路在线状态 */
	uint16_t offline_cnt;     /* 当前离线计数 */

	volatile uint32_t gimbal_rx_time_ms; /* 云台反馈接收时刻 */
	volatile uint8_t gimbal_data_valid;  /* 云台反馈数据有效 */
}Board_Status_t;

/* 板间通信对象，函数指针便于统一调度 */
typedef struct Board_Struct_t{
	Board_Status_t* status;  /* 链路状态 */
	Board_Tx_Pkt_t* tx_pkt;  /* 下板发送缓存 */
  Board_Rx_Meg_t* rx_meg;  /* 上板反馈缓存 */

	void (*heartbeat)(struct Board_Struct_t *board); /* 心跳更新 */

	void (*tx_01)(struct Board_Struct_t* board); /* D1 整车状态 */
	void (*tx_02)(struct Board_Struct_t* board); /* D2 云台目标 */
	void (*tx_03)(struct Board_Struct_t* board); /* D3 射击信息 */
	void (*tx_04)(struct Board_Struct_t* board); /* D4 血量数据 */
	void (*tx_05)(struct Board_Struct_t* board); /* D5 遥控控制 */

	void (*rx_01)(struct Board_Struct_t* board, uint8_t *rxBuf); /* C1 */
	void (*rx_02)(struct Board_Struct_t* board, uint8_t *rxBuf); /* C2 */

	void (*init)(struct Board_Struct_t *board); /* 绑定接口 */

}Board_t;



extern  Board_t  board;

void Board_Init(Board_t* board);

void Board_Heart_Beat(Board_t* board);

void Board_Tx_Pkt_01(Board_t* board);

void Board_Tx_Pkt_02(Board_t* board);

void Board_Tx_Pkt_03(Board_t* board);

void Board_Tx_Pkt_04(Board_t* board);

void Board_Tx_Pkt_05(Board_t* board);

void Board_Rx_Meg_01(Board_t* board,uint8_t* rxbuf);

void Board_Rx_Meg_02(Board_t* board,uint8_t* rxbuf);


#endif



