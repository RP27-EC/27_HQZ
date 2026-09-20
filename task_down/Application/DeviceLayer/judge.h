#ifndef __JUDGE_H
#define __JUDGE_H

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "rp_config.h"
#include "judge_protocol.h"
#include <stdint.h>
/* Exported macro ------------------------------------------------------------*/
#define JUDGE_OFFLINE_CNT_MAX 1000

/* ========================== 命令码宏 ========================== */
#define ID_game_status               0x0001U
#define ID_game_result               0x0002U
#define ID_game_robot_HP             0x0003U
#define ID_event_data                0x0101U
#define ID_referee_warning           0x0104U
#define ID_dart_info                 0x0105U
#define ID_robot_status              0x0201U
#define ID_power_heat_data           0x0202U
#define ID_robot_pos                 0x0203U
#define ID_buff                      0x0204U
#define ID_hurt_data                 0x0206U
#define ID_shoot_data                0x0207U
#define ID_projectile_allowance      0x0208U
#define ID_rfid_status               0x0209U
#define ID_robot_interaction_data    0x0301U
#define ID_map_command               0x0303U
#define ID_map_robot_data            0x0305U
#define ID_map_data                  0x0307U
#define ID_custom_info               0x0308U
#define ID_set_video_channel         0x0F01U
#define ID_query_video_channel       0x0F02U
#define ID_radar_enemy_HP             0x0A02U
#define ID_radar_enemy_ammo           0x0A03U
#define ID_radar_enemy_team_status    0x0A04U
#define ID_radar_enemy_robot_status   0x0A05U

/* ========================== 结构体长度 ========================== */
/* 注意：长度 = 数据段长度，不含帧头(5)+cmd_id(2)+CRC16(2) */
#define LEN_game_status              11U
#define LEN_game_result              1U
#define LEN_game_robot_HP            20U
#define LEN_event_data               4U
#define LEN_referee_warning          3U
#define LEN_dart_info                3U
#define LEN_robot_status             17U
#define LEN_power_heat_data          14U
#define LEN_robot_pos                12U
#define LEN_buff                     8U
#define LEN_hurt_data                1U
#define LEN_shoot_data               7U
#define LEN_projectile_allowance     8U
#define LEN_rfid_status              5U
#define LEN_robot_interaction_data   112U
#define LEN_map_command              12U
#define LEN_map_robot_data           48U
#define LEN_map_data                 105U
#define LEN_custom_info              34U
#define LEN_set_video_channel        1U
#define LEN_query_video_channel      1U
#define LEN_radar_enemy_HP            12U
#define LEN_radar_enemy_ammo          10U
#define LEN_radar_enemy_team_status   8U
#define LEN_radar_enemy_robot_status  41U

/*---------------------结构体-----------------------*/
/* -------------------- 0x0001 -------------------- */
/**
 * @brief  比赛状态数据，固定以 1 Hz 频率发送
 * @note   0x0001
 */
typedef struct __attribute__((packed))
{
    uint8_t  game_type     : 4; /* bit 0-3：比赛类型
                                   1：RoboMaster 机甲大师超级对抗赛
                                   2：RoboMaster 机甲大师高校单项赛
                                   3：ICRA RoboMaster 高校人工智能挑战赛
                                   4：RoboMaster 机甲大师高校联盟赛 3V3 对抗
                                   5：RoboMaster 机甲大师高校联盟赛步兵对抗 */
    uint8_t  game_progress : 4; /* bit 4-7：当前比赛阶段
                                   0：未开始比赛
                                   1：准备阶段
                                   2：十五秒裁判系统自检阶段
                                   3：五秒倒计时
                                   4：比赛中
                                   5：比赛结算中 */
    uint16_t stage_remain_time; /* 当前阶段剩余时间，单位：秒 */
    uint64_t SyncTimeStamp;     /* UNIX 时间，当机器人正确连接到裁判系统的 NTP 服务器后生效 */
} game_status_t;

/* -------------------- 0x0002 -------------------- */
/**
 * @brief  比赛结果数据，比赛结束触发发送
 * @note   0x0002
 */
typedef struct __attribute__((packed))
{
    uint8_t winner; /* 0：平局，1：红方胜利，2：蓝方胜利 */
} game_result_t;

/* -------------------- 0x0003 -------------------- */
/**
 * @brief  机器人血量数据，固定以 3 Hz 频率发送
 * @note   0x0003
 */
typedef struct __attribute__((packed))
{
    uint16_t ally_1_robot_HP;
    uint16_t ally_2_robot_HP;
    uint16_t ally_3_robot_HP;
    uint16_t ally_4_robot_HP;

    int16_t damage_difference;    /* 己方总伤害 - 对方总伤害 */

    uint16_t ally_7_robot_HP;
    uint16_t ally_outpost_HP;
    uint16_t ally_base_HP;

    uint16_t enemy_outpost_HP;
    uint16_t enemy_base_HP;
} game_robot_HP_t;


/* -------------------- 0x0101 -------------------- */
/**
 * @brief  场地事件数据，固定以 1 Hz 频率发送
 * @note   0x0101
 *
 *  event_data 位域定义（bit0 为最低位）：
 *  bit 0     己方与资源区不重叠的补给区占领状态   1=已占领
 *  bit 1     己方与资源区重叠的补给区占领状态     1=已占领
 *  bit 2     己方补给区占领状态（仅 RMUL）        1=已占领
 *  bit 3-4   己方小能量机关激活状态               0=未激活 1=已激活 2=正在激活
 *  bit 5-6   己方大能量机关激活状态               0=未激活 1=已激活 2=正在激活
 *  bit 7-8   己方中央高地占领状态                 1=己方 2=对方
 *  bit 9-10  己方梯形高地占领状态                 1=已占领
 *  bit11-19  对方飞镖最后一次击中己方前哨站/基地时间 0-420 s
 *  bit20-22  对方飞镖最后一次击中目标             0=无 1=前哨站 2=基地固定 3=基地随机固定 4=基地随机移动 5=基地末端移动
 *  bit23-24  中心增益点占领状态（仅 RMUL）        0=无 1=己方 2=对方 3=双方
 *  bit25-26  己方堡垒增益点占领状态               0=无 1=己方 2=对方 3=双方
 *  bit27-28  己方前哨站增益点占领状态             0=无 1=己方 2=对方
 *  bit29     己方基地增益点占领状态               1=已占领
 *  bit30-31  保留
 */
typedef struct __attribute__((packed))
{
    uint32_t event_data;
} event_data_t;

/* -------------------- 0x0104 -------------------- */
/**
 * @brief  裁判警告数据，己方判罚/判负时触发发送，其余时间以 1 Hz 频率发送
 * @note   0x0104
 */
typedef struct __attribute__((packed))
{
    uint8_t level;              /* 己方最后一次受到判罚的等级
                                   1：双方黄牌，2：黄牌，3：红牌，4：判负 */
    uint8_t offending_robot_id; /* 违规机器人 ID（如红 1 机器人 ID 为 1，蓝 1 为 101）
                                   判负和双方黄牌时，该值为 0 */
    uint8_t count;              /* 对应判罚等级的违规次数（开局默认 0） */
} referee_warning_t;

/* -------------------- 0x0105 -------------------- */
/**
 * @brief  飞镖发射相关数据，固定以 1 Hz 频率发送
 * @note   0x0105
 */
typedef struct __attribute__((packed))
{
    uint8_t  dart_remaining_time; /* 己方飞镖发射剩余时间，单位：秒 */
    uint16_t dart_info;           /* bit 0-2：最近一次己方飞镖击中目标
                                     bit 3-5：对方被击中目标累计次数
                                     bit 6-8：飞镖当前选定目标
                                     bit 9-15：保留 */
} dart_info_t;

/* -------------------- 0x0201 -------------------- */
/**
 * @brief  机器人性能体系数据，固定以 10 Hz 频率发送
 * @note   0x0201
 */
typedef struct __attribute__((packed))
{
    uint8_t  robot_id;
    uint8_t  robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;

    float shooter_barrel_speed_limit;

    uint8_t power_management_gimbal_output  : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
    uint8_t reserved                        : 5;
} robot_status_t;


/* -------------------- 0x0202 -------------------- */
/**
 * @brief  实时底盘缓冲能量和射击热量数据，固定以 10 Hz 频率发送
 * @note   0x0202
 */
typedef struct __attribute__((packed))
{
    uint16_t reserved1;
    uint16_t reserved2;
    float    reserved3;
    uint16_t buffer_energy;             /* 缓冲能量，单位：J */
    uint16_t shooter_17mm_1_barrel_heat; /* 17 mm 发射机构热量 */
    uint16_t shooter_42mm_barrel_heat;   /* 42 mm 发射机构热量 */
} power_heat_data_t;

/* -------------------- 0x0203 -------------------- */
/**
 * @brief  机器人位置数据，固定以 1 Hz 频率发送
 * @note   0x0203
 */
typedef struct __attribute__((packed))
{
    float x;     /* 本机器人位置 x 坐标，单位：m */
    float y;     /* 本机器人位置 y 坐标，单位：m */
    float angle; /* 本机器人测速模块朝向，单位：度，正北为 0° */
} robot_pos_t;

/* -------------------- 0x0204 -------------------- */
/**
 * @brief  机器人增益和底盘能量数据，固定以 3 Hz 频率发送
 * @note   0x0204
 */
typedef struct __attribute__((packed))
{
    uint8_t  recovery_buff;      /* 回血增益（百分比） */
    uint16_t cooling_buff;       /* 射击热量冷却增益（直接值） */
    uint8_t  defence_buff;       /* 防御增益（百分比） */
    uint8_t  vulnerability_buff; /* 负防御增益（百分比） */
    uint16_t attack_buff;        /* 攻击增益（百分比） */
    uint8_t  remaining_energy;   /* 机器人剩余能量比例标识，详见协议位域说明 */
} buff_t;

/* -------------------- 0x0206 -------------------- */
/**
 * @brief  伤害状态数据，伤害发生后发送
 * @note   0x0206
 */
typedef struct __attribute__((packed))
{
    uint8_t armor_id : 4;            /* 扣血装甲/测速模块 ID */
    uint8_t HP_deduction_reason : 4; /* 血量变化类型，详见协议枚举 */
} hurt_data_t;

/* -------------------- 0x0207 -------------------- */
/**
 * @brief  实时射击数据，弹丸发射后发送
 * @note   0x0207
 */
typedef struct __attribute__((packed))
{
    uint8_t bullet_type;       /* 弹丸类型：bit1-17 mm，bit2-42 mm */
    uint8_t shooter_number;    /* 发射机构 ID：1-17 mm，3-42 mm */
    uint8_t launching_frequency; /* 弹丸射速，单位：Hz */
    float   initial_speed;     /* 弹丸初速度，单位：m/s */
} shoot_data_t;

/* -------------------- 0x0208 -------------------- */
/**
 * @brief  允许发弹量，固定以 10 Hz 频率发送
 * @note   0x0208
 */
typedef struct __attribute__((packed))
{
    uint16_t projectile_allowance_17mm;   /* 17 mm 弹丸允许发弹量 */
    uint16_t projectile_allowance_42mm;   /* 42 mm 弹丸允许发弹量 */
    uint16_t remaining_gold_coin;         /* 剩余金币数量 */
    uint16_t projectile_allowance_fortress; /* 堡垒增益点储备 17 mm 弹丸允许发弹量 */
} projectile_allowance_t;

/* -------------------- 0x0209 -------------------- */
/**
 * @brief  机器人 RFID 模块状态，固定以 3 Hz 频率发送
 * @note   0x0209
 */
typedef struct __attribute__((packed))
{
    uint32_t rfid_status;   /* 位图，详见协议 1.2 表 1-18 */
    uint8_t  rfid_status_2; /* 扩展位图，bit0-bit1 见协议说明 */
} rfid_status_t;


/* -------------------- 0x0301 -------------------- */
/**
 * @brief  机器人交互数据，发送方触发发送，频率上限 30 Hz
 * @note   0x0301
 */
typedef struct __attribute__((packed))
{
    uint16_t data_cmd_id; /* 子内容 ID */
    uint16_t sender_id;   /* 发送者 ID */
    uint16_t receiver_id; /* 接收者 ID */
    uint8_t  user_data[112]; /* 内容数据段，最大 112 字节 */
} robot_interaction_data_t;

/* -------------------- 0x0303 -------------------- */
/**
 * @brief  选手端小地图交互数据，选手端触发发送
 * @note   0x0303
 */
typedef struct __attribute__((packed))
{
    float    target_position_x; /* 目标位置 x 坐标，单位：m */
    float    target_position_y; /* 目标位置 y 坐标，单位：m */
    uint8_t  cmd_keyboard;      /* 云台手按下的键盘按键通用键值，无按键为 0 */
    uint8_t  target_robot_id;   /* 对方机器人 ID，发送坐标时为 0 */
    uint16_t cmd_source;        /* 信息来源 ID，详见协议附录 */
} map_command_t;

/* -------------------- 0x0305 -------------------- */
/**
 * @brief  选手端小地图接收雷达数据，频率上限 5 Hz
 * @note   0x0305
 */
typedef struct __attribute__((packed))
{
    uint16_t opponent_hero_position_x;
    uint16_t opponent_hero_position_y;
    uint16_t opponent_engineer_position_x;
    uint16_t opponent_engineer_position_y;
    uint16_t opponent_infantry_3_position_x;
    uint16_t opponent_infantry_3_position_y;
    uint16_t opponent_infantry_4_position_x;
    uint16_t opponent_infantry_4_position_y;
    uint16_t opponent_aerial_position_x;
    uint16_t opponent_aerial_position_y;
    uint16_t opponent_sentry_position_x;
    uint16_t opponent_sentry_position_y;

    uint16_t ally_hero_position_x;
    uint16_t ally_hero_position_y;
    uint16_t ally_engineer_position_x;
    uint16_t ally_engineer_position_y;
    uint16_t ally_infantry_3_position_x;
    uint16_t ally_infantry_3_position_y;
    uint16_t ally_infantry_4_position_x;
    uint16_t ally_infantry_4_position_y;
    uint16_t ally_aerial_position_x;
    uint16_t ally_aerial_position_y;
    uint16_t ally_sentry_position_x;
    uint16_t ally_sentry_position_y;
} map_robot_data_t;

/* -------------------- 0x0307 -------------------- */
/**
 * @brief  选手端小地图接收路径数据，频率上限 1 Hz
 * @note   0x0307
 */
typedef struct __attribute__((packed))
{
    uint8_t  intention;         /* 1：攻击，2：防守，3：移动 */
    uint16_t start_position_x;  /* 路径起点 x 坐标，单位：dm */
    uint16_t start_position_y;  /* 路径起点 y 坐标，单位：dm */
    int8_t   delta_x[49];       /* 路径点 x 轴增量数组，单位：dm */
    int8_t   delta_y[49];       /* 路径点 y 轴增量数组，单位：dm */
    uint16_t sender_id;         /* 发送者 ID */
} map_data_t;

/* -------------------- 0x0308 -------------------- */
/**
 * @brief  选手端小地图接收机器人数据，频率上限 3 Hz
 * @note   0x0308
 */
typedef struct __attribute__((packed))
{
    uint16_t sender_id;    /* 发送者 ID */
    uint16_t receiver_id;  /* 接收者 ID */
    uint8_t  user_data[30]; /* UTF-16 编码字符，最大 30 字节 */
} custom_info_t;

/* -------------------- 0x0F01 -------------------- */
/**
 * @brief  设置图传出图信道，频率上限 1 Hz
 * @note   0x0F01
 */
typedef struct __attribute__((packed))
{
    uint8_t channel; /* 1~6：设置信道为 1~6；反馈见协议说明 */
} set_video_channel_t;

/* -------------------- 0x0F02 -------------------- */
/**
 * @brief  查询当前出图信道，频率上限 2 Hz
 * @note   0x0F02
 */
typedef struct __attribute__((packed))
{
    uint8_t query; /* 发送时填 0；反馈值 0~6 含义见协议说明 */
} query_video_channel_t;


/*
 * @brief  对方机器人血量信息，由信号发射源发送给雷达
 * @note   以 10 Hz 频率持续发送，数据长度为 12 字节
 *
 * @details
 * 当对应机器人未上场或被罚下时，血量值为 0。
 * 本命令不包含空中机器人、前哨站和基地血量。
 */
typedef struct __attribute__((packed))
{
    uint16_t enemy_hero_HP;       /* 偏移 0：对方 1 号英雄机器人血量 */
    uint16_t enemy_engineer_HP;   /* 偏移 2：对方 2 号工程机器人血量 */
    uint16_t enemy_infantry_3_HP; /* 偏移 4：对方 3 号步兵机器人血量 */
    uint16_t enemy_infantry_4_HP; /* 偏移 6：对方 4 号步兵机器人血量 */
    uint16_t enemy_sentry_HP;     /* 偏移 8：对方 7 号哨兵机器人血量 */
		uint32_t update_timestamp;

} radar_enemy_HP_t;


/*
 * @brief  对方机器人剩余允许发弹量信息，由信号发射源发送给雷达
 * @note   以 10 Hz 频率持续发送，数据长度为 10 字节
 *
 * @details
 * 步兵、空中和哨兵机器人的数值包含堡垒增益点提供的
 * 储备允许发弹量。
 *
 * 工程机器人没有发弹量字段。
 */
typedef struct __attribute__((packed))
{
    uint16_t enemy_hero_ammo;       /* 偏移 0：对方 1 号英雄机器人允许发弹量 */
    uint16_t enemy_infantry_3_ammo; /* 偏移 2：对方 3 号步兵机器人允许发弹量 */
    uint16_t enemy_infantry_4_ammo; /* 偏移 4：对方 4 号步兵机器人允许发弹量 */
    uint16_t enemy_aerial_ammo;     /* 偏移 6：对方 6 号空中机器人允许发弹量 */
    uint16_t enemy_sentry_ammo;     /* 偏移 8：对方 7 号哨兵机器人允许发弹量 */
	  uint32_t update_timestamp;

} radar_enemy_ammo_t;


/* 
 * @brief  对方队伍宏观状态信息，由信号发射源发送给雷达
 * @note   以 10 Hz 频率持续发送，数据长度为 8 字节
 *
 * @details
 * 包含对方剩余金币、累计总金币及场地交互模块状态。
 */
typedef struct __attribute__((packed))
{
    uint16_t remaining_gold_coin; /* 对方剩余金币 */
    uint16_t total_gold_coin;     /* 对方累计总金币 */
    uint32_t field_status;        /* 对方队伍宏观/场地状态 */
	  uint32_t update_timestamp;
} radar_enemy_team_status_t;


/*
 * 单个机器人增益信息。
 *
 * 对应 0x0A05 中每个机器人的连续 7 字节数据。
 */
typedef struct __attribute__((packed))
{
    uint8_t  recovery_buff;       /* 回血增益，百分比 */
    uint16_t cooling_buff;        /* 射击热量冷却增益 */
    uint8_t  defence_buff;        /* 防御增益，百分比 */
    uint8_t  vulnerability_buff;  /* 负防御/易伤增益，百分比 */
    uint16_t attack_buff;         /* 攻击增益，百分比 */
} radar_enemy_robot_buff_t;


/* 
 * @brief  对方各机器人当前增益效果和主要状态
 * @note   以 10 Hz 频率持续发送，数据长度为 41 字节
 *
 * @details
 * 包含对方英雄、工程、3 号步兵、4 号步兵和哨兵的增益数据；
 * 另外包含对方哨兵姿态及五台机器人的主要状态。
 *
 * 本命令不包含空中机器人状态。
 */
typedef struct __attribute__((packed))
{
    radar_enemy_robot_buff_t hero;       /* 偏移 0：对方英雄增益，7 字节 */
    radar_enemy_robot_buff_t engineer;   /* 偏移 7：对方工程增益，7 字节 */
    radar_enemy_robot_buff_t infantry_3; /* 偏移 14：对方 3 号步兵增益，7 字节 */
    radar_enemy_robot_buff_t infantry_4; /* 偏移 21：对方 4 号步兵增益，7 字节 */
    radar_enemy_robot_buff_t sentry;     /* 偏移 28：对方哨兵增益，7 字节 */

    uint8_t sentry_mode;                 /* 偏移 35：对方哨兵当前姿态
                                          * 0：未定义/无效
                                          * 1：进攻姿态
                                          * 2：防御姿态
                                          * 3：移动姿态
                                          * 4：强化进攻姿态
                                          * 5：强化防御姿态
                                          * 6：强化移动姿态 */

    uint8_t hero_status;                 /* 偏移 36：对方英雄主要状态 */
    uint8_t engineer_status;             /* 偏移 37：对方工程主要状态 */
    uint8_t infantry_3_status;           /* 偏移 38：对方 3 号步兵主要状态 */
    uint8_t infantry_4_status;           /* 偏移 39：对方 4 号步兵主要状态 */
    uint8_t sentry_status;               /* 偏移 40：对方哨兵主要状态
                                          *
                                          * 主要状态枚举：
                                          * 0：存活
                                          * 1：战亡
                                          * 2：无敌但不虚弱
                                          * 3：无敌且虚弱
                                          *
                                          * 机器人异常离线不会影响该状态判断。 */
		uint32_t update_timestamp;

} radar_enemy_robot_status_t;

typedef struct __attribute__((packed))
{
    radar_enemy_HP_t            radar_enemy_HP;
    radar_enemy_ammo_t          radar_enemy_ammo;
    radar_enemy_team_status_t   radar_enemy_team_status;
    radar_enemy_robot_status_t  radar_enemy_robot_status;
	  uint32_t current_timestamp;

} radar_information_status_t;


typedef struct{
	game_status_t              game_status;           
	game_result_t              game_result;          
  game_robot_HP_t            game_robot_HP;        
	event_data_t               event_data;           
	referee_warning_t          referee_warning;    
	dart_info_t                dart_info;            
	robot_status_t             robot_status;        
	power_heat_data_t          power_heat_data;       
	robot_pos_t                robot_pos;             
	buff_t                     buff;            
	hurt_data_t                hurt_data;           
	shoot_data_t               shoot_data;         
	projectile_allowance_t     projectile_allowance;
	rfid_status_t              rfid_status;         
	robot_interaction_data_t   robot_interaction_data;
	map_command_t              map_command;       
	map_robot_data_t           map_robot_data;      
	map_data_t                 map_data;           
	custom_info_t              custom_info;      
	set_video_channel_t        set_video_channel;   
	query_video_channel_t      query_video_channel;
	
	radar_information_status_t radar_information_status;

}Judge_Info_t;


typedef enum{
	J_HERO=0,
	J_ENGINEER,
	J_INFANTRY_3,
	J_INFANTRY_4,
	J_SENTRY,
	J_OUTPOST,
	J_BASE,
	J_ROBOT_CNT,
	
}Judge_Robot_Class_e;


typedef struct{
	uint8_t  robot_id;                       /* 本机器人 ID */
  uint8_t  game_progress;
	
  uint16_t blood[J_ROBOT_CNT];

  uint16_t shooter_barrel_heat_limit;     /* 机器人射击热量上限 */
	uint16_t shooter_17mm_1_barrel_heat;    /* 17 mm 发射机构热量 */
	
  uint16_t chassis_power_limit;           /* 机器人底盘功率上限 */
  uint16_t buffer_energy;                 /* 缓冲能量，单位：J */
  
	uint8_t launching_frequency;            /* 弹丸射速，单位：Hz */
  float   initial_speed;                  /* 弹丸初速度，单位：m/s */
	uint16_t projectile_allowance_17mm;     /* 17 mm 弹丸允许发弹量 */
	
	
  uint16_t enemy_blood[J_ROBOT_CNT];      /* 对方血量 */    
  uint16_t enemy_ammo[J_ROBOT_CNT];       /* 对方允许发弹量 */
	uint16_t enemy_remaining_gold;
  uint8_t enemy_robot_status[J_ROBOT_CNT];
	
}Judge_Pkt_t;


typedef struct
{
	uint16_t offline_cnt_max;
	dev_work_state_t status;
	uint16_t offline_cnt;
}Judge_Status_t;

typedef struct Judge_Struct_t
{
	Judge_Info_t* info;
	Judge_Pkt_t* pkt;
	Judge_Status_t* status;
	
	void (*heartbeat)(struct Judge_Struct_t *judge);
	
	void (*rx)(uint16_t id, uint8_t *rxBuf);

	void (*init)(struct Judge_Struct_t *judge);
	
}Judge_t;


typedef struct
{
	float speed_now;
	uint16_t shoot_num;
	
	uint16_t lower_237;
	uint16_t speed_237;
	uint16_t speed_238;
	uint16_t speed_239;
	uint16_t speed_240;
	uint16_t speed_241;
	uint16_t speed_242;
	uint16_t speed_243;
	uint16_t speed_244;
	uint16_t speed_245;
	uint16_t speed_246;
	uint16_t speed_247;
	uint16_t speed_248;
	uint16_t speed_249;
	uint16_t speed_250;
	uint16_t higher_250;
	uint16_t num;
	float mean;  //弹速平均值
	float variance;//弹速平方差
	
	uint32_t shooting_cmd_excute_tick;
	uint16_t shooting_cmd_excute_tick_buf[100];
	float shooting_cmd_excute_tick_mean;
	float shooting_cmd_excute_tick_variance;
	
	int16_t temperature_L;
	uint8_t shooting_flag;
	uint8_t shoot_mode;
	
}bullet_data_t;


extern Judge_t judge;

extern bullet_data_t  shoot_statistics;

void Judge_Init(Judge_t* judge);
void Judge_Heart_Beat(Judge_t* judge);
void Judge_Data_Update(uint16_t id, uint8_t *rxBuf);

void Shooting_Cmd_Excute_Tick_Calculating(uint8_t flag);
void Speed_Statistic(void);

#endif
