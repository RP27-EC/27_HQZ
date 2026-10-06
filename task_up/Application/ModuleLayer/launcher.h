#ifndef __LAUNCHER_H
#define __LAUNCHER_H

#include <stdint.h>


/* 发射机构运行阶段 */
typedef enum
{
    LAUNCHER_SLEEP = 0,  /* 休眠，输出关闭 */
    LAUNCHER_SPINUP,     /* 摩擦轮升速 */
    LAUNCHER_INIT,       /* 拨盘初始定位 */
    LAUNCHER_READY,      /* 待发 */
    LAUNCHER_SINGLE,     /* 单发供弹 */
    LAUNCHER_REPEAT,     /* 连发供弹 */
    LAUNCHER_REVERSE,    /* 堵转退让 */
    LAUNCHER_RELOAD,     /* 退让后复位 */
    LAUNCHER_STOPPING,   /* 停机降速 */
    LAUNCHER_FAULT,      /* 故障锁定 */
} launcher_state_e;

typedef struct
{
    launcher_state_e state;    /* 当前运行阶段 */
    uint32_t state_tick;       /* 阶段进入时刻，ms */
    uint32_t last_repeat_tick; /* 上次连发时刻，ms */
    uint32_t jam_tick;         /* 堵转确认计数，ms */

    float fric_target_rpm;  /* 目标转速，rpm */
    float fric_l_speed_rpm; /* 左轮反馈转速，rpm */
    float fric_r_speed_rpm; /* 右轮反馈转速，rpm */

    int32_t dial_angle;        /* 累计角度，count */
    int32_t dial_target_angle; /* 目标角度，count */
    int32_t dial_zero_angle;   /* 上电零点，count */

    uint8_t enabled;          /* 1 = 机构使能 */
    uint8_t fric_ready;       /* 1 = 摩擦轮达速 */
    uint8_t dial_online;      /* 1 = 拨盘在线 */
    uint8_t last_shoot_level; /* 上周期发射电平 */
    uint8_t fault;            /* 1 = 机构故障 */
} launcher_t;

extern launcher_t launcher;

typedef enum
{
    LAUNCHER_HEAT_NONE = 0, // 无有效初始参数，禁发
    LAUNCHER_HEAT_REFEREE, // 裁判有效，值1
    LAUNCHER_HEAT_ESTIMATE, // 断链本地估算，值2
    LAUNCHER_HEAT_TRAINING, // 固定参数训练，值3
} launcher_heat_source_e;

typedef struct
{
    float current_a; // 反馈电流，A
    float speed_rpm; // 反馈转速，rpm
    uint32_t feedback_tick; // 反馈接收时刻，ms
    uint32_t feedback_seq; // 反馈序号，uint32循环
    uint32_t feedback_age_ms; // 反馈距今时间，ms
    uint8_t online; // 电机在线，0/1
} launcher_fric_observation_t;

typedef struct
{
    float heat; // 裁判校准后的估计，热量单位
    float referee_heat; // 最新裁判值，热量单位
    float heat_limit; // 使用中上限，热量单位
    float cooling_rate; // 使用中冷却，热量单位/s
    float remaining; // 剩余预算，热量单位
    float target_rate; // 连发射频，发/s
    launcher_heat_source_e source; // 参数来源，0~3
    uint8_t ready; // 初始热量已建立，0/1
    uint8_t blocked; // 热停发锁定，0/1
    launcher_fric_observation_t fric_l; // 左轮观测，单位见类型
    launcher_fric_observation_t fric_r; // 右轮观测，单位见类型
} launcher_heat_t;

extern launcher_heat_t launcher_heat;

void Launcher_Init(void);
void Launcher_Work(void);

#endif
