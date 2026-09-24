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

void Launcher_Init(void);
void Launcher_Work(void);

#endif
