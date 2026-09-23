/* launcher.h - 发射机构控制 */

#ifndef __LAUNCHER_H
#define __LAUNCHER_H

#include <stdint.h>

typedef enum
{
    LAUNCHER_SLEEP = 0,   // 休眠: 摩擦轮停转
    LAUNCHER_SPINUP,      // 摩擦轮升速
    LAUNCHER_INIT,        // 拨盘回零
    LAUNCHER_READY,       // 待发
    LAUNCHER_SINGLE,      // 单发中
    LAUNCHER_REPEAT,      // 连发中
    LAUNCHER_REVERSE,     // 反转退弹
    LAUNCHER_RELOAD,      // 补弹回位
    LAUNCHER_STOPPING,    // 摩擦轮降速停机
    LAUNCHER_FAULT,       // 故障锁定
} launcher_state_e;

typedef struct
{
    launcher_state_e state;
    uint32_t state_tick;        // 当前状态进入时刻
    uint32_t last_repeat_tick;  // 连发上次触发时刻
    uint32_t jam_tick;          // 卡弹检测起始时刻

    float fric_target_rpm;      // 摩擦轮目标转速
    float fric_l_speed_rpm;     // 左轮实际转速
    float fric_r_speed_rpm;     // 右轮实际转速

    int32_t dial_angle;         // 拨盘当前累计角度
    int32_t dial_target_angle;  // 拨盘目标角度
    int32_t dial_zero_angle;    // 上电时的零点

    uint8_t enabled;            // 发射使能
    uint8_t fric_ready;         // 摩擦轮达标
    uint8_t dial_online;        // 拨盘在线
    uint8_t single_pending;     // 单发待执行
    uint8_t last_shoot_level;   // 上一拍发射电平, 做边沿
    uint8_t fault;              // 故障标志
} launcher_t;

extern launcher_t launcher;

void Launcher_Init(void);
void Launcher_Work(void);

#endif
