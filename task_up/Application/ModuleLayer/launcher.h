#ifndef __LAUNCHER_H
#define __LAUNCHER_H

#include <stdint.h>

typedef enum
{
    LAUNCHER_SLEEP = 0,
    LAUNCHER_SPINUP,
    LAUNCHER_INIT,
    LAUNCHER_READY,
    LAUNCHER_SINGLE,
    LAUNCHER_REPEAT,
    LAUNCHER_REVERSE,
    LAUNCHER_RELOAD,
    LAUNCHER_STOPPING,
    LAUNCHER_FAULT,
} launcher_state_e;

typedef struct
{
    launcher_state_e state;
    uint32_t state_tick;
    uint32_t last_repeat_tick;
    uint32_t jam_tick;

    float fric_target_rpm;
    float fric_l_speed_rpm;
    float fric_r_speed_rpm;

    int32_t dial_angle;
    int32_t dial_target_angle;
    int32_t dial_zero_angle;

    uint8_t enabled;
    uint8_t fric_ready;
    uint8_t dial_online;
    uint8_t last_shoot_level;
    uint8_t fault;
} launcher_t;

extern launcher_t launcher;

void Launcher_Init(void);
void Launcher_Work(void);

#endif
