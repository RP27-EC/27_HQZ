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
    LAUNCHER_REJECT_NONE = 0, // 无拒绝，值0
    LAUNCHER_REJECT_DISABLED, // 许可关闭，值1
    LAUNCHER_REJECT_BUSY, // 单发或启动忙，值2
    LAUNCHER_REJECT_FRIC_NOT_READY, // 转轮未达速，值3
    LAUNCHER_REJECT_DIAL_OFFLINE, // 拨盘离线，值4
    LAUNCHER_REJECT_HEAT, // 热量禁止，值5
    LAUNCHER_REJECT_START_FAILED, // 启动超时，值6
    LAUNCHER_REJECT_SINGLE_TIMEOUT, // 单发未到位，值7
    LAUNCHER_REJECT_FRIC_JAM, // 转轮解堵中，值8
    LAUNCHER_REJECT_FRIC_FAULT, // 转轮锁停，值9
    LAUNCHER_REJECT_RELEASE_REQUIRED, // 等待释放，值10
    LAUNCHER_REJECT_ABORTED, // 许可中断，值11
} launcher_reject_e;

typedef enum
{
    LAUNCHER_FRIC_NORMAL = 0, // 常规速度环，值0
    LAUNCHER_FRIC_BOOST, // 限时增强，值1
    LAUNCHER_FRIC_LOCKED, // 失败锁停，值2
} launcher_fric_state_e;

typedef struct
{
    uint32_t received; // 收到事件数，uint32循环
    uint32_t accepted; // 接受单发数，uint32循环
    uint32_t rejected; // 拒绝事件数，uint32循环
    uint32_t completed; // 单发到位数，uint32循环
    uint32_t timed_out; // 单发超时数，uint32循环
    uint32_t start_failed; // 启动失败数，uint32循环
    uint32_t aborted; // 已接受中止数，uint32循环
    uint32_t jam_count; // 转轮堵转数，uint32循环
    uint32_t recovery_ok; // 解堵成功数，uint32循环
    uint32_t recovery_failed; // 解堵失败数，uint32循环
    int64_t dial_target; // 拨盘目标，count
    int32_t dial_angle; // 拨盘反馈角，count
    float dial_speed; // 拨盘反馈速度，deg/s
    float dial_speed_target; // 拨盘目标速度，deg/s
    float dial_output; // 拨盘控制电流，原始值
    float dial_current; // 拨盘反馈电流，原始值
    float fric_output[2]; // 左右轮控制电流，原始值
    float fric_speed[2]; // 左右轮反馈速度，rpm
    float fric_current[2]; // 左右轮反馈电流，A
    launcher_reject_e reject_reason; // 最近事件结果，0~11
    launcher_reject_e inhibit_reason; // 当前禁止原因，0~11
    launcher_fric_state_e fric_state; // 转轮阶段，0~2
    uint32_t jam_elapsed_ms; // 堵转确认时间，ms
    uint32_t boost_elapsed_ms; // 增强运行时间，ms
    uint8_t single_seq; // 最近单发序号，0~255
    uint8_t reset_seq; // 最近复位序号，0~255
    uint8_t start_pending; // 单发启动等待，0/1
    uint8_t release_required; // 供弹等待释放，0/1
} launcher_debug_t;

extern volatile launcher_debug_t launcher_debug;

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
