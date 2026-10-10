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
    float fric_l_output_raw; // 左轮最终电流指令，原始量
    float fric_r_output_raw; // 右轮最终电流指令，原始量
} launcher_t;

extern launcher_t launcher;

typedef enum
{
    LAUNCHER_SPEED_OK = 0, // 允许新发射，值0
    LAUNCHER_SPEED_LINK, // D6链路失效，值1
    LAUNCHER_SPEED_SOURCE, // 裁判源离线，值2
    LAUNCHER_SPEED_TIMEOUT, // 保留超时编号，值3
    LAUNCHER_SPEED_ASSOCIATION, // 保留关联编号，值4
    LAUNCHER_SPEED_HIGH, // 高弹速锁止，值5
    LAUNCHER_SPEED_WAITING, // 保留兼容编号，值6
    LAUNCHER_SPEED_REARM, // 切换后隔离反馈，值7
} launcher_speed_reason_e;

typedef struct
{
    float target_rpm; // 最终摩擦轮目标，rpm
    float single_target_rpm; // 单发修正目标，rpm
    float repeat_target_rpm; // 连发修正目标，rpm
    float latest_mps; // 最近有效弹速，m/s
    float average_mps; // 当前模式三条均值，m/s
    float sample_sum_mps; // 同目标弹速和，m/s
    uint32_t wait_tick; // 单发实际开始时刻，ms
    uint32_t feedback_age_ms; // 当前样本年龄，ms
    uint32_t guard_count; // 高弹速事件次数，循环
    uint32_t limit_count; // 达25m/s次数，循环
    uint32_t timeout_count; // 发射反馈超时次数，循环
    uint32_t association_count; // 关联异常次数，循环
    uint16_t event_seq; // 最近事件序号，uint16循环
    uint16_t pending_seq; // 受理前事件序号，循环
    uint8_t sample_count; // 本组样本数，0~2
    uint8_t waiting; // 学习等待弹速，0/1
    uint8_t source_ready; // 弹速源及链路有效，0/1
    uint8_t feed_ready; // 弹速允许新发射，0/1
    uint8_t at_limit; // 修正到转速边界，0/1
    uint8_t guard_latched; // 高弹速锁止，0/1
    launcher_speed_reason_e block_reason; // 新发射阻止原因，0~7
} launcher_speed_t;

extern launcher_speed_t launcher_speed;

typedef enum
{
    LAUNCHER_DIAL_REJECT_NONE = 0, // 无拒绝，值0
    LAUNCHER_DIAL_REJECT_HEAT, // 热量不足，值1
    LAUNCHER_DIAL_REJECT_BUSY, // 动作未结束，值2
    LAUNCHER_DIAL_REJECT_OFFLINE, // 通信或电机离线，值3
    LAUNCHER_DIAL_REJECT_INTERLOCK, // 失能或互锁，值4
    LAUNCHER_DIAL_REJECT_START, // 启动发送超时，值5
    LAUNCHER_DIAL_REJECT_TIMEOUT, // 单发未完成，值6
    LAUNCHER_DIAL_REJECT_FRIC, // 摩擦轮未达速，值7
    LAUNCHER_DIAL_REJECT_SPEED, // 弹速反馈或保护，值8
} launcher_dial_reject_e;

typedef struct
{
    launcher_state_e state; // 拨盘独立阶段，0~9
    launcher_dial_reject_e reject_reason; // 最近拒绝原因，0~8
    int64_t target_angle; // 累计目标，count
    int64_t target_error; // 目标减反馈，count
    float speed_target_dps; // 最近速度目标，deg/s
    int16_t output_current_raw; // 最近电流指令，原始量
    uint8_t hold_allowed; // 允许保持，0/1
    uint8_t feed_allowed; // 允许供弹，0/1
    uint8_t pending_single; // 等待启动单发，0/1
    uint8_t trigger_ready; // 已释放触发可重发，0/1
    uint8_t run_tx_status; // 启动HAL结果，0~3
    uint8_t torque_tx_status; // 力矩HAL结果，0~3
    uint8_t stop_tx_status; // 停机HAL结果，0~3
    uint32_t accepted_count; // 已接纳单发数，次
    uint32_t rejected_count; // 拒绝单发数，次
    uint32_t completed_count; // 到位单发数，次
    uint32_t timeout_count; // 未完成单发数，次
    uint32_t start_timeout_count; // 启动超时数，次
    uint32_t state_tick; // 拨盘阶段起点，ms
    uint32_t pending_tick; // 等待启动起点，ms
    uint32_t settle_tick; // 到位确认起点，ms
    uint8_t settling; // 正在确认到位，0/1
} launcher_dial_t;

extern launcher_dial_t launcher_dial;

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
