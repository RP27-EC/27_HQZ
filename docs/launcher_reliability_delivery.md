# 发射可靠性：完整修改单元

2026-10-06。生产代码已落地；未编译、未烧录、未自测。下列代码块直接取自交付源码，函数、类型与配置均完整。

## 操作与工程边界

- 两板同步更新。D1 byte6 为单发事件序号，byte7 为在线 S1 下位→中位产生的复位序号，均为 uint8 循环；原 byte5 位域保持不变。新旧固件混用不支持可靠单发及锁停复位。
- S1 中位自动转轮，仍受现有遥控升降/狗洞许可限制。重连后 S2 需先回中再上拨；键鼠需释放左键再按下。原键鼠分支绕过遥控升降/狗洞互锁的边界保留，未扩展其他机构行为。
- 单发达速后才接受；忙时/未达速点击直接拒绝。已接受单发松手仍执行，启动发送等待≤50 ms；正式运行500 ms未到位则停止，释放再按从实际位置重试，已预占热量不退回。
- 摩擦轮一次限时增强失败后锁停；断联重连不解除。在线 S1 下位→中位复位，键鼠模式同样使用此动作。成功解堵后旧单发作废，需释放再触发供弹。
- 当前默认无裁判训练：上限300、冷却150热量单位/s、最高15发/s。比赛前将上板训练宏改为0，确认有效D3参数与热量。BOARD_JUDGE_ENABLE=0为遗留配置，当前裁判初始化与接收不受此宏控制。
- 拨盘控制/反馈电流保留原始值；KT驱动历史安培换算注释不一致，本次不按这些注释推定物理电流。RM摩擦轮反馈电流由现有驱动换算为A。
- 新增参数仅为人工调试初值，不能视作机械解堵或单发效果已验证。未编写测试、Mock或测试脚本，未执行构建/烧录、自测循环；按用户规范跳过。

## Keil Watch

| 表达式 | 含义 |
| --- | --- |
| launcher_debug.received / accepted / rejected | 上板观察到的新事件 / 接受单发 / 拒绝事件；不累计序号差值 |
| launcher_debug.completed / timed_out / start_failed / aborted | 到位 / 500ms未到位 / 50ms启动失败 / 已接受请求中止 |
| launcher_debug.reject_reason | 最近事件结果，接受新单发时清零；日常空闲不擦除 |
| launcher_debug.inhibit_reason | 当前供弹禁止原因，0表示无门槛阻挡，忙碌为2 |
| launcher_debug.fric_state | 0常规、1增强、2锁停 |
| launcher_debug.jam_count / recovery_ok / recovery_failed | 转轮堵转及恢复统计 |
| launcher_debug.dial_target / dial_angle / dial_speed_target / dial_speed | 目标及反馈，count或deg/s |
| launcher_debug.dial_output / dial_current | 控制及反馈电流，KT原始值；控制值不证明CAN已到达 |
| launcher_debug.fric_output / fric_speed / fric_current | 左/右轮控制原始值、rpm及反馈A |
| launcher_debug.start_pending / release_required | 已接受单发等待启动 / 等待供弹释放 |
| board.tx_pkt->shoot_pkt.single_seq / reset_seq | 下板生成的单发及复位序号 |
| Board_Rx_Info.shoot_pkt.single_seq / reset_seq | 上板收到的单发及复位序号 |
| launcher_heat | 热量来源、就绪、预算、射频及双轮反馈新鲜度 |

拒绝/禁止原因：0无、1许可关闭、2忙、3转轮未达速、4拨盘离线、5热量禁止、6启动超时、7单发未到位、8解堵中、9转轮锁停、10等待释放、11许可中断。启动失败与超时针对已接受请求计数，不额外计入 rejected；aborted包含这些未完成请求。

## 人工验收场景

| 场景 | 观察目标 |
| --- | --- |
| 达速后鼠标短按 / S2快速上拨再回中 | accepted和completed各增加1；松手不取消 |
| 上一发未完成再次点击 | rejected增加、原因2；旧单发不变，无后续补射 |
| 未达速点击 | 拒绝原因3；达速后不补该次单发 |
| 拨盘启动发送失败 | start_pending最多50ms；失败计数增加，旧请求不续射 |
| 单发未到位 | 500ms停止、timed_out增加，释放再按重新取实际角度 |
| 遥控断联再重连，S1中位 | 许可满足后自动转轮；保持S2上位或左键按下不直接供弹 |
| 转轮堵转，随后恢复 | 停供弹、一次≤200ms增强，双轮连续100ms达速后恢复常规；需再触发 |
| 转轮增强失败 / 增强中断联 | 输出归零锁停；重连不复位，在线S1下→中才复位 |
| 训练与比赛配置切换 | 训练无需裁判；关闭训练后未获初始有效热量时禁发 |

## 完整配置

源码：[task_up/Application/ConfigLayer/launcher_config.h](G:/STM32_ALL/RM_code/Train_code_plus/task_up/Application/ConfigLayer/launcher_config.h)

```c
#ifndef __LAUNCHER_CONFIG_H
#define __LAUNCHER_CONFIG_H

/* 摩擦轮常规速度控制 */
#define LAUNCHER_DIAL_ENABLE              1u // 拨盘控制使能，0/1
#define LAUNCHER_REPEAT_ENABLE            1u // 连发使能，0/1
#define LAUNCHER_FRIC_TARGET_RPM          1500.0f // 轮速目标，rpm
#define LAUNCHER_FRIC_RAMP_RPM_PER_MS     20.0f // 升速斜率，rpm/ms
#define LAUNCHER_FRIC_STOP_RAMP_RPM_PER_MS 20.0f // 降速斜率，rpm/ms
#define LAUNCHER_FRIC_STOP_SPEED_RPM      100.0f // 停转门槛，rpm
#define LAUNCHER_FRIC_STOP_CONFIRM_MS     1000u // 停转确认，ms
#define LAUNCHER_FRIC_READY_TOL_RPM       500.0f // 达速容差，rpm
#define LAUNCHER_FRIC_READY_TIME_MS       100u // 达速确认，ms
#define LAUNCHER_FRIC_KP                  2.0f // 比例，原始值/rpm
#define LAUNCHER_FRIC_KI                  1.0f // 积分，原始值/rpm每拍
#define LAUNCHER_FRIC_KD                  0.0f // 微分，原始值/rpm每拍
#define LAUNCHER_FRIC_INTEGRAL_MAX        500.0f // 误差积分限幅，rpm拍
#define LAUNCHER_FRIC_KFF                 0.0f // 前馈系数，非负
#define LAUNCHER_FRIC_OUT_MAX             5000.0f // 控制电流限幅，原始值
#define LAUNCHER_FRIC_L_DIRECTION         1.0f // 方向符号，仅±1
#define LAUNCHER_FRIC_R_DIRECTION         -1.0f // 方向符号，仅±1

/* 转轮堵转仅尝试一次增强 */
#define LAUNCHER_FRIC_START_GRACE_MS       500u // 启动检测宽限，ms
#define LAUNCHER_FRIC_JAM_SPEED_RPM        300.0f // 堵转速度门槛，rpm
#define LAUNCHER_FRIC_JAM_OUTPUT_RAW       3000.0f // 控制电流门槛，原始值
#define LAUNCHER_FRIC_JAM_CONFIRM_MS       200u // 堵转确认时间，ms
#define LAUNCHER_FRIC_BOOST_KP             4.0f // 增强比例，原始值/rpm
#define LAUNCHER_FRIC_BOOST_TIME_MS        200u // 单次增强上限，ms
#define LAUNCHER_DIAL_START_RETRY_MS       50u // 单发启动等待上限，ms

/* 拨盘：65536 count 为一圈，一发走一圈。 */
#define LAUNCHER_DIAL_AUTO_RESET_ENABLE   0u // 历史定位开关，固定0
#define LAUNCHER_DIAL_READY_HOLD_ENABLE   1u // 待发位置保持，0/1

#define LAUNCHER_DIAL_ANGLE_KP            0.08f // 比例，deg/s/count
#define LAUNCHER_DIAL_ANGLE_KI            0.0f // 积分，deg/s/count每拍
#define LAUNCHER_DIAL_ANGLE_KD            0.0f // 微分，deg/s/count每拍
#define LAUNCHER_DIAL_ANGLE_INTEGRAL_MAX  0.0f // 误差积分限幅，count拍
#define LAUNCHER_DIAL_ANGLE_DEADBAND      0.0f // 位置误差死区，count

#define LAUNCHER_DIAL_SPEED_KP            0.40f // 比例，原始值/(deg/s)
#define LAUNCHER_DIAL_SPEED_KI            0.05f // 积分增益，按1ms拍
#define LAUNCHER_DIAL_SPEED_KD            0.0f // 微分增益，按1ms拍
#define LAUNCHER_DIAL_SPEED_INTEGRAL_MAX  500.0f // 速度误差积分，deg/s拍
#define LAUNCHER_DIAL_SPEED_OUT_MAX       2000.0f // 电流限幅，原始值

#define LAUNCHER_DIAL_ANGLE_SIGN          1.0f // 方向符号，仅±1
#define LAUNCHER_DIAL_SPEED_SIGN          1.0f // 方向符号，仅±1
#define LAUNCHER_DIAL_OUTPUT_SIGN         1.0f // 方向符号，仅±1
#define LAUNCHER_DIAL_DIRECTION           1.0f // 方向符号，仅±1

#define LAUNCHER_DIAL_CURRENT_LIMIT       2000.0f // 电流限幅，原始值≤2000
#define LAUNCHER_DIAL_MAX_SPEED_DPS       7000u // 位置环速度上限，deg/s

#define LAUNCHER_DIAL_RESET_ANGLE         31259.0f // 定位目标，count
#define LAUNCHER_DIAL_RESET_TIMEOUT_MS    1000u // 定位超时，ms

#define LAUNCHER_DIAL_ONE_SHOT_ANGLE      65536.0f // 单发位移，count
#define LAUNCHER_DIAL_REVERSE_ANGLE       65536.0f // 反向退让，count
#define LAUNCHER_DIAL_STOP_ERROR          500.0f // 到位容差，count
#define LAUNCHER_DIAL_SINGLE_TIMEOUT_MS   500u // 单发运行超时，ms
#define LAUNCHER_DIAL_REVERSE_TIMEOUT_MS  200u // 反转超时，ms
#define LAUNCHER_DIAL_RELOAD_TIMEOUT_MS   200u // 退让复位超时，ms

/* 连发独立速度环：15 圈/s。 */
#define LAUNCHER_DIAL_REPEAT_SPEED_DPS    5400u // 连发速度上限，deg/s
#define LAUNCHER_DIAL_REPEAT_KP           0.35f // 比例，原始值/(deg/s)
#define LAUNCHER_DIAL_REPEAT_KI           0.0f // 积分增益，按1ms拍
#define LAUNCHER_DIAL_REPEAT_KD           0.0f // 微分增益，按1ms拍
#define LAUNCHER_DIAL_REPEAT_INTEGRAL_MAX 500.0f // 速度误差积分，deg/s拍
#define LAUNCHER_DIAL_REPEAT_OUT_MAX      2000.0f // 电流限幅，原始值

/* 堵转后反向退让，再回到原供弹目标。 */
#define LAUNCHER_DIAL_JAM_ENABLE          0u // 历史反转开关，固定0
#define LAUNCHER_DIAL_JAM_CURRENT_RAW     600 // 堵转反馈电流，原始值
#define LAUNCHER_DIAL_JAM_SPEED_DPS       10 // 堵转速度门槛，deg/s
#define LAUNCHER_DIAL_JAM_CONFIRM_TICKS   200u // 堵转确认拍数，1ms/拍

#define LAUNCHER_DIAL_SAFE_STOP_RETRY_MS  50u // 停机发送重试间隔，ms

/* 拨盘释放后的主动制动，避免直接断力后的惯性和回弹。 */
#define LAUNCHER_DIAL_BRAKE_KP            0.2f // 比例，原始值/(deg/s)
#define LAUNCHER_DIAL_BRAKE_KI            0.0f // 积分增益，按1ms拍
#define LAUNCHER_DIAL_BRAKE_KD            0.0f // 微分增益，按1ms拍
#define LAUNCHER_DIAL_BRAKE_INTEGRAL_MAX  0.0f // 速度误差积分，deg/s拍
#define LAUNCHER_DIAL_BRAKE_OUT_MAX       1500.0f // 电流限幅，原始值
#define LAUNCHER_DIAL_BRAKE_STOP_SPEED_DPS 20u // 制动停转门槛，deg/s
#define LAUNCHER_DIAL_BRAKE_TIMEOUT_MS    120u // 制动超时，ms

/* 17 mm热量预算与限频 */
#define LAUNCHER_HEAT_PER_SHOT             10.0f // 每发增热，热量单位
#define LAUNCHER_HEAT_WARN                200.0f // 降速起点，热量单位
#define LAUNCHER_HEAT_SATURATE             50.0f // 平衡区起点，热量单位
#define LAUNCHER_HEAT_MARGIN               20.0f // 安全余量，热量单位，至少10
#define LAUNCHER_HEAT_STOP LAUNCHER_HEAT_MARGIN // 连发停发余量，热量单位
#define LAUNCHER_HEAT_RESUME               30.0f // 恢复余量，热量单位
#define LAUNCHER_HEAT_MAX_RATE             15.0f // 最高射频，发/s
#define LAUNCHER_HEAT_D3_TIMEOUT_MS        100u // 热量链路超时，ms
#define LAUNCHER_HEAT_TRAINING_ENABLE        1u // 固定参数训练，0/1
#define LAUNCHER_HEAT_TRAINING_LIMIT     300.0f // 训练上限，热量单位
#define LAUNCHER_HEAT_TRAINING_COOLING   150.0f // 训练冷却，热量单位/s

#endif
```

## 完整类型与接口

### rc_mouse_shot_t — task_down/Application/ProtocolLayer/rc_protocol.h

```c
typedef struct
{
    uint32_t press_seq; // 左键按下序号，uint32循环
    uint32_t release_seq; // 左键释放序号，uint32循环
    uint8_t pressed; // 当前左键电平，0/1
} rc_mouse_shot_t;
```

### Board_Shoot_Pkt_t — task_down/Application/ProtocolLayer/board_protocol.h

```c
typedef struct{
	uint8_t launch_state; /* 摩擦轮许可，0/1 */
  uint8_t shoot_mode;   /* 0 = 单发，1 = 连发 */
	uint8_t shoot_level;  /* 发射触发电平，0/1 */
    uint8_t single_seq; // 单发事件序号，0~255循环
    uint8_t reset_seq; // 人工复位序号，0~255循环
}Board_Shoot_Pkt_t;
```

### Board_Shoot_Pkt_t — task_up/Application/ProtocolLayer/communicate.h

```c
typedef struct
{
    uint8_t launch_state; /* 摩擦轮许可，0/1 */
    uint8_t shoot_mode;   /* 0 = 单发，1 = 连发 */
    uint8_t shoot_level;  /* 发射触发电平，0/1 */
    uint8_t is_hole;      /* 狗洞请求，0/1 */
    uint8_t single_seq; // 单发事件序号，0~255循环
    uint8_t reset_seq; // 人工复位序号，0~255循环
} Board_Shoot_Pkt_t;
```

### launcher_reject_e — task_up/Application/ModuleLayer/launcher.h

```c
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
```

### launcher_fric_state_e — task_up/Application/ModuleLayer/launcher.h

```c
typedef enum
{
    LAUNCHER_FRIC_NORMAL = 0, // 常规速度环，值0
    LAUNCHER_FRIC_BOOST, // 限时增强，值1
    LAUNCHER_FRIC_LOCKED, // 失败锁停，值2
} launcher_fric_state_e;
```

### launcher_debug_t — task_up/Application/ModuleLayer/launcher.h

```c
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
```

### launcher_runtime_t — task_up/Application/ModuleLayer/launcher.c

```c
typedef struct
{
    uint32_t start_tick; // 启动请求时刻，ms
    uint32_t power_tick; // 转轮使能时刻，ms
    uint32_t jam_tick; // 疑似堵转起点，ms
    uint32_t boost_tick; // 增强开始时刻，ms
    uint8_t seq_seen; // 报文序号基准有效，0/1
    uint8_t single_seq; // 最近单发序号，0~255
    uint8_t reset_seq; // 最近复位序号，0~255
    uint8_t start_pending; // 已接受单发待启动，0/1
    uint8_t release_required; // 供弹等待释放，0/1
    uint8_t power_seen; // 转轮使能基准有效，0/1
    uint8_t jam_seen; // 堵转确认基准有效，0/1
    launcher_fric_state_e fric_state; // 转轮恢复阶段，0~2
} launcher_runtime_t;
```

```c
void Rc_GetMouseShotSnapshot(rc_mouse_shot_t *snapshot);
extern volatile launcher_debug_t launcher_debug;
```

## 新增状态声明

下板 `launch.c`：

```c
static uint8_t launch_single_seq;
static uint8_t launch_reset_seq;
static uint8_t launch_s1_seen;
static uint8_t launch_s1_previous;
static uint8_t launch_source = 2u;
static uint8_t launch_mouse_armed;
static uint32_t launch_mouse_press_seq;
static uint32_t launch_mouse_release_seq;
```

下板 `rc_protocol.c`：

```c
static volatile rc_mouse_shot_t rc_mouse_shot;
static uint8_t rc_mouse_shot_seen;
```

上板 `launcher.c`：

```c
volatile launcher_debug_t launcher_debug;
static launcher_runtime_t launcher_runtime;
```

## 完整函数

### Rc_GetMouseShotSnapshot

源码：[task_down/Application/ProtocolLayer/rc_protocol.c](G:/STM32_ALL/RM_code/Train_code_plus/task_down/Application/ProtocolLayer/rc_protocol.c)

```c
void Rc_GetMouseShotSnapshot(rc_mouse_shot_t *snapshot)
{
    uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    snapshot->press_seq = rc_mouse_shot.press_seq;
    snapshot->release_seq = rc_mouse_shot.release_seq;
    snapshot->pressed = rc_mouse_shot.pressed;
    __set_PRIMASK(irq_state);
}
```

### rc_update

源码：[task_down/Application/ProtocolLayer/rc_protocol.c](G:/STM32_ALL/RM_code/Train_code_plus/task_down/Application/ProtocolLayer/rc_protocol.c)

```c
void rc_update(rc_dev_t *rc_sen, uint8_t *rxBuf)
{

	rc_data_t *rc_info = rc_sen->info;
    uint8_t mouse_pressed = rxBuf[12] & 0x01u;
    uint8_t reconnect = (rc_info->offline_cnt >= rc_info->offline_max_cnt) ? 1u : 0u;
    /* 重连首帧只建立按键基准 */
    if ((rc_mouse_shot_seen != 0u) && (reconnect == 0u))
    {
        if ((mouse_pressed != 0u) && (rc_mouse_shot.pressed == 0u))
        {
            rc_mouse_shot.press_seq++;
        }
        else if ((mouse_pressed == 0u) && (rc_mouse_shot.pressed != 0u))
        {
            rc_mouse_shot.release_seq++;
        }
    }
    rc_mouse_shot.pressed = mouse_pressed;
    rc_mouse_shot_seen = 1u;
	rc_info->offline_cnt=0; /* 收到帧则在线 */
	/* 遥控器 */
	rc_info->ch0 = (rxBuf[0] | rxBuf[1] << 8) & 0x07FF; /* 右横 */
	rc_info->ch0 -= 1024;
	rc_info->ch1 = (rxBuf[1] >> 3 | rxBuf[2] << 5) & 0x07FF; /* 右纵 */
	rc_info->ch1 -= 1024;
	rc_info->ch2 = (rxBuf[2] >> 6 | rxBuf[3] << 2 | rxBuf[4] << 10) & 0x07FF; /* 左横 */
	rc_info->ch2 -= 1024;
	rc_info->ch3 = (rxBuf[4] >> 1 | rxBuf[5] << 7) & 0x07FF; /* 左纵 */
	rc_info->ch3 -= 1024;

	rc_info->thumbwheel.value = ((int16_t)rxBuf[16] | ((int16_t)rxBuf[17] << 8)) & 0x07ff; /* 波轮 */
	rc_info->thumbwheel.value -= 1024;

	if(abs(rc_info->thumbwheel.value)>660)
	{
		rc_info->thumbwheel.value=0;
	}

	rc_info->s1.value = ((rxBuf[5] >> 4) & 0x000C) >> 2; /* S1 档位 */
	rc_info->s2.value = (rxBuf[5] >> 4) & 0x0003; /* S2 档位 */
	/*遥控器限位置零*/
	if(rc_dev.info->ch3== -660)
	{
		rc_dev.info->ch3=0;
	}

	/* 键鼠 */
	rc_info->mouse_vx = rxBuf[6]  | (rxBuf[7 ] << 8); /* 鼠标 X */
	rc_info->mouse_vy = rxBuf[8]  | (rxBuf[9 ] << 8); /* 鼠标 Y */
	rc_info->mouse_vz = rxBuf[10] | (rxBuf[11] << 8); /* 鼠标滚轮 */
  rc_info->mouse_btn_l.value = rxBuf[12] & 0x01; /* 左键 */
  rc_info->mouse_btn_r.value = rxBuf[13] & 0x01; /* 右键 */
  rc_info->key_v   =  rxBuf[14] | (rxBuf[15] << 8); /* 键盘位图 */
  rc_info->update_seq++;

  rc_info->W.value = 	KEY_PRESSED_W;
  rc_info->S.value =    KEY_PRESSED_S;
  rc_info->A.value = 	KEY_PRESSED_A;
  rc_info->D.value = 	KEY_PRESSED_D;
  rc_info->Shift.value = 	KEY_PRESSED_SHIFT;
  rc_info->Ctrl.value  = 	KEY_PRESSED_CTRL;
  rc_info->Q.value = 	KEY_PRESSED_Q;
  rc_info->E.value = 	KEY_PRESSED_E;
  rc_info->R.value = 	KEY_PRESSED_R;
  rc_info->F.value = 	KEY_PRESSED_F;
  rc_info->G.value = 	KEY_PRESSED_G;
  rc_info->Z.value = 	KEY_PRESSED_Z;
  rc_info->X.value = 	KEY_PRESSED_X;
  rc_info->C.value = 	KEY_PRESSED_C;
  rc_info->V.value = 	KEY_PRESSED_V;
  rc_info->B.value = 	KEY_PRESSED_B;

	rc_info->offline_cnt = 0;
	tt1 = tt2;
	tt2 = micros();
	ttp1 = tt2 - tt1;

}
```

### Launch_Data_Update

源码：[task_down/Application/ModuleLayer/launch.c](G:/STM32_ALL/RM_code/Train_code_plus/task_down/Application/ModuleLayer/launch.c)

```c
static void Launch_Data_Update(Launch_t *launch)
{
    rc_mouse_shot_t mouse;
    uint8_t s1;
    uint8_t s2;
    uint8_t keyboard;
    uint8_t previous_s2;

    Rc_GetMouseShotSnapshot(&mouse);
    launch->state = L_LOCK;
    launch->mode = SINGLE_SHOT;
    launch->shoot_level = 0u;
    if (rc_dev.work_state != DEV_ONLINE)
    {
        Launch_Reset_Shoot_Arm();
        Launch_Reset_S2_Filter();
        launch_s1_seen = 0u;
        launch_source = 2u;
        launch_mouse_armed = 0u;
        launch_mouse_press_seq = mouse.press_seq;
        launch_mouse_release_seq = mouse.release_seq;
        return;
    }

    s1 = (uint8_t)rc_dev.info->s1.value;
    if ((launch_s1_seen != 0u) &&
        (launch_s1_previous == RC_SW_DOWN) && (s1 == RC_SW_MID))
    {
        launch_reset_seq++;
    }
    launch_s1_seen = 1u;
    launch_s1_previous = s1;
    keyboard = Chassis_Input_IsKeyboardMode();
    if (launch_source != keyboard)
    {
        Launch_Reset_Shoot_Arm();
        Launch_Reset_S2_Filter();
        launch_mouse_press_seq = mouse.press_seq;
        launch_mouse_release_seq = mouse.release_seq;
        launch_mouse_armed = (mouse.pressed == 0u) ? 1u : 0u;
        launch_source = keyboard;
    }

    if (keyboard != 0u)
    {
        launch->state = L_UNLOCK;
        if ((mouse.pressed == 0u) ||
            (mouse.release_seq != launch_mouse_release_seq))
        {
            launch_mouse_armed = 1u;
        }
        if ((mouse.press_seq != launch_mouse_press_seq) &&
            (launch_mouse_armed != 0u))
        {
            launch_single_seq++;
        }
        launch_mouse_press_seq = mouse.press_seq;
        launch_mouse_release_seq = mouse.release_seq;
        if ((mouse.pressed != 0u) && (launch_mouse_armed != 0u))
        {
            launch->shoot_level = 1u;
            launch->mode = (rc_dev.info->mouse_btn_l.status == long_press) ?
                           REPEAT_SHOT : SINGLE_SHOT;
        }
        return;
    }

    launch_mouse_press_seq = mouse.press_seq;
    launch_mouse_release_seq = mouse.release_seq;
#if BOARD_LIFT_ENABLE
    if ((board.tx_pkt->gimbal_target_pkt.is_hole != 0u) ||
        (board.rx_meg->state_meg.is_down != 2u))
    {
        Launch_Reset_Shoot_Arm();
        Launch_Reset_S2_Filter();
        return;
    }
#endif
    s2 = Launch_Filter_S2((uint8_t)rc_dev.info->s2.value);
    if (((s1 != RC_SW_UP) && (s1 != RC_SW_MID)) ||
        ((s1 == RC_SW_UP) && (s2 == RC_SW_DOWN)))
    {
        Launch_Reset_Shoot_Arm();
        return;
    }

    launch->state = L_UNLOCK;
    previous_s2 = launch_shoot_previous_switch;
    if (s2 == RC_SW_MID)
    {
        launch_shoot_armed = 1u;
    }
    if ((launch_shoot_switch_seen != 0u) &&
        (launch_shoot_armed != 0u) && (s2 == RC_SW_UP))
    {
        launch->mode = (s1 == RC_SW_UP) ? REPEAT_SHOT : SINGLE_SHOT;
        launch->shoot_level = 1u;
        if ((previous_s2 != RC_SW_UP) && (launch->mode == SINGLE_SHOT))
        {
            launch_single_seq++;
        }
    }
    launch_shoot_switch_seen = 1u;
    launch_shoot_previous_switch = s2;
}
```

### Launch_Cmd_Transmit

源码：[task_down/Application/ModuleLayer/launch.c](G:/STM32_ALL/RM_code/Train_code_plus/task_down/Application/ModuleLayer/launch.c)

```c
static void Launch_Cmd_Transmit(Launch_t *launch)
{
    uint32_t irq_state = __get_PRIMASK();
    /* 防止发送任务读到半更新字段 */
    __disable_irq();
    board.tx_pkt->shoot_pkt.launch_state = (uint8_t)launch->state;
    board.tx_pkt->shoot_pkt.shoot_mode = (uint8_t)launch->mode;
    board.tx_pkt->shoot_pkt.shoot_level = launch->shoot_level;
    board.tx_pkt->shoot_pkt.single_seq = launch_single_seq;
    board.tx_pkt->shoot_pkt.reset_seq = launch_reset_seq;
    __set_PRIMASK(irq_state);
}
```

### Board_Tx_Pkt_01

源码：[task_down/Application/ProtocolLayer/board_protocol.c](G:/STM32_ALL/RM_code/Train_code_plus/task_down/Application/ProtocolLayer/board_protocol.c)

```c
void Board_Tx_Pkt_01(Board_t* board)
{
	board->status->gimbal_d1_tx_ok = 0u;
	board->status->gimbal_d2_tx_ok = 0u;
	memset(pkt_01, 0, 8); /* 清空缓存 */
	
	pkt_01[0] |= (board->tx_pkt->car_pkt.car_state & 0x03) << 0;   /* 车辆状态 */
	pkt_01[0] |= (board->tx_pkt->car_pkt.gimbal_mode & 0x01) << 2; /* 云台模式 */
	pkt_01[0] |= (board->tx_pkt->car_pkt.vision_mode & 0x07) << 3; /* 视觉模式 */
	pkt_01[0] |= (board->tx_pkt->car_pkt.game_start & 0x01) << 6;  /* 比赛开始 */
	pkt_01[0] |= (board->tx_pkt->car_pkt.my_color & 0x01) << 7;    /* 己方颜色 */
	
	uint16_t t1,t2; /* 速度压缩值 */
	
	t1 = float_to_uint(board->tx_pkt->car_pkt.v_x,-8000.f,8000.f,16); /* v_x 压缩值 */
	t2 = float_to_uint(board->tx_pkt->car_pkt.v_y,-8000.f,8000.f,16); /* v_y 压缩值 */
	
	pkt_01[1] = t1>>8; /* v_x 高字节 */
	pkt_01[2] = t1;    /* v_x 低字节 */
	pkt_01[3] = t2>>8; /* v_y 高字节 */
	pkt_01[4] = t2;    /* v_y 低字节 */

									 
	pkt_01[5] |= (board->tx_pkt->shoot_pkt.launch_state & 0x01) << 0; /* 发射许可 */
	pkt_01[5] |= (board->tx_pkt->shoot_pkt.shoot_mode & 0x01) << 1; /* 发射模式 */
	pkt_01[5] |= (board->tx_pkt->shoot_pkt.shoot_level & 0x01) << 2; /* 触发电平 */
	pkt_01[5] |= (board->tx_pkt->gimbal_target_pkt.is_hole & 0x01) << 3; /* 过洞标志 */
    pkt_01[6] = board->tx_pkt->shoot_pkt.single_seq;
    pkt_01[7] = board->tx_pkt->shoot_pkt.reset_seq;
	

	board->status->gimbal_d1_tx_ok =
		(CAN_SendData(&hfdcan2, ID_PKT_01, pkt_01) == HAL_OK) ? 1u : 0u;
	
	
}
```

### Board_Rx_Pkt_01

源码：[task_up/Application/ProtocolLayer/communicate.c](G:/STM32_ALL/RM_code/Train_code_plus/task_up/Application/ProtocolLayer/communicate.c)

```c
static void Board_Rx_Pkt_01(uint8_t *rxbuf)
{
    Board_Rx_Info.state_pkt.car_state = rxbuf[0] & 0x03;          /* 车辆状态 */
    Board_Rx_Info.state_pkt.gimbal_mode = (rxbuf[0] >> 2) & 0x01; /* 云台模式 */
    Board_Rx_Info.state_pkt.vision_mode = (rxbuf[0] >> 3) & 0x07; /* 视觉模式 */
    Board_Rx_Info.state_pkt.game_start = (rxbuf[0] >> 6) & 0x01;  /* 比赛开始 */
    Board_Rx_Info.state_pkt.my_color = (rxbuf[0] >> 7) & 0x01;    /* 己方颜色 */

    Board_Rx_Shoot_Flags = rxbuf[5] & 0x0Fu;
    Board_Rx_Info.shoot_pkt.launch_state = rxbuf[5] & 0x01;       /* 发射许可 */
    Board_Rx_Info.shoot_pkt.shoot_mode = (rxbuf[5] >> 1) & 0x01;  /* 发射模式 */
    Board_Rx_Info.shoot_pkt.shoot_level = (rxbuf[5] >> 2) & 0x01; /* 触发电平 */
    Board_Rx_Info.shoot_pkt.is_hole = (rxbuf[5] >> 3) & 0x01;     /* 过洞标志 */
    Board_Rx_Info.shoot_pkt.single_seq = rxbuf[6];
    Board_Rx_Info.shoot_pkt.reset_seq = rxbuf[7];
}
```

### Launcher_DialAtTarget

源码：[task_up/Application/ModuleLayer/launcher.c](G:/STM32_ALL/RM_code/Train_code_plus/task_up/Application/ModuleLayer/launcher.c)

```c
static uint8_t Launcher_DialAtTarget(int64_t target)
{
    uint32_t raw_error = (uint32_t)target - (uint32_t)Launcher_DialAngle();
    int64_t error = (raw_error <= 0x7FFFFFFFu) ? (int64_t)raw_error :
                    (int64_t)raw_error - 4294967296LL;

    return (Launcher_AbsInt64(error) <=
            (int64_t)LAUNCHER_DIAL_STOP_ERROR) ? 1u : 0u;
}
```

### Launcher_DialApplyTorque

源码：[task_up/Application/ModuleLayer/launcher.c](G:/STM32_ALL/RM_code/Train_code_plus/task_up/Application/ModuleLayer/launcher.c)

```c
static void Launcher_DialApplyTorque(int16_t current)
{
    if ((dail_motor.W_iqControl != NULL) && (dail_motor.tx_W_cmd != NULL))
    {
        dail_motor.W_iqControl(&dail_motor, current); /* 写入电流环 */
        launcher_debug.dial_output = (float)current;
        (void)dail_motor.tx_W_cmd(&dail_motor, TORQUE_CLOSE_LOOP_ID);
    }
}
```

### Launcher_DialStop

源码：[task_up/Application/ModuleLayer/launcher.c](G:/STM32_ALL/RM_code/Train_code_plus/task_up/Application/ModuleLayer/launcher.c)

```c
static HAL_StatusTypeDef Launcher_DialStop(void)
{
    Launcher_DialClearPid();
    if (dail_motor.tx_W_cmd == NULL)
    {
        return HAL_ERROR;
    }

    launcher_debug.dial_output = 0.0f;
    return dail_motor.tx_W_cmd(&dail_motor, MOTOR_STOP_ID);
}
```

### Launcher_DialPositionControl

源码：[task_up/Application/ModuleLayer/launcher.c](G:/STM32_ALL/RM_code/Train_code_plus/task_up/Application/ModuleLayer/launcher.c)

```c
static void Launcher_DialPositionControl(int64_t target)
{
    float speed_target;    /* 位置环输出的速度目标 */
    int16_t current_output;/* 速度环输出的力矩电流 */
    uint32_t raw_error;
    int64_t angle_error;

    if (Launcher_DialOnline() == 0u)
    {
        Launcher_DialClearPid();
        Launcher_DialApplyTorque(0);
        return;
    }

    /* 模差保留跨界与大角度精度 */
    raw_error = (uint32_t)target - (uint32_t)Launcher_DialAngle();
    angle_error = (raw_error <= 0x7FFFFFFFu) ? (int64_t)raw_error :
                  (int64_t)raw_error - 4294967296LL;
    launcher_dial_angle_pid.target = (float)angle_error;
    launcher_dial_angle_pid.measure = 0.0f;
    launcher_dial_angle_pid.err =
        launcher_dial_angle_pid.target - launcher_dial_angle_pid.measure;
    single_pid_ctrl(&launcher_dial_angle_pid);

    launcher_debug.dial_speed_target = launcher_dial_angle_pid.out;
    speed_target = constrain(launcher_dial_angle_pid.out, /* 限制速度目标 */
                             -(float)LAUNCHER_DIAL_MAX_SPEED_DPS,
                             (float)LAUNCHER_DIAL_MAX_SPEED_DPS);

    launcher_dial_speed_pid.target = speed_target; /* 速度目标 */
    launcher_dial_speed_pid.measure =
        LAUNCHER_DIAL_SPEED_SIGN *
        (float)dail_motor.KT_motor_info.rx_info.speed; /* 反馈速度 */
    launcher_dial_speed_pid.err =
        launcher_dial_speed_pid.target - launcher_dial_speed_pid.measure;
    single_pid_ctrl(&launcher_dial_speed_pid);

    current_output = (int16_t)constrain( /* 电流输出限幅 */
        LAUNCHER_DIAL_OUTPUT_SIGN * launcher_dial_speed_pid.out,
        -LAUNCHER_DIAL_CURRENT_LIMIT,
        LAUNCHER_DIAL_CURRENT_LIMIT);
    Launcher_DialApplyTorque(current_output);
}
```

### Launcher_DialSpeedControl

源码：[task_up/Application/ModuleLayer/launcher.c](G:/STM32_ALL/RM_code/Train_code_plus/task_up/Application/ModuleLayer/launcher.c)

```c
static void Launcher_DialSpeedControl(void)
{
    int16_t current_output; /* 连发速度环输出 */

    if (Launcher_DialOnline() == 0u)
    {
        Launcher_DialClearPid();
        Launcher_DialApplyTorque(0);
        return;
    }

    launcher_dial_repeat_pid.target = /* 连发目标速度 */
        LAUNCHER_DIAL_DIRECTION * launcher_heat.target_rate *
        (LAUNCHER_DIAL_ONE_SHOT_ANGLE * 360.0f / 65536.0f);
    launcher_debug.dial_speed_target = launcher_dial_repeat_pid.target;
    launcher_dial_repeat_pid.measure =
        LAUNCHER_DIAL_SPEED_SIGN *
        (float)dail_motor.KT_motor_info.rx_info.speed;
    launcher_dial_repeat_pid.err =
        launcher_dial_repeat_pid.target - launcher_dial_repeat_pid.measure;
    single_pid_ctrl(&launcher_dial_repeat_pid);

    current_output = (int16_t)constrain( /* 连发电流限幅 */
        LAUNCHER_DIAL_OUTPUT_SIGN * launcher_dial_repeat_pid.out,
        -LAUNCHER_DIAL_CURRENT_LIMIT,
        LAUNCHER_DIAL_CURRENT_LIMIT);
    Launcher_DialApplyTorque(current_output);
}
```

### Launcher_DialBrakeControl

源码：[task_up/Application/ModuleLayer/launcher.c](G:/STM32_ALL/RM_code/Train_code_plus/task_up/Application/ModuleLayer/launcher.c)

```c
static void Launcher_DialBrakeControl(void)
{
    int16_t current_output;

    if (Launcher_DialOnline() == 0u)
    {
        Launcher_DialClearPid();
        Launcher_DialApplyTorque(0);
        return;
    }

    launcher_debug.dial_speed_target = 0.0f;
    launcher_dial_brake_pid.target = 0.0f;
    launcher_dial_brake_pid.measure =
        LAUNCHER_DIAL_SPEED_SIGN *
        (float)dail_motor.KT_motor_info.rx_info.speed;
    launcher_dial_brake_pid.err =
        launcher_dial_brake_pid.target -
        launcher_dial_brake_pid.measure;
    single_pid_ctrl(&launcher_dial_brake_pid);

    current_output = (int16_t)constrain(
        LAUNCHER_DIAL_OUTPUT_SIGN * launcher_dial_brake_pid.out,
        -LAUNCHER_DIAL_CURRENT_LIMIT,
        LAUNCHER_DIAL_CURRENT_LIMIT);
    Launcher_DialApplyTorque(current_output);
}
```

### Launcher_DialSafeStop

源码：[task_up/Application/ModuleLayer/launcher.c](G:/STM32_ALL/RM_code/Train_code_plus/task_up/Application/ModuleLayer/launcher.c)

```c
static void Launcher_DialSafeStop(uint32_t now)
{
    if (((launcher_dial_stopped == 0u) ||
         ((now - launcher_dial_stop_tick) >=
          LAUNCHER_DIAL_SAFE_STOP_RETRY_MS)) &&
        (Launcher_DialStop() == HAL_OK))
    {
        launcher_dial_stopped = 1u;
        launcher_dial_stop_tick = now;
    }

    launcher.jam_tick = 0u;
}
```

### Launcher_DialControl

源码：[task_up/Application/ModuleLayer/launcher.c](G:/STM32_ALL/RM_code/Train_code_plus/task_up/Application/ModuleLayer/launcher.c)

```c
static void Launcher_DialControl(uint8_t shoot_active)
{
    uint32_t now = HAL_GetTick();

    /* 重新发射时取消制动。 */
    if (shoot_active != 0u)
    {
        launcher_dial_braking = 0u;
    }
    else
    {
        if ((launcher.state != LAUNCHER_SLEEP) &&
            (launcher.state != LAUNCHER_STOPPING) &&
            (launcher.state != LAUNCHER_FAULT))
        {
            launcher.state = LAUNCHER_READY;
            launcher.state_tick = now;
            launcher.jam_tick = 0u;
        }

        if (launcher_dial_stopped == 0u)
        {
            if (launcher_dial_braking == 0u)
            {
                launcher_dial_braking = 1u;
                launcher_dial_brake_tick = now;
                Launcher_DialClearPid();
            }

            Launcher_DialBrakeControl();
            if ((fabsf((float)dail_motor.KT_motor_info.rx_info.speed) <=
                 (float)LAUNCHER_DIAL_BRAKE_STOP_SPEED_DPS) ||
                ((now - launcher_dial_brake_tick) >=
                 LAUNCHER_DIAL_BRAKE_TIMEOUT_MS))
            {
                launcher_dial_braking = 0u;
                Launcher_DialSafeStop(now);
            }
            return;
        }

        launcher_dial_braking = 0u;
        Launcher_DialSafeStop(now);
        return;
    }
#if LAUNCHER_DIAL_ENABLE
#if !LAUNCHER_DIAL_READY_HOLD_ENABLE
    if (launcher.state == LAUNCHER_READY)
    {
        Launcher_DialStop();
        return;
    }
#endif

    if (launcher.state == LAUNCHER_REPEAT)
    {
        Launcher_DialSpeedControl();
    }
    else
    {
        Launcher_DialPositionControl(launcher_dial_target);
    }
#else
    if (dail_motor.tx_W_cmd != NULL)
    {
        (void)dail_motor.tx_W_cmd(&dail_motor, MOTOR_CLOSE_ID);
    }
#endif
}
```

### Launcher_FricControl

源码：[task_up/Application/ModuleLayer/launcher.c](G:/STM32_ALL/RM_code/Train_code_plus/task_up/Application/ModuleLayer/launcher.c)

```c
static void Launcher_FricControl(uint8_t enable)
{
    for (uint8_t i = 0u; i < SHOOT_FRIC_NUM; i++)
    {
        pid_ctrl_t *pid = rm_motor[i].ctrl->speed_ctrl; /* 当前速度环 */
        float direction = (i == SHOOT_FRIC_L) ?
                          LAUNCHER_FRIC_L_DIRECTION :
                          LAUNCHER_FRIC_R_DIRECTION;

        if ((enable != 0u) && (Launcher_FricOnline(i) != 0u))
        {
            pid->kp = (launcher_runtime.fric_state == LAUNCHER_FRIC_BOOST) ?
                      LAUNCHER_FRIC_BOOST_KP : LAUNCHER_FRIC_KP;
            pid->target = direction * launcher.fric_target_rpm; /* 目标转速 */
            pid->measure = (float)rm_motor[i].rx_info->encoder_speed; /* 反馈转速 */
            pid->err = pid->target - pid->measure;

            if (launcher.state == LAUNCHER_STOPPING)
            {
                pid->integral = 0.0f;
            }

            single_pid_ctrl(pid);
            rm_motor[i].tx_info->torque = /* 力矩限幅 */
                constrain(pid->out,
                          -LAUNCHER_FRIC_OUT_MAX,
                          LAUNCHER_FRIC_OUT_MAX);
        }
        else
        {
            pid->integral = 0.0f;
            pid->last_err = 0.0f;
            pid->out = 0.0f;
            rm_motor[i].tx_info->torque = 0.0f;
        }
    }

    RM_Group.group_set_torque(&RM_Group);
}
```

### Launcher_Init

源码：[task_up/Application/ModuleLayer/launcher.c](G:/STM32_ALL/RM_code/Train_code_plus/task_up/Application/ModuleLayer/launcher.c)

```c
void Launcher_Init(void)
{
    pid_ctrl_t *pid; /* 摩擦轮速度环临时指针 */

    memset((void *)&launcher_debug, 0, sizeof(launcher_debug));
    memset(&launcher_runtime, 0, sizeof(launcher_runtime));
    memset(&launcher_heat, 0, sizeof(launcher_heat));
    memset(&launcher_heat_runtime, 0, sizeof(launcher_heat_runtime));
    launcher_heat_runtime.update_tick = HAL_GetTick();
    launcher_heat.blocked = 1u;

    launcher.state = LAUNCHER_SLEEP;
    launcher.state_tick = 0u;
    launcher.last_repeat_tick = 0u;
    launcher.jam_tick = 0u;
    launcher.fric_target_rpm = 0.0f;
    launcher.fric_l_speed_rpm = 0.0f;
    launcher.fric_r_speed_rpm = 0.0f;
    launcher.dial_angle = 0;
    launcher.dial_target_angle = 0;
    launcher.dial_zero_angle = 0;
    launcher.enabled = 0u;
    launcher.fric_ready = 0u;
    launcher.dial_online = 0u;
    launcher.last_shoot_level = 0u;
    launcher.fault = 0u;

    launcher_fric_ready_count = 0u;
    launcher_fric_stop_count = 0u;
    launcher_dial_last_online = 0u;
    launcher_dial_stopped = 1u;
    launcher_dial_braking = 0u;
    launcher_dial_brake_tick = 0u;
    launcher_dial_stop_tick = 0u;
    launcher_dial_target = 0;

    for (uint8_t i = 0u; i < SHOOT_FRIC_NUM; i++)
    {
        pid = rm_motor[i].ctrl->speed_ctrl;
        pid->kp = LAUNCHER_FRIC_KP;
        pid->ki = LAUNCHER_FRIC_KI;
        pid->kd = LAUNCHER_FRIC_KD;
        pid->integral = 0.0f;
        pid->integral_max = LAUNCHER_FRIC_INTEGRAL_MAX;
        pid->out_max = LAUNCHER_FRIC_OUT_MAX;
        pid->deadband = 0.0f;
        pid->d_filter_alpha = 0.0f;
        pid->out = 0.0f;
    }

    launcher_dial_angle_pid.kp = LAUNCHER_DIAL_ANGLE_KP;
    launcher_dial_angle_pid.ki = LAUNCHER_DIAL_ANGLE_KI;
    launcher_dial_angle_pid.kd = LAUNCHER_DIAL_ANGLE_KD;
    launcher_dial_angle_pid.integral = 0.0f;
    launcher_dial_angle_pid.integral_max = LAUNCHER_DIAL_ANGLE_INTEGRAL_MAX;
    launcher_dial_angle_pid.out_max = (float)LAUNCHER_DIAL_MAX_SPEED_DPS;
    launcher_dial_angle_pid.deadband = LAUNCHER_DIAL_ANGLE_DEADBAND;
    launcher_dial_angle_pid.d_filter_alpha = 0.0f;
    launcher_dial_angle_pid.out = 0.0f;

    launcher_dial_speed_pid.kp = LAUNCHER_DIAL_SPEED_KP;
    launcher_dial_speed_pid.ki = LAUNCHER_DIAL_SPEED_KI;
    launcher_dial_speed_pid.kd = LAUNCHER_DIAL_SPEED_KD;
    launcher_dial_speed_pid.integral = 0.0f;
    launcher_dial_speed_pid.integral_max =
        LAUNCHER_DIAL_SPEED_INTEGRAL_MAX;
    launcher_dial_speed_pid.out_max = LAUNCHER_DIAL_SPEED_OUT_MAX;
    launcher_dial_speed_pid.deadband = 0.0f;
    launcher_dial_speed_pid.d_filter_alpha = 0.0f;
    launcher_dial_speed_pid.out = 0.0f;

    launcher_dial_repeat_pid.kp = LAUNCHER_DIAL_REPEAT_KP;
    launcher_dial_repeat_pid.ki = LAUNCHER_DIAL_REPEAT_KI;
    launcher_dial_repeat_pid.kd = LAUNCHER_DIAL_REPEAT_KD;
    launcher_dial_repeat_pid.integral = 0.0f;
    launcher_dial_repeat_pid.integral_max =
        LAUNCHER_DIAL_REPEAT_INTEGRAL_MAX;
    launcher_dial_repeat_pid.out_max = LAUNCHER_DIAL_REPEAT_OUT_MAX;
    launcher_dial_repeat_pid.deadband = 0.0f;
    launcher_dial_repeat_pid.d_filter_alpha = 0.0f;
    launcher_dial_repeat_pid.out = 0.0f;

    launcher_dial_brake_pid.kp = LAUNCHER_DIAL_BRAKE_KP;
    launcher_dial_brake_pid.ki = LAUNCHER_DIAL_BRAKE_KI;
    launcher_dial_brake_pid.kd = LAUNCHER_DIAL_BRAKE_KD;
    launcher_dial_brake_pid.integral = 0.0f;
    launcher_dial_brake_pid.integral_max =
        LAUNCHER_DIAL_BRAKE_INTEGRAL_MAX;
    launcher_dial_brake_pid.out_max = LAUNCHER_DIAL_BRAKE_OUT_MAX;
    launcher_dial_brake_pid.deadband = 0.0f;
    launcher_dial_brake_pid.d_filter_alpha = 0.0f;
    launcher_dial_brake_pid.out = 0.0f;
}
```

### Launcher_RejectSingle

源码：[task_up/Application/ModuleLayer/launcher.c](G:/STM32_ALL/RM_code/Train_code_plus/task_up/Application/ModuleLayer/launcher.c)

```c
static void Launcher_RejectSingle(launcher_reject_e reason)
{
    launcher_debug.rejected++;
    launcher_debug.reject_reason = reason;
}
```

### Launcher_CancelSingle

源码：[task_up/Application/ModuleLayer/launcher.c](G:/STM32_ALL/RM_code/Train_code_plus/task_up/Application/ModuleLayer/launcher.c)

```c
static void Launcher_CancelSingle(launcher_reject_e reason)
{
    if ((launcher_runtime.start_pending != 0u) ||
        (launcher.state == LAUNCHER_SINGLE))
    {
        launcher_debug.aborted++;
        launcher_debug.reject_reason = reason;
    }
    launcher_runtime.start_pending = 0u;
    launcher_runtime.release_required = 1u;
    launcher_dial_target = (int64_t)Launcher_DialAngle();
    launcher.state = LAUNCHER_READY;
    launcher_dial_braking = 0u;
}
```

### Launcher_PublishDebug

源码：[task_up/Application/ModuleLayer/launcher.c](G:/STM32_ALL/RM_code/Train_code_plus/task_up/Application/ModuleLayer/launcher.c)

```c
static void Launcher_PublishDebug(void)
{
    launcher.dial_target_angle = (int32_t)launcher_dial_target;
    launcher_debug.dial_target = launcher_dial_target;
    launcher_debug.dial_angle = Launcher_DialAngle();
    launcher_debug.dial_speed = (float)dail_motor.KT_motor_info.rx_info.speed;
    launcher_debug.dial_current = (float)dail_motor.KT_motor_info.rx_info.current;
    launcher_debug.fric_state = launcher_runtime.fric_state;
    launcher_debug.start_pending = launcher_runtime.start_pending;
    launcher_debug.release_required = launcher_runtime.release_required;
    for (uint8_t i = 0u; i < SHOOT_FRIC_NUM; i++)
    {
        launcher_debug.fric_output[i] = rm_motor[i].tx_info->torque;
        launcher_debug.fric_speed[i] = (float)rm_motor[i].rx_info->encoder_speed;
        launcher_debug.fric_current[i] = rm_motor[i].rx_info->torque_current;
    }
}
```

### Launcher_FricRecoveryUpdate

源码：[task_up/Application/ModuleLayer/launcher.c](G:/STM32_ALL/RM_code/Train_code_plus/task_up/Application/ModuleLayer/launcher.c)

```c
static void Launcher_FricRecoveryUpdate(uint32_t now)
{
    uint8_t blocked = 0u;
    launcher_debug.jam_elapsed_ms = 0u;
    launcher_debug.boost_elapsed_ms = 0u;
    if (launcher_runtime.fric_state == LAUNCHER_FRIC_LOCKED)
    {
        return;
    }
    if (launcher_runtime.power_seen == 0u)
    {
        launcher_runtime.power_seen = 1u;
        launcher_runtime.power_tick = now;
        launcher_runtime.jam_seen = 0u;
    }
    if (launcher_runtime.fric_state == LAUNCHER_FRIC_BOOST)
    {
        launcher_debug.boost_elapsed_ms = now - launcher_runtime.boost_tick;
        if ((launcher.fric_ready != 0u) &&
            ((now - launcher_runtime.boost_tick) <= LAUNCHER_FRIC_BOOST_TIME_MS))
        {
            launcher_runtime.fric_state = LAUNCHER_FRIC_NORMAL;
            launcher_runtime.power_tick = now;
            launcher_runtime.jam_seen = 0u;
            launcher_debug.recovery_ok++;
        }
        else if ((now - launcher_runtime.boost_tick) >= LAUNCHER_FRIC_BOOST_TIME_MS)
        {
            launcher_runtime.fric_state = LAUNCHER_FRIC_LOCKED;
            launcher.fault = 1u;
            launcher_debug.recovery_failed++;
        }
        return;
    }
    if ((now - launcher_runtime.power_tick) < LAUNCHER_FRIC_START_GRACE_MS)
    {
        return;
    }
    for (uint8_t i = 0u; i < SHOOT_FRIC_NUM; i++)
    {
        if ((Launcher_FricOnline(i) != 0u) &&
            (fabsf((float)rm_motor[i].rx_info->encoder_speed) < LAUNCHER_FRIC_JAM_SPEED_RPM) &&
            (fabsf(rm_motor[i].tx_info->torque) >= LAUNCHER_FRIC_JAM_OUTPUT_RAW))
        {
            blocked = 1u;
        }
    }
    if (blocked == 0u)
    {
        launcher_runtime.jam_seen = 0u;
        return;
    }
    if (launcher_runtime.jam_seen == 0u)
    {
        launcher_runtime.jam_seen = 1u;
        launcher_runtime.jam_tick = now;
    }
    launcher_debug.jam_elapsed_ms = now - launcher_runtime.jam_tick;
    if (launcher_debug.jam_elapsed_ms >= LAUNCHER_FRIC_JAM_CONFIRM_MS)
    {
        launcher_runtime.fric_state = LAUNCHER_FRIC_BOOST;
        launcher_runtime.boost_tick = now;
        launcher_runtime.jam_seen = 0u;
        launcher_debug.jam_count++;
        Launcher_CancelSingle(LAUNCHER_REJECT_FRIC_JAM);
        Launcher_UpdateFrictionReady(0u);
        for (uint8_t i = 0u; i < SHOOT_FRIC_NUM; i++)
        {
            rm_motor[i].ctrl->speed_ctrl->integral = 0.0f;
        }
    }
}
```

### Launcher_Work

源码：[task_up/Application/ModuleLayer/launcher.c](G:/STM32_ALL/RM_code/Train_code_plus/task_up/Application/ModuleLayer/launcher.c)

```c
void Launcher_Work(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t irq_state = __get_PRIMASK();
    Board_Shoot_Pkt_t command;
    uint8_t link_online;
    uint8_t fric_on;
    uint8_t single_event = 0u;
    uint8_t reset_event = 0u;
    uint8_t fric_enable = 0u;
    uint8_t dial_active = 0u;
    uint8_t force_stop = 0u;
    uint8_t continuous = 0u;
    launcher_reject_e inhibit = LAUNCHER_REJECT_NONE;

    __disable_irq();
    command = Board_Rx_Info.shoot_pkt;
    link_online = (Board_HeartBeat.status == DEV_ONLINE) ? 1u : 0u;
    __set_PRIMASK(irq_state);
    launcher.dial_angle = Launcher_DialAngle();
    launcher.dial_online = Launcher_DialOnline();
    Launcher_HeatUpdate(now);
    if ((launcher_dial_last_online == 0u) && (launcher.dial_online != 0u))
    {
        launcher.dial_zero_angle = launcher.dial_angle;
    }
    launcher_dial_last_online = launcher.dial_online;

    if (link_online == 0u)
    {
        launcher_runtime.seq_seen = 0u;
    }
    else if (launcher_runtime.seq_seen == 0u)
    {
        /* 首次接管不执行旧事件 */
        launcher_runtime.seq_seen = 1u;
        launcher_runtime.single_seq = command.single_seq;
        launcher_runtime.reset_seq = command.reset_seq;
        launcher_runtime.release_required = 1u;
    }
    else
    {
        single_event = (command.single_seq != launcher_runtime.single_seq) ? 1u : 0u;
        reset_event = (command.reset_seq != launcher_runtime.reset_seq) ? 1u : 0u;
        launcher_runtime.single_seq = command.single_seq;
        launcher_runtime.reset_seq = command.reset_seq;
    }
    launcher_debug.single_seq = command.single_seq;
    launcher_debug.reset_seq = command.reset_seq;
    if (single_event != 0u)
    {
        launcher_debug.received++;
    }
    if (reset_event != 0u)
    {
        launcher_runtime.fric_state = LAUNCHER_FRIC_NORMAL;
        launcher_runtime.power_seen = 0u;
        launcher_runtime.jam_seen = 0u;
        launcher.fault = 0u;
        Launcher_CancelSingle(LAUNCHER_REJECT_ABORTED);
        if (single_event != 0u)
        {
            Launcher_RejectSingle(LAUNCHER_REJECT_ABORTED);
            single_event = 0u;
        }
        Launcher_UpdateFrictionReady(0u);
        force_stop = 1u;
    }
    /* 新事件已由下板确认释放再按下 */
    if ((link_online != 0u) &&
        ((command.shoot_level == 0u) || (single_event != 0u)))
    {
        launcher_runtime.release_required = 0u;
    }
    launcher.last_shoot_level = command.shoot_level;
    fric_on = ((link_online != 0u) && (command.launch_state != 0u) &&
               (Launcher_FricOnline(SHOOT_FRIC_L) != 0u) &&
               (Launcher_FricOnline(SHOOT_FRIC_R) != 0u)) ? 1u : 0u;

    if (fric_on == 0u)
    {
        uint8_t was_sleep = (launcher.state == LAUNCHER_SLEEP) ? 1u : 0u;
        uint8_t hard_stop = (launcher_runtime.fric_state != LAUNCHER_FRIC_NORMAL) ? 1u : 0u;
        if (launcher_runtime.fric_state == LAUNCHER_FRIC_BOOST)
        {
            launcher_runtime.fric_state = LAUNCHER_FRIC_LOCKED;
            launcher_debug.recovery_failed++;
            launcher.fault = 1u;
        }
        if (single_event != 0u)
        {
            Launcher_RejectSingle(LAUNCHER_REJECT_DISABLED);
        }
        Launcher_CancelSingle(LAUNCHER_REJECT_ABORTED);
        launcher_runtime.power_seen = 0u;
        launcher_runtime.jam_seen = 0u;
        launcher_debug.jam_elapsed_ms = 0u;
        launcher_debug.boost_elapsed_ms = 0u;
        launcher.enabled = 0u;
        Launcher_UpdateFrictionReady(0u);
        force_stop = 1u;
        launcher.fric_target_rpm = Launcher_Ramp(launcher.fric_target_rpm, 0.0f,
                                                LAUNCHER_FRIC_STOP_RAMP_RPM_PER_MS);
        if (hard_stop != 0u)
        {
            launcher.fric_target_rpm = 0.0f;
            launcher.state = LAUNCHER_FAULT;
            inhibit = LAUNCHER_REJECT_FRIC_FAULT;
        }
        else
        {
            if ((launcher.fric_l_speed_rpm <= LAUNCHER_FRIC_STOP_SPEED_RPM) &&
                (launcher.fric_r_speed_rpm <= LAUNCHER_FRIC_STOP_SPEED_RPM))
            {
                if (launcher_fric_stop_count < LAUNCHER_FRIC_STOP_CONFIRM_MS)
                {
                    launcher_fric_stop_count++;
                }
            }
            else
            {
                launcher_fric_stop_count = 0u;
            }
            if ((was_sleep != 0u) ||
                (launcher_fric_stop_count >= LAUNCHER_FRIC_STOP_CONFIRM_MS))
            {
                launcher.state = LAUNCHER_SLEEP;
                launcher.fric_target_rpm = 0.0f;
                launcher_fric_stop_count = 0u;
            }
            else
            {
                launcher.state = LAUNCHER_STOPPING;
                fric_enable = 1u;
            }
            inhibit = LAUNCHER_REJECT_DISABLED;
        }
        goto output;
    }

    launcher.enabled = 1u;
    launcher_fric_stop_count = 0u;
    launcher.fric_target_rpm = LAUNCHER_FRIC_TARGET_RPM;
    Launcher_UpdateFrictionReady(1u);
    Launcher_FricRecoveryUpdate(now);
    if (launcher_runtime.fric_state != LAUNCHER_FRIC_NORMAL)
    {
        inhibit = (launcher_runtime.fric_state == LAUNCHER_FRIC_BOOST) ?
                  LAUNCHER_REJECT_FRIC_JAM : LAUNCHER_REJECT_FRIC_FAULT;
        if (single_event != 0u)
        {
            Launcher_RejectSingle(inhibit);
        }
        Launcher_CancelSingle(inhibit);
        force_stop = 1u;
        fric_enable = (launcher_runtime.fric_state == LAUNCHER_FRIC_BOOST) ? 1u : 0u;
        if (fric_enable == 0u)
        {
            launcher.fric_target_rpm = 0.0f;
            launcher.state = LAUNCHER_FAULT;
            Launcher_UpdateFrictionReady(0u);
        }
        goto output;
    }
    fric_enable = 1u;
    if ((launcher.state == LAUNCHER_SLEEP) || (launcher.state == LAUNCHER_STOPPING) ||
        (launcher.state == LAUNCHER_FAULT))
    {
        launcher.state = LAUNCHER_READY;
        launcher_dial_target = (int64_t)launcher.dial_angle;
        Launcher_DialClearPid();
    }

#if LAUNCHER_DIAL_ENABLE
    if (launcher.dial_online == 0u)
#else
    if (1)
#endif
    {
        inhibit = LAUNCHER_REJECT_DIAL_OFFLINE;
        if (single_event != 0u)
        {
            Launcher_RejectSingle(inhibit);
        }
        Launcher_CancelSingle(inhibit);
        force_stop = 1u;
        goto output;
    }

    if ((launcher_heat.ready == 0u) || (launcher_heat.heat > launcher_heat.heat_limit))
    {
        inhibit = LAUNCHER_REJECT_HEAT;
        Launcher_CancelSingle(inhibit);
        force_stop = 1u;
    }
    if (single_event != 0u)
    {
        launcher_reject_e reason = inhibit;
        if ((launcher.state == LAUNCHER_SINGLE) || (launcher.state == LAUNCHER_REPEAT) ||
            (launcher_runtime.start_pending != 0u))
        {
            reason = LAUNCHER_REJECT_BUSY;
        }
        else if (reason == LAUNCHER_REJECT_NONE)
        {
            if (launcher_runtime.release_required != 0u)
            {
                reason = LAUNCHER_REJECT_RELEASE_REQUIRED;
            }
            else if (launcher.fric_ready == 0u)
            {
                reason = LAUNCHER_REJECT_FRIC_NOT_READY;
            }
            else if (Launcher_HeatReserveSingle() == 0u)
            {
                reason = LAUNCHER_REJECT_HEAT;
            }
        }
        if (reason != LAUNCHER_REJECT_NONE)
        {
            Launcher_RejectSingle(reason);
        }
        else
        {
            launcher_debug.accepted++;
            launcher_debug.reject_reason = LAUNCHER_REJECT_NONE;
            launcher_dial_target = (int64_t)Launcher_DialAngle() +
                (int64_t)(LAUNCHER_DIAL_DIRECTION * LAUNCHER_DIAL_ONE_SHOT_ANGLE);
            launcher.state = LAUNCHER_SINGLE;
            launcher_runtime.start_pending = 1u;
            launcher_runtime.start_tick = now;
            launcher_dial_braking = 0u;
            Launcher_DialClearPid();
        }
    }

    if (launcher.state == LAUNCHER_SINGLE)
    {
        if (launcher_runtime.start_pending != 0u)
        {
            if ((now - launcher_runtime.start_tick) >= LAUNCHER_DIAL_START_RETRY_MS)
            {
                launcher_debug.start_failed++;
                launcher_debug.reject_reason = LAUNCHER_REJECT_START_FAILED;
                Launcher_CancelSingle(LAUNCHER_REJECT_START_FAILED);
                inhibit = LAUNCHER_REJECT_START_FAILED;
                force_stop = 1u;
            }
            else if (Launcher_DialRun() == HAL_OK)
            {
                launcher_dial_stopped = 0u;
                launcher_runtime.start_pending = 0u;
                launcher.state_tick = now;
            }
        }
        if ((launcher.state == LAUNCHER_SINGLE) && (launcher_runtime.start_pending == 0u))
        {
            if (Launcher_DialAtTarget(launcher_dial_target) != 0u)
            {
                launcher_debug.completed++;
                launcher.state = LAUNCHER_READY;
                Launcher_DialClearPid();
            }
            else if ((now - launcher.state_tick) >= LAUNCHER_DIAL_SINGLE_TIMEOUT_MS)
            {
                launcher_debug.timed_out++;
                Launcher_CancelSingle(LAUNCHER_REJECT_SINGLE_TIMEOUT);
                inhibit = LAUNCHER_REJECT_SINGLE_TIMEOUT;
                force_stop = 1u;
            }
            else
            {
                dial_active = 1u;
            }
        }
    }
    else if (force_stop == 0u)
    {
#if LAUNCHER_REPEAT_ENABLE
        continuous = ((command.shoot_mode != 0u) && (command.shoot_level != 0u) &&
                      (launcher_runtime.release_required == 0u) &&
                      (launcher.fric_ready != 0u) && (launcher_heat.ready != 0u) &&
                      (launcher_heat.blocked == 0u) && (launcher_heat.target_rate > 0.0f)) ? 1u : 0u;
#endif
        if (continuous != 0u)
        {
            if ((launcher_dial_stopped == 0u) || (Launcher_DialRun() == HAL_OK))
            {
                launcher_dial_stopped = 0u;
                launcher_dial_braking = 0u;
                if (launcher.state != LAUNCHER_REPEAT)
                {
                    Launcher_DialClearPid();
                    Launcher_HeatStartRepeat();
                }
                launcher.state = LAUNCHER_REPEAT;
                dial_active = 1u;
            }
        }
        else if (launcher.state == LAUNCHER_REPEAT)
        {
            launcher.state = LAUNCHER_READY;
            launcher_dial_target = (int64_t)Launcher_DialAngle();
        }
    }

    if (inhibit == LAUNCHER_REJECT_NONE)
    {
        if (launcher_runtime.release_required != 0u)
        {
            inhibit = LAUNCHER_REJECT_RELEASE_REQUIRED;
        }
        else if (launcher.fric_ready == 0u)
        {
            inhibit = LAUNCHER_REJECT_FRIC_NOT_READY;
        }
        else if ((launcher_heat.ready == 0u) || (launcher_heat.blocked != 0u))
        {
            inhibit = LAUNCHER_REJECT_HEAT;
        }
        else if ((launcher.state == LAUNCHER_SINGLE) || (launcher.state == LAUNCHER_REPEAT))
        {
            inhibit = LAUNCHER_REJECT_BUSY;
        }
    }

output:
    launcher_debug.inhibit_reason = inhibit;
    Launcher_FricControl(fric_enable);
    if (force_stop != 0u)
    {
        launcher_debug.dial_speed_target = 0.0f;
        Launcher_DialSafeStop(now);
    }
    else if (launcher_runtime.start_pending == 0u)
    {
        Launcher_DialControl(dial_active);
    }
    Launcher_PublishDebug();
}
```

