# task_down 下板控制工程

> STM32H723VGTx + FreeRTOS 的整车输入、底盘、发射决策和超电通信主机。

`task_down` 负责接收遥控器和键鼠，统一生成本车控制命令，执行四轮底盘控制，并向上板发送 D1/D2/D5。上板只执行云台、发射和升降动作，下板则掌握整车状态、控制来源、发射许可和狗洞请求。

这份 README 分成两层：

- 前半部分讲职责边界、架构和设计取舍。
- 后半部分展开输入映射、底盘、跟随、小陀螺、发射、超电、协议、问题复盘和调试方法。

## 1. 工程定位

```text
DBUS / 键鼠
    |
    v
CommandTask
  rc_interrupt_update
  keyboard_update
    |
    v
CtrlTask
  Board_Debug_Gimbal_Command
  Chassis_Input_Update
  Chassis_Follow_UpdateMode
  Chassis_Spin_UpdateMode
  Chassis_Follow_Update
  Chassis_Spin_Update
  Chassis_Control_Update
  Launch_Work
  SuperCap_Tx
    |
    +--> FDCAN1: 四轮 0x200、超电 0x222
    +--> FDCAN2: D1/D2/D5
    +<-- FDCAN2: C1/C2
```

下板当前是唯一的遥控器/键鼠主机。这样做是为了避免两个 MCU 同时解释同一套输入，减少控制权冲突和状态重复。

## 2. 当前职责

下板负责：

- DBUS 18 字节帧解析。
- 遥控器摇杆、拨杆、拨轮和键鼠状态。
- RC/键盘控制源切换。
- 底盘直控、跟随云台、小陀螺三种运动来源。
- 四轮逆解和速度闭环。
- 发射许可、S2 消抖、上电重新解锁和打符联锁。
- 狗洞请求下发和云台机械模式命令。
- 超电 CAN 保活和反馈解析。
- D1/D2/D5 周期发送，C1/C2 接收。

下板不负责：

- 云台电机 PID。
- IMU 姿态闭环。
- 发射摩擦轮和拨盘执行。
- 升降电机执行。
- 自瞄、视觉、裁判和正式功率限制。
- 上层 UI。

## 3. 设计重点

### 3.1 单一遥控器主机

输入链固定为：

```text
遥控器 -> rc_dev
键鼠   -> rc_dev
        |
        v
下板统一决策
        |
        v
D1/D2/D5 -> 上板执行
```

D5 不需要让上板重新判断“这是遥控器还是键鼠”。下板先把两种输入统一成 Yaw/Pitch 角速度，再按 `0.1 deg/s` 量化。

### 3.2 所有底盘来源统一成 `chassis_cmd_t`

底盘输入、跟随、小陀螺、键鼠最终都写入同一个命令结构：

```c
typedef struct
{
    float vx;
    float vy;
    float wz;
    uint8_t valid;
    chassis_source_e source;
} chassis_cmd_t;
```

来源枚举：

```text
CHASSIS_SRC_NONE
CHASSIS_SRC_RC
CHASSIS_SRC_RC_FOLLOW
CHASSIS_SRC_SPIN
CHASSIS_SRC_KEYBOARD
```

固定处理顺序：

```text
输入解析
  -> 跟随覆盖 wz / 平移参考系
  -> 小陀螺覆盖 wz / 平移参考系
  -> 四轮逆解
  -> 速度环
  -> 力矩输出
```

这样每个模式只需要接管自己关心的分量，不会各自直接操作电机。

### 3.3 跟随模式把 C2 当作安全输入

跟随模式使用上板 C2 的 Yaw 机械角。代码不会无脑相信最后一帧：

```text
C2 曾经收到
+gimbal_rx_time_ms 距当前 <= 50 ms
+Yaw 数值有效
+相邻两拍 yaw_error 跳变 <= 30 deg
```

任一条件失败：

- `fault_latched = 1`
- `active = 0`
- `cmd.valid = 0`
- 底盘输出清零

故障需要退出跟随档位再进入才能恢复。

### 3.4 小陀螺平移使用云台参考系

小陀螺旋转时，如果平移仍按车体坐标系解释，操作方向会随云台角度变化。当前实现：

```text
theta = yaw_mec
vx' = cos(theta) * vx - sin(theta) * vy
vy' = sin(theta) * vx + cos(theta) * vy
```

如果 C2 超过 50 ms：

- 自转保留。
- `vx/vy` 清零。

这个策略是有意保守的：姿态不可信时，不进行方向相关平移。

### 3.5 底盘控制优先检查“对象、数据、在线状态”

底盘更新链：

```text
cmd != NULL
enabled
cmd.valid
vx/vy/wz 非 NaN 且有限
四轮对象完整
四轮全部在线
逆解
速度环
组帧发送
```

任何一步失败都调用停机：

- 四轮目标清 0。
- 发送 `0x200` 全零力矩。
- `fault = 1`。

这部分主要是提交 `770a417` 后补强的。之前底盘任务调用和空指针检查不够完整，容易出现“命令链还在跑，但对象状态不可信”的情况。

### 3.6 发射许可独立于上板执行

下板只把许可、模式和触发电平写入 D1：

```text
launch_state
shoot_mode
shoot_level
```

上板不需要知道 S1/S2 的原始值，也不需要知道打符请求背后的按键逻辑。

发射安全条件：

- 遥控在线。
- S2 上电后发生过一次变化。
- S2 经过 15 ms 消抖。
- 打符期间禁止发射。
- 升降不在上位时禁止发射。
- 小陀螺档位禁止发射。

### 3.7 超电先只做通信

当前超电控制输出全部为 0：

```c
SUPERCAP_CAP_SWITCH         = 0u
SUPERCAP_TURBO_MODE         = 0u
SUPERCAP_PRE_CHARGE_ENABLE  = 0u
SUPERCAP_POWER_BUFFER       = 0u
SUPERCAP_POWER_LIMIT        = 0u
SUPERCAP_POWER_OUT_LIMIT    = 0
SUPERCAP_POWER_IN_LIMIT     = 0u
```

先验证 `0x222/0x211` 和电压电流反馈，再逐步启用预充、放电和功率协同。这是为了避免未验证超电把底盘调试变成不可复现问题。

## 4. main 和任务模型

### 4.1 main

```text
SCB_EnableICache
HAL_Init
SystemClock_Config
PeriphCommonClock_Config
MX_GPIO_Init
MX_DMA_Init
MX_FDCAN1_Init
MX_FDCAN2_Init
MX_UART5_Init
DEVICE_Init
DRIVER_Init
osKernelInitialize
MX_FREERTOS_Init
osKernelStart
```

`DEVICE_Init()`：

```text
SuperCap_Init
rc_dev.init
board.init
launch.init
rm_motor_list_init
Chassis_Follow_Init
Chassis_Spin_Init
Chassis_Input_Init
Chassis_Control_Init
```

`DRIVER_Init()`：

```text
USART5_Init
CAN2_Filter_Init
CAN1_Filter_Init
```

### 4.2 任务

系统 tick 为 1 kHz。

| 任务 | 优先级 | 栈 | 周期 | 职责 |
|---|---:|---:|---:|---|
| `CtrlTask` | `osPriorityHigh` | 2048 bytes | 1 ms 循环 | 整车控制主链 |
| `ConnectTask` | `osPriorityHigh` | 2048 bytes | 1 ms | D1/D2/D5 |
| `CommandTask` | `osPriorityHigh` | 1024 bytes | 1 ms | 遥控和键鼠 |
| `MonitorTask` | `osPriorityAboveNormal` | 2048 bytes | 1 ms | 电机、遥控、板间、超电 |

`StartCtrlTask()` 顺序：

```text
Board_Debug_Gimbal_Command
Chassis_Input_Update
Chassis_Follow_UpdateMode
Chassis_Spin_UpdateMode
Chassis_Follow_Update
Chassis_Spin_Update
Chassis_Control_Update
launch.work
SuperCap_Tx
```

`StartConnectTask()`：

```text
D1
D2
D5
osDelay(1)
```

板间心跳、遥控心跳、RM 电机心跳和超电心跳放在 `MonitorTask`。

### 4.3 weak 壳和强制定义

`Core/Src/freertos.c` 中保留：

```text
StartMonitorTask
StartCtrlTask
StartCommandTask
StartConnectTask
```

四个 weak 空壳。实际实现位于 `Application/TaskLayer`。禁止在应用层再复制其他任务名，否则会重新出现 `L6200E`。

## 5. 外设和通信

### 5.1 外设

| 外设 | 引脚/参数 | 用途 |
|---|---|---|
| FDCAN1 | PD0/PD1，经典 CAN | 四轮和超电 |
| FDCAN2 | PB5/PB6，经典 CAN | D1/D2/D5 和 C1/C2 |
| UART5 | TX PC12，RX PD2 | DBUS |
| UART5 格式 | 100000 baud，9 位，偶校验，2 停止位 | DBUS 物理层 |
| DMA1 Stream0 | 18 字节循环缓冲 | RxToIdle |
| SWD | PA13/PA14 | 调试 |
| PWR_5V_EN | PC15 | 板载 5V 使能 |
| PWR_OUT1_EN | PC14 | 外设电源 |
| PWR_OUT2_EN | PC13 | 外设电源 |
| BUZZER | PB15 | 预留 |

### 5.2 FDCAN1

| ID | 方向 | 内容 |
|---|---|---|
| `0x200` | 下板发送 | 四轮组帧 |
| `0x201` | 下板接收 | 左前 |
| `0x202` | 下板接收 | 左后 |
| `0x203` | 下板接收 | 右前 |
| `0x204` | 下板接收 | 右后 |
| `0x222` | 下板发送 | 超电控制 |
| `0x211` | 下板接收 | 超电反馈 |

### 5.3 FDCAN2

| ID | 方向 | 内容 |
|---|---|---|
| `0xC1` | 下板接收 | 上板设备状态 |
| `0xC2` | 下板接收 | 云台姿态 |
| `0xD1` | 下板发送 | 整车和发射状态 |
| `0xD2` | 下板发送 | 云台目标 |
| `0xD5` | 下板发送 | 遥控/键鼠控制量 |

### 5.4 CAN 波特率风险

`.ioc` 目标：

```text
FDCAN clock = 100 MHz
Baud = 1 Mbps
```

生成代码当前参数：

```text
HSE = 24 MHz
PLL2M = 24
PLL2N = 200
PLL2P = 4
FDCAN clock = 50 MHz
Prescaler = 5
TQ = 20
Baud = 500 kbps
```

上板按 1 Mbps 配置。双板首次联调时，如果抓包发现一端有发送但另一端没有有效 ACK，先查这里，再查协议。

## 6. 遥控与键鼠输入

### 6.1 DBUS 接收

UART5 使用 DMA RxToIdle：

```text
只接受 IDLE 事件
只接受 size == 18
解析后立即重启接收
错误回调也重启接收
```

`rc_update()` 解析：

```text
ch0/ch1/ch2/ch3
S1/S2
拨轮
鼠标 dx/dy/dz
鼠标按键
键盘 bitmap
```

`keyboard_update()` 维护每个按键的状态：

```text
release
release_to_press
short_press
long_press
press_to_release
```

### 6.2 遥控通道

| 通道 | 含义 | 当前用途 |
|---|---|---|
| `ch0` | 右横 | 底盘转向、小陀螺输入、部分 D5 Yaw |
| `ch1` | 右纵 | D5 Pitch、机械模式 Pitch 微调 |
| `ch2` | 左横 | 底盘 `vy` |
| `ch3` | 左纵 | 底盘 `vx` |
| `s1` | 左开关 | 控制源、跟随、小陀螺、发射模式 |
| `s2` | 右开关 | 跟随、小陀螺、发射模式 |
| 拨轮 | 左拨轮 | 预留，不参与当前主控 |

遥控直控：

```text
vx = -ch3 * CHASSIS_MAX_VX / 660
vy =  ch2 * CHASSIS_MAX_VY / 660
wz =  ch0 * CHASSIS_MAX_WZ / 660
```

输入先做死区，再归一化到 `[-1, 1]`。

### 6.3 键鼠模式

键鼠模式成立：

```text
键盘输入开启
遥控在线
keyboard_source_active == 1
S1 上位
```

F 键在 S1 上位时切换输入源。

Z/X/C：

| 键 | 模式 |
|---|---|
| Z | 跟随 |
| X | 机械/普通直控 |
| C | 小陀螺 |

运动输入：

```text
W/S      前后
A/D      左右
Q/E      旋转
Shift    1.5x
Ctrl     0.5x
```

鼠标控制量会先进入 `rc_interrupt_update()` 的 10 点移动平均，再由 D5 转换成角速度。

### 6.4 机械模式和打符

S1 下位时，下板发送：

```text
gimbal_mode = 0
yaw_mec_tar = BOARD_MEC_YAW_FRONT_RAD = 0
pitch_mec_tar += ch1 / 660 * 0.002 rad
pitch_mec_tar 限幅 -7.5..30 deg
```

打符入口：

```text
S1 上位
S2 中位
B 键按下沿
```

打符期间：

- 云台锁机械模式。
- Yaw 机械目标固定 0。
- Pitch 目标固定 0。
- 发射锁。
- 退出时等待升降上报上位，或 5000 ms 超时。

## 7. 底盘控制

### 7.1 四轮逆解

```text
LF = -vx + vy + wz
LB = -vx - vy + wz
RF =  vx + vy + wz
RB =  vx - vy + wz
```

总速度限幅使用 Manhattan 近似：

```text
trans = abs(vx) + abs(vy)
rotate = abs(wz)
total = trans + rotate
```

超过 `CHASSIS_CTRL_MAX_SPEED = 80` 时：

- 旋转最多占 60%。
- 平移按剩余量缩放。

### 7.2 速度环

```c
CHASSIS_SPEED_KP = 0.8f
CHASSIS_SPEED_KI = 0.0f
CHASSIS_SPEED_KD = 0.0f
```

当前是纯 P。这样调参快，但抗负载能力有限。正式高速运动前应该重新评估积分和功率限制。

力矩上限：

| 来源 | 上限 |
|---|---:|
| 普通 | 2 N*m |
| 跟随 | 3 N*m |
| 小陀螺 | 4 N*m |

### 7.3 零速区域

```text
|target| < 0.5
|feedback| < 1.0
```

同时落入时：

- 清空 PID。
- 输出 0。

避免静止时速度环反复抽搐。

### 7.4 底盘安全

底盘整组停输出条件：

- 命令空。
- 控制未使能。
- 命令无效。
- 浮点 NaN 或超过安全范围。
- 四轮对象不完整。
- 四轮任一离线。
- 输出接口为空。

## 8. 跟随模式

### 8.1 选择

遥控：

```text
S1 上位
S2 上位或中位
```

键鼠：

```text
Z 键选择跟随
```

### 8.2 跟随误差

```text
yaw_error = wrap_pi(yaw_mec - CHASSIS_FOLLOW_CENTER_RAD)
```

平移按误差旋转，语义为“在云台坐标系下控制底盘”。

### 8.3 参数

```c
CHASSIS_FOLLOW_KP                  = 20.0f
CHASSIS_FOLLOW_MAX_WZ              = 20.0f
CHASSIS_FOLLOW_WZ_STEP             = 0.2f
CHASSIS_FOLLOW_FRICTION_FF         = 7.0f
CHASSIS_FOLLOW_DEADBAND_DEG        = 7.0f
CHASSIS_FOLLOW_TURN_LOCK_DEG       = 150.0f
CHASSIS_FOLLOW_TURN_UNLOCK_DEG     = 20.0f
CHASSIS_FOLLOW_YAW_JUMP_LIMIT_DEG  = 30.0f
CHASSIS_FOLLOW_TIMEOUT_MS          = 50u
CHASSIS_FOLLOW_BLEND_TIME_MS       = 200u
CHASSIS_FOLLOW_TORQUE_LIMIT_NM     = 3.0f
```

大误差锁定旋转方向，小误差释放，用于抑制中心点来回抖。进入跟随时 200 ms 融合，退出时回手动角速度。

### 8.4 故障

- C2 年龄超过 50 ms。
- C2 从未收到。
- Yaw 为 NaN 或越界。
- Yaw 误差跳变超过 30 deg。

故障锁存后退出跟随档位再进入，避免异常反馈反复触发。

## 9. 小陀螺模式

### 9.1 选择

遥控：

```text
S1 上位
S2 下位
```

键鼠：

```text
C 键选择
```

### 9.2 参数

```c
CHASSIS_SPIN_MAX_WZ       = 20.0f
CHASSIS_SPIN_BASE_WZ      = 20.0f
CHASSIS_SPIN_TRIM_WZ      = 0.0f
CHASSIS_SPIN_STEP         = 0.1f
CHASSIS_SPIN_RC_DEADBAND  = 30.0f
CHASSIS_SPIN_TORQUE_LIMIT = 4.0f
```

`TRIM_WZ = 0` 表示当前小陀螺以固定基础角速度自转，`ch0` 不参与调节。

### 9.3 平移安全

```text
C2 有效
  -> 按 yaw_mec 旋转 vx/vy
C2 超时
  -> 清 vx/vy
  -> 保留自转
```

这是实际调试中很重要的一个取舍：不因为云台反馈异常就让底盘完全停机，但也不允许用旧姿态做平移。

## 10. 发射决策

### 10.1 键鼠

键鼠模式：

```text
左键按下 -> 发射
左键长按 -> 连发
松开     -> 单发模式并取消触发
```

### 10.2 遥控

S2 上电后必须先变化一次：

```text
launch_shoot_switch_seen
launch_shoot_armed
```

否则上电时 S2 已经在上位会直接发射。

S2 消抖：

```c
BOARD_LAUNCH_S2_DEBOUNCE_TICKS = 15u
```

档位组合：

| S1 | S2 | 结果 |
|---|---|---|
| 上位 | 上位 | 连发 |
| 中位 | 上位 | 单发 |
| 上位/中位 | 中位 | 解锁但无触发 |
| 上位 | 下位 | 小陀螺，锁发射 |
| 其他 | 任意 | 锁发射 |

### 10.3 联锁

```text
is_hole != 0
或
is_down != 2
```

时发射锁定。

这里的 `is_down` 表示上板升降状态，不是“上板是否在线”。

## 11. 超电通信

### 11.1 帧

控制：

| 字节 | 内容 |
|---|---|
| 0 | buffer |
| 1..2 | power_limit |
| 3..4 | power_out_limit |
| 5..6 | power_in_limit |
| 7 | cap_switch、turbo、pre_charge |

反馈：

| 字节 | 内容 |
|---|---|
| 0..1 | chassis_power |
| 2..3 | voltage_raw |
| 4..5 | current_raw |
| 6 | ability、pre_charge_mode |

### 11.2 当前策略

超电只保证链路存活：

- `SuperCap_Tx()` 周期发送控制帧。
- `SuperCap_Rx()` 刷新电压、电流、底盘功率。
- 100 ms 无反馈判离线。
- 所有输出开关和功率上限保持 0。

## 12. 板间协议

### 12.1 D1

| 字节 | 位 | 含义 |
|---|---|---|
| 0 | 1:0 | `car_state` |
| 0 | 2 | `gimbal_mode` |
| 5 | 0 | `launch_state` |
| 5 | 1 | `shoot_mode` |
| 5 | 2 | `shoot_level` |
| 5 | 3 | `is_hole` |

### 12.2 D2

| 字节 | 字段 | 单位 | 量程 |
|---|---|---|---|
| 0..1 | Pitch IMU | deg | -360..360 |
| 2..3 | Yaw IMU | deg | -360..360 |
| 4..5 | Pitch 机械 | rad | -4..4 |
| 6..7 | Yaw 机械 | rad | -4..4 |

### 12.3 D5

| 字节 | 含义 |
|---|---|
| 0 | valid、ctrl_source、cmd_type |
| 1 | 鼠标按键 |
| 2..3 | Yaw 角速度，0.1 deg/s/LSB |
| 4..5 | Pitch 角速度，0.1 deg/s/LSB |
| 6..7 | 保留 |

键鼠鼠标量先乘：

```c
Yaw gain   =  3.0f
Pitch gain = -3.0f
```

后限制：

```text
Yaw   ±300 deg/s
Pitch ±150 deg/s
```

### 12.4 C1/C2

C1 提供云台、摩擦轮、拨盘在线位和升降状态。C2 提供云台机械角和 IMU 角。

板间心跳：

- C1 或 C2 任一收到都清零 `offline_cnt`。
- `MonitorTask` 每 1 ms 累加。
- 50 ms 没有 C1/C2 判离线。
- C2 单独记录 `gimbal_rx_time_ms`，供跟随和小陀螺判断反馈新鲜度。

## 13. 问题复盘

### 13.1 底盘任务调用和指针检查

提交：`770a417 修复底盘任务调用和指针安全检查`

问题：

- 底盘控制链调用关系和对象安全检查不完整。
- `Chassis_Control_PidUpdate()`、`Chassis_Control_Output()` 原来是 `void`，出现空对象也无法阻止继续执行。
- 初始化时没有检查四轮对象的 `ctrl`、`speed_ctrl` 和 `tx_info`。

处理：

- 关键函数返回成功/失败。
- 初始化失败时 `enabled = 0`、`fault = 1`。
- 失败直接整组停机。
- 删除旧 observe 任务的信号量调用，避免底盘任务被无效同步阻塞。

### 13.2 小陀螺平移参考系

提交：`c325662 小陀螺平移的实现以及解放yaw轴`

问题：

- 早期小陀螺只做纯旋转，平移语义不完整。
- Yaw 被底盘占用后，小陀螺平移还按车体坐标系，操作手感不稳定。

处理：

- 新增 `CHASSIS_SPIN_TRANSLATION_*` 配置。
- 使用 C2 Yaw 机械角做平移旋转。
- C2 超时只清平移，保留自转。
- `CHASSIS_SPIN_TRIM_WZ` 从 5 改为 0，明确当前以固定基础速度自转。

### 13.3 机械模式和发射时序

提交：`078980a 机械模式的调整以及发射机构的时序问题解决`

问题：

- 机械模式调整后，发射状态在同一控制周期出现前后不一致。
- 发射许可、模式和触发位如果分开读取，可能在切换瞬间读到混合状态。

处理：

- 使用同一拍发射状态快照。
- 下板 `Launch_Cmd_Transmit()` 统一写 D1。
- 上板用 `Board_Rx_Shoot_Flags` 读取同一字节。

### 13.4 升降 fault 退出

提交：`8ecb25f 修复进入fault`

问题：

- 升降进入 fault 后没有可靠退出边沿。
- 上电或静止检测误入 fault 后无法继续动作。

处理：

- 只有 `cmd_changed` 才允许恢复。
- 保持 fault 时持续输出 0。
- 恢复时根据 `home_valid` 和 `is_hole` 决定重新找零、下降或上升。

### 13.5 S2 回中弹跳和上电误发射

问题：

- S2 回中时机械触点抖动，发射档位瞬间跳变。
- 上电时如果 S2 已经在上位，会被首次采样直接当成发射。

处理：

- 15 ms 软件消抖。
- 上电后必须先发生一次档位变化，才允许解锁。
- 遥控离线或板间条件不满足时清除解锁状态。

### 13.6 跟随故障反复触发

问题：

- C2 偶发丢帧或 Yaw 跳变时，如果每拍自行恢复，底盘会在安全与运动间抖动。
- 旧姿态用于平移会引发不可预测方向。

处理：

- 50 ms 超时。
- Yaw 跳变超过 30 deg 锁存故障。
- 必须退出跟随档位后重新进入。
- 小陀螺 C2 超时清平移。

### 13.7 超电接入顺序

提交：`754aa0a 超电通信测试未验证`

问题：

- 超电控制和底盘功率限制同时接入会让调试问题难定位。
- 超电反馈异常可能直接影响底盘输出。

处理：

- 先只保活通信。
- 所有功率输出开关为 0。
- 先验证 0x222/0x211，再考虑预充和功率协同。

## 14. 调试顺序

### 14.1 遥控和键鼠

- 先确认 DBUS 18 字节帧。
- 确认 S1/S2 档位定义。
- 确认右摇杆、左摇杆、鼠标方向。
- 确认 F/Z/X/C 模式切换。
- 确认遥控离线后所有输入归零。

### 14.2 底盘

1. 单轮低速正反转。
2. 四轮同向。
3. 四轮平移。
4. 原地旋转。
5. 斜向运动。
6. 速度环阶跃。
7. 零速区域。
8. 低限矩跟随。

### 14.3 跟随

- 先确认 C2 的 Yaw 机械角符号。
- 确认 7 deg 死区和方向。
- 确认 150/20 deg 锁向阈值。
- 拔掉 CAN 或暂停上板发送，确认 50 ms 后底盘停机。
- 制造超过 30 deg 的 Yaw 跳变，确认故障锁存。

### 14.4 小陀螺

- 先只转不平移。
- 确认 20 rad/s 基础角速度的实际效果。
- 再打开云台参考系平移。
- 断开 C2，确认平移清零但自转保留。

### 14.5 发射

- 不装弹，确认 S2 消抖。
- 上电时把 S2 放上位，确认不会直接发射。
- 确认 S1 中位单发、S1 上位连发。
- 确认打符和升降条件能锁发射。

### 14.6 超电

- 只抓 0x222/0x211。
- 确认电压电流换算。
- 确认 100 ms 离线检测。
- 暂时不要打开功率输出。

## 15. 编译

```powershell
UV4 -r task_down\MDK-ARM\DM-MC02.uvprojx -t DM-MC02 -o build_down.log
```

当前保留日志：

```text
Code=55852
RO-data=1144
RW-data=1516
ZI-data=28408
0 Error(s), 0 Warning(s)
```

## 16. Keil Watch 建议

```text
rc_dev.work_state
rc_data
keyboard_source_active
chassis_input_cmd
chassis_follow
chassis_spin
chassis_ctrl
wheel_motor
launch.state
launch.mode
launch.heart
board.status
board.tx_pkt
board.rx_meg
board_lift_dbg
supercap
hfdcan1.Instance->ECR
hfdcan2.Instance->ECR
```

排障优先级：

1. 遥控在线和通道值。
2. 底盘命令来源和有效位。
3. 四轮在线和力矩输出。
4. C2 时间戳与 Yaw 跳变。
5. D1 发射位和升降状态。
6. 超电在线状态。

## 17. 当前边界

- 上下板 CAN 波特率存在配置不一致风险。
- 超电只保活，不参与功率。
- 正式功率限制未实现。
- 裁判、视觉和自瞄未接入。
- 底盘速度环是纯 P，高速负载下需要复核。
- 升降临时行程影响发射许可条件。
- 看门狗未启用。
- 没有实车验证时只能说明编译和逻辑检查通过。

## 18. 目录

```text
task_down/
  Application/
    AlgorithmLayer/   PID、数学工具
    ConfigLayer/      底盘、通信、超电、设备参数
    DeviceLayer/      遥控、超电、电机对象
    DriverLayer/      FDCAN、UART、DWT、GPIO
    HardwareLayer/    RM 电机驱动
    ModuleLayer/      输入、跟随、小陀螺、底盘、发射
    ProtocolLayer/    板间、遥控、超电、CAN 分发
    TaskLayer/        Command、Ctrl、Connect、Monitor
  Core/               CubeMX 主程序和中断
  Drivers/            STM32H7 HAL/CMSIS
  Middlewares/        FreeRTOS
  MDK-ARM/            Keil 工程和产物
```

## 19. 验收清单

- `DM-MC02` 全量 rebuild 为 0 errors。
- DBUS 18 字节帧稳定，遥控掉线后输入归零。
- 上下板 CAN 波特率经仪器确认一致。
- C1/C2 50 ms 内没有持续离线。
- 四轮单轮、同向、平移、旋转和斜向均通过。
- 普通底盘、跟随、小陀螺分别完成限矩验证。
- 跟随 C2 超时和 Yaw 跳变能锁存故障。
- 小陀螺 C2 超时后平移清零。
- S2 消抖和上电重新解锁有效。
- 打符和升降不在上位时禁止发射。
- 超电通信正常，但输出仍保持关闭。
- 所有协议和 CAN ID 改动同步检查 `task_up`。
