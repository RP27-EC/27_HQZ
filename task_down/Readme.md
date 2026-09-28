# task_down 下板控制工程

`task_down` 运行在 STM32H723VGTx 上，是整车输入、底盘、发射决策和超电通信主机。下板负责解析 DBUS 遥控器和键鼠，组合底盘模式，执行四轮速度环，生成 D1/D2/D5 发给上板，并接收 C1/C2 获取云台状态和姿态反馈。

```text
DBUS / 键鼠
    |
    v
CommandTask 解析
    |
    v
CtrlTask
  Board_Debug_Gimbal_Command
  Chassis_Input_Update
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

当前不启用自瞄、视觉串级、裁判系统、正式功率限制和下层本机 IMU 控制。超电当前只保活通信，不参与功率输出。

## 1. 快速开始

### 1.1 硬件与工具

| 项目 | 当前配置 |
|---|---|
| 主控 | STM32H723VGTx |
| HSE | 24 MHz |
| 主频 | PLL1 480 MHz，HCLK 240 MHz |
| FDCAN 时钟 | 代码配置 PLL2，当前参数见 CAN 章节 |
| 调试器 | J-Link、ST-Link 或 DAPLink，SWD |
| IDE | Keil MDK5 |
| 工程 | `task_down/MDK-ARM/DM-MC02.uvprojx` |
| Target | `DM-MC02` |
| 编译环境 | ARM Compiler 5.06 update 7（Keil 日志记录） |
| 遥控接收 | UART5，DBUS |

首次带电前必须架空底盘、拆弹，并准备急停和 CAN 分析仪。轮组方向、速度环和力矩限幅未标定时，不允许直接落地高速运行。

### 1.2 编译

Keil 图形界面：

```text
打开 task_down/MDK-ARM/DM-MC02.uvprojx
Target: DM-MC02
Build/Rebuild
```

命令行：

```powershell
UV4 -r task_down\MDK-ARM\DM-MC02.uvprojx -t DM-MC02 -o build_down.log
```

构建产物：

```text
task_down/MDK-ARM/DM-MC02/DM-MC02.axf
task_down/MDK-ARM/DM-MC02/DM-MC02.hex
```

工程内保留的 2026-09-28 构建日志结果：

```text
Program Size: Code=55852 RO-data=1144 RW-data=1516 ZI-data=28408
"DM-MC02\DM-MC02.axf" - 0 Error(s), 0 Warning(s).
```

修改源码后必须重新执行目标 `DM-MC02` 的全量 rebuild。

### 1.3 上电顺序

1. 整车断电，底盘架空，发射机构空载。
2. 确认 FDCAN1/FDCAN2 终端电阻、CANH/CANL 和共地正确。
3. 确认上下板 CAN 波特率一致。当前下板生成代码存在与上板 1 Mbps 不一致的风险，必须先用分析仪确认。
4. 上电后先看遥控器是否在线，S1/S2 是否处于预期档位。
5. 不接底盘动力时确认 D1/D2/D5 发送和 C1/C2 接收。
6. 确认上板云台已归中、升降上报到上位后，再测试发射许可。
7. 底盘先单轮、再四轮同向、再平移、旋转和斜向。
8. 确认基础速度环稳定后，再启用跟随和小陀螺。
9. 超电当前只验证通信，不让它参与输出。

## 2. 启动与任务

### 2.1 main 启动顺序

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

`DEVICE_Init()` 顺序：

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

### 2.2 FreeRTOS 任务

系统 tick 为 1 kHz。

| 任务 | 优先级 | 栈配置 | 周期 | 主要工作 |
|---|---:|---:|---:|---|
| `CtrlTask` | `osPriorityHigh` | 2048 bytes (`512 * 4`) | 静态循环加 `osDelay(1)` | 整车控制主链 |
| `ConnectTask` | `osPriorityHigh` | 2048 bytes (`512 * 4`) | `BOARD_COMM_D1D2_PERIOD_MS = 1` | 发 D1/D2/D5 |
| `CommandTask` | `osPriorityHigh` | 1024 bytes (`256 * 4`) | `osDelay(1)` | 遥控和键盘状态更新 |
| `MonitorTask` | `osPriorityAboveNormal` | 2048 bytes (`512 * 4`) | `osDelay(1)` | 电机、遥控、板间和超电心跳 |

`StartCtrlTask()` 固定顺序：

```text
Board_Debug_Gimbal_Command
Chassis_Input_Update
Chassis_Follow_UpdateMode
Chassis_Spin_UpdateMode
Chassis_Follow_Update(&chassis_input_cmd)
Chassis_Spin_Update(&chassis_input_cmd)
Chassis_Control_Update(&chassis_input_cmd)
launch.work
SuperCap_Tx
```

顺序会影响跟随、小陀螺和普通底盘输入的覆盖关系，不能随意交换。

### 2.3 任务函数重名问题

`Core/Src/freertos.c` 中保留 `StartMonitorTask`、`StartCtrlTask`、`StartCommandTask`、`StartConnectTask` 的 weak 空壳，实际实现位于 `Application/TaskLayer`。禁止在应用层复制其他任务同名强定义，否则会再次出现 `L6200E` 重复符号。

## 3. 外设与通信

### 3.1 外设映射

| 外设 | 引脚/参数 | 用途 |
|---|---|---|
| FDCAN1 | PD0/PD1，经典 CAN | 四轮和超电 |
| FDCAN2 | PB5/PB6，经典 CAN | C1/C2 接收和 D1/D2/D5 发送 |
| UART5 | TX PC12，RX PD2，100000 baud | DBUS 遥控器 |
| UART5 格式 | 9 位数据、偶校验、2 停止位 | DBUS 原始帧 |
| DMA1 Stream0 | 18 字节循环缓冲 | UART5 RxToIdle |
| SWD | PA13/PA14 | 调试 |
| 电源控制 | PC13/PC14/PC15 | 板载电源使能 |
| 蜂鸣器 | PB15 | 预留输出 |

FDCAN 过滤器当前全通，实际 ID 分发在 `CAN1_rxDataHandler()` 和 `CAN2_rxDataHandler()`。

### 3.2 CAN1 映射

| CAN ID | 方向 | 内容 |
|---|---|---|
| `0x200` | 下板发送 | 四轮组控制，槽位按 LF/LB/RF/RB |
| `0x201` | 下板接收 | 左前轮反馈 |
| `0x202` | 下板接收 | 左后轮反馈 |
| `0x203` | 下板接收 | 右前轮反馈 |
| `0x204` | 下板接收 | 右后轮反馈 |
| `0x222` | 下板发送 | 超电控制帧 |
| `0x211` | 下板接收 | 超电反馈帧 |

### 3.3 CAN2 映射

| CAN ID | 方向 | 内容 |
|---|---|---|
| `0xC1` | 下板接收 | 上板设备在线状态、升降状态 |
| `0xC2` | 下板接收 | 云台机械角/IMU 角反馈 |
| `0xD1` | 下板发送 | 整车状态、发射状态、狗洞请求 |
| `0xD2` | 下板发送 | 云台目标角 |
| `0xD5` | 下板发送 | 遥控/键鼠控制量 |

### 3.4 CAN 波特率风险

`task_down/DM-MC02.ioc` 记录的 FDCAN 目标是：

```text
FDCAN clock = 100 MHz
Target baud = 1 Mbps
```

当前生成代码 `Core/Src/main.c`：

```text
HSE = 24 MHz
PLL2M = 24
PLL2N = 200
PLL2P = 4
FDCAN clock = 50 MHz
Prescaler = 5
TQ = 1 + 14 + 5 = 20
Baud = 50 MHz / 5 / 20 = 500 kbps
```

上板 CAN1/CAN2 是按 1 Mbps 配置的。若双板收发出现“发送成功但对端没有有效帧”，先确认该时钟配置，不要先怀疑协议字段。修正时必须同时检查 `DM-MC02.ioc`、`Core/Src/main.c` 和上下板总线上其他设备。

### 3.5 DBUS 接收

UART5 使用 DMA Idle：

```text
帧长 = 18 字节
只有 IDLE 事件且 size == 18 才进入解析
解析后立即重启 DMA 接收
串口错误回调会重启接收
```

`rc_update()` 解析通道、S1/S2、鼠标、键盘位图和拨轮；`keyboard_update()` 更新短按、长按和边沿状态。

## 4. 板间协议

本节与 `task_up/Readme.md` 必须同步维护。

### 4.1 D1，下板到上板

D1 固定 8 字节，发送前清零。

| 字节 | 位 | 含义 |
|---|---|---|
| `Byte 0` | bit 1:0 | `car_state` |
| `Byte 0` | bit 2 | `gimbal_mode`，0 机械，1 速控 |
| `Byte 5` | bit 0 | `launch_state` |
| `Byte 5` | bit 1 | `shoot_mode` |
| `Byte 5` | bit 2 | `shoot_level` |
| `Byte 5` | bit 3 | `is_hole` |

### 4.2 D2，下板到上板

D2 大端发送四个 16 位值：

| 字节 | 字段 | 单位 | 量程 |
|---|---|---|---|
| `0..1` | Pitch IMU 目标 | deg | `-360..360` |
| `2..3` | Yaw IMU 目标 | deg | `-360..360` |
| `4..5` | Pitch 机械目标 | rad | `-4..4` |
| `6..7` | Yaw 机械目标 | rad | `-4..4` |

编码函数为：

```text
raw = (value - min) / (max - min) * 65535
```

当前 `float_to_uint()` 不主动限幅，写入前必须保证输入在量程内。

### 4.3 D5，下板到上板

| 字节 | 含义 |
|---|---|
| `Byte 0` | bit0 有效，bit1 输入源，bit2 控制类型 |
| `Byte 1` | 鼠标左右键位 |
| `Byte 2..3` | Yaw 角速度 `int16_t`，0.1 deg/s/LSB |
| `Byte 4..5` | Pitch 角速度 `int16_t`，0.1 deg/s/LSB |
| `Byte 6..7` | 保留，当前为零 |

当前 `cmd_type` 始终为 0，即角速度包。键鼠模式下鼠标速度先乘：

```c
BOARD_D5_MOUSE_YAW_GAIN    =  3.0f
BOARD_D5_MOUSE_PITCH_GAIN  = -3.0f
```

再限制到：

```text
Yaw   ±300 deg/s
Pitch ±150 deg/s
```

最后按 `0.1 deg/s` 量化。上板保留 `cmd_type = 1` 的鼠标增量分支，但下板当前不会发送该类型。

### 4.4 C1，上板到下板

上板定义：

| 位 | 含义 |
|---|---|
| bit 0 | Yaw 电机在线 |
| bit 1 | Pitch 电机在线 |
| bit 2 | 升降电机在线 |
| bit 3 | 右摩擦轮在线 |
| bit 4 | 左摩擦轮在线 |
| bit 5 | 拨盘在线 |

`Byte 1` 为升降状态：

```text
0 = 下位或下行堵转停机
1 = 运动中
2 = 上位/未完成找零
3 = 故障
```

下板当前只消费 Byte 0 的 bit 0/1/3/4/5 和 Byte 1。变量 `state_meg.is_down` 的名字容易误解，它表示上板升降状态。

### 4.5 C2，上板到下板

| 字节 | 字段 | 单位 | 量程 |
|---|---|---|---|
| `0..1` | Yaw 机械角 | rad | `-4..4` |
| `2..3` | Pitch 机械角 | rad | `-4..4` |
| `4..5` | Yaw IMU 角 | deg | `-360..360` |
| `6..7` | Pitch IMU 角 | deg | `-360..360` |

C2 数据用于：

- 跟随模式的云台相对角。
- 小陀螺的平移参考系。
- 调试观测。

### 4.6 心跳

`BOARD_OFFLINE_CNT_MAX = 50`。

- C1 或 C2 任一收到都会清零同一个 `board.status->offline_cnt`。
- `MonitorTask` 每 1 ms 累加计数。
- 连续 50 ms 没有 C1/C2，`board.status->status = DEV_OFFLINE`。
- C2 额外记录 `gimbal_rx_time_ms` 并置 `gimbal_data_valid`。
- 跟随和小陀螺各自检查 C2 年龄不超过 50 ms。
- 注意综合离线只表示“至少收到了 C1/C2 之一”，并不分别判断两块报文是否都完整。

## 5. 遥控与键鼠输入

### 5.1 DBUS 通道

| 变量 | 物理输入 | 当前用途 |
|---|---|---|
| `ch0` | 右摇杆左右 | 底盘 Yaw 速度、D5 Yaw 角速度或小陀螺调节 |
| `ch1` | 右摇杆上下 | D5 Pitch 角速度，机械模式 Pitch 微调 |
| `ch2` | 左摇杆左右 | 底盘左右平移 |
| `ch3` | 左摇杆上下 | 底盘前后运动 |
| `s1` | 左侧开关 | 控制来源、跟随/小陀螺、发射模式 |
| `s2` | 右侧开关 | 跟随/小陀螺/发射 |
| 拨轮 | 左拨轮 | 预留，当前不参与主控制 |

档位定义：

```text
RC_SW_UP   = 1
RC_SW_MID  = 3
RC_SW_DOWN = 2
```

### 5.2 遥控底盘输入

当 S1 为上或下时，允许底盘直控：

```text
vx = -ch3 / 660 * CHASSIS_MAX_VX
vy =  ch2 / 660 * CHASSIS_MAX_VY
wz =  ch0 / 660 * CHASSIS_MAX_WZ
```

输入先应用死区，再归一化到 `[-1, 1]`。

当前参数：

```c
#define CHASSIS_MAX_VX       35.0f
#define CHASSIS_MAX_VY       35.0f
#define CHASSIS_MAX_WZ       25.0f
#define CHASSIS_RC_DEADBAND  30.0f
```

### 5.3 键鼠模式

键鼠模式成立条件：

```text
CHASSIS_KEYBOARD_INPUT_ENABLE = 1
遥控器在线
keyboard_source_active == 1
S1 上位
```

F 键在 S1 上位时切换 `keyboard_source_active`。

键鼠底盘模式：

| 按键 | 模式 |
|---|---|
| Z | 跟随 |
| X | 机械/普通直控 |
| C | 小陀螺 |

运动映射：

```text
W/S      前后
A/D      左右
Q/E      旋转
Shift    加速 1.5x
Ctrl     减速 0.5x
```

### 5.4 机械模式和打符

S1 下拨时，`Board_Debug_Gimbal_Command()`：

- 发送 `gimbal_mode = 0`。
- Yaw 机械目标固定为 `BOARD_MEC_YAW_FRONT_RAD = 0`。
- Pitch 机械目标按 `ch1` 逐步累加，步长为 `0.002 rad`。
- Pitch 范围限制为 `-7.5..30 deg`。
- 底盘仍保留平移和转向输入。

打符入口：

```text
S1 上位
S2 中位
B 键按下沿
```

B 键翻转狗洞请求。请求置位时锁定机械模式并给出：

```text
Yaw 机械目标 = 0
Pitch 机械目标 = BOARD_HOLE_PITCH_TARGET_RAD = 0
```

退出打符会等待上板升降上报 `is_down == 2`，或等待 `BOARD_HOLE_EXIT_TIMEOUT_MS = 5000 ms`。

## 6. 底盘控制

### 6.1 输入与模式链

控制链固定为：

```text
Chassis_Input_Update
  -> Chassis_Follow_Update
  -> Chassis_Spin_Update
  -> Chassis_Control_Update
```

`Chassis_Input_Update()` 只生成统一指令：

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

来源：

```text
CHASSIS_SRC_NONE
CHASSIS_SRC_RC
CHASSIS_SRC_RC_FOLLOW
CHASSIS_SRC_SPIN
CHASSIS_SRC_KEYBOARD
```

### 6.2 四轮逆解

```text
LF = -vx + vy + wz
LB = -vx - vy + wz
RF =  vx + vy + wz
RB =  vx - vy + wz
```

当总速度超过 `CHASSIS_CTRL_MAX_SPEED = 80` 时：

- 旋转保留量最高为总限幅的 60%。
- 剩余量再按比例缩放平移。

### 6.3 速度环

当前为纯 P：

```c
#define CHASSIS_SPEED_KP   0.8f
#define CHASSIS_SPEED_KI   0.0f
#define CHASSIS_SPEED_KD   0.0f
```

力矩限幅按来源切换：

| 来源 | 限幅 |
|---|---:|
| 普通遥控/键鼠 | `CHASSIS_TEST_TORQUE_LIMIT_NM = 2.0 N*m` |
| 云台跟随 | `CHASSIS_FOLLOW_TORQUE_LIMIT_NM = 3.0 N*m` |
| 小陀螺 | `CHASSIS_SPIN_TORQUE_LIMIT_NM = 4.0 N*m` |

零速区域：

```text
目标绝对值 < 0.5
反馈绝对值 < 1.0
```

进入零速区域时清空 PID 状态并输出 0，避免静止时来回抽动。

### 6.4 底盘安全

底盘输出前检查：

- `cmd != NULL`。
- `enabled != 0`。
- `cmd.valid != 0`。
- `vx/vy/wz` 非 NaN 且绝对值小于 1000000。
- 四个轮对象、状态、控制器、速度环全部存在。
- 四轮全部在线。

任一条件失败都会调用 `Chassis_Control_Stop()`，四轮力矩清零并发送 `0x200` 组帧。

## 7. 跟随模式

### 7.1 选择条件

遥控：

```text
S1 上位
S2 上位或中位
```

键鼠：

```text
Z 键选择跟随
```

### 7.2 有效条件

- C2 数据必须接收过。
- C2 距当前时间不超过 `CHASSIS_FOLLOW_TIMEOUT_MS = 50 ms`。
- Yaw 机械角必须是有效数值且绝对值不超过 `pi + 0.01`。
- 连续两拍 Yaw 误差跳变不能超过 `CHASSIS_FOLLOW_YAW_JUMP_LIMIT_DEG = 30 deg`。

任一条件失败会锁存跟随故障，四轮输出清零。退出跟随档位后清除故障锁存。

### 7.3 控制参数

```c
#define CHASSIS_FOLLOW_KP                 20.0f
#define CHASSIS_FOLLOW_MAX_WZ             20.0f
#define CHASSIS_FOLLOW_WZ_STEP            0.2f
#define CHASSIS_FOLLOW_FRICTION_FF        7.0f
#define CHASSIS_FOLLOW_DEADBAND_DEG       7.0f
#define CHASSIS_FOLLOW_TURN_LOCK_DEG      150.0f
#define CHASSIS_FOLLOW_TURN_UNLOCK_DEG    20.0f
#define CHASSIS_FOLLOW_BLEND_TIME_MS      200u
```

跟随误差为：

```text
yaw_error = wrap_pi(yaw_mec - CHASSIS_FOLLOW_CENTER_RAD)
```

平移分量会按 `yaw_error` 旋转，使操作手在云台坐标系下控制。进入跟随时有 200 ms 融合，退出时斜坡回手动角速度。

## 8. 小陀螺模式

### 8.1 选择条件

遥控：

```text
S1 上位
S2 下位
```

键鼠：

```text
C 键选择小陀螺
```

### 8.2 参数

```c
#define CHASSIS_SPIN_MAX_WZ               20.0f
#define CHASSIS_SPIN_BASE_WZ              20.0f
#define CHASSIS_SPIN_TRIM_WZ              0.0f
#define CHASSIS_SPIN_STEP                 0.1f
#define CHASSIS_SPIN_RC_DEADBAND          30.0f
#define CHASSIS_SPIN_GIMBAL_TIMEOUT_MS    50u
```

当前 `CHASSIS_SPIN_TRIM_WZ = 0`，所以 `ch0` 对旋转速度没有实际调节作用，小陀螺以基础角速度 20 旋转。

### 8.3 平移参考系

```c
#define CHASSIS_SPIN_TRANSLATION_ENABLE       1u
#define CHASSIS_SPIN_TRANSLATION_FRAME_GIMBAL 1u
```

C2 有效且未超过 50 ms 时，平移按云台 Yaw 机械角旋转。C2 超时后：

- 平移 `vx/vy` 清零。
- 自转角速度仍由小陀螺状态机输出。

## 9. 发射决策

### 9.1 键鼠

键鼠模式下发射直接解锁：

- 鼠标左键按下时 `shoot_level = 1`。
- 短按单发。
- 左键长按切换连发。

### 9.2 遥控

安全解锁：

- 上电后必须先让 S2 档位发生一次变化。
- 防止上电时 S2 已经在上位直接发射。

S2 消抖：

```c
#define BOARD_LAUNCH_S2_DEBOUNCE_TICKS 15u
```

发射状态：

| S1 | S2 | 结果 |
|---|---|---|
| 上位 | 上位 | 连发 |
| 中位 | 上位 | 单发 |
| 上位/中位 | 中位 | 已解锁但不出弹 |
| 上位 | 下位 | 小陀螺，发射锁定 |
| 其他 | 任意 | 发射锁定 |

### 9.3 打符和升降联锁

当狗洞请求有效，或上板升降状态不是 `is_down == 2` 时：

```text
launch_state = L_LOCK
shoot_mode = SINGLE_SHOT
shoot_level = 0
```

即未完成找零、不在上位或正在打符时禁止发射。

### 9.4 D1 输出

每拍把决策写入：

```text
board.tx_pkt->shoot_pkt.launch_state
board.tx_pkt->shoot_pkt.shoot_mode
board.tx_pkt->shoot_pkt.shoot_level
```

## 10. 超电通信

### 10.1 CAN

| 方向 | ID |
|---|---|
| 下板发送 | `0x222` |
| 下板接收 | `0x211` |

控制帧：

| 字节 | 内容 |
|---|---|
| `0` | `power_buffer` |
| `1..2` | `power_limit` 大端 |
| `3..4` | `power_out_limit` 大端 |
| `5..6` | `power_in_limit` 大端 |
| `7` | bit0 `cap_switch`，bit1 `turbo_mode`，bit2 `pre_charge_enable` |

反馈帧：

```text
chassis_power = int16 big-endian
voltage_raw   = int16 big-endian
current_raw   = int16 big-endian
ability, pre_charge_mode 位于 Byte 6
```

### 10.2 当前行为

`supercap_config.h` 当前：

```c
#define SUPERCAP_CAP_SWITCH            0u
#define SUPERCAP_TURBO_MODE            0u
#define SUPERCAP_PRE_CHARGE_ENABLE     0u
#define SUPERCAP_POWER_BUFFER          0u
#define SUPERCAP_POWER_LIMIT           0u
#define SUPERCAP_POWER_OUT_LIMIT       0
#define SUPERCAP_POWER_IN_LIMIT        0u
```

因此超电只做周期通信保活，不让超电参与功率输出。`rp_config.h` 的 `CAP_SWITCH` 是旧总开关宏，不改变上述实际发送值。

离线超时：

```c
#define SUPERCAP_OFFLINE_TIMEOUT_MS 100u
```

反馈换算：

```text
cap_voltage = scale(voltage_raw, 0.0, 25.0)
cap_current = scale(current_raw, -16.0, 16.0)
```

## 11. 安全与故障

- 遥控离线时底盘输入清零，发射锁定，D5 `valid = 0`。
- 四轮任一离线时底盘全部停输出。
- 跟随模式 C2 超时或 Yaw 异常时锁存故障并停底盘。
- 小陀螺 C2 超时时保留自转但清除平移。
- 板间 C1/C2 连续 50 ms 无帧时板间状态离线。
- 发射需要云台归中、升降在上位、S2 已完成上电后的首次动作。
- 修改 CAN ID、字节序或协议布局时必须同步上下板。
- 看门狗未配置，当前异常保护主要依赖任务、遥控和 CAN 心跳。

## 12. 调参入口

### 12.1 输入与底盘

参数文件：

```text
Application/ConfigLayer/chassis_config.h
```

常用参数：

```text
CHASSIS_MAX_VX
CHASSIS_MAX_VY
CHASSIS_MAX_WZ
CHASSIS_RC_DEADBAND
CHASSIS_SPEED_KP
CHASSIS_TEST_TORQUE_LIMIT_NM
CHASSIS_ZERO_TARGET_BAND
CHASSIS_STOP_SPEED_BAND
CHASSIS_FOLLOW_*
CHASSIS_SPIN_*
```

部分宏当前是预留或旧阶段参数：

```text
CHASSIS_TURN_CYCLE_SPEED
CHASSIS_FIXED_CURRENT_LIMIT_A
CHASSIS_FIXED_TORQUE_LIMIT_NM
CHASSIS_CTRL_MAX_SPEED 仅用于逆解限幅
CHASSIS_LENGTH_M / WIDTH / DIAGONAL / MASS / WHEEL_RADIUS
```

几何质量参数当前不参与这套简化四轮速度环。

### 12.2 遥控和板间

```text
Application/ConfigLayer/board_comm_config.h
Application/ProtocolLayer/board_protocol.h
```

当前关键值：

```text
BOARD_COMM_D1D2_PERIOD_MS       1
BOARD_COMM_D5_ENABLE            1
BOARD_LAUNCH_S2_DEBOUNCE_TICKS  15
BOARD_LIFT_ENABLE               1
BOARD_OFFLINE_CNT_MAX           50
BOARD_D5_YAW_RATE_MAX_DEG_S     300
BOARD_D5_PITCH_RATE_MAX_DEG_S   150
```

### 12.3 超电

```text
Application/ConfigLayer/supercap_config.h
```

恢复超电输出前，必须先验证通信帧、电压电流反馈、预充逻辑和功率保护。当前配置不应直接用于正式功率控制。

## 13. Keil Watch 调试变量

```text
rc_dev.work_state
rc_data
keyboard_source_active
chassis_input_cmd
chassis_follow
chassis_spin
chassis_ctrl
wheel_motor
wheel_group
launch.state
launch.mode
launch.heart
launch.shoot_level
board.status
board.tx_pkt
board.rx_meg
board_lift_dbg
board_hole_request
board_hole_exit_pending
supercap
hfdcan1.Instance->ECR
hfdcan2.Instance->ECR
```

常用判断：

- 底盘不动：看 `rc_dev.work_state`、`chassis_input_cmd.valid`、`chassis_ctrl.state.fault`、四轮在线位。
- 单轮反了：检查对应 `rxId`、左/右前定义和轮组反馈 ID。
- 底盘抽搐：看零速区域、速度环 Kp 和反馈速度。
- 跟随失效：看 `chassis_follow.selected`、`active`、`fault_latched`、`board.status->gimbal_rx_time_ms`。
- 小陀螺不平移：看 C2 是否在 50 ms 内。
- 发射不动作：看 S1/S2、S2 是否解锁、`launch.shoot_level`、`launch.state`、`is_down`。
- 板间离线：看 `board.status->offline_cnt`、FDCAN2 波特率和 CAN2 过滤器。
- 超电离线：看 `supercap.state`、`rx_count`、`last_rx_ms`。

## 14. 历史问题与对策

| 问题 | 当前处理 |
|---|---|
| `StartUITask` 重名导致 `L6200E` | `connect_task.c` 改为 `StartConnectTask`，保留 weak 壳 |
| `ConnectTask` 没有发送 D1/D2 | 增加 1 ms 发送任务 |
| D1/D2 发送缓存有脏数据 | 发送前 `memset` |
| D3/D4、裁判、视觉、UI 等旧链路残留 | 当前从活动代码删除，只保留 D1/D2/D5 和 C1/C2 |
| 旧整车模块一次性初始化风险大 | `DEVICE_Init()` 改为当前底盘、遥控、发射、超电和板间模块 |
| 底盘任务调用和指针安全 | 四轮对象、控制器和发送接口逐级判空 |
| 四轮反馈 C2 超时导致跟随危险 | 50 ms 超时和 Yaw 跳变故障锁存 |
| 小陀螺平移参考系错误 | 有效时按云台 Yaw 旋转，超时清平移 |
| S2 回中接触弹跳 | 15 ms 软件消抖 |
| 上电 S2 已在上位误发射 | 上电后先动作一次才解锁 |
| 打符时云台和发射冲突 | 打符锁机械模式，升降未到上位时禁发 |
| 超电尚未完成正式联调 | 所有输出开关保持 0，只保活通信 |
| 下板本机 IMU 残留 | 当前活动链不使用本机 IMU |
| 上下板 CAN 波特率不一致风险 | `.ioc` 目标 1 Mbps，生成代码计算 500 kbps，需仪器确认 |

## 15. 常见问题排查

| 现象 | 优先检查 |
|---|---|
| 遥控离线 | DBUS 接线、UART5 PD2、波特率、9 位偶校验、18 字节帧、DMA Idle 回调 |
| 板间无 C1/C2 | FDCAN2 波特率、终端电阻、ID、过滤器、上板发送任务 |
| D1/D2/D5 无帧 | `BOARD_COMM_TX_ENABLE`、`StartConnectTask` 是否强定义、FDCAN2 发送 |
| 底盘输入无效 | S1 档位、遥控在线、死区、`cmd.source`、`cmd.valid` |
| 键鼠切不过来 | F 键、S1 是否上位、`keyboard_source_active` |
| 四轮只动一部分 | 反馈 ID 与槽位、FDCAN1 终端、电机离线计数 |
| 底盘方向错误 | 逆解符号、`CHASSIS_KEY_*_SIGN`、单轮接线 |
| 原地零速抖动 | 零速区域、Kp、轮速反馈噪声、力矩限幅 |
| 跟随不接管 | S1/S2 档位、C2 数据年龄、Yaw 跳变故障 |
| 跟随退出跳变 | 200 ms 融合、`WZ_STEP` |
| 小陀螺不自转 | S1/S2 档位或 C 键、`CHASSIS_SPIN_ENABLE` |
| 小陀螺平移无效 | C2 超时、`CHASSIS_SPIN_GIMBAL_TIMEOUT_MS` |
| 发射锁死 | S2 是否动作解锁、升降是否 `is_down == 2`、是否打符 |
| 发射档位乱 | S1/S2 组合、15 ms 消抖、遥控帧稳定性 |
| 超电不参与输出 | 当前设计如此，所有输出参数为 0 |
| 超电离线 | `0x211` 反馈、FDCAN1、100 ms 超时、`supercap.rx_count` |

## 16. 目录结构

```text
task_down/
  Application/
    AlgorithmLayer/   PID、数学工具
    ConfigLayer/      底盘、通信、超电、设备参数
    DeviceLayer/      遥控、超电、电机对象
    DriverLayer/      FDCAN、UART、DWT、GPIO
    HardwareLayer/    RM 电机协议
    ModuleLayer/      输入、跟随、小陀螺、底盘、发射
    ProtocolLayer/    板间、遥控、超电、CAN 分发
    TaskLayer/        Command、Ctrl、Connect、Monitor 任务
  Core/               CubeMX 生成的内核和中断
  Drivers/            STM32H7 HAL/CMSIS
  Middlewares/        FreeRTOS
  MDK-ARM/            Keil 工程与产物
```

## 17. 验收清单

- `DM-MC02` 全量 rebuild 为 0 errors。
- DBUS 18 字节帧稳定解析，遥控离线后所有输入归零。
- 上下板 CAN 波特率经仪器确认一致。
- D1/D2/D5 能按 1 ms 周期发送。
- C1/C2 50 ms 内无持续离线。
- 四轮单轮方向、四轮同向、平移、旋转、斜向均通过台架测试。
- 普通速度环、跟随、小陀螺分别完成限矩验证。
- 跟随 C2 超时和 Yaw 跳变能锁存故障并停车。
- 小陀螺自转稳定，C2 超时后平移清零。
- S2 消抖有效，上电不会因 S2 已在上位直接发射。
- 打符期间禁止发射，升降不在上位时禁止发射。
- 超电通信正常，但输出仍保持关闭。
- 所有 CAN ID、协议字段或数据方向改动同步检查 `task_up`。
