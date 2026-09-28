# task_up 上板控制工程

`task_up` 运行在 STM32F407IGHx 上，是云台、发射机构和狗洞升降的执行板。整车输入、遥控器/键鼠解析、底盘控制和发射决策由下板 `task_down` 负责；上板只接收 D1/D2/D5，执行云台闭环、发射机构控制和升降状态机，再通过 C1/C2 返回状态。

```text
下板 task_down
  D1: 整车状态、云台模式、发射许可、狗洞请求
  D2: 云台机械角/IMU 角目标
  D5: 遥控或键鼠控制量
        |
        v
上板 task_up
  BMI088 -> IMU 姿态
  Yaw/Pitch DM4310 -> 云台闭环
  M3508 x2 -> 摩擦轮
  KT4005 -> 拨盘
  RM2006 -> 狗洞升降
  C1/C2 + CAN 电机输出
```

当前代码不包含自瞄、视觉串级、UI、裁判系统和正式功率限制。上板本机遥控器链路已经移除，手动控制只接受下板 D5。

## 1. 快速开始

### 1.1 硬件与工具

| 项目 | 当前配置 |
|---|---|
| 主控 | STM32F407IGHx，外部 12 MHz 晶振 |
| 系统时钟 | SYSCLK 168 MHz |
| 调试器 | J-Link、ST-Link 或 DAPLink，SWD |
| IDE | Keil MDK5 |
| 工程 | `task_up/MDK-ARM/My_C.uvprojx` |
| Target | `My_C` |
| 编译环境 | ARM Compiler 5.06 update 7（Keil 日志记录） |
| 板间通信 | CAN2，经典 CAN |
| IMU | BMI088，SPI1 |

电机与 CAN 连接：

| 设备 | 型号 | 控制帧/ID | 反馈 ID | CAN |
|---|---|---|---|---|
| Pitch 云台 | DM4310 | `0x01` | `0x11` | CAN1 |
| Yaw 云台 | DM4310 | `0x02` | `0x12` | CAN2 |
| 左摩擦轮 | M3508 | `0x200` 槽 0 | `0x201` | CAN1 |
| 右摩擦轮 | M3508 | `0x200` 槽 1 | `0x202` | CAN1 |
| 升降 | RM2006 | `0x200` 槽 3 | `0x204` | CAN1 |
| 拨盘 | KT4005 | `0x141` | `0x141` | CAN1 |

首次带电前必须拆弹、架空摩擦轮、可靠固定云台，并准备 CAN 分析仪和急停手段。机械方向、机械中值、限位和 PID 未标定时，不允许直接进行整机高速运行。

### 1.2 编译

Keil 图形界面：

```text
打开 task_up/MDK-ARM/My_C.uvprojx
Target: My_C
Build/Rebuild
```

命令行：

```powershell
UV4 -r task_up\MDK-ARM\My_C.uvprojx -t My_C -o build_up.log
```

构建产物：

```text
task_up/MDK-ARM/My_C/My_C.axf
task_up/MDK-ARM/My_C/My_C.hex
```

工程内保留的 2026-09-28 构建日志结果：

```text
Program Size: Code=71296 RO-data=2840 RW-data=2752 ZI-data=25272
".\My_C\My_C.axf" - 0 Error(s), 0 Warning(s).
```

修改源码后必须重新执行目标 `My_C` 的全量 rebuild，上表只代表对应日志时刻。

### 1.3 上电顺序

1. 整机断电，云台可靠固定，发射机构空载。
2. 检查 CANH/CANL 不反接，总线两端 120 欧终端正确。
3. 确认上下板 CAN 波特率一致。当前上板 CAN 为 1 Mbps，下板生成代码的参数存在不一致风险，详见 CAN 章节。
4. 先只上控制电，观察下板是否发送 D1/D2/D5。
5. 确认上板收到 D1/D2，`Board_HeartBeat.status == DEV_ONLINE`。
6. 确认 C1/C2 持续返回，下板能收到云台反馈。
7. 确认 IMU 在线并完成校准。
8. 让 `car_state == 1`，先观察云台归中，再验证机械模式和速控模式。
9. 云台稳定后，再验证单发、连发和升降。
10. 每次只接一个执行机构，先空载，再逐步加负载。

## 2. 启动与任务

### 2.1 main 启动顺序

```text
HAL_Init
SystemClock_Config
MX_GPIO_Init
MX_DMA_Init
MX_SPI1_Init
MX_CAN1_Init
MX_CAN2_Init
MX_USB_DEVICE_Init
DEVICE_Init
Module_Init
DRIVER_Init
MX_FREERTOS_Init
osKernelStart
```

| 函数 | 工作 |
|---|---|
| `DEVICE_Init()` | IMU、RM 电机组、KT 拨盘、DM 云台电机组、发射机构 |
| `Module_Init()` | 绑定并初始化云台 PID、绑定升降对象 |
| `DRIVER_Init()` | 初始化 CAN1/CAN2 过滤器并启动 CAN |
| `MX_FREERTOS_Init()` | 创建 `ControlTask`、`MonitorTask`、`LedTask` |

IWDG 初始化代码存在，但 `main.c` 中 `MX_IWDG_Init()` 当前被注释，看门狗未启用。

### 2.2 FreeRTOS 任务

系统 tick 为 1 kHz。

| 任务 | 优先级 | 栈 | 周期 | 主要工作 |
|---|---:|---:|---:|---|
| `ControlTask` | `osPriorityRealtime` | 1024 words | 1 ms | IMU、云台、升降、DM 发送、发射、C1/C2 |
| `MonitorTask` | `osPriorityRealtime` | 512 words | 1 ms | IMU、DM、RM、KT 和板间心跳 |
| `LedTask` | `osPriorityAboveNormal` | 256 words | 开机后 5 Hz | 绿灯状态指示 |

`StartControlTask()` 固定顺序：

```text
IMU update
IMU debug snapshot
Module_Work
  Gimbal_Work
  Lift_Work
gimbal_can_send
Launcher_Work
Send_To_Down_Board
```

这个顺序不能随意交换。云台模式切换、升降故障恢复和发射时序都依赖 1 ms 周期。

`gimbal_can_send()` 同时检查：

```text
Board_HeartBeat.status == DEV_ONLINE
Board_Rx_Info.state_pkt.car_state != 0
```

任一条件不满足时，DM 云台组执行卸力并发送卸力帧。

## 3. 外设与 CAN

### 3.1 引脚与外设

| 外设 | 引脚/参数 | 用途 |
|---|---|---|
| SPI1 | PB3/PB4/PA7，软件 NSS | BMI088 |
| BMI088 加速度计片选 | `CS1_ACCEL`，PA4 | 加速度计 |
| BMI088 陀螺仪片选 | `CS1_GYRO`，PB0 | 陀螺仪 |
| CAN1 | PD0/PD1，1 Mbps | Pitch、摩擦轮、拨盘、升降 |
| CAN2 | PB5/PB6，1 Mbps | Yaw、D1/D2/D5、C1/C2 |
| USB OTG FS | USB CDC | 调试接口 |
| TIM2 | HAL 时基 | `HAL_GetTick()` |
| LED | PH10/PH11/PH12 | 状态指示 |
| IWDG | 已配置代码，未启动 | 当前无看门狗保护 |

SPI1 当前为主机模式、8 位、CPOL=1、CPHA=2Edge、MSB First、预分频 16。APB2 为 84 MHz 时，SPI 时钟约为 5.25 Mbit/s。

### 3.2 CAN1

| CAN ID | 方向 | 内容 |
|---|---|---|
| `0x01` | 上板发送 | Pitch DM4310 控制帧 |
| `0x11` | 上板接收 | Pitch DM4310 反馈 |
| `0x141` | 双向 | KT4005 拨盘命令/反馈 |
| `0x200` | 上板发送 | M3508 组控制：槽 0 左摩擦轮，槽 1 右摩擦轮，槽 3 升降 |
| `0x201` | 上板接收 | 左摩擦轮反馈 |
| `0x202` | 上板接收 | 右摩擦轮反馈 |
| `0x204` | 上板接收 | 升降 RM2006 反馈 |

### 3.3 CAN2

| CAN ID | 方向 | 内容 |
|---|---|---|
| `0x02` | 上板发送 | Yaw DM4310 控制帧 |
| `0x12` | 上板接收 | Yaw DM4310 反馈 |
| `0xD1` | 下板发送 | 整车状态、发射状态、狗洞请求 |
| `0xD2` | 下板发送 | 云台目标角 |
| `0xD5` | 下板发送 | 遥控/键鼠控制量 |
| `0xC1` | 上板发送 | 电机在线状态、升降状态 |
| `0xC2` | 上板发送 | 云台机械角/IMU 角反馈 |

CAN1/CAN2 使用经典 CAN，启用自动离线恢复，关闭自动重传，发送 FIFO 优先级使能。过滤器为全通掩码，实际分发由 `CAN1_rxDataHandler()` 和 `CAN2_rxDataHandler()` 完成。

### 3.4 CAN 波特率风险

上板：

```text
APB1 = 42 MHz
Prescaler = 3
TQ = 1 + 10 + 3 = 14
Baud = 42 MHz / 3 / 14 = 1 Mbps
```

下板当前生成代码：

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

`task_down/DM-MC02.ioc` 记录的目标是 FDCAN 100 MHz、1 Mbps，和生成代码不一致。双板通信前必须用 CAN 分析仪或示波器确认实际波特率。该问题优先于 D1/D2 字段排查。

## 4. 板间协议

本节与 `task_down/Readme.md` 必须同步维护。D1/D2/D5 或 C1/C2 任一字段改动，不允许只改一侧。

### 4.1 D1，下板到上板

D1 固定 8 字节，下板发送前会清零缓存。

| 字节 | 位 | 含义 |
|---|---|---|
| `Byte 0` | bit 1:0 | `car_state`，0 为卸力，非 0 为允许控制 |
| `Byte 0` | bit 2 | `gimbal_mode`，0 为机械模式，1 为速控模式 |
| `Byte 5` | bit 0 | `launch_state`，发射许可 |
| `Byte 5` | bit 1 | `shoot_mode`，0 单发，1 连发 |
| `Byte 5` | bit 2 | `shoot_level`，发射触发电平 |
| `Byte 5` | bit 3 | `is_hole`，狗洞请求 |

D1 其余位当前为零，不应解释成其他功能。

### 4.2 D2，下板到上板

D2 按大端序发送 4 个 16 位线性压缩值：

```text
raw = (value - min) / (max - min) * 65535
```

| 字节 | 字段 | 单位 | 量程 |
|---|---|---|---|
| `0..1` | `pitch_imu_tar` | deg | `-360..360` |
| `2..3` | `yaw_imu_tar` | deg | `-360..360` |
| `4..5` | `pitch_mec_tar` | rad | `-4..4` |
| `6..7` | `yaw_mec_tar` | rad | `-4..4` |

上板解码后写入 `Board_Rx_Info.gimbal_target_pkt`。机械角在云台模块中转换成 deg，IMU 角保持 deg。

下板发送函数本身没有额外限幅，目标值必须在写入发送缓存前限制在量程内。

### 4.3 D5，下板到上板

| 字节 | 含义 |
|---|---|
| `Byte 0` | bit0 有效，bit1 输入源，bit2 控制类型 |
| `Byte 1` | 鼠标左右键位 |
| `Byte 2..3` | `int16_t` 大端，Yaw 角速度，0.1 deg/s/LSB |
| `Byte 4..5` | `int16_t` 大端，Pitch 角速度，0.1 deg/s/LSB |
| `Byte 6..7` | 保留 |

`ctrl_source`：0 为遥控，1 为键鼠。

`cmd_type`：0 为角速度包，1 为鼠标增量包。

下板当前始终发送 `cmd_type = 0`。键鼠模式也会先在下板转换成 Yaw/Pitch 角速度。上板保留鼠标增量分支，但当前数据流不走该分支。

### 4.4 C1，上板到下板

| 字节 | 位 | 含义 |
|---|---|---|
| `Byte 0` | bit 0 | Yaw 电机在线 |
| `Byte 0` | bit 1 | Pitch 电机在线 |
| `Byte 0` | bit 2 | 升降电机在线 |
| `Byte 0` | bit 3 | 右摩擦轮在线 |
| `Byte 0` | bit 4 | 左摩擦轮在线 |
| `Byte 0` | bit 5 | 拨盘在线 |
| `Byte 1` | 全部 | 升降状态 |

C1 Byte 1：

```text
0 = 下位或下行堵转停机
1 = 正在运动
2 = 上位等待或未完成找零
3 = 升降故障
```

下板把 C1 Byte 1 保存为 `board.rx_meg->state_meg.is_down`。该变量实际是上板升降状态，不是“下板是否在线”。下板当前没有消费 C1 Byte 0 的 bit 2。

### 4.5 C2，上板到下板

| 字节 | 字段 | 单位 | 量程 |
|---|---|---|---|
| `0..1` | `yaw_mec` | rad | `-4..4` |
| `2..3` | `pitch_mec` | rad | `-4..4` |
| `4..5` | `yaw_imu` | deg | `-360..360` |
| `6..7` | `pitch_imu` | deg | `-360..360` |

上板打包时对四个值做限幅。下板 C2 数据用于跟随云台角速度、小陀螺平移变换和状态观测。

### 4.6 心跳

`Board_HeartBeat.offline_cnt_max = 100`。

- D1、D2、D5 收到后分别清零 `offline_cnt_1/2/5`。
- `C_Board_Communicate_HeartBeat()` 每 1 ms 累加三个计数。
- D1 或 D2 任一达到 100，`Board_HeartBeat.status = DEV_OFFLINE`。
- D5 超时只影响手动控制，不直接让综合在线状态离线。
- 板间离线时云台 DM 组卸力，C1/C2 仍继续发送本地状态。

## 5. 云台控制

### 5.1 单位与机械角

| 数据 | 单位 |
|---|---|
| IMU 角度 | deg |
| IMU 角速度 | deg/s |
| 电机机械角 | 内部使用 deg |
| 电机机械角速度 | rad/s |
| DM 输出力矩 | N*m |
| D2 机械目标 | rad |
| D2 IMU 目标 | deg |

机械角：

```text
yaw_mec_angle   = wrap_180(yaw_motor.motor_angle_deg   - GIMBAL_YAW_MIDDLE_DEG)
pitch_mec_angle = wrap_180(pitch_motor.motor_angle_deg - GIMBAL_PITCH_MIDDLE_DEG)
```

当前机械中值：

```c
#define GIMBAL_YAW_MIDDLE_DEG      (-22.224138f)
#define GIMBAL_PITCH_MIDDLE_DEG    148.573157f
```

这两个值必须按整车安装重新标定。

Pitch 机械限位：

```c
#define GIMBAL_PITCH_MIN_DEG       (-7.5f)
#define GIMBAL_PITCH_MAX_DEG       30.0f
```

### 5.2 模式仲裁

| 模式 | 选择条件 | 说明 |
|---|---|---|
| `G_SLEEP` | 板间离线或 `car_state == 0` | 卸力 |
| `G_INIT` | 安全条件满足且 `init_flag == 0` | 上电首次归中 |
| `G_MEC` | `init_flag == 1` 且 D1 `gimbal_mode == 0` | 机械编码器位置环 |
| `G_RATE` | `init_flag == 1` 且 D1 `gimbal_mode != 0` | 速控模式 |
| `G_GYRO` | 当前仲裁不选择 | 保留枚举 |
| `G_AUTO` | 当前仲裁不选择 | 保留枚举 |

安全条件只有板间在线和 `car_state != 0`。任一条件不满足时进入 `G_SLEEP`，同时清除归中完成标志和计时。

### 5.3 上电归中

归中步进为 `0.25 deg/ms`，完成条件：

```text
Yaw/Pitch 角度误差 <= 2.0 deg
Yaw/Pitch 机械速度 <= 0.5 rad/s
斜坡目标与最终目标误差 <= 0.1 deg
连续 30 ms
```

超时时间为 6000 ms，超时后强制允许进入后续模式。归中目标来自 D2 机械目标角。

### 5.4 机械模式

```text
目标机械角
  -> 位置外环 P/D
  -> 按剩余误差计算制动限速
  -> 电机速度内环 PID
  -> 力矩
```

小误差区叠加：

```text
hold_torque = hold_kp * angle_error - hold_kd * speed
```

Yaw：

| 参数 | 值 |
|---|---:|
| 最大速度 | 120 deg/s |
| 制动减速度 | 8 rad/s^2 |
| 保持 Kp | 0.8 N*m/deg |
| 保持 Kd | 0.08 N*m/(rad/s) |
| 保持力矩限幅 | 3 N*m |

Pitch：

| 参数 | 值 |
|---|---:|
| 最大速度 | 90 deg/s |
| 制动减速度 | 6 rad/s^2 |
| 保持 Kp | 0.5 N*m/deg |
| 保持 Kd | 0.06 N*m/(rad/s) |
| 保持力矩限幅 | 3 N*m |

Pitch 在机械模式下叠加余弦重力补偿。

### 5.5 速控模式

手动输入有效条件：

```text
Board_HeartBeat.offline_cnt_5 < 100
remote_cmd_pkt.valid != 0
car_state != 0
```

输入源：

- 遥控源直接使用 D5 的 Yaw/Pitch 角速度。
- 键鼠且 `cmd_type == 1` 时使用鼠标增量累加虚拟目标角。
- 当前下板 `cmd_type` 始终为 0，因此实际走角速度分支。

输出结构：

```text
Yaw:
  操作手角速度 + 松杆保持环修正
  -> Yaw IMU 角速度内环
  -> Yaw 力矩

Pitch:
  操作手角速度 + 松杆保持环修正
  -> Pitch 机械速度内环
  -> Pitch 力矩
```

松杆保持：

- 进入阈值 `2.0 deg/s`，退出阈值 `8.0 deg/s`。
- 阈值之间线性混合。
- Yaw 用 IMU Yaw 角。
- Pitch 用机械编码器角。
- IMU 安装在偏航部分，Pitch 运动时 IMU 欧拉 Pitch 不能提供可用的俯仰相对角。

默认保持环：

```c
#define GIMBAL_YAW_HOLD_KP                 10.0f
#define GIMBAL_YAW_HOLD_KI                 0.003f
#define GIMBAL_YAW_HOLD_OUT_MAX            150.0f
#define GIMBAL_PITCH_HOLD_KP               50.0f
#define GIMBAL_PITCH_HOLD_KI               0.0f
#define GIMBAL_PITCH_HOLD_OUT_MAX          1000.0f
```

### 5.6 重力补偿与限幅

```text
gravity = sign * (K * cos(pitch_mec_angle - middle_angle) + B)
```

默认值：

```c
#define GIMBAL_GRAVITY_ENABLE      1
#define GIMBAL_GRAVITY_K_NM        1.1f
#define GIMBAL_GRAVITY_B_NM        0.0f
#define GIMBAL_GRAVITY_SIGN        1.0f
#define GIMBAL_GRAVITY_MIDDLE_DEG  0.0f
#define GIMBAL_TORQUE_LIMIT        6.0f
```

补偿方向错误时，Pitch 可能持续顶向一侧、下垂加剧或回不到目标。先确认机械方向，再调 `gravity_sign` 和 `gravity_k_nm`。

模式切换会清空 PID 历史、对齐当前目标、开启斜坡，并在切换帧先输出 0 力矩。

## 6. 发射机构

### 6.1 设备与参数

| 设备 | 类型 | 索引/ID | 作用 |
|---|---|---|---|
| 左摩擦轮 | M3508 | `0x200` 槽 0，反馈 `0x201` | 发弹摩擦 |
| 右摩擦轮 | M3508 | `0x200` 槽 1，反馈 `0x202` | 发弹摩擦 |
| 拨盘 | KT4005 | `0x141` | 单发、连发供弹 |

```c
#define LAUNCHER_FRIC_TARGET_RPM       1500.0f
#define LAUNCHER_FRIC_READY_TOL_RPM    500.0f
#define LAUNCHER_FRIC_READY_TIME_MS    100u
#define LAUNCHER_DIAL_ONE_SHOT_ANGLE   65536.0f
#define LAUNCHER_DIAL_REPEAT_SPEED_DPS 5400u
#define LAUNCHER_DIAL_CURRENT_LIMIT    2000.0f
```

### 6.2 使能条件

```text
Board_HeartBeat.status == DEV_ONLINE
D1 launch_state != 0
左、右摩擦轮在线
拨盘在线
```

当前发射总使能不直接检查 `car_state`。实际安全前置由下板 `launch_state` 决策、打符互锁和电机在线条件共同保证。修改 D1 语义时必须保留这个约束。

### 6.3 状态机

```text
SLEEP
  -> READY
  -> SINGLE
  -> READY
  -> REPEAT
  -> READY
  -> STOPPING
  -> SLEEP
```

辅助状态：

- `REVERSE`：卡弹后反转退让。
- `RELOAD`：退让后回到原供弹目标。
- `FAULT`：故障锁定。
- `SPINUP`、`INIT`：保留状态，当前默认流程从 `SLEEP` 直接进入 `READY`，拨盘自动回零关闭。

触发规则：

- 单发：`shoot_mode == 0` 且 `shoot_level` 出现 0 到 1 上升沿。
- 连发：`shoot_mode == 1` 且 `shoot_level != 0`。
- 单发每次增加 `65536 count`。
- 连续发射使用独立速度环。
- 松开触发后先制动，速度低于 20 deg/s 或 120 ms 超时后发送 `MOTOR_STOP_ID`。

摩擦轮：

- 目标 1500 rpm。
- 左右方向分别由 `LAUNCHER_FRIC_L_DIRECTION` 和 `LAUNCHER_FRIC_R_DIRECTION` 修正。
- 双轮都达到容差并持续 100 ms 后置 `fric_ready`。
- 总使能关闭后按 `20 rpm/ms` 降速，双轮低于 100 rpm 并连续 1000 ms 后进入 `SLEEP`。

### 6.4 卡弹恢复

当前默认关闭：

```c
#define LAUNCHER_DIAL_JAM_ENABLE       0u
```

打开后检测低速高流，连续 200 ms 后反转 65536 count，再回到原供弹目标。启用前必须架空拨盘并记录：

```text
dail_motor.KT_motor_info.rx_info.current
dail_motor.KT_motor_info.rx_info.speed
launcher.jam_tick
launcher_jam_count
```

## 7. 狗洞升降

### 7.1 状态与上报

| 状态 | C1 上报值 | 说明 |
|---|---:|---|
| `LIFT_READY_DOWN` | 0 | 下位到位 |
| `LIFT_STALL_STOP` | 0 | 下行堵转停机 |
| `LIFT_HOMING_UP`、`LIFT_RETRACT_DOWN`、`LIFT_ALIGN_DOWN`、`LIFT_MOVING_DOWN`、`LIFT_MOVING_UP` | 1 | 运动中 |
| `LIFT_WAIT`、`LIFT_READY_UP` | 2 | 等待或上位 |
| `LIFT_FAULT` | 3 | 故障锁定 |

### 7.2 找零

上电后等待 2000 ms 或收到狗洞请求，向上找顶。找顶条件为电流达到阈值，并且静止窗口或低速条件满足，连续 500 ms 后记录顶部零点；随后向下回转 5 圈作为上位。

```c
#define LIFT_HOME_SPEED_RPM             2865.0f
#define LIFT_HOME_CURRENT_RAW           520.0f
#define LIFT_HOME_STALL_SPEED_RPM       1.0f
#define LIFT_HOME_STATIONARY_WINDOW_MS  500u
#define LIFT_HOME_STATIONARY_COUNTS     40.0f
#define LIFT_HOME_CONFIRM_MS            500u
```

### 7.3 行程与对齐

```c
#define LIFT_POSITION_COUNTS_PER_UNIT   1.0f
/* 临时测试行程，验证后改回 316 圈 */
#define LIFT_TRAVEL_TURNS               280.0f
#define LIFT_TRAVEL_COUNTS              (LIFT_TRAVEL_TURNS * 8192.0f)
```

当前 280 圈只是临时测试值，装车确认后必须更新。

下降前云台必须在 `G_MEC` 且归中完成，Yaw/Pitch 误差不超过 5 deg，速度不超过 0.2 rad/s。

### 7.4 故障码

| 码 | 含义 |
|---:|---|
| 1 | 找顶超时 |
| 2 | 运动超时 |
| 3 | 超程 |
| 4 | 堵转电流确认 |
| 5 | 堵转无进展 |

进入 `LIFT_FAULT` 后输出 0。只有狗洞命令再次发生边沿变化，才允许重新找零或恢复运动。下行堵转进入 `LIFT_STALL_STOP` 并保留复位路径；上行堵转直接进入故障。

## 8. 安全联锁

- D1 或 D2 任一连续 100 ms 未更新，板间链路判离线。
- 板间离线时云台 DM 组卸力。
- `car_state == 0` 时云台不出力，升降停止。
- D5 超时或无效时，上板手动速度归零。
- 发射许可必须由下板通过 D1 给出。
- 发射还需左右摩擦轮和拨盘在线。
- 发弹释放后先主动制动再停机。
- 升降找顶、运动、超程、堵转和电流异常都会进入故障或堵转停机。
- 看门狗当前未启用，异常恢复依赖 FreeRTOS 任务和 CAN/电机心跳。

## 9. 调参与调试

### 9.1 云台

默认值在 `Application/ModuleLayer/gimbal.h`。

Keil Watch 在线变量：

```text
gimbal_tune.gravity_enable
gimbal_tune.gravity_k_nm
gimbal_tune.gravity_b_nm
gimbal_tune.gravity_sign
gimbal_tune.gravity_middle_deg
gimbal_tune.pitch_torque_limit_nm
gimbal_tune.yaw_torque_limit_nm
gimbal_tune.yaw_hold_kp
gimbal_tune.yaw_hold_ki
gimbal_tune.yaw_hold_integral_max
gimbal_tune.yaw_hold_out_max
gimbal_tune.pitch_hold_kp
gimbal_tune.pitch_hold_ki
gimbal_tune.pitch_hold_integral_max
gimbal_tune.pitch_hold_out_max
gimbal_tune.rate_hold_enter_deg_s
gimbal_tune.rate_hold_exit_deg_s
gimbal_tune.manual_yaw_sign
gimbal_tune.manual_pitch_sign
gimbal_tune.yaw_manual_rate_max_deg_s
gimbal_tune.pitch_manual_rate_max_deg_s
gimbal_tune.mouse_yaw_deg_per_count
gimbal_tune.mouse_pitch_deg_per_count
gimbal_tune.mouse_yaw_sign
gimbal_tune.mouse_pitch_sign
gimbal_tune.mouse_rate_ff_dps_per_count
gimbal_tune.mouse_deadband_count
```

推荐顺序：

1. 标定 Yaw/Pitch 机械中值。
2. 确认电机方向、机械限位和编码器符号。
3. 限矩先降到 1 N*m 以内，调 Pitch 机械内环和外环。
4. 调 Yaw 机械环，再调速控模式 Yaw/Pitch 内环。
5. 最后调重力补偿、松杆保持和鼠标手感。

### 9.2 发射

参数在 `Application/ConfigLayer/launcher_config.h`。

调参顺序：

1. 只开摩擦轮，确认左右轮转向。
2. 调摩擦轮速度环。
3. 单发调拨盘角度环和速度环。
4. 连发单独调 `LAUNCHER_DIAL_REPEAT_*`。
5. 停机无回弹后再考虑打开卡弹恢复。

当前保留项：

- `LAUNCHER_FRIC_RAMP_RPM_PER_MS` 未参与启动升速。
- `LAUNCHER_FRIC_KFF` 未参与摩擦轮控制。
- `LAUNCHER_DIAL_AUTO_RESET_ENABLE = 0`。
- `LAUNCHER_SPINUP`、`LAUNCHER_INIT` 当前默认流程不可达。

### 9.3 升降

参数在 `Application/ConfigLayer/lift_config.h`，在线结构为 `lift_tune` 和 `lift_debug`。

1. 用 `lift_debug.mode = 1/2` 小输出确认上下方向。
2. 确认 `LIFT_UP_DIRECTION`、`LIFT_OUTPUT_DIRECTION`、`LIFT_SPEED_DIRECTION`。
3. 低输出找顶，确认电流阈值不误判。
4. 记录找顶后的 `encoder_sum`，修正行程圈数。
5. 先调位置环，再调速度环，最后验证超时、超程和堵转。

### 9.4 常用 Watch

```text
imu_dbg
Gimbal.gimbal_mode
Gimbal.base_info
Gimbal.feedforward
Gimbal.pid_info
gimbal_tune
launcher.state
launcher.enabled
launcher.fric_ready
launcher.fric_l_speed_rpm
launcher.fric_r_speed_rpm
launcher.dial_angle
launcher.dial_target_angle
launcher.fault
lift.state
lift.fault_code
lift.top_zero
lift.top_target
lift.bottom_target
lift.home_valid
lift_debug
lift_tune
Board_HeartBeat
Board_Rx_Info
Board_Tx_Info
```

## 10. 历史问题与对策

| 问题 | 当前处理 |
|---|---|
| `StartUITask` 重复定义导致 `L6200E` | 任务强定义改为正确名称，weak 壳不再重复实现 |
| 机械模式与发射时序相互干扰 | 固定 ControlTask 顺序，并用云台模式和发射状态拆开 |
| S2 回中弹跳导致发射抖动 | 下板增加 15 ms 软件消抖 |
| 上电时 S2 已在上位误发射 | 上电后 S2 必须先动作一次才解锁 |
| 升降误入 fault 后无法退出 | 故障后等待狗洞命令边沿再恢复 |
| 找顶时零速误判为到顶 | 电流设为主判据，并要求持续确认 |
| 升降行程不确定 | 当前使用 280 圈临时值，正式值需实测 |
| IMU 装偏航部分影响 Pitch 保持 | Pitch 保持环改用机械编码器 |
| 重力补偿方向未定 | `gravity_sign` 和 `gravity_k_nm` 保留为标定项 |
| 卡弹恢复未经完整验证 | 默认关闭 |
| 上下板 CAN 波特率存在不一致风险 | 双板联调前用仪器确认并修正配置 |
| IWDG 未启用 | 当前不能依赖看门狗自动复位异常 |

## 11. 常见问题排查

| 现象 | 优先检查 |
|---|---|
| C1/C2 没有帧 | CAN2 波特率、终端电阻、ID、`Send_To_Down_Board()` |
| D1/D2 有帧但上板不收 | 上下板波特率、CANH/CANL、过滤器、ID |
| 云台无输出 | `Board_HeartBeat.status`、D1 `car_state`、DM 在线状态 |
| D5 超时 | 下板遥控在线、`BOARD_COMM_D5_ENABLE`、D5 发送函数 |
| 云台抖动 | 机械中值、PID 限幅、IMU 零偏、CAN 周期、重力补偿 |
| Pitch 下垂 | `gravity_sign`、`gravity_k_nm`、机械中值 |
| Yaw 松杆漂移 | 保持环 Kp/Ki、D5 死区、机械回差 |
| 归中后不动 | `init_flag`、D2 目标量程、`car_state`、DM 在线状态 |
| 单发不动作 | D1 `shoot_level` 上升沿、拨盘在线、KT 使能状态 |
| 连发不稳 | 连发速度环、电流限幅、拨盘阻力 |
| 摩擦轮方向相反 | `LAUNCHER_FRIC_L_DIRECTION`、`LAUNCHER_FRIC_R_DIRECTION` |
| 发射许可一直为 0 | 下板 S2 是否解锁、是否打符、升降是否到上位 |
| 升降不进上位 | 找顶电流、低速阈值、静止窗口、编码器方向 |
| 升降 fault 不恢复 | 查看 `fault_code`，切换狗洞命令边沿 |
| 编译重复符号 | 检查 TaskLayer 与 `freertos.c` 是否有同名强定义 |

## 12. 目录结构

```text
task_up/
  Application/
    AlgorithmLayer/   PID、数学、滤波、IMU 算法依赖
    ConfigLayer/      电机、通信、发射、升降参数
    DeviceLayer/      IMU、电机对象、LED、传感器抽象
    DriverLayer/      CAN、DWT 等底层驱动
    HardwareLayer/    DM、RM、KT 电机协议实现
    ModuleLayer/      云台、发射、升降状态机
    ProtocolLayer/    板间协议、CAN 分发
    TaskLayer/        FreeRTOS 周期任务
  Core/               CubeMX 生成的内核和中断
  Drivers/            STM32F4 HAL/CMSIS
  Middlewares/        FreeRTOS、USB Device
  USB_DEVICE/         USB CDC 应用
  MDK-ARM/            Keil 工程与产物
```

## 13. 验收清单

- `My_C` 全量 rebuild 为 0 errors。
- 上下板 CAN 波特率经仪器确认一致。
- D1/D2 在 100 ms 内无持续离线。
- D5 掉线时上板手动速度为 0。
- 板间离线 100 ms 内云台卸力。
- 云台机械中值和 Pitch 限位已实测。
- 云台归中、机械模式和速控模式均完成台架验证。
- 单发每次只转一圈，连发松触发后能制动停机。
- 摩擦轮实际转向与方向宏一致。
- 升降找顶、上位、下位、回程均实测。
- 升降超程和堵转能进入 fault 且不会自动反复重启。
- 升降正式行程从 280 圈更新为实测值。
- 协议、CAN ID 或机械方向改动同步检查 `task_down`。
