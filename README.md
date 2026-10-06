# RoboMaster 上下板控制工程

基于 STM32、HAL 和 FreeRTOS 的上下板控制程序。下板负责遥控器/键鼠输入、四轮底盘和控制指令生成；上板负责云台、IMU、摩擦轮、拨盘及狗洞升降机构。两板通过经典 CAN 交换控制量和反馈。

本文基础说明按 `405f7d8`（2026-10-03）整理，热量控制与 D3 说明更新于 2026-10-06。默认配置用于整车联调：跟随、小陀螺、发射、升降和裁判热量链路已接入；UI 和底盘功率限制未启用。热量改动尚未编译或实机验证。

## 目录

- [快速开始](#快速开始)
- [工程结构与源码入口](#工程结构与源码入口)
- [硬件与总线](#硬件与总线)
- [启动与任务调度](#启动与任务调度)
- [遥控器与键鼠操作](#遥控器与键鼠操作)
- [控制模块](#控制模块)
- [板间 CAN 协议](#板间-can-协议)
- [配置与调参](#配置与调参)
- [调试与故障排查](#调试与故障排查)
- [验证与维护](#验证与维护)
- [开发记录与当前边界](#开发记录与当前边界)

## 快速开始

### 1. 准备构建环境

| 项目 | 上板 | 下板 |
| --- | --- | --- |
| MCU | STM32F407IGHx，Cortex-M4 | STM32H723VGTx，Cortex-M7 |
| Keil 工程 | `task_up/MDK-ARM/My_C.uvprojx` | `task_down/MDK-ARM/DM-MC02.uvprojx` |
| Target | `My_C` | `DM-MC02` |
| 工程记录的编译器 | ARM Compiler 5.06 update 7，build 960 | ARM Compiler 5.06 update 7，build 960 |
| 工程记录的设备包 | `Keil.STM32F4xx_DFP.2.17.1` | `Keil.STM32H7xx_DFP.4.1.3` |
| 构建输出目录 | `task_up/MDK-ARM/My_C/` | `task_down/MDK-ARM/DM-MC02/` |
| 主要产物 | `My_C.axf`、`My_C.hex` | `DM-MC02.axf`、`DM-MC02.hex` |

前置依赖：

- Windows、Keil MDK / µVision，以及可用的 ARM Compiler 5 授权。
- 安装表中设备包；若换用其他版本，重新检查启动文件、链接内存布局和 HAL 兼容性。
- HAL、CMSIS、FreeRTOS 等源码已随工程保存，不需要安装 npm 或 Python 依赖。
- CubeMX 用于修改 `.ioc` 和重新生成外设代码，日常 Keil 构建无需重新生成。
- 下载需要与目标板匹配的 SWD 调试器、驱动、Flash Algorithm 和供电。

工程当前没有 CMake、GCC 或跨平台构建入口。不要仅把编译器切换为 ARM Compiler 6 后假定行为一致。

### 2. 编译上下板

在仓库根目录打开 PowerShell。下面命令会寻找常见 Keil 安装位置，并将日志写入系统临时目录；安装在其他位置时，直接给 `$uv4Path` 赋实际路径。

```powershell
$uv4Path = (Get-Command UV4.exe -ErrorAction SilentlyContinue).Source
if (-not $uv4Path) {
    $uv4Path = @('C:\Keil_v5\UV4\UV4.exe', 'D:\Keil_v5\UV4\UV4.exe') |
        Where-Object { Test-Path -LiteralPath $_ } |
        Select-Object -First 1
}
if (-not $uv4Path) { throw '未找到 UV4.exe，请将 $uv4Path 设置为实际安装路径' }

$upLog = Join-Path $env:TEMP 'Train_code_plus_up_build.log'
$downLog = Join-Path $env:TEMP 'Train_code_plus_down_build.log'
Start-Process -FilePath $uv4Path -ArgumentList @(
    '-b', 'task_up\MDK-ARM\My_C.uvprojx', '-t', 'My_C', '-o', $upLog
) -Wait -WindowStyle Hidden
Start-Process -FilePath $uv4Path -ArgumentList @(
    '-b', 'task_down\MDK-ARM\DM-MC02.uvprojx', '-t', 'DM-MC02', '-o', $downLog
) -Wait -WindowStyle Hidden
Get-Content -LiteralPath $upLog
Get-Content -LiteralPath $downLog
```

- 两个工程均需确认最终摘要为 `0 Error(s)`；同时检查 warning 和链接内存占用。
- `-b` 为增量构建，完整重建可将其替换为 `-r`。
- GUI：打开对应 `.uvprojx` → 选择表中 Target → Build / Rebuild。
- `.axf` 用于源码调试，`.hex` 用于下载；旧产物存在不代表本次构建成功。
- 本次 README 更新未执行编译，以上是可复用构建步骤，不是本次编译通过声明。

### 3. 下载与首次运行

1. 将两板 CAN 收发器侧的 CAN_H、CAN_L 和参考地连接，核对终端电阻及供电。
2. 遥控接收机接下板 UART5 接口；上板默认从下板获取手动输入。
3. 在 Keil 的 Target Options → Debug / Utilities 中选择实际调试器与对应芯片下载算法，分别下载上下板固件。
4. 架空底盘，清空发射通道，给云台和升降机构留出完整运动空间。初次调试使用 S1 中位、S2 中位，避免先进入键鼠模式。
5. 查看下板 `rc_dev`、`board.status` 和四轮在线状态；查看上板 `Board_HeartBeat`、`imu_dbg` 和电机状态。
6. 云台安全条件满足后会归中；升降在通信、整车使能和电机在线条件满足后，可在上电延时约 2 s 后自动找顶。S1 中位不等于上板机构全部卸力。
7. 先验证机械模式和单轴方向，再验证跟随、小陀螺、升降，最后验证发射。

## 工程结构与源码入口

```text
Train_code_plus/
├── task_up/                 上板主工程
│   ├── Application/         业务、算法、设备与协议
│   ├── Core/                CubeMX 外设初始化、启动入口和 RTOS 配置
│   ├── Drivers/             STM32F4 HAL、CMSIS
│   ├── Middlewares/         FreeRTOS、USB Device
│   ├── USB_DEVICE/          USB 设备接口
│   ├── MDK-ARM/             Keil 工程及生成产物
│   └── My_C.ioc
├── task_down/               下板主工程
│   ├── Application/
│   ├── Core/
│   ├── Drivers/             STM32H7 HAL、CMSIS
│   ├── Middlewares/         FreeRTOS
│   ├── MDK-ARM/
│   └── DM-MC02.ioc
├── 01_LED/                  独立外设参考工程
└── AGENTS.md                仓库协作与验证约定
```

`01_LED` 不参与当前上下板主程序。两板各自维护 Application，不应把其中一板的同名头文件直接替换到另一板。

| 分层 | 职责 | 阅读入口 |
| --- | --- | --- |
| ConfigLayer | 功能宏、控制参数、设备/用户配置 | 上板 `board_remote_config.h`、`gimbal_init_config.h`、`launcher_config.h`、`lift_config.h`；下板 `chassis_config.h`、`board_comm_config.h` |
| DriverLayer | CAN/FDCAN、UART DMA、SPI、时基等封装 | 两板 `driver.c`、`drv_can.c`、`drv_uart.c` |
| HardwareLayer | DM、RM、KT 等电机底层协议 | 上板 `DM_Motor.c`、`RM_motor.c`、`KT_motor.c`；下板 `RM_motor.c` |
| DeviceLayer | 电机对象、IMU、遥控器、超电等设备 | 两板 `device.c`、`motor.c`、`Sensor/rc_sensor.c` |
| AlgorithmLayer | PID、滤波、数学工具、CRC、功率算法 | 两板 `PID.c`、`rp_math.c`；上板 IMU 解算另见 DeviceLayer/Imu |
| ModuleLayer | 云台、发射、升降与底盘控制 | 上板 `gimbal.c`、`launcher.c`、`lift.c`；下板 `chassis_input.c`、`chassis_control.c`、`chassis_follow.c`、`chassis_spin.c`、`launch.c` |
| ProtocolLayer | 报文编码、解码和接收分发 | 上板 `communicate.c`；下板 `board_protocol.c`；两板 `can_protocol.c` |
| TaskLayer | 业务周期与监控调度 | 两板 `control_task.c`、`monitor_task.c`；下板 `Command_Task.c`、`connect_task.c` |

下板还保留 ControlLayer、ServiceLayer、SolvingLayer、ParameterLayer 等旧整车组织方式。当前默认运行路径以实际初始化分支和任务调用为准；UserLayer 中的旧发射入口不代表当前控制链。

## 硬件与总线

### 总线拓扑

```text
遥控接收机 ── UART5 ── 下板 STM32H723
                         ├─ FDCAN1 ── 四轮电机、超电
                         └─ FDCAN2 ─────────────┐
                                              │ 板间 CAN
                         上板 STM32F407 ── CAN2 ┘ + Yaw DM 电机
                              └─ CAN1 ── Pitch DM、双摩擦轮、KT 拨盘、RM2006 升降
```

两板均使用标准 ID、8 字节经典 CAN 数据帧。下板采用 FDCAN 外设，但 `FrameFormat` 配置为 `FDCAN_FRAME_CLASSIC`，不是 CAN-FD 数据帧。

| 板卡 / 外设 | RX / TX 芯片引脚 | 用途 |
| --- | --- | --- |
| 上板 CAN1 | PD0 / PD1 | Pitch、摩擦轮、拨盘、升降 |
| 上板 CAN2 | PB5 / PB6 | Yaw、板间通信 |
| 下板 FDCAN1 | PD0 / PD1 | 四轮、超电 |
| 下板 FDCAN2 | PB5 / PB6 | 板间通信 |
| 下板 FDCAN3 | PD12 / PD13 | 外设已初始化，接收分发暂无业务 |
| 下板 UART5 | RX PD2；TX PC12 已配置引脚 | 遥控接收，当前串口设为 RX 模式 |

表中为 MCU 引脚，板上插座位置需对照实物原理图。芯片 CAN_RX / CAN_TX 经收发器转换后才是 CAN_H / CAN_L。

位时序按当前源码推导：

- 上板：HSE 12 MHz → SYSCLK 168 MHz → APB1 42 MHz；`42 MHz / [3 × (1 + 10 + 3)] = 1 Mbit/s`。
- 下板：HSE 24 MHz，PLL2Q 为 100 MHz；`100 MHz / [5 × (1 + 14 + 5)] = 1 Mbit/s`。
- UART5：100000 baud、8 个有效数据位、偶校验、2 停止位，18 字节遥控帧。HAL 配置为 `UART_WORDLENGTH_9B`，其中包含校验位。

### 电机与设备 ID

| 所属总线 | 设备 | 发送 / 接收 ID |
| --- | --- | --- |
| 上板 CAN1 | Pitch DM | 控制 `0x01`，反馈 `0x11` |
| 上板 CAN2 | Yaw DM | 控制 `0x02`，反馈 `0x12` |
| 上板 CAN1 | 左 / 右摩擦轮，RM 3508 单电机配置 | 组控 `0x200` 第 1 / 2 路，反馈 `0x201` / `0x202` |
| 上板 CAN1 | 升降 RM2006 | 组控 `0x200` 第 4 路，反馈 `0x204` |
| 上板 CAN1 | 拨盘 KT4005 | `0x141`，命令和反馈共用 ID |
| 下板 FDCAN1 | 左前 / 左后 / 右前 / 右后轮 | 反馈依次为 `0x201` / `0x202` / `0x203` / `0x204` |
| 下板 FDCAN1 | 超电调试协议 | 下发 `0x222`，反馈 `0x211` |

电机绑定以 DeviceLayer 的 `motor.c`、`motor.h` 和 ProtocolLayer 的 `can_protocol.c` 为准；部分旧 ConfigLayer 枚举注释保留历史设备布局。

## 启动与任务调度

### 上板

`Core/Src/main.c`：外设初始化 → USB Device → `DEVICE_Init()` → `Module_Init()` → `DRIVER_Init()` → 创建 RTOS 任务。

- `DEVICE_Init()`：IMU、遥控对象、RM/KT/DM 电机和发射模块。
- `Module_Init()`：`Gimbal_Init()`、`Lift_Init()`。
- `DRIVER_Init()`：驱动及接收链；CAN 过滤器由此初始化。
- IMU 默认选择 EKF：`IMU_USE_EKF=1`、`IMU_USE_MAHONY=0`。

| 任务 | 当前状态 / 周期 | 实际工作 |
| --- | --- | --- |
| ControlTask | 启用，目标 1 ms，`osDelayUntil` | IMU 更新 → `imu_dbg` 快照 → 云台 → 升降 → DM 输出 → 发射/RM 输出 → 反馈发送调度 |
| MonitorTask | 启用，循环后 `osDelay(1)` | IMU、电机、遥控和板间心跳 |
| LedTask | 启用 | LED 状态提示 |
| CommunityTask | 关闭 | 原任务无业务，通信已并入 ControlTask |

上板采用 CMSIS-RTOS v1 接口。云台 DM 输出在板间离线或 `car_state=0` 时卸力。

### 下板

`Core/Src/main.c`：外设初始化 → `DEVICE_Init()` → `DRIVER_Init()` → 创建 RTOS 任务。

默认 `BOARD_COMM_DEBUG=1`，设备初始化走遥控、板间通信、四轮、发射指令、跟随、小陀螺和底盘输入/控制分支。旧 `infantry.work()` 分支未运行。

| 任务 | 当前状态 / 周期 | 实际工作 |
| --- | --- | --- |
| CommandTask | 启用，循环后 `osDelay(1)` | 遥控帧解析、键鼠按键状态更新 |
| CtrlTask | 启用，循环后 `osDelay(1)` | 云台/狗洞指令 → 底盘输入 → 跟随 → 小陀螺 → 四轮控制 → 发射指令 → 超电保活 |
| ConnectTask | 启用，默认间隔 1 ms | 到期D3先尝试；容量足够再发整组D1/D2/D5；D4默认关闭 |
| MonitorTask | 启用，循环后 `osDelay(1)` | 四轮、遥控、板间和超电心跳 |
| UpdataTask | 默认不创建 | 由 `!BOARD_COMM_DEBUG` 分支控制 |
| UITask | 默认不创建 | `BOARD_UI_ENABLE=0` |

下板采用 CMSIS-RTOS v2 创建任务。`osDelay(1)` 是循环末尾延时，实际周期含执行耗时，不能据此承诺每次严格 1 ms。两板独立看门狗初始化当前均被注释。

## 遥控器与键鼠操作

S1、S2 指源码解析的拨杆名称；上/中/下对应 `RC_SW_UP` / `RC_SW_MID` / `RC_SW_DOWN`。操作手应先通过 `rc_dev.info` 核对实物拨杆与通道方向。

### 遥控模式

| S1 | S2 | 底盘 / 云台 | 发射行为 |
| --- | --- | --- | --- |
| 上 | 上 | 底盘跟随；云台速控自稳 | 解锁后连发 |
| 上 | 中 | 底盘跟随；云台速控自稳；B 可切换狗洞 | 解锁后待发，升降互锁另行限制 |
| 上 | 下 | 小陀螺；云台速控自稳 | 锁定 |
| 中 | 上 | 底盘无有效直控输入；云台仍走非机械模式 | 解锁后单发 |
| 中 | 中 | 底盘无有效直控输入 | 解锁后待发 |
| 中 | 下 | 底盘无有效直控输入 | 锁定 |
| 下 | 任意 | 底盘直接平移/转向；云台机械模式 | 锁定 |

遥控发射还有前置条件：遥控在线、升降上报 `2`、无狗洞请求。上电/掉线或互锁复位后，必须先改变一次 S2 档位才能解锁；S2 新档位需连续稳定约 15 个控制周期。S2 中位待发会给出摩擦轮许可。

| 输入 | 当前用途 |
| --- | --- |
| ch3 | 底盘纵向输入，源码带负号 |
| ch2 | 底盘横向输入 |
| ch0 | 直接模式底盘转向；跟随/小陀螺时生成云台 Yaw 角速度，小陀螺模块另处理旋转 |
| ch1 | 云台 Pitch；机械模式积分机械目标，非机械模式通过 D5 发送角速度 |
| R | 遥控模式切换前/后机械零位预置，S1 任意位置可记录，机械档使用该预置 |
| B | 仅 S1 上、S2 中时以按下边沿切换狗洞请求 |

S1 中位只使当前底盘输入无效；调试指令仍会在遥控在线时置 `car_state=1`，不能将中位当作整车断力开关。

### 键鼠模式

前提：遥控在线，S1 上位，按 F 切换输入源。初始键鼠底盘模式为跟随。S1 离开上位时键鼠暂不生效，遥控掉线会清除启用标志；R 掉头基准在退出有效键鼠模式时复位。

| 按键 / 鼠标 | 功能 |
| --- | --- |
| F | S1 上位时切换遥控/键鼠输入 |
| W / S | 前进 / 后退 |
| A / D | 左移 / 右移 |
| Shift | 平移及普通键盘旋转倍率 1.5 |
| Ctrl | 速度倍率 0.5；与 Shift 同按时 Shift 优先 |
| Z | 跟随档：鼠标控制云台，底盘自动跟转 |
| X | 机械档：Yaw 锁在前/后机械零位，鼠标 X 直接转底盘 |
| C | 小陀螺档，允许平移并按云台方向转换 |
| Q / E | 输入层生成正/反旋转；跟随和小陀螺会按各自模块覆盖最终旋转量 |
| 鼠标 Y | 云台 Pitch 控制 |
| 鼠标左键 | 按下触发单发，按键状态进入长按后切连发；松开取消触发 |
| R，X 档 | 翻转前/后机械基准；后向基准时 WASD 平移整体反向 |
| R，Z 档 | 云台世界角掉头约 180°，动作期间冻结底盘自动跟转 |
| R，C 档 | 当前没有对应掉头动作 |
| B | 仍需 S1 上、S2 中，沿用狗洞请求逻辑 |

键鼠 D5 当前发送角速度：Yaw 为 `mouse_x × 5`，Pitch 为 `mouse_y × -3`，分别限幅至 ±200、±150 deg/s。X 档鼠标 X 的底盘转向增益为 `0.0873`，该项限幅 ±3.5，Q/E 项另行叠加。

**进入键鼠模式即给出摩擦轮许可，不需要按左键才启动摩擦轮。** 当前下板键鼠分支在遥控升降/狗洞互锁检查之前返回，不能假定键鼠发射同样受这些互锁保护；上板发射使能条件也未补上该限制。

## 控制模块

### 云台

实际模式选择顺序：安全条件失败 → `G_SLEEP`；未初始化 → `G_INIT`；D1 `gimbal_mode=0` → `G_MEC`；其余 → `G_RATE`。

- `G_INIT`：独立归中参数和 PID，目标机械角 Yaw/Pitch 均为 0°；连续满足位置、速度等判据 30 ms 后完成。
- `G_MEC`：Yaw 使用机械位置外环和 IMU 速度内环，带速度规划、前馈处理及力矩约束；键鼠机械档的 Pitch 沿用手动速控/保持路径。
- `G_RATE`：D5 输入角速度，起停斜坡和松杆保持环；跟随、小陀螺默认使用这一模式。
- `G_GYRO`、`G_AUTO` 有分支代码，但当前仲裁器不直接选择；不能据此声称自瞄已接通。
- Pitch 机械范围 -7.5°～30°，两轴默认力矩上限 6 N·m；Pitch 重力补偿默认启用。
- 当前机械中点：Yaw -22.224138°、Pitch 148.573157°，属于本机构标定值。

当前 `gimbal_safety_allows_control()` 仅检查板间在线和 `car_state`，没有将 IMU 校准完成、IMU 错误或两轴在线显式纳入这个使能门槛。首次运行需要主动观察这些状态，不能把“云台开始控制”当作传感器已就绪的证明。

归中超时默认 6000 ms；源码超时后仍置 `init_flag=1`，不是进入故障停机。验收归中时需同时检查实际角度，不能只看完成标志。

### 底盘

当前路径：`Chassis_Input_Update()` → `Chassis_Follow_Update()` / `Chassis_Spin_Update()` → `Chassis_Control_Update()`。

- 四轮逆解：LF=`-vx+vy+wz`，LB=`-vx-vy+wz`，RF=`vx+vy+wz`，RB=`vx-vy+wz`。
- 轮速反馈由 RM 驱动将 rpm 转换为 rad/s，并按电机类型处理减速比；`CHASSIS_MAX_VX/VY/WZ` 直接参与轮速合成，未通过轮径和底盘尺寸形成完整的机体速度换算，不能将其中 vx/vy 当作 m/s。
- 默认四轮速度环为纯 P：`kp=0.8`、`ki=kd=0`；低目标、低反馈区域清 PID 并停止输出。
- 四轮在线检查、异常输入检查和限矩由 `chassis_control.c` 处理。
- 基础限矩 2 N·m，跟随和小陀螺各为 4 N·m；功率限制和规划器默认关闭。

跟随使用 C2 的机械 Yaw 相对角，平移按实际云台方向转换，旋转由角度误差和指令前馈生成。默认死区 0.5°、反馈超时 50 ms、单次角度跳变阈值 30°，并有大误差方向锁定和输出斜坡。锁存故障后需退出相关档位，核对反馈再重入。

键鼠 Z 档 R 掉头通过 D5 下发角速度，C2 的 IMU Yaw 判断转角；接近 180°或到 2500 ms 超时后减速。动作结束取实际机械相对角作为新跟随中心，减少底盘接手时的突转。反馈过期或切走跟随档会取消动作。

### 发射机构

下板 `launch.c` 生成许可、模式和触发电平；上板 `launcher.c` 执行摩擦轮升降速及拨盘状态机。

- 主要状态：SLEEP → SPINUP → READY → SINGLE / REPEAT；关闭许可后走 STOPPING。
- INIT、REVERSE、RELOAD、FAULT 等状态也已定义；默认自动拨盘定位和堵转退让关闭。
- 摩擦轮目标 1500 rpm，升/降速斜坡 20 rpm/ms，达速误差容许 500 rpm，连续确认 100 ms。
- 拨盘单发目标增量 65536 count，一圈供一发；连发独立速度环由热量预算限频，最高 5400 deg/s（15 发/s）。
- 单发按触发边沿处理；连发使用持续触发电平。默认连发、拨盘控制、待发位置保持均开启。
- 上板发射许可依赖板间在线、下板许可和摩擦轮在线；拨盘在线后才参与供弹。
- 关闭许可时摩擦轮按斜坡降速，拨盘主动制动；不能把关闭许可描述为所有输出瞬时归零。

#### 发射热量预算

- `launcher_heat` 可直接在 Keil Watch 查看；`heat` / `referee_heat` / `remaining` 为本地估计、裁判值、剩余热量，`target_rate` 单位为发/s。
- `source`：0 无初始参数、1 裁判有效、2 断链估算、3 固定参数训练；`ready` 表示初始热量已建立，`blocked` 表示热停发。
- 单发先计入 10 热量单位，取消、超时或空拨不主动退还；连发按正向供弹位移累计，制动尾段继续计热，反向与回到已计热位置不重复累计。
- 每周期按实际时间冷却，停机仍冷却；新的有效裁判热量直接覆盖本地估计，允许上调或下调。重复源序号不覆盖；源热量或 D3 失效后，恢复时首份有效快照重新校准。
- 余量 ≥200 时最高 15 发/s；200→50 线性降到 `cooling_rate/10`；50→20 保持冷却平衡射频；低于 20 停拨盘，恢复至 30 才解除。平衡射频限制在 0～15 发/s。
- `LAUNCHER_HEAT_MARGIN` 默认 20 热量单位，连发停发阈值引用此余量；单发预占后也必须保留此余量，因此默认余量至少 30 才能获准。`remaining` 仍表示上限减估算值，未扣除安全余量。
- 单发被拒绝后必须松开重按；连发保持按下时冷却后自动继续。已预占的单发可完成，但裁判校准或限额变化使估计超过上限时中止。
- 默认需首次取得有效参数与热量快照才允许供弹；随后裁判失效仍沿用最后有效参数估算，上下板失联仍走原停机逻辑。
- 无裁判训练：在上板 `launcher_config.h` 设置 `LAUNCHER_HEAT_TRAINING_ENABLE=1u`，填写 `LAUNCHER_HEAT_TRAINING_LIMIT`（须大于 30）与 `LAUNCHER_HEAT_TRAINING_COOLING`（须大于 0）。默认均为 0，禁止供弹；训练模式使用固定参数，不使用裁判校准。
- `launcher_heat.fric_l/fric_r` 提供电流 A、转速 rpm、反馈时刻 ms、序号、距今时间和在线状态；尚未用电流识别出弹。
- 参数集中在 `launcher_config.h`；阈值排序必须满足 `WARN > SATURATE >= RESUME > STOP >= PER_SHOT > 0`。

实弹测试需确认一圈一发、反馈连续性和制动余弹；20 热量单位的停发余量不代表已验证不会超热量。

### 狗洞升降

上板 `lift.c` 控制 RM2006，下板通过 D1 的 `is_hole` 请求动作。

1. 等待电机、板间通信和整车使能。
2. 无有效零点时，上电延时约 2 s 或收到请求后找顶。
3. 顶部判定结合电流、速度和位置变化窗口，随后向下回退 5 个电机圈。
4. 进入上位待机；收到过洞请求后先对齐云台，再下压。
5. 取消请求后上升；超时、过流或行程异常按状态机停止/报错。

- 当前配置行程为 280 个电机圈，源码仍标为临时测试值；不能未经标定直接换算成机构毫米行程。
- 下板拒绝后向基准时的入洞请求；过洞期间强制云台前向零位，并把键鼠跟随基准复位为前方。
- 上板另检查前向角度、对齐误差和角速度，避免只凭“达到后向目标”就允许下压。
- 退出请求后下板继续保持云台对齐，直到上报 `2` 或退出等待达到 5000 ms；该超时不代表机构一定已回上位。
- 找顶和运动超时当前均为 90000 ms，调整时结合机械限位与电流实测。

| C1 byte1 | 当前升降上报含义 |
| --- | --- |
| 0 | 下位就绪，或 `LIFT_STALL_STOP` |
| 1 | 找顶、回退、对齐、上/下移动等过程 |
| 2 | 上位就绪，或 `LIFT_WAIT` |
| 3 | `LIFT_FAULT` |

`2` 不等于已找零，需结合 `lift.home_valid`、`lift.state` 和实际位置确认。

### 超电与保留模块

- `SUPERCAP_BRINGUP_ENABLE=1`：下板发送保活并解析反馈，离线超时 100 ms。
- `SUPERCAP_CAP_SWITCH=0`，turbo、预充及功率字段均为 0，当前不允许超电参与功率输出。
- 新 `supercap.c` 调试链与旧 `cap.c` / `cap_protocol.c` 路径不同，不应混用配置。
- 裁判系统、UI、视觉、旧整车决策与功率算法存在源码，但不构成当前默认启用功能。

## 板间 CAN 协议

源码依据：下板 `Application/ProtocolLayer/board_protocol.c` 与上板 `Application/ProtocolLayer/communicate.c`。板间接口为下板 FDCAN2 ↔ 上板 CAN2。

### 编码约定与发送周期

- 以下 byte 下标从 0 开始；16 位字段高字节在前。
- 浮点量程压缩为无符号 16 位：`raw = (value-min)/(max-min) × 65535`，解码做反向映射；不是 IEEE 754 浮点直接传输。
- 机械角使用 rad；IMU 角使用 deg；D5 角速度使用有符号 int16，量化 0.1 deg/s。
- D1/D2/D5 保持 1 ms 任务周期；下板先检查是否有整组发送空间，容量不足时延后，清除本组成功标志，避免半组入队及错误计入掉头交接。
- C1/C2 各以最近成功时刻计每 5 ms 到期，交错发送，每次上板 ControlTask 最多一帧；失败不推进周期，避免1 kHz重复姿态反馈占满总线。
- D3 默认启用，每次成功入队后等待 10 ms；到期后在控制组前尝试，失败下个任务周期重试，不忙等。D4独立开关，默认关闭。
- 入队成功不等于上板已收到；最终以`Board_Rx_Info.heat_pkt.rx_tick`的新鲜度和D1/D2心跳判断通信。

| ID | 方向 | 内容 | 默认发送 |
| --- | --- | --- | --- |
| `0xD1` | 下 → 上 | 整车状态、发射控制、过洞请求 | 是 |
| `0xD2` | 下 → 上 | 四个云台目标角 | 是 |
| `0xD3` | 下 → 上 | 热量上限、当前热量、冷却速率、源序号及有效标志 | 是，每 10 ms |
| `0xD4` | 下 → 上 | 8 字节血量字段透传 | 否，上板仅更新心跳 |
| `0xD5` | 下 → 上 | 遥控/键鼠控制量 | 是 |
| `0xC1` | 上 → 下 | 电机在线、升降状态、视觉保留字段 | 是 |
| `0xC2` | 上 → 下 | 云台机械角和 IMU 角 | 是 |

### D1：整车与发射状态

| 字节 | 编码 |
| --- | --- |
| 0 | bit0～1：`car_state`；bit2：`gimbal_mode`；bit3～5：`vision_mode`；bit6：`game_start`；bit7：`my_color` |
| 1～2 | `v_x`，uint16 量程 [-8000, 8000] |
| 3～4 | `v_y`，uint16 量程 [-8000, 8000] |
| 5 | bit0：发射许可；bit1：0 单发 / 1 连发；bit2：触发电平；bit3：过洞请求 |
| 6～7 | 0，保留 |

上板当前只解析 byte0 和 byte5；D1 速度字段没有在该接收函数中用于控制。

### D2：目标角

| 字节 | 目标 | 量程 / 单位 |
| --- | --- | --- |
| 0～1 | Pitch IMU | [-360, 360] deg |
| 2～3 | Yaw IMU | [-360, 360] deg |
| 4～5 | Pitch 机械 | [-4, 4] rad |
| 6～7 | Yaw 机械 | [-4, 4] rad |

D2 字段会解码缓存，但是否参与控制取决于上板模式；默认非机械档选择 G_RATE，不直接跟踪 D2 的 IMU 角目标。

### D3 / D4：裁判热量与血量链路

| D3 字节 | 内容 | 编码 |
| --- | --- | --- |
| 0～1 | 热量上限 | uint16，热量单位，高字节在前 |
| 2～3 | 17 mm 第一枪管当前热量 | uint16，热量单位，高字节在前 |
| 4～5 | 冷却速率 | uint16，热量单位/s，高字节在前 |
| 6 | 热量源更新序号 | uint8，0～255 循环 |
| 7 | 源有效性 | bit0 参数有效；bit1 热量有效；其余为 0 |

下板分别记录裁判 `0x0201` 参数与 `0x0202` 热量源时刻，默认超时为 1500 / 300 ms；旧缓存继续发送时会清除对应有效位。上板 D3 默认 100 ms 超时，重复源序号不重复校准，源热量失效或 D3 超时后重新接收时重建序号基准。

D3 布局已替换旧弹速／射频格式，必须同步更新上下板固件。D4 原样发送 `blood_pkt.blood[0..7]`，默认关闭，上板仍只清心跳。

### D5：手动控制

| 字节 | 编码 |
| --- | --- |
| 0 | bit0：valid；bit1：0 遥控 / 1 键鼠；bit2：0 角速度 / 1 鼠标增量 |
| 1 | bit0：鼠标左键；bit1：鼠标右键 |
| 2～3 | int16 Yaw 控制量；角速度格式乘 0.1 得 deg/s |
| 4～5 | int16 Pitch 控制量；角速度格式乘 0.1 得 deg/s |
| 6～7 | 0，保留 |

当前下板对遥控和键鼠都发送 bit2=0。上板保留 bit1=1、bit2=1 的鼠标增量解析，不能只改发送端量纲而不检查上板控制路径。

### C1：设备与升降状态

| 字节 | 编码 |
| --- | --- |
| 0 | bit0：Yaw 在线；bit1：Pitch 在线；bit2：升降在线；bit3：右摩擦轮在线；bit4：左摩擦轮在线；bit5：拨盘在线；bit6：视觉在线 |
| 1 | 升降报告值，见升降状态表 |
| 2～3 | 视觉 Yaw 保留目标，当前编码为 0° |
| 4～5 | 视觉 Pitch 保留目标，当前编码为 0° |
| 6 | bit0：找到视觉目标，当前为 0 |
| 7 | 0，保留 |

下板字段名 `state_meg.is_down` 实际保存完整 byte1 升降报告值，不能按布尔“是否下降”理解。

### C2：云台反馈

| 字节 | 反馈 | 量程 / 单位 |
| --- | --- | --- |
| 0～1 | Yaw 机械角 | [-4, 4] rad |
| 2～3 | Pitch 机械角 | [-4, 4] rad |
| 4～5 | Yaw IMU 角 | [-360, 360] deg |
| 6～7 | Pitch IMU 角 | [-360, 360] deg |

机械角编码步长约 0.000122 rad（0.007°），IMU 角步长约 0.011°。下板仅在收到 C2 时刷新 `gimbal_rx_time_ms`，用于跟随、小陀螺与掉头的反馈新鲜度判断。

### 心跳与兼容性

- 上板对 D1/D2 分别计数，任意一个达到 100 个 MonitorTask 周期即判板间离线；D3 热量有效性独立判断，D4 默认关闭不影响综合在线判定。
- D5 单独计数，手动输入还需检查有效位和控制链中的超时条件。
- 下板收到 C1 或 C2 都清综合在线计数，阈值 50 个 MonitorTask 周期；C1 正常不能替代 C2 姿态新鲜度。
- 协议没有版本协商。修改 ID、位布局、量程、字节序或单位时，必须同步修改并验证上下板，成对下载固件。

## 配置与调参

所有路径相对各自板卡目录，表中为本文对应源码的默认值。

| 板卡 / 文件 | 关键项 | 默认值 / 作用 |
| --- | --- | --- |
| 上板 `ConfigLayer/board_remote_config.h` | `GIMBAL_DOWN_RC_ENABLE` / `GIMBAL_LOCAL_RC_ENABLE` | 1 / 0；下板输入启用，本地输入关闭 |
| 上板同上 | `BOARD_FEEDBACK_PERIOD_MS` | 5；C1/C2各自周期，单轮最多一帧 |
| 上板 `ConfigLayer/rp_config.h` | `IMU_USE_EKF` / `IMU_USE_MAHONY` | 1 / 0 |
| 上板 `ConfigLayer/gimbal_init_config.h` | HOME、TIMEOUT、STABLE | 0° / 0°；6000 ms；30 ms |
| 上板 `ConfigLayer/gimbal_turn_config.h` | `GIMBAL_MEC_YAW_USE_TURN_PATH` | 0；旧独立转向路径未启用 |
| 上板 `ConfigLayer/launcher_config.h` | DIAL / REPEAT / AUTO_RESET / JAM | 1 / 1 / 0 / 0 |
| 上板 `ConfigLayer/lift_config.h` | TRAVEL / AUTO_HOME_DELAY | 280 电机圈；2000 ms |
| 下板 `ConfigLayer/board_comm_config.h` | DEBUG / TX / D5 | 均为 1 |
| 下板同上 | D3 / D4 / CAP / UI | 1 / 0 / 0 / 0；D3 每 10 ms |
| 下板同上 | `BOARD_JUDGE_ENABLE` | 遗留值 0；当前 USART1、裁判对象和心跳无条件初始化，不能用此宏判断裁判接收 |
| 上板 `ConfigLayer/launcher_config.h` | HEAT_TRAINING_ENABLE / LIMIT / COOLING | 均为 0；无初始裁判快照时禁止供弹 |
| 上板同上 | HEAT_MARGIN / HEAT_RESUME | 20 / 30 热量单位；恢复阈值须大于余量 |
| 下板同上 | `BOARD_LIFT_ENABLE` | 1；参与下板过洞/发射指令互锁 |
| 下板 `ConfigLayer/chassis_config.h` | BRINGUP / RC / KEYBOARD / FOLLOW / SPIN | 均为 1 |
| 下板同上 | PLANNER / FEEDFORWARD / POWER_LIMIT | 均为 0 |
| 下板同上 | MAX_VX / MAX_VY / MAX_WZ | 25 / 25 / 20，现有控制域 |
| 下板同上 | FOLLOW_KP / FOLLOW_MAX_WZ / FOLLOW_TIMEOUT | 20 / 40 / 50 ms |
| 下板同上 | SPIN_BASE_WZ / SPIN_MAX_WZ | 25 / 25 |
| 下板 `ConfigLayer/supercap_config.h` | BRINGUP / CAP_SWITCH | 1 / 0 |

部分云台机械中点、手动输入和保持环参数目前仍在 `ModuleLayer/gimbal.h`，不是全部已集中到 ConfigLayer。新增参数优先放对应配置头文件，修改现有值前确认控制代码实际读取位置。

调参顺序：

1. 标定安装中点、机械限位、电机/IMU方向和遥控通道。
2. 架空验证单轴速度环和输出方向，再调云台归中。
3. 调云台机械档、速控档与松杆保持。
4. 调四轮基础环，再调跟随方向、死区、前馈和斜坡。
5. 单独标定升降零位、行程及上下行堵转阈值。
6. 最后调摩擦轮和拨盘，实测弹速/射频；恢复裁判及功率链时补做整车验证。

调试器写入 `gimbal_tune`、`lift_tune` 只改变 RAM，复位后恢复源码默认值。确认有效后将参数写回实际初始化使用的配置，重新构建验证。

## 调试与故障排查

### 建议观察变量

| 板卡 | 变量 | 用途 |
| --- | --- | --- |
| 上板 | `imu_dbg`、`imu_dev.work_state` | 原始惯性数据、姿态、角速度、校准和错误状态 |
| 上板 | `Gimbal`、`gimbal_tune` | 实际模式、归中判据、角度目标、保持环参数 |
| 上板 | `Board_Rx_Info`、`Board_HeartBeat` | D1/D2/D5 内容及各帧离线计数 |
| 上板 | `dm_motor`、`rm_motor`、`dail_motor` | 两轴、摩擦轮和拨盘反馈/在线状态 |
| 上板 | `launcher`、`launcher_heat` | 发射状态、热量余量、射频、热停发及双轮标定观测 |
| 上板 | `lift`、`lift_tune`、`lift_debug` | 找顶判据、行程、互锁和手动调试 |
| 下板 | `rc_dev.info`、`rc_dev.work_state` | 通道、拨杆、键位、鼠标值与在线状态 |
| 下板 | `board.tx_pkt`、`board.rx_meg`、`board.status` | 板间指令、姿态、升降报告与 C2 时间戳 |
| 下板 | `board.status->heat_d3_tx_ok_count/fail_count` | D3入队成功/失败次数，观察计数增量 |
| 下板 | `board.status->heat_d3_tx_gap_ms/max_gap_ms` | D3最近/最大成功间隔，ms |
| 下板 | `board.status->control_tx_defer_count` | 整组容量不足的延后次数 |
| 上板 | `board_feedback_debug` | C1/C2成功时刻、成功/失败计数与无邮箱延后计数 |
| 下板 | `chassis_input_cmd`、`chassis_ctrl` | 输入来源、四轮目标/反馈、输出及故障 |
| 下板 | `chassis_follow`、`chassis_spin` | 模式选择、反馈有效性、跟随输出及锁存状态 |
| 下板 | `board_lift_dbg`、`board_hole_request`、`board_hole_exit_pending` | B 边沿、后向拒绝与退出等待 |
| 下板 | `launch`、`supercap` | 发射指令，超电收发计数、反馈和离线状态 |

`lift_debug.mode` 非零会进入手动调试路径并优先于正常状态机安全分支，不能把它当作只读观察项。动作测试后应恢复为 0；实际运行前重新核对输出。

### 常见现象

| 现象 | 优先检查 |
| --- | --- |
| 下板遥控一直离线 | UART5 RX PD2、100000/8E2、18 字节帧、DMA/IDLE 接收、`rc_dev` 状态 |
| 上板不使能 | D1/D2 是否持续到达、`car_state`、IMU 校准/错误、两轴在线、实际 `Gimbal` 模式 |
| 有 C1 但跟随不起作用 | C2 ID/解码、`gimbal_data_valid` 与 `gimbal_rx_time_ms`、反馈量纲、跟随锁存故障 |
| F 按了无效 | S1 是否上位、遥控是否在线、是否有新按下边沿 |
| R 后 W 方向异常 | 当前 Z/X/C 档、机械角与掉头基准、是否重复施加平移反向 |
| 跟随反转或抖动 | Yaw 角符号、`FOLLOW_WZ_SIGN`、电机方向、前馈符号、跳变阈值；先确认方向再加增益 |
| 摩擦轮突然启动 | 是否进入键鼠模式，或遥控 S2 已解锁并停在待发中位 |
| 拨盘不供弹 | 许可、KT 反馈、触发、`launcher_heat.ready/blocked/source`；无裁判训练需显式配置 |
| B 无响应 | S1 上/S2 中、按下边沿、后向基准拒绝、`BOARD_LIFT_ENABLE` |
| 升降上报 2 但未找零 | 查看是否 `LIFT_WAIT`，检查 `home_valid`，不能只看报告值 |
| 升降提前停止 | 上下行电流阈值、位置进展判据、方向符号、`fault_code`、超行程保护 |
| 超电在线但不供电 | 当前 CAP_SWITCH 和功率字段为 0，属于保活配置 |
| UI/裁判数据无效果 | UI 宏；裁判 USART1 接收、D3 源有效位和帧时间；D4 上板仍不解析 |

## 验证与维护

当前无单元测试框架。代码修改后需编译对应工程；协议、整车互锁或跨板模式修改需要同时编译两板。

### 台架验收顺序

| 阶段 | 操作 | 验收要点 |
| --- | --- | --- |
| 编译 | 构建对应 Target | 0 errors，核对 warning、内存与本次产物 |
| 设备在线 | 无负载观察遥控、电机、IMU | 数据变化、在线判断与反馈方向正确 |
| 板间通信 | 观察 D1/D2/D5、C1/C2 | 周期、单位、字节序、时间戳与超时正确 |
| 机械云台 | 归中、Pitch 边界、R 前后切换 | 实际到位、无撞限位、无持续震荡；区分归中超时放行 |
| 基础底盘 | 架空测试前后、横移、转向 | 四轮方向正确，输入无效或轮离线时输出符合代码保护 |
| 跟随 | 遥控上/中 S2，键鼠 Z | 中心稳定，平移朝向正确，C2 失效/跳变能退出或停控 |
| 小陀螺 | 遥控 S2 下，键鼠 C | 自稳、平移变换、旋转方向和反馈失效处理正确 |
| 键鼠掉头 | 分别在 X、Z 按 R | 两种行为符合映射，结束后跟随中心无明显突变 |
| 升降 | 自动找顶、B 下压/回升、后向拒绝 | 标定、对齐、行程、电流阈值、报告状态与实物一致 |
| 发射 | 清空通道后测试许可、单发、长按连发、停机 | 摩擦轮达速、拨盘供弹和停止；补验键鼠升降互锁边界 |
| 失联 | 分别断开遥控、板间链路、C2、相关电机反馈 | 模块按各自保护路径停止/卸力/降速，不保留危险旧指令 |

实车联调记录应包含板卡、固件提交、宏配置、供电、负载、参数、异常及复现步骤。仅编译通过不能替代 CAN、遥控器和电机台架验证。

### 修改约定

- C99 风格，模块前缀命名，宏全大写；新增模块放对应 Layer。
- 注释解释原因与约束，删除失效实现；警示标签使用 `TODO:`、`FIXME:`、`NOTE:`、`WARNING:`。
- 不手工改 MDK-ARM 生成文件；重新生成 CubeMX 前检查 USER CODE 区域和外设配置差异。
- 修改协议同步检查两板；修改功能宏还需检查初始化、任务创建、接收分发和输出路径。
- 提交信息用简短中文；PR 说明修改目的、板卡、编译结果、硬件验证和兼容性。
- 不提交日志、临时脚本、备份及本地调试产物；`.gitignore` 已覆盖多数 Keil 构建文件。
- 每次修改输入映射、宏默认值、协议或构建步骤，同时更新本 README。

## 开发记录与当前边界

以下提交仅用于定位功能演进，描述来自 Git 历史；最终行为以当前源码和台架记录为准。

| 提交 | 记录主题 | 相关模块 |
| --- | --- | --- |
| `fb6827c` | 单发可用 | 上板 launcher / 拨盘 |
| `76a7a18` | 连发参数调整 | 上板发射参数 |
| `754aa0a` | 超电通信测试未验证 | 下板超电调试链 |
| `f0efeb6` | 升降模式完成 | 上板 lift、下板过洞指令 |
| `3f7c8dc` | 归中调试 | 上板云台 |
| `1b36a3f` | R 转向调试 | 云台机械转向 |
| `56a1928` | 机械模式仍有很小超调 | 云台控制参数 |
| `06c57c3` | 底盘跟随调试 | 下板 chassis_follow |
| `405f7d8` | R 问题基本解决，键鼠映射仍需修改 | 下板输入、跟随、协议与指令生成 |

当前需要继续跟踪的边界：

- 键鼠映射仍处调试阶段；Z/X/C、R、过洞与发射组合需整车复验。
- 键鼠发射分支绕过下板遥控升降互锁，上板也未以升降/过洞状态作为发射总许可条件。
- 云台归中超时按完成处理；升降 WAIT 上报 2，完成标志不能替代实物到位判断。
- 云台当前使能门槛未显式检查 IMU 就绪和两轴在线，传感器/电机状态需独立核对。
- 升降行程仍为临时测试配置，方向、电流及速度阈值均需按机构实测。
- 裁判热量控制已接入，尚未编译或实机验证；UI、完整功率管理、自瞄未在默认链路完整启用。
- 看门狗初始化关闭；需恢复并验证各任务执行与异常处理后再作为正式运行配置。
- 本次文档已核对源码、Keil 工程配置和 Git 历史，未进行固件编译或硬件验证。

第三方组件的许可证保留在对应 Drivers / Middlewares 目录；仓库未提供统一的根目录许可证，分发时应分别核对各组件许可。
