# 03_yuntai_up 上板云台控制工程

底层通信部分和接口部分的代码暂时移植的源码，可能还有些乱码，这些我将在后续修改

接下来一些是我在编写代码时候的一些idea吧，后面的是对工程的整体描述

pid.c中，我一开始想用的是是否可以使用前馈+弱kp的pid模式，类似于底盘部分的思想，在和ai的探讨后还是否定了这一想法

但是前馈这个思想还是可以耦合到其它的一些部分

pitch轴需要重力前馈

学过刚体的都知道，旋转的时候会有转矩，那云台在高速转动的时候是否也可以给一个前馈去补偿呢    好吧，这个加速度前馈好像有一些复杂，后续我再去研究研究，不过似乎这部分一般这里也不会去用

这些我应该会写在gimbal.c

feedforward我会放在gimbal里

那么pid.c这里就只是简单的单环和串级双环吗，其实还会有一些小算法

像微分先行，kd加个低通滤波，抗积分饱和等等，这些就比较简单了，代码中也会有体现
gimbal.c中，我是有参考了步兵的源码进行代码的编写

云台的解算

我们知道，机械即编码器的话，它的角速度反馈是很差的

于是乎我们选择陀螺仪去进行角速度的反馈

init初始化的时候需要对齐二者

初始使用机械，后面使用陀螺仪

所以这部分还是沿用了源码的部分
在操作的时候，如果我们仅通过给目标值，然后通过误差pid计算的话，就会导致有延迟，手感会很差，那我们是否可以将遥控器的期望值直接赋给内环的输出呢，这部分暂时我先写在云台部分，后续再遥控器部分代码再补齐

但是不太对，遥控器输出的和内环输出的量纲不统一，应该是加到内环的目标值才对
速控还是位控呢，速控的话手感更好，位控的话可能不够丝滑

但是速控没办法控制对应位置

当然，自瞄用位控，操作手操作用速控呢，

或者两者结合起来，角速度输入加较小角度修正

最终我应该先采用速控操作手，位控是自瞄

不过后面想了一下我觉得我会把pitch轴换成位控，yaw轴速控，速控也不会是纯速控，会加点放飘移

pitch轴本身能转的角度就很小，直接位控可能会更稳
还有一个云台自稳对吧

小陀螺模式的时候，云台需要输出一个和底盘反向的角速度

通常这个角速度需要加前馈，纯PID响应太慢了

似乎还需要有些预测，因为这是有小延迟的，需再加个小数值

这样才能小陀螺走直线的时候也不会偏对吧

那这部分就等底盘之后再去进行一个编写，暂时先留着
重力补偿的话参数等再测试，用的比较简单的重力补偿
阻力补偿的话一般就给yaw轴，但也有可能不需要，我再看看实际效果再考虑加不加吧
然后在写代码的过程像一些容易跳变的地方，比如像归中等可能会突然有很大误差的时候，我会使用斜坡函数，逐渐步进，防止突然跳变，从而造成受伤等的情况

还有一些地方可以加斜坡
然后其实已经差不多了吧，大概就先这样了







## 1. 工程简介

本工程是 RoboMaster 步兵上板云台控制程序，基于 STM32F407 和 FreeRTOS，实现两轴云台的控制与上下板通信。

当前主要功能：

- BMI088 IMU 姿态解算与角速度反馈
- Pitch、Yaw 两个 DM4310 MIT 模式电机控制
- 机械模式、陀螺位置模式、纯速控模式
- 遥控器和鼠标角速度输入
- 上下板 CAN 通信
- 8 路串级 PID
- 模式切换目标斜坡
- 初始化检测与超时保护
- Pitch 重力补偿接口
- Yaw 阻力补偿伪代码预留
- DM、板间通信、IMU 在线检测与安全归零

当前工程已完成 Keil 编译验证，但机械中值、PID、重力补偿和手动控制手感仍需实车标定。

## 2. 硬件与接口

### 主控

- MCU：STM32F407
- RTOS：FreeRTOS
- IMU：BMI088
- IMU 解算：当前配置使用 EKF
- 遥控器接口：USART3，DBUS，100000 baud，偶校验

### 电机分配

| 轴 | CAN | 控制 ID | 反馈 ID |
|---|---:|---:|---:|
| Pitch | CAN1 | `0x01` | `0x11` |
| Yaw | CAN2 | `0x02` | `0x12` |

Yaw 与下板通信共用 CAN2。

## 3. 目录结构

```text
03_yuntai_me/
├─ Application/
│  ├─ AlgorithmLayer/     PID、数学、滤波、功率限制等算法
│  ├─ ConfigLayer/       运行配置、驱动配置、设备配置
│  ├─ DeviceLayer/       IMU、遥控器、电机对象等设备层
│  ├─ DriverLayer/       CAN、UART、SPI、TIM 等底层驱动
│  ├─ HardwareLayer/     DM、RM、HT、KT 等电机硬件驱动
│  ├─ ModuleLayer/       云台模块
│  │  ├─ gimbal.c
│  │  ├─ gimbal.h
│  │  ├─ module.c
│  │  └─ module.h
│  ├─ ProtocolLayer/     CAN 协议与上下板通信
│  │  ├─ can_protocol.c
│  │  ├─ can_protocol.h
│  │  ├─ communicate.c
│  │  └─ communicate.h
│  ├─ TaskLayer/         控制、监控、LED、社区任务
│  └─ UserLayer/         用户初始化代码
├─ Core/                 STM32CubeMX 生成代码
├─ Drivers/              HAL、CMSIS、DSP
├─ Middlewares/          FreeRTOS、USB Device
├─ MDK-ARM/              Keil 工程
├─ USB_DEVICE/           USB CDC 配置
├─ My_C.ioc              CubeMX 配置文件
└─ Readme.md
```

## 4. 云台控制结构

### 4.1 数据流

```text
下板 CAN 目标
    -> Board_Rx_Info
    -> 云台模式仲裁
    -> 角度环或角速度环
    -> 重力、阻力、力矩前馈
    -> DM4310 力矩输出
```

### 4.2 PID 前馈分层

PID 只负责反馈计算：

```text
PID输出 = P + I + D
```

角速度前馈加在内环目标：

```text
内环目标角速度 =
    角度外环输出
    + 操作手角速度前馈
```

力矩前馈加在最终输出：

```text
最终力矩 =
    内环PID输出
    + 重力补偿
    + 阻力补偿
    + 其他力矩前馈
```

## 5. 云台模式

```c
typedef enum
{
    G_SLEEP = 0,
    G_INIT,
    G_MEC,
    G_GYRO,
    G_RATE,
} gimbal_mode_e;
```

| 模式 | 说明 |
|---|---|
| `G_SLEEP` | 休眠，输出零力矩 |
| `G_INIT` | 上电归中，使用机械目标和目标斜坡 |
| `G_MEC` | 机械模式，电机角度外环 + 电机速度内环 |
| `G_GYRO` | 陀螺位置模式，IMU 角度外环 + IMU 角速度内环 |
| `G_RATE` | 纯速控模式，操作手目标角速度 + IMU 角速度内环 |

当前下板只发送 1 位云台模式：

```text
gimbal_mode == 0  -> G_MEC
gimbal_mode != 0  -> G_RATE
```

`G_GYRO` 当前保留，但暂时不通过现有板间模式字选择。

## 6. 初始化流程

1. 等待下板、IMU 和两个 DM4310 在线。
2. 进入 `G_INIT`。
3. Yaw 使用最短路径斜坡归中。
4. Pitch 使用斜坡归中并受机械限位保护。
5. 判断实际角度、机械速度和斜坡目标是否到位。
6. 连续稳定时间达到 30 ms 后初始化完成。
7. 超过 6 s 仍未完成时执行超时退出。

初始化相关参数位于 `gimbal.c` 的 `Gimbal_Init()` 和 `gimbal.h`。

## 7. 手动控制

### 遥控器模式

`car_state == 1` 时：

- 遥控器右侧左右通道控制 Yaw 目标角速度
- 遥控器右侧上下通道控制 Pitch 目标角速度

### 键鼠模式

`car_state == 2` 时：

- 鼠标 X 控制 Yaw 目标角速度
- 鼠标 Y 控制 Pitch 目标角速度

当前没有把普通键盘按键映射为云台角速度。

## 8. 上下板通信

### 上板接收

| CAN ID | 内容 |
|---|---|
| `0xD1` | 整车状态、云台模式、发射状态 |
| `0xD2` | Yaw/Pitch 机械目标与 IMU 目标 |
| `0xD3` | 裁判系统数据，当前只接收不处理 |
| `0xD4` | 血量数据，当前只接收不处理 |

### 上板发送

| CAN ID | 内容 |
|---|---|
| `0xC1` | 电机状态、升降状态、视觉占位 |
| `0xC2` | Yaw/Pitch 机械角与 IMU 角反馈 |

板间心跳由 `monitor_task.c` 每 1 ms 更新。连续收不到 `0xD1` 或 `0xD2` 后，板间状态变为离线。

## 9. 任务分配

| 任务 | 周期 | 主要工作 |
|---|---:|---|
| `ControlTask` | 1 ms | IMU 更新、遥控输入处理、云台计算、DM 输出、板间发送 |
| `MonitorTask` | 1 ms | IMU 心跳、DM 心跳、遥控器心跳、板间心跳 |
| `CommunityTask` | 1 ms | 预留 |
| `LedTask` | 1 ms | LED 状态 |

## 10. 关键参数

关键参数位于 `Application/ModuleLayer/gimbal.h`。

### 机械参数

```c
#define GIMBAL_YAW_MIDDLE_DEG      0.0f
#define GIMBAL_PITCH_MIDDLE_DEG    0.0f
#define GIMBAL_PITCH_MIN_DEG       (-7.5f)
#define GIMBAL_PITCH_MAX_DEG       30.0f
#define GIMBAL_TORQUE_LIMIT        3.0f
```

需实车重新确认。

### 手动控制

```c
#define GIMBAL_RC_AXIS_MAX                 660.0f
#define GIMBAL_RC_AXIS_DEADBAND            20.0f
#define GIMBAL_MANUAL_YAW_RATE_DEG_S       300.0f
#define GIMBAL_MANUAL_PITCH_RATE_DEG_S     150.0f
#define GIMBAL_MOUSE_YAW_RATE_GAIN         1.0f
#define GIMBAL_MOUSE_PITCH_RATE_GAIN       1.0f
#define GIMBAL_RATE_CMD_RAMP_DEG_S_PER_MS  6.0f
```

### 速控保持

```c
#define GIMBAL_RATE_HOLD_KP                0.5f
#define GIMBAL_RATE_HOLD_DEADBAND_DEG_S    5.0f
```

设置过大会导致速控手感变慢，设置过小则 Yaw 漂移和 Pitch 下垂会更明显。

### 重力补偿

```c
#define GIMBAL_GRAVITY_ENABLE      1
#define GIMBAL_GRAVITY_K_NM        0.3f
#define GIMBAL_GRAVITY_B_NM        0.0f
#define GIMBAL_GRAVITY_MIDDLE_DEG  0.0f
```

需实车确认方向和机械中值。

## 11. PID 初始参数

| 控制环 | Kp | Ki | Kd | 输出限幅 |
|---|---:|---:|---:|---:|
| Yaw 陀螺外环 | 20.0 | 0.05 | 0.0 | 500 |
| Yaw 陀螺内环 | 0.04 | 0.0 | 0.0 | 10 |
| Pitch 陀螺外环 | 56.0 | 0.0 | 0.0 | 100 |
| Pitch 陀螺内环 | 0.03 | 0.0 | 0.0 | 10 |
| Yaw 机械外环 | 20.0 | 0.0 | 0.0 | 500 |
| Yaw 机械内环 | 0.1 | 0.0 | 0.0 | 10 |
| Pitch 机械外环 | 1.6 | 0.0 | 0.0 | 10 |
| Pitch 机械内环 | 1.2 | 0.0 | 0.0 | 10 |

这些值为初始值，直接搬过来的，需重测。

## 12. 安全保护

以下任一条件不满足时，云台双轴力矩归零：

- 板间通信离线
- 整车进入休眠
- Yaw 或 Pitch 电机离线
- IMU 未在线或未完成校准
- 机械角、IMU 角或目标值非法
- 模式切换第一帧

DM 脱落或离线后，必须重新满足接管条件才会恢复输出。

## 13. 编译方法

使用 Keil MDK 打开：

```text
03_yuntai_me/MDK-ARM/My_C.uvprojx
```

Target：

```text
My_C
```

当前编译结果：

- 增量编译：`0 errors / 0 warnings`
- 全量 rebuild：`0 errors`
- 全量警告主要来自原有 RM、HT 和 CMSIS DSP 文件

构建产物：

```text
03_yuntai_me/MDK-ARM/My_C/My_C.axf
03_yuntai_me/MDK-ARM/My_C/My_C.hex
```

## 14. 上板调试顺序

1. 不接电机，确认程序启动、IMU、CAN 和板间心跳正常。
2. 确认 Pitch/Yaw 电机方向、反馈 ID 和角速度符号。
3. 确认机械中值、Pitch 上下限和电机零位。
4. 小力矩测试正方向，建议先将总力矩限制降到 `0.3~0.5 N·m`。
5. 调 Yaw/Pitch 速度内环。
6. 调机械角度外环。
7. 调陀螺位置外环。
8. 标定重力补偿方向、幅值和机械中值。
9. 最后开启阻力补偿或较大力矩限制。

## 15. 当前待办

- 标定 `GIMBAL_YAW_MIDDLE_DEG`
- 标定 `GIMBAL_PITCH_MIDDLE_DEG`
- 确认 Pitch 机械限位
- 确认重力补偿方向和幅值
- 调整遥控器与鼠标输入增益
- 调整速控模式角度保持系数
- 根据实车效果决定是否启用 Yaw 阻力补偿
- 如需显式选择 `G_GYRO`，扩展下板模式协议

## 16. 注意事项

- 第一次上电不要让云台带弹丸或带负载运行。
- 未确认电机方向前必须关闭两轴输出。
- 重力补偿启用后，首次测试应使用低力矩上限。
- 修改机械中值后，需要重新检查初始化、限位和重力补偿。
- 模式切换和离线恢复必须保证不会突发大力矩。
