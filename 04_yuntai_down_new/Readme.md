# 03_yuntai_down 下板云台通信工程

本工程是云台上下板组合测试用的下板程序，基于 STM32H723VGTx 和 FreeRTOS。

当前阶段只保留最小云台测试链：

```text
USART5 DBUS 遥控器
    -> 下板遥控解析
    -> CAN2 D1/D2/D5
    -> 上板云台控制
```

下板负责遥控输入和板间通信，不负责云台电机闭环。

## 当前运行模式

调试开关位于：

```text
Application/ConfigLayer/board_comm_config.h
```

当前配置：

```c
#define BOARD_COMM_DEBUG                1u
#define BOARD_COMM_TX_ENABLE            1u
#define BOARD_COMM_D1D2_PERIOD_MS       1u
#define BOARD_COMM_D5_ENABLE            1u

#define BOARD_CAP_ENABLE                0u
#define BOARD_JUDGE_ENABLE              0u
#define BOARD_UI_ENABLE                 0u
```

调试模式下运行 FreeRTOS 调度、USART5 DBUS 解析、D1/D2/D5 发送、遥控心跳和板间心跳。

调试模式下不运行下板 BMI088、infantry.work()、gimbal.work()、底盘、发射、视觉、UI、裁判和超电。D3/D4 当前关闭。

## 硬件与接口

- MCU：STM32H723VGTx
- RTOS：FreeRTOS
- 调试器：Keil MDK
- 遥控接口：USART5 DBUS
- 板间通信：CAN2，1 Mbit/s

USART5 DBUS 配置为 100000 baud、9 位数据、2 位停止位、偶校验、只接收。

## CAN2 板间协议

| CAN ID | 方向 | 当前用途 |
|---|---|---|
| `0xD1` | 下板到上板 | 整车状态、云台模式 |
| `0xD2` | 下板到上板 | Yaw/Pitch 机械目标和 IMU 目标 |
| `0xD5` | 下板到上板 | 操作手 Yaw/Pitch 目标角速度 |
| `0xC1` | 上板到下板 | 上板电机状态、云台状态 |
| `0xC2` | 上板到下板 | Yaw/Pitch 机械角和 IMU 角反馈 |

D1/D2/D5 由 `connect_task.c` 周期发送，当前 D1/D2 周期为 1 ms。

## D1 状态

调试模式下，遥控在线时发送：

```text
car_state   = 1
gimbal_mode = 1
```

遥控离线时发送：

```text
car_state   = 0
gimbal_mode = 0
```

`gimbal_mode = 1` 让上板进入 `G_RATE` 速控模式。

## D5 手操角速度

当前布局：

```text
byte0 bit0: valid
byte0 bit1: ctrl_source
byte1:      button_bits，当前固定为 0
byte2-3:    Yaw 角速度，int16 大端，单位 0.1 deg/s
byte4-5:    Pitch 角速度，int16 大端，单位 0.1 deg/s
byte6-7:    预留
```

遥控器右摇杆左右控制 Yaw 角速度，上下控制 Pitch 角速度。遥控在线时 `valid = 1`，离线时为 `0`。

当前 D5 只转发遥控器右摇杆，键鼠暂时不能控制云台。

相关参数：

```c
#define BOARD_RC_AXIS_DEADBAND          20.0f
#define BOARD_RC_AXIS_MAX               660.0f
#define BOARD_D5_YAW_RATE_MAX_DEG_S     300.0f
#define BOARD_D5_PITCH_RATE_MAX_DEG_S   150.0f
#define BOARD_D5_RATE_LSB_DEG_S         0.1f
```

## 任务说明

| 任务 | 主要工作 |
|---|---|
| `CommandTask` | DBUS 遥控和键鼠解析 |
| `ConnectTask` | 发送 D1、D2、D5 |
| `CtrlTask` | 更新 D1 在线状态和云台模式 |
| `MonitorTask` | 遥控心跳、板间心跳 |

## 编译方法

打开 Keil 工程：

```text
03_yuntai_down/MDK-ARM/DM-MC02.uvprojx
```

Target：`DM-MC02`。

构建产物：

```text
03_yuntai_down/MDK-ARM/DM-MC02/DM-MC02.axf
03_yuntai_down/MDK-ARM/DM-MC02/DM-MC02.hex
```

最近验证结果：全量 rebuild `0 errors / 44 warnings`，警告均为原工程保留警告。

## 第一次联调顺序

1. 上下板供电，先不接负载。
2. 检查 CANH/CANL、共地和 120 欧姆终端电阻。
3. 检查 USART5 DBUS 遥控在线。
4. 用 Keil Watch 检查 D1/D2/D5 和上板 `offline_cnt_1/2/5`。
5. 确认上板 D5 `valid = 1`。
6. 先轻推右摇杆，再测试云台 Yaw/Pitch 方向。
7. 遥控回中，确认云台不漂移。
8. 遥控失联，确认上板约 100 ms 内卸力。
9. 最后再调整上板速度内环参数。

## 当前限制

- D5 暂不支持键鼠。
- 下板工程已物理删除底盘、发射、视觉、UI、裁判、超电、IMU 和电机控制源码；D3/D4 不发送。
- 下板调试模式不运行完整步兵控制。
- 上板机械中值、PID、重力补偿和遥控方向仍需实车标定。
- 修改 D5 字节序或协议布局时，必须同步修改上板解析代码。
