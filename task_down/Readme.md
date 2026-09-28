# task_down 下板控制工程

基于 STM32H723VGTx 和 FreeRTOS，负责底盘、发射、超电和上下板通信。

## 当前功能

- UART5 DBUS 遥控解析
- 新底盘输入、跟随、小陀螺和四轮闭环
- 发射机构状态、拨盘和摩擦轮控制
- 超电保活与 CAN 反馈接收
- FDCAN2 向上板发送 D1/D2/D5
- FDCAN2 接收上板 C1/C2 并维护离线状态

主控制链路：

```text
UART5 DBUS
    -> CommandTask 遥控解析
    -> CtrlTask 底盘/发射/超电控制
    -> FDCAN1 四轮与超电
    -> FDCAN2 D1/D2/D5
```

## 板间协议

| CAN ID | 方向 | 内容 |
|---|---|---|
| `0xD1` | 下板到上板 | 车辆状态、云台模式、发射状态 |
| `0xD2` | 下板到上板 | 云台机械目标和 IMU 目标 |
| `0xD5` | 下板到上板 | 遥控/键鼠角速度控制 |
| `0xC1` | 上板到下板 | 电机在线状态和抬升状态 |
| `0xC2` | 上板到下板 | 云台机械角和 IMU 角反馈 |

D1 保留字节由发送缓存清零，当前只有已定义位参与控制。D3/D4、裁判系统和视觉协议已移除。

## 任务

| 任务 | 主要工作 |
|---|---|
| `CommandTask` | DBUS 遥控和键鼠输入解析 |
| `CtrlTask` | 底盘、发射、超电周期控制 |
| `ConnectTask` | 发送 D1、D2、D5 |
| `MonitorTask` | 遥控和板间链路心跳 |

## 编译

打开工程：

```text
task_down/MDK-ARM/DM-MC02.uvprojx
```

Target：`DM-MC02`。

```powershell
UV4 -b task_down\MDK-ARM\DM-MC02.uvprojx -t DM-MC02 -o build_down.log
```

构建产物：

```text
task_down/MDK-ARM/DM-MC02/DM-MC02.axf
task_down/MDK-ARM/DM-MC02/DM-MC02.hex
```

## 当前边界

- 已删除旧底盘、下板旧云台、`infantry`、UI、视觉、observe、旧超电、裁判、D3/D4 和下板本机 IMU。
- 软件已完成全量重建验证，硬件接口和整车动作仍需台架确认。
- 修改 CAN ID、字节序或协议布局时，必须同步修改上板。
