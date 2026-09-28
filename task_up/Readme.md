# task_up 上板控制工程

基于 STM32F407 和 FreeRTOS，负责云台、发射、升降和上下板通信。

## 当前功能

- BMI088 IMU 姿态解算与角速度反馈
- Pitch、Yaw 两轴 DM4310 闭环控制
- 机械模式、IMU 位置模式和纯速控模式
- 摩擦轮、拨盘和抬升机构控制
- FDCAN2 接收下板 D1/D2/D5
- FDCAN2 返回 C1/C2
- USB CDC 调试接口
- 板间、IMU 和电机离线保护

主控制链路：

```text
FDCAN2 D1/D2/D5
    -> ControlTask
    -> 云台、发射、升降
    -> CAN1/CAN2 电机输出

BMI088 + DM4310 反馈
    -> C1/C2 状态与姿态回传
```

手动控制输入只来自下板 D5。上板本机 USART3 遥控链路已删除。

## 板间协议

| CAN ID | 方向 | 内容 |
|---|---|---|
| `0xD1` | 下板到上板 | 车辆状态、云台模式、发射状态 |
| `0xD2` | 下板到上板 | 云台机械目标和 IMU 目标 |
| `0xD5` | 下板到上板 | 遥控/键鼠角速度控制 |
| `0xC1` | 上板到下板 | 电机在线状态和抬升状态 |
| `0xC2` | 上板到下板 | 云台机械角和 IMU 角反馈 |

D3/D4、裁判系统、UI 和视觉协议已移除。C1 保留位保持为零。

## 任务

| 任务 | 周期 | 主要工作 |
|---|---:|---|
| `ControlTask` | 1 ms | IMU、云台、发射、升降和板间发送 |
| `MonitorTask` | 1 ms | IMU、电机和板间心跳 |
| `LedTask` | 1 ms | 运行状态指示 |

## 外设

- `SPI1`：BMI088
- `CAN1`：Pitch、摩擦轮、拨盘和抬升电机
- `CAN2`：Yaw 电机及上下板通信
- `USB_OTG_FS`：USB CDC

## 编译

打开工程：

```text
task_up/MDK-ARM/My_C.uvprojx
```

Target：`My_C`。

```powershell
UV4 -b task_up\MDK-ARM\My_C.uvprojx -t My_C -o build_up.log
```

构建产物：

```text
task_up/MDK-ARM/My_C/My_C.axf
task_up/MDK-ARM/My_C/My_C.hex
```

## 标定与验证

- 机械中值、Pitch 限位、PID 和重力补偿需要实车标定。
- 首次测试应限制力矩并脱开弹丸和负载。
- 软件已完成全量重建验证，电机方向、CAN 接线和整车动作仍需台架确认。
- 修改 D1/D2/D5 或 C1/C2 布局时，必须同步修改下板。
