# 上板模块文档

[返回上板工程说明](../Readme.md) · [返回项目总览](../../README.md)

本目录对应 STM32F407 上板运行链路。任务入口在 `Application/TaskLayer/control_task.c`；模式/状态控制位于 ModuleLayer，器件采集和电机协议位于 DeviceLayer/HardwareLayer。

## 按模块讲解的顺序

| 模块 | 一句话说清职责 | 输入 → 处理 → 输出 | 优先打开的源码 |
| --- | --- | --- | --- |
| IMU | 将 BMI088 采样转成云台可用的姿态/速率 | SPI 原始量 → 坐标变换、零偏、EKF → `imu_data.base_info` | [imu_sensor.c](../Application/DeviceLayer/Sensor/imu_sensor.c)、[bmi_EKF.c](../Application/DeviceLayer/Imu/bmi_EKF.c) |
| 云台 | 按模式将目标与反馈转成两轴力矩 | D1/D2/D5、IMU/DM → 目标仲裁、闭环、补偿 → DM Yaw/Pitch | [gimbal.c](../Application/ModuleLayer/gimbal.c)、[gimbal.h](../Application/ModuleLayer/gimbal.h) |
| 升降 | 建立零点后按编码器目标执行上下运动 | D1 请求、RM 反馈、云台状态 → FSM、双环、保护 → 升降输出和 C1 状态 | [lift.c](../Application/ModuleLayer/lift.c)、[lift_config.h](../Application/ConfigLayer/lift_config.h) |
| 发射 | 独立控制摩擦轮与拨盘，并管理热量 | D1 许可/触发、D3、电机反馈 → 速度/位置环、热量状态 → RM/KT 命令 | [launcher.c](../Application/ModuleLayer/launcher.c)、[launcher_config.h](../Application/ConfigLayer/launcher_config.h) |
| 电机驱动 | 将控制量与设备专用协议互相转换 | 模块输出/总线反馈 → 打包、解码、单位换算、心跳 → 电机对象 | [motor.c](../Application/DeviceLayer/motor.c)、[can_protocol.c](../Application/ProtocolLayer/can_protocol.c) |
| 板间协议 | 接收下板控制并回传执行状态 | D1–D5 → 解码对象；本板状态 → C1/C2 | [communicate.c](../Application/ProtocolLayer/communicate.c) |

讲每个模块时依次回答五件事：**谁调用它、输入是什么、核心算法/状态是什么、输出到哪里、什么条件会限制输出**。具体参数和函数阅读顺序见下方模块页；全项目讲解口径见[根 README](../../README.md#项目讲解与源码速查)。

### 一轮上板控制中的先后关系

`StartControlTask()` 更新 IMU 后调用 `Module_Work()`，该函数先 `Gimbal.work()` 再 `Lift_Work()`；随后发送 DM 命令，执行 `Launcher_Work()`，最后尝试 C1/C2 回传。模块在同一任务中顺序执行，部分跨模块标志会由后一模块更新并在下一轮被前一模块消费。CAN 接收中断还可能在任务执行期间更新协议对象，不能把整轮理解为原子事务。

```mermaid
flowchart LR
  CAN[下板 D1-D5 / CAN2] --> CONTROL[1 ms ControlTask]
  SPI[BMI088 SPI1] --> IMU[IMU + EKF]
  IMU --> CONTROL
  CONTROL --> G[Gimbal]
  CONTROL --> L[Lift FSM]
  CONTROL --> S[Launcher + heat]
  G --> DM[DM motors]
  L --> RM[RM2006]
  S --> SHOOT[RM3508 + KT4005]
  G --> FB[C1/C2]
  L --> FB
  S --> FB
  FB --> CAN
```

| 模块 README | 重点 |
| --- | --- |
| [云台](gimbal.md) | G_SLEEP/G_INIT/G_MEC/G_RATE 仲裁、级联控制、Pitch 重力补偿 |
| [升降机构](lift.md) | 找顶、零点有效性、对齐/行程、故障和 C1 状态 |
| [发射与热量](launcher.md) | 摩擦轮/拨盘状态、D3 新鲜度、预算与限频 |
| [电机](motors.md) | DM/RM/KT 实例、CAN、单位转换和在线状态 |
| [IMU](imu.md) | BMI088 SPI、EKF、姿态单位和校准状态 |
| [板间通信](board-link.md) | D1–D5 接收、C1/C2 回传与有效性 |

## 推荐阅读路线

| 调试目的 | 阅读路线 | 关键状态/变量 |
| --- | --- | --- |
| 上电后云台不动 | [板间通信](board-link.md) → [IMU](imu.md) → [电机](motors.md) → [云台](gimbal.md) | D1 heartbeat、`imu_dev.work_state`、DM online、`Gimbal` mode/output |
| 云台跟手或归中异常 | [云台](gimbal.md) → [IMU](imu.md) → [电机](motors.md) | D5 `valid/source/cmd_type`、目标/反馈单位、力矩限幅 |
| 升降没有响应 | [升降](lift.md) → [板间通信](board-link.md) → [云台](gimbal.md) | `home_valid`、`fault_code`、`is_hole`、Pitch/Yaw 对齐条件 |
| 拨盘拒绝供弹 | [发射](launcher.md) → [板间通信](board-link.md) → [下板裁判](../../task_down/docs/referee.md) | D1许可、D3 flags/age、`launcher_heat.source/ready/blocked` |

## 公共时序与单位约定

- ControlTask 以 1 ms 节拍运行；IMU 任务调用频率不等于 BMI088 实际数据采样率。
- 云台算法中的 IMU角/手动角速度主要使用 deg、deg/s；DM 电机反馈位置接口使用机械角换算，协议 C2 对机械角映射为 rad。
- 升降位置状态使用 encoder count，RM 电流字段是驱动原始计数；任何 count→mm 或 raw→A 换算都需要机构/驱动参数依据。
- `G_INIT` 的状态标志需与到位误差、速度和稳定计时一起看；超时完成不代表机构已经到达安全机械中心。

模块页中的当前宏值是源码默认值；若运行时 Watch 改写了 RAM 参数，实际控制值可能与编译默认值不同。

调参前先确认模块运行状态、反馈单位和设备在线状态；上电测试安全步骤见[上板 README](../Readme.md#troubleshooting--safety)。
