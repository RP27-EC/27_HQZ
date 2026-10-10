# 下板四轮 RM3508 控制与 CAN 映射

[返回下板模块索引](README.md) · [底盘闭环](chassis.md) · [板间命令](board-link.md)

## 实例、总线与反馈 ID

四个轮实例定义在 `task_down/Application/DeviceLayer/motor.c`，均为 RM3508 减速电机，绑定 FDCAN1；命令组标准 ID 为 `0x200`，同帧四槽按 LF/LB/RF/RB 排列。反馈电机 ID 对应 `0x201`–`0x204`，接收后更新独立 `rm_rx_t` 与在线状态。

| 实例索引 | 电机逻辑名 | `rxId` 槽 | 反馈标准 ID | 发送组槽 |
| --- | --- | ---: | ---: | ---: |
| `WHEEL_LF` | 左前 | 0 | `0x201` | `0x200` byte 0–1 |
| `WHEEL_LB` | 左后 | 1 | `0x202` | `0x200` byte 2–3 |
| `WHEEL_RF` | 右前 | 2 | `0x203` | `0x200` byte 4–5 |
| `WHEEL_RB` | 右后 | 3 | `0x204` | `0x200` byte 6–7 |

表中的实际输出编码由 `HardwareLayer/RM_motor.c` 组帧函数决定；不要把 feedback ID 当成命令组 ID，也不要仅交换软件数组而不核对底盘实体轮位和电机方向。

RM 标准反馈帧 byte 0–1 为编码器角、2–3 为电机轴 RPM、4–5 为电流原始计数、byte 6 为温度。驱动把 RPM 按 `_3508_REDUCT_RATIO` 换成输出轴 rad/s 写入 `rx_info->speed`，保留 `encoder_speed` 作为 RPM。组控制帧的每个槽为高字节在前的 16-bit 电流计数；`tx_info->torque` 则先按型号换算成原始电流，不可直接把 N·m 数值写入 CAN 字节。

## 控制信号和量纲

`DeviceLayer/motor.c` 装配 `wheel_group` 和速度 PID；`ModuleLayer/chassis_control.c` 在每个 CtrlTask 周期执行在线检查、速度目标、速度反馈、控制器计算和组力矩发送。当前四轮为纯 P 速度环，`CHASSIS_SPEED_KP=0.8`、KI/KD 为 0；控制输出单位为 N·m，默认 bring-up 源限矩 2 N·m，跟随与小陀螺源限矩 4 N·m。随后功率限制可按同一比例进一步缩小四个输出。

| 数据 | 代码使用方式 | 单位边界 |
| --- | --- | --- |
| `rx_info->encoder_speed` | RM 反馈原始转速；也传给功率预测模型 | rpm（电机轴） |
| `rx_info->speed` | RM 驱动根据型号换算后的输出轴角速度 | rad/s（已除减速比） |
| `rx_info->current` | 电调反馈/诊断 | 驱动原始值；不默认等于 A |
| `tx_info->torque` | 速度控制器输出并进入 RM 组帧 | N·m 控制域，非 CAN 原始 16-bit 数值 |
| `state->status` | 四轮全在线门槛 | 心跳判定；不能说明机械联轴/供电/轮子转动正常 |

## 停机与在线判据

`Chassis_Control_CheckOnline()` 必须看到四个实例对象、反馈结构、控制器和 `DEV_ONLINE` 状态全部有效。任一电机离线、命令无效/非有限数、对象指针缺失或输出函数缺失，调用停止路径清零四轮 `tx_info->torque` 并发组帧。此保护是整组停机，不会用剩余三轮继续控制。

零目标且反馈低于停止带时清理 PID 内部状态，防止残余输出。功率缩放位于 PID 限矩之后、写入电机之前；观察 `power_limit_state.scale` 可判断是否发生二次削减。

## 诊断路径

| 现象 | 优先查看 |
| --- | --- |
| 某一轮始终离线 | FDCAN1 RX filter、反馈 ID、总线物理层、对应 `wheel_state[i]` 与 `offline_cnt` |
| 轮序错/转向反 | `WHEEL_*` 与实体接线映射、`order_correction`、正负目标及测速方向 |
| 轮子有反馈但无输出 | `chassis_ctrl.state.enabled/fault/cmd.valid`、四轮在线门槛、输出力矩和 0x200 TX 统计 |
| 输出偏小 | 查看源限矩、`Power_Limit_Apply()` 的 scale、目标功率/回退状态，再检查速度环误差 |
| 轮速单位混乱 | 区分 `speed` 与 `encoder_speed`；不要混用 RPM、rad/s 与 raw |

配置集中在 `task_down/Application/ConfigLayer/chassis_config.h` 与 `power_limit_config.h`。首次上电抬离地面，单独低速检查四轮方向和转速反馈；温度/错误状态的存在不代表本工程拥有统一超温硬切断逻辑。

### 从控制目标到 RM 电流帧

```text
chassis_cmd_t(vx, vy, wz)
  → LF/LB/RF/RB 轮速目标
  → 读取输出轴 speed(rad/s)
  → 速度 P 控制器产生 torque(N·m)
  → 每轮来源限矩 + 公共功率 scale
  → RM 型号换算为 raw current
  → FDCAN1 标准 ID 0x200，4 个 big-endian 槽
```

反馈也需区分原始与换算字段：`encoder_speed` 是转子 RPM；`speed` 是按减速比换算后的输出轴 rad/s；两者不是同一个量。怀疑轮序时应同时对照软件枚举、反馈 ID、槽位和实体左前/左后/右前/右后，避免仅凭数组下标推断线束位置。

## 按源码讲解驱动适配

| 文件与函数 | 输入 → 输出 | 职责边界 |
| --- | --- | --- |
| [motor.c](../Application/DeviceLayer/motor.c) | 实例配置 → `wheel_group`、各轮PID/反馈对象 | 轮序、型号和总线绑定，不计算整车运动学 |
| [can_protocol.c](../Application/ProtocolLayer/can_protocol.c) `CAN1_rxDataHandler()` | FDCAN1标准ID/字节 → 对应轮更新 | 同一总线还需区分超电0x211 |
| [RM_motor.c](../Application/HardwareLayer/RM_motor.c) `rm_motor_update()` | RM字节 → 编码器/rpm/current/temperature | 反馈解码并刷新在线计数 |
| 同文件 `RPM_to_Rads()` / `Torque_to_Raw_Current()` | 型号与rpm/N·m → 输出轴rad/s/原始电流 | 单位换算依赖 `_3508_Reduction` 分支 |
| `Group_Motor_Set_Torque()` | 四轮原始电流 → 0x200四槽 | 合帧发送；槽位与各反馈ID对应 |
| [chassis_control.c](../Application/ModuleLayer/chassis_control.c) | 轮速误差 → 候选/最终力矩 | 运动学、闭环、限矩和限功位于业务模块 |
| [drv_can.c](../Application/DriverLayer/drv_can.c) `CAN_SendData()` | 标准ID/数据 → HAL入队结果 | 不提供电机执行确认 |

### 反馈与功率模型为何保留两套速度

轮速闭环使用输出轴 `speed`，功率模型使用转子 `encoder_speed`；模型系数是按转子rpm拟合的，不能将输出轴rad/s直接代入。同理，模型输入电流是发送协议原始计数，不能直接替换为A或N·m。

同名 `RM_motor.c` 在上下板各有一份，上板摩擦轮/升降使用单电机原始电流分支，下板四轮使用减速电机力矩换算分支。定位问题先核对实际构建板卡，避免改错另一板驱动。

可用于讲解：“四轮电机驱动负责反馈解码、减速比和电流换算，底盘模块计算速度闭环并决定最终力矩；一次0x200命令包含四轮输出，任何轮离线由整组保护停机。”
