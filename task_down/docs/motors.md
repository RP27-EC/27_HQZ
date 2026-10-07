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
