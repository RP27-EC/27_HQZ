# 上板云台控制

[返回上板模块索引](README.md) · [返回上板工程说明](../Readme.md) · [协议字段](board-link.md)

## 控制链路

```mermaid
flowchart LR
  BMI[BMI088 / EKF] --> STATE[IMU角度与角速度]
  ENC[DM电机编码器] --> STATE
  D1[D1使能与模式] --> FSM[G_SLEEP / G_INIT / 运行模式]
  D2[D2目标角] --> TARGET[模式目标选择]
  D5[D5遥控角速度或鼠标增量] --> TARGET
  STATE --> CTRL[位置/速度控制与前馈]
  TARGET --> CTRL
  CTRL --> LIMIT[力矩限幅]
  LIMIT --> DM[DM Yaw / Pitch]
  CTRL --> LIFT[升降对齐与Pitch零位互锁]
```

主实现为 `task_up/Application/ModuleLayer/gimbal.c`、`gimbal.h`。`control_task.c` 每 1 ms 更新传感器和模块；设备初始化、DM CAN 收发及在线状态分别由 `DeviceLayer/motor.c` 和 `HardwareLayer/DM_Motor.c` 管理。

## 模式状态与选择

| 状态 | 进入/退出条件 | 实际动作 |
| --- | --- | --- |
| `G_SLEEP` | 板间心跳离线或 D1 `car_state==0` | 两轴输出清零并撤销初始化完成标志 |
| `G_INIT` | 安全条件恢复且尚未完成初始化 | 以机械目标归中；位置误差、速度和稳定时间均参与完成判定 |
| `G_MEC` | `init_flag` 有效且 D1 `gimbal_mode==0` | Yaw 机械位置保持；Pitch 使用速率通路和机械限制 |
| `G_RATE` | `init_flag` 有效且 D1 `gimbal_mode!=0` | 操作角速度指令进入速率闭环，松杆时位置保持环接管 |
| `G_GYRO`、`G_AUTO` | 枚举/分支存在 | 当前选择函数没有由 D1 选入这两个模式 |

`G_INIT` 稳定阈值为 30 ms，最长等待 6000 ms。超时分支也会置 `init_flag`，因此“初始化完成”不能单独证明云台已到机械零位。板间失联恢复后会重新初始化；检查实际编码器角和机构空间后再允许高速动作。

模式切换清理相关 PID 状态并对齐目标，切换首周期置零输出。手动输入源由 D5 的来源位区分遥控/键鼠；键鼠格式携带鼠标增量，角速度格式的量化单位为 0.1 deg/s/LSB。`GIMBAL_LOCAL_RC_ENABLE=0`，当前上板不直接启用本地遥控输入。

## 两轴闭环与补偿

- **Yaw 机械保持**：电机机械角误差形成速率目标，IMU 角速度用于内环反馈；位置保持和速度保持各有死区。静摩擦补偿只在误差死区外参与，速度反馈经过低通。
- **速率与松杆保持**：操作速率幅值低于 2 deg/s 时保持环进入；高于 8 deg/s 时完全交还手动输入，中间区域用于平滑接管。Yaw、Pitch 保持环参数分别配置。
- **Pitch 输出**：速率控制结果与重力补偿合成后再限矩；重力项使用中点角、正负方向、幅值和偏置，不是固定常数。
- **单位边界**：IMU角度及人工输入命令使用 deg、deg/s；机械位置控制接口有 deg/rad 转换；DM 输出为 N·m。跨协议字段请以[板间协议页](board-link.md)的映射范围为准。

## 常用配置与调参入口

| 文件/宏 | 当前值 | 作用与单位 |
| --- | --- | --- |
| `ConfigLayer/gimbal_init_config.h`：`GIMBAL_INIT_*_HOME_DEG` | Yaw/Pitch 均 0 deg | 初始化目标；机械中点仍需与实物标定对应 |
| `GIMBAL_INIT_TIMEOUT_MS` / `GIMBAL_INIT_STABLE_MS` | 6000 / 30 ms | 初始化等待上限与连续稳定时间 |
| `GIMBAL_INIT_*_MAX_RATE_DEG_S` | Yaw 200、Pitch 45 deg/s | 归中目标速率限制；速度规划开关当前为 0 |
| `GIMBAL_INIT_*_TORQUE_LIMIT_NM` | 两轴 6 N·m | 初始化状态力矩限幅 |
| `GIMBAL_INIT_PITCH_GRAVITY_*` | 开启；K=1.1 N·m、B=0 N·m、中点 0 deg | Pitch 余弦重力补偿参数 |
| `gimbal.h`：`GIMBAL_YAW_MIDDLE_DEG` / `GIMBAL_PITCH_MIDDLE_DEG` | -22.224138 / 148.573157 deg | 当前机械中点常量；与归中目标不是同一组参数 |
| `GIMBAL_TORQUE_LIMIT` | 6 N·m | 运行输出最终限幅 |
| `GIMBAL_RATE_HOLD_ENTER/EXIT_DEG_S` | 2 / 8 deg/s | 速控松杆保持环接管门限 |

运行控制中较常观察的参数（均定义在 `task_up/Application/ModuleLayer/gimbal.h`）：

| 参数 | 当前值 | 作用/单位 |
| --- | ---: | --- |
| `GIMBAL_MANUAL_YAW_RATE_DEG_S` / `...PITCH...` | 200 / 150 deg/s | 摇杆满量程映射到的速率上限 |
| `GIMBAL_RC_AXIS_MAX` / `GIMBAL_RC_AXIS_DEADBAND` | 660 / 20 | 摇杆归一化满量程和死区 |
| `GIMBAL_MOUSE_YAW/PITCH_DEG_PER_COUNT` | 0.04 / 0.04 deg/count | 鼠标位移到目标角增量 |
| `GIMBAL_RATE_CMD_RAMP_DEG_S_PER_MS` | 6 deg/s/ms | 速率命令斜坡变化限制 |
| `GIMBAL_MEC_YAW_MAX_RATE_DEG_S` | 300 deg/s | 机械 Yaw 运动目标限速 |
| `GIMBAL_MEC_YAW_FRICTION_FF_NM` | 0.3 N·m | 死区外静摩擦补偿幅值 |
| `GIMBAL_YAW_HOLD_KP/KI` | 10 / 0.003 | Yaw 位置保持 PI；输出 deg/s |
| `GIMBAL_PITCH_HOLD_KP/KI` | 50 / 0 | Pitch 位置保持 PI；输出 deg/s |
| `GIMBAL_MEC_HOLD_KP` | 1.5 | Yaw 机械位置保持 P；宏注释定义为 N·m/deg |
| `GIMBAL_MEC_HOLD_RATE_KP` / `OUT_MAX` | 0.1 / 6.0 | Yaw 角速度保持控制器增益与输出限幅，N·m 域 |

归中状态使用 `gimbal_init_config.h` 另设四个位置/速度环（Yaw 外环 Kp/Kd=0.1/1.0、输出上限 500；Yaw 内环 Kp/Kd=1.5/0.2、输出上限 100；Pitch 外环 Kp=1.6、输出上限 10；Pitch 内环 Kp=1.2、输出上限 10）。这些数值是控制器内部输出域，最后仍由两轴 6 N·m 限幅约束；不应仅凭表中 PID 输出上限推导实际电机力矩。

角速度指令/保持 PID 的增益单位随控制误差和输出单位组合变化，不能跨 P/I/D 项直接比较数值。Yaw/Pitch 位置误差先以 deg 计算，而电机输出力矩最终都经过 N·m 限幅。

当前值不等于已完成实车标定。尤其机械中点、Pitch 行程端点、重力方向和力矩限幅需按实际机构确认。

## 排查顺序

1. 看 `Board_HeartBeat.status` 和 D1 `car_state`，确认不是 `G_SLEEP` 保护。
2. 看 IMU error/cali 状态、`imu_dev.work_state`、两轴编码器和电机在线位，避免用 PID 掩盖无效反馈。
3. 对照 `Gimbal` 当前模式、目标角/角速度、位置误差、内环反馈与最终力矩，分清目标没有到达还是输出受限。
4. 速控漂移检查 D5 `valid/source/cmd_type`、控制方向和松杆接管阈值；Pitch 下垂检查重力补偿符号/中点。
5. 检查 C1/C2 心跳和发送计数；MCU 入队成功不等于下板收到。

调试观察入口：`Gimbal`、`gimbal_feedforward`、`gimbal_init_info`、`imu_dbg`、`Board_Rx_Info.remote_cmd_pkt`、`board_feedback_debug`。首次调试卸除弹丸、限制力矩并留出两轴全行程空间。

### 目标到输出的 Watch 路径

| 顺序 | 建议字段 | 用途 |
| --- | --- | --- |
| 1 | `Board_HeartBeat.status`、`Board_Rx_Info.state_pkt.car_state` | 判断是否被 `G_SLEEP` 保护 |
| 2 | `Gimbal.gimbal_mode`、`init_info.init_flag` | 确认选中模式与归中状态，必要时查超时标志 |
| 3 | `Gimbal.base_info`、`Board_Rx_Info.gimbal_target_pkt`、`remote_cmd_pkt` | 对照角度单位、角速度、机械目标与手动输入 |
| 4 | 位置/速度误差、hold 状态、feedforward | 分辨目标生成、反馈方向和保持/补偿分支 |
| 5 | `tx_info.torque`、DM 反馈与 CAN 发送状态 | 判断是否在软件限幅后仍有命令以及反馈是否跟随 |

字段层级按 `gimbal.h` 当前结构定义核实。若只观察最终 torque，无法区分“目标未生成”“反馈无效”“输出被限幅”或“驱动器未执行”。
