# task_up 上板控制工程

> STM32F407IGHx + FreeRTOS 的云台、发射机构和狗洞升降执行板。

`task_up` 接收下板 D1/D2/D5，负责云台闭环、发射状态机和升降状态机，通过 C1/C2 回传状态与姿态。这个工程的边界很明确：下板决定“能不能动、往哪动、是否允许发射”，上板只负责把命令变成安全、连续、可观测的执行动作。

本文分成两层阅读：

- 前半部分是架构、设计取舍和问题复盘，适合快速评审。
- 后半部分是协议、状态机、控制公式、调参入口和排查表，适合接手上号。

## 1. 工程定位

```text
下板 task_down                         上板 task_up
  遥控器 / 键鼠                           BMI088
  整车模式和安全决策                      DM4310 Yaw/Pitch
  发射决策                                M3508 摩擦轮
  云台目标                               KT4005 拨盘
  升降请求                               RM2006 狗洞升降
        |                                      |
        |        D1 / D2 / D5                  |
        +------------------------------------->|
                                               |
        |<--------------------------------------+
                    C1 / C2 状态与反馈
```

上板职责：

- BMI088 姿态解算和角速度反馈。
- 上电归中。
- Yaw/Pitch 机械位置模式。
- Yaw/Pitch 速控模式。
- 松杆保持、重力补偿和模式切换保护。
- 双摩擦轮速度环。
- 拨盘单发、连发、停机制动和可选卡弹恢复。
- 狗洞升降找顶、回退、上下位运动和堵转保护。
- 板间心跳、设备在线状态和 C1/C2 回传。

上板不负责：

- 遥控器和键鼠解析。
- 底盘运动。
- 整车模式选择。
- 发射许可决策。
- 自瞄、视觉串级、UI、裁判和正式功率限制。

这样可以避免两个 MCU 同时读取遥控器、同时做模式判断，避免命令权冲突。

## 2. 设计重点

### 2.1 D1/D2 是安全链路，D5 是操作链路

上板没有把三路报文塞进一个心跳：

| 报文 | 作用 | 超时后果 |
|---|---|---|
| D1 | 整车状态、云台模式、发射状态、狗洞请求 | 连续 100 ms 丢失，板间整体离线 |
| D2 | 云台机械角/IMU 角目标 | 连续 100 ms 丢失，板间整体离线 |
| D5 | 遥控或键鼠角速度 | 连续 100 ms 丢失，手动速度归零 |

判离线逻辑：

```text
D1 或 D2 任一达到 100 ms
  -> Board_HeartBeat.status = DEV_OFFLINE
  -> gimbal_can_send() 先卸力再发帧
```

D5 只影响手动操作，不让整个板间链路离线。这个拆分是上板安全设计的基础。

### 2.2 `car_state` 是第二道闸门

即使板间报文还在，`car_state == 0` 仍然不允许云台闭环：

```c
if ((Board_HeartBeat.status == DEV_ONLINE) &&
    (Board_Rx_Info.state_pkt.car_state != 0))
{
    DM_Group.group_set_torque(&DM_Group);
}
else
{
    DM_Group.group_sleep(&DM_Group);
    DM_Group.group_set_torque(&DM_Group);
}
```

升降也只在上板在线且 `car_state != 0` 时参与运动。这样下板可以通过一条状态位迅速让上板卸力。

### 2.3 云台模式切换先清历史，再切目标

上板当前活跃模式：

```text
G_SLEEP -> G_INIT -> G_MEC
                  -> G_RATE
```

仲裁顺序：

1. 板间离线或 `car_state == 0`：`G_SLEEP`。
2. 安全条件满足但 `init_flag == 0`：`G_INIT`。
3. 归中完成且 D1 `gimbal_mode == 0`：`G_MEC`。
4. 归中完成且 D1 `gimbal_mode != 0`：`G_RATE`。

`G_GYRO` 和 `G_AUTO` 保留枚举，但当前仲裁不会主动选择。

切换模式时执行：

- 清空全部 PID 积分和微分历史。
- 把目标角对齐到当前反馈角。
- 重置松杆保持目标。
- 切换帧输出 0 力矩。
- 下一帧从当前角度开始爬升。

这解决的是模式切换时的瞬时冲击，而不是 PID 参数本身。

### 2.4 Pitch 保持环使用机械编码器

IMU 安装在偏航部分。云台俯仰时，IMU 欧拉角中的 Pitch 无法稳定表达 Pitch 电机相对角。用 IMU Pitch 做松杆保持，误差可能接近零，保持环就不会出力。

当前选择：

- Yaw 保持使用 IMU Yaw 角。
- Pitch 保持使用机械编码器角。
- Pitch 速度内环使用机械速度反馈并换算到 deg/s。

这是一个由机械结构决定控制量选择的例子。

### 2.5 机械模式采用“限速 + 柔顺保持”

机械模式控制链：

```text
位置 P/D
  -> 按剩余误差计算制动速度上限
  -> 速度环
  -> 力矩
```

制动速度上限：

```text
v_limit = sqrt(2 * a_decel * abs(error))
```

再与最大速度取较小值。大误差时不会一直高速冲，接近目标时速度自然收敛。

小误差区叠加：

```text
hold_torque = Kp_hold * angle_error - Kd_hold * speed
```

通过误差权重从速度环平滑过渡到位置柔顺项，降低静止附近来回抖动。

### 2.6 发射状态机把安全解锁和动作执行拆开

下板负责 S2 消抖、上电后重新解锁、打符禁发和升降联锁。上板只执行 D1 给出的：

```text
launch_state
shoot_mode
shoot_level
```

上板还要求：

- 板间在线。
- 左右摩擦轮在线。
- 拨盘在线。

发射状态快照使用 `Board_Rx_Shoot_Flags`，同一拍只读一个字节，避免一次控制周期内多个字段前后不一致。

### 2.7 升降找顶采用双条件确认

只靠电流会误判，只靠静止也会误判。找顶条件：

- 电流达到阈值。
- 静止窗口或低速条件满足。
- 条件连续维持 500 ms。

进入故障后不会自动重试。只有狗洞命令发生边沿变化，才允许恢复。这解决了“上电误入 fault 后卡死”的问题。

## 3. 启动和任务模型

### 3.1 main 启动顺序

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

初始化职责：

| 函数 | 内容 |
|---|---|
| `DEVICE_Init()` | IMU、RM 电机组、KT 拨盘、DM 云台组、发射机构 |
| `Module_Init()` | 云台 PID、升降对象绑定 |
| `DRIVER_Init()` | CAN 过滤器、CAN 启动 |
| `MX_FREERTOS_Init()` | Control、Monitor、Led 任务 |

IWDG 代码存在，但 `MX_IWDG_Init()` 当前被注释。

### 3.2 FreeRTOS 任务

系统 tick 为 1 kHz。

| 任务 | 优先级 | 栈 | 周期 | 职责 |
|---|---:|---:|---:|---|
| `ControlTask` | `osPriorityRealtime` | 1024 words | 1 ms | IMU、云台、升降、DM 输出、发射、通信 |
| `MonitorTask` | `osPriorityRealtime` | 512 words | 1 ms | IMU、DM、RM、KT、板间心跳 |
| `LedTask` | `osPriorityAboveNormal` | 256 words | 开机后 5 Hz | 状态灯 |

ControlTask 固定顺序：

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

不能随意交换顺序：

- 云台在发送前必须完成目标更新。
- 升降故障可能需要在同一拍清输出。
- 发射状态比云台发送晚一拍时，会出现机械模式和发射动作争同一帧的问题。

## 4. 外设与 CAN

### 4.1 外设

| 外设 | 引脚/参数 | 用途 |
|---|---|---|
| SPI1 | PB3/PB4/PA7，软件 NSS | BMI088 |
| BMI088 ACC CS | PA4 | 加速度计 |
| BMI088 GYRO CS | PB0 | 陀螺仪 |
| CAN1 | PD0/PD1 | Pitch、摩擦轮、拨盘、升降 |
| CAN2 | PB5/PB6 | Yaw、D1/D2/D5、C1/C2 |
| USB OTG FS | USB CDC | 调试 |
| TIM2 | HAL 时基 | `HAL_GetTick()` |
| LED | PH10/PH11/PH12 | 状态指示 |

SPI1：主机模式、8 位、CPOL=1、CPHA=2Edge、MSB First、16 分频。APB2 84 MHz 时约 5.25 MHz。

### 4.2 CAN1

| ID | 方向 | 内容 |
|---|---|---|
| `0x01` | 上板发送 | Pitch DM4310 |
| `0x11` | 上板接收 | Pitch DM4310 反馈 |
| `0x141` | 双向 | KT4005 拨盘 |
| `0x200` | 上板发送 | 左摩擦轮槽 0、右摩擦轮槽 1、升降槽 3 |
| `0x201` | 上板接收 | 左摩擦轮反馈 |
| `0x202` | 上板接收 | 右摩擦轮反馈 |
| `0x204` | 上板接收 | 升降 RM2006 反馈 |

### 4.3 CAN2

| ID | 方向 | 内容 |
|---|---|---|
| `0x02` | 上板发送 | Yaw DM4310 |
| `0x12` | 上板接收 | Yaw DM4310 反馈 |
| `0xD1` | 下板发送 | 整车状态和发射状态 |
| `0xD2` | 下板发送 | 云台目标 |
| `0xD5` | 下板发送 | 遥控/键鼠速度 |
| `0xC1` | 上板发送 | 电机在线和升降状态 |
| `0xC2` | 上板发送 | 云台机械角/IMU 角 |

过滤器当前为全通掩码，按 ID 在协议层分发。

### 4.4 CAN 波特率风险

上板按 1 Mbps 配置：

```text
APB1 = 42 MHz
Prescaler = 3
TQ = 1 + 10 + 3 = 14
Baud = 42 MHz / 3 / 14 = 1 Mbps
```

下板 `.ioc` 目标是 1 Mbps，但当前生成代码按参数计算为：

```text
FDCAN clock = 24 MHz / 24 * 200 / 4 = 50 MHz
Prescaler = 5
TQ = 1 + 14 + 5 = 20
Baud = 50 MHz / 5 / 20 = 500 kbps
```

这是双板上电联调的第一优先级问题。若协议字段都对但完全没有对端接收，先用分析仪确认波特率。

## 5. 板间协议

D1/D2/D5、C1/C2 必须与 `task_down/Readme.md` 同步维护。

### 5.1 D1，下板到上板

| 字节 | 位 | 含义 |
|---|---|---|
| Byte 0 | bit 1:0 | `car_state` |
| Byte 0 | bit 2 | `gimbal_mode` |
| Byte 5 | bit 0 | `launch_state` |
| Byte 5 | bit 1 | `shoot_mode` |
| Byte 5 | bit 2 | `shoot_level` |
| Byte 5 | bit 3 | `is_hole` |

发送端每次先清零 8 字节，保留位不携带历史状态。

### 5.2 D2，下板到上板

| 字节 | 字段 | 单位 | 量程 |
|---|---|---|---|
| 0..1 | Pitch IMU 目标 | deg | -360..360 |
| 2..3 | Yaw IMU 目标 | deg | -360..360 |
| 4..5 | Pitch 机械目标 | rad | -4..4 |
| 6..7 | Yaw 机械目标 | rad | -4..4 |

压缩公式：

```text
raw = (value - min) / (max - min) * 65535
```

上板解码后进入 `Board_Rx_Info.gimbal_target_pkt`。

### 5.3 D5，下板到上板

| 字节 | 含义 |
|---|---|
| 0 | bit0 valid，bit1 ctrl_source，bit2 cmd_type |
| 1 | 鼠标按键 |
| 2..3 | Yaw 角速度 int16，0.1 deg/s/LSB |
| 4..5 | Pitch 角速度 int16，0.1 deg/s/LSB |
| 6..7 | 保留 |

下板当前始终发送 `cmd_type = 0`。上板保留鼠标增量分支，但当前不使用。

### 5.4 C1，上板到下板

| 位 | 含义 |
|---|---|
| bit 0 | Yaw 电机在线 |
| bit 1 | Pitch 电机在线 |
| bit 2 | 升降电机在线 |
| bit 3 | 右摩擦轮在线 |
| bit 4 | 左摩擦轮在线 |
| bit 5 | 拨盘在线 |

Byte 1 为升降状态：

```text
0 = 下位或下行堵转
1 = 运动
2 = 上位/等待找零
3 = 故障
```

### 5.5 C2，上板到下板

| 字节 | 字段 | 单位 | 量程 |
|---|---|---|---|
| 0..1 | Yaw 机械角 | rad | -4..4 |
| 2..3 | Pitch 机械角 | rad | -4..4 |
| 4..5 | Yaw IMU 角 | deg | -360..360 |
| 6..7 | Pitch IMU 角 | deg | -360..360 |

### 5.6 心跳

| 通道 | 超时阈值 | 超时效果 |
|---|---:|---|
| D1 | 100 ms | 板间离线 |
| D2 | 100 ms | 板间离线 |
| D5 | 100 ms | 手动速度归零 |

## 6. 云台控制

### 6.1 单位和机械角

| 量 | 单位 |
|---|---|
| IMU 角度 | deg |
| IMU 角速度 | deg/s |
| 电机机械角 | deg |
| 电机机械速度 | rad/s |
| 力矩 | N*m |
| D2 机械目标 | rad |
| D2 IMU 目标 | deg |

机械角：

```text
yaw_mec_angle   = wrap_180(yaw_motor_angle - GIMBAL_YAW_MIDDLE_DEG)
pitch_mec_angle = wrap_180(pitch_motor_angle - GIMBAL_PITCH_MIDDLE_DEG)
```

默认中值：

```c
GIMBAL_YAW_MIDDLE_DEG    = -22.224138f
GIMBAL_PITCH_MIDDLE_DEG  = 148.573157f
```

Pitch 限位：

```c
GIMBAL_PITCH_MIN_DEG = -7.5f
GIMBAL_PITCH_MAX_DEG = 30.0f
```

### 6.2 上电归中

归中步进：

```text
Yaw 0.25 deg/ms
Pitch 0.25 deg/ms
```

完成条件：

```text
Yaw/Pitch 角度误差 <= 2 deg
Yaw/Pitch 机械速度 <= 0.5 rad/s
目标与斜坡目标误差 <= 0.1 deg
连续 30 ms
```

超过 6000 ms 时强制结束归中，避免机构卡住导致永远无法进入模式。

### 6.3 机械模式参数

| 参数 | Yaw | Pitch |
|---|---:|---:|
| 最大速度 | 120 deg/s | 90 deg/s |
| 制动减速度 | 8 rad/s^2 | 6 rad/s^2 |
| 保持 Kp | 0.8 N*m/deg | 0.5 N*m/deg |
| 保持 Kd | 0.08 N*m/(rad/s) | 0.06 N*m/(rad/s) |
| 保持力矩 | 3 N*m | 3 N*m |

### 6.4 速控模式

输入有效条件：

```text
D5 offline_cnt_5 < 100
D5 valid != 0
car_state != 0
```

速度目标：

```text
Yaw:
  操作手速度 + hold_blend * Yaw 保持环输出

Pitch:
  操作手速度 + hold_blend * Pitch 保持环输出
```

松杆保持交接：

| 速度幅值 | 保持环权重 |
|---|---|
| `<= 2 deg/s` | 1 |
| `2..8 deg/s` | 线性过渡 |
| `>= 8 deg/s` | 0 |

默认保持参数：

```c
GIMBAL_YAW_HOLD_KP   = 10.0f
GIMBAL_YAW_HOLD_KI   = 0.003f
GIMBAL_PITCH_HOLD_KP = 50.0f
GIMBAL_PITCH_HOLD_KI = 0.0f
```

### 6.5 重力补偿

```text
gravity = sign * (K * cos(pitch_mec_angle - middle_angle) + B)
```

默认：

```c
GIMBAL_GRAVITY_K_NM = 1.1f
GIMBAL_GRAVITY_B_NM = 0.0f
GIMBAL_GRAVITY_SIGN = 1.0f
```

补偿值必须通过实车确认。方向错误时会出现持续顶向一侧或松杆无法回位。

### 6.6 输出限幅

默认单轴限矩：

```c
GIMBAL_TORQUE_LIMIT = 6.0f
```

`gimbal_tune` 支持运行时修改：

```text
gravity_enable
gravity_k_nm
gravity_b_nm
gravity_sign
pitch_torque_limit_nm
yaw_torque_limit_nm
yaw_hold_kp / ki
pitch_hold_kp / ki
rate_hold_enter_deg_s / exit
manual_yaw_sign / pitch_sign
mouse_yaw_deg_per_count / pitch
```

## 7. 发射机构

### 7.1 电机

| 设备 | 控制 | 反馈 |
|---|---|---|
| 左摩擦轮 | `0x200` 槽 0 | `0x201` |
| 右摩擦轮 | `0x200` 槽 1 | `0x202` |
| 拨盘 | `0x141` | `0x141` |

### 7.2 状态机

```text
SLEEP -> READY -> SINGLE -> READY
              -> REPEAT -> READY
              -> STOPPING -> SLEEP
              -> REVERSE -> RELOAD -> READY/REPEAT
              -> FAULT
```

### 7.3 单发

单发触发条件：

```text
shoot_mode == 0
shoot_level 0 -> 1 上升沿
```

动作：

```text
dial_target += 65536 count
```

一圈对应一发。到位容差为 500 count，超时 500 ms。

### 7.4 连发

连发触发条件：

```text
shoot_mode == 1
shoot_level != 0
```

速度使用了独立 PID，而不是共用单发位置环。目标速度为 5400 deg/s，约 15 圈/s。

### 7.5 停机

松触发后：

1. 进入制动环，目标速度为 0。
2. 速度低于 20 deg/s，或 120 ms 超时。
3. 发送停机命令。

不做直接断力，是为了避免拨盘惯性和回弹。

### 7.6 卡弹恢复

当前默认关闭：

```c
LAUNCHER_DIAL_JAM_ENABLE = 0u
```

启用后：

- 低速 + 高电流连续 200 tick。
- 反向退让 65536 count。
- 回到原供弹目标。
- 单发回 READY，连发回 REPEAT。

该功能还未完整验证，不能作为正式比赛依赖。

## 8. 狗洞升降

### 8.1 状态

| 状态 | 含义 |
|---|---|
| `LIFT_WAIT` | 等待上电找零或狗洞命令 |
| `LIFT_HOMING_UP` | 向上找顶 |
| `LIFT_RETRACT_DOWN` | 到顶后回转 |
| `LIFT_READY_UP` | 上位待命 |
| `LIFT_ALIGN_DOWN` | 下降前云台对齐 |
| `LIFT_MOVING_DOWN` | 下降中 |
| `LIFT_MOVING_UP` | 上升中 |
| `LIFT_READY_DOWN` | 下位待命 |
| `LIFT_STALL_STOP` | 下行堵转，等待复位 |
| `LIFT_FAULT` | 故障锁定 |

### 8.2 找零

等待时间：

```text
LIFT_AUTO_HOME_DELAY_MS = 2000
```

找顶：

```text
速度 2865 rpm
电流阈值 520 raw
低速阈值 1 rpm
静止窗口 500 ms
静止位移 40 count
确认时长 500 ms
超时 90000 ms
```

到顶后回转 5 圈作为上位。

### 8.3 行程

当前临时值：

```c
LIFT_TRAVEL_TURNS = 280.0f
LIFT_TRAVEL_COUNTS = 280 * 8192
```

注释要求验证后改回 316 圈。正式装车前不能直接把 280 当最终值。

### 8.4 故障码

| 码 | 含义 |
|---:|---|
| 1 | 找顶超时 |
| 2 | 运动超时 |
| 3 | 超程 |
| 4 | 堵转电流确认 |
| 5 | 堵转无进展 |

故障恢复要求狗洞命令边沿，不允许在同一状态下自动重试。

## 9. 安全联锁

```text
板间 D1/D2 离线
  -> 云台和升降停止

car_state == 0
  -> 云台和升降停止

D5 超时
  -> 手动速度清零

摩擦轮/拨盘离线
  -> 发射总使能为 0

升降超时/超程/堵转
  -> 输出清零，进入 fault 或 stall stop

狗洞命令边沿
  -> 唯一允许退出升降 fault 的条件
```

IWDG 当前未启用，异常恢复主要依赖任务心跳和 CAN 设备心跳。

## 10. 历史问题复盘

### 10.1 `L6200E` 重复符号

早期 `connect_task.c` 里的函数名误写成 `StartUITask()`，和旧 UI 任务重复定义，Keil 链接报 `Symbol StartUITask multiply defined`。

处理：

- `connect_task.c` 改为 `StartConnectTask()`。
- `Core/Src/freertos.c` 仅保留 weak 空壳。

现在的维护规则是：任务函数强定义只能存在一份。

### 10.2 机械模式和发射时序

提交 `078980a` 处理了机械模式和发射机构的时序问题。核心不是调 PID，而是保证：

- 云台模式在一拍内完成仲裁。
- 发射状态在同一拍有稳定快照。
- D1 发送和上板执行间的状态不会前后错位。

上板后来使用 `Board_Rx_Shoot_Flags`，把发射许可、模式、触发位拆成同一字节快照，避免一次控制周期读到不同步字段。

### 10.3 升降 fault 无法退出

提交 `8ecb25f` 增加了 `cmd_changed` 判断。之前的 fault 状态没有明确的退出边沿，导致异常进入后只能复位。

现在的恢复方式：

```text
若 cmd_changed == 0
  -> 保持 fault，输出 0
否则
  -> 按 home_valid 和 is_hole 重新进入找零/下降/上升
```

### 10.4 找顶误判

旧逻辑只看电流或静止，容易在上电静止时认为已经到顶。现在必须电流满足，并且静止/低速辅助条件满足，还要连续确认。

### 10.5 Pitch 松杆保持

提交历史记录了 Yaw/Pitch 保持环调整以及 Pitch 换机械编码器。这个问题的根因是 IMU 的安装位置，而不是参数不够大。

### 10.6 升降 280 圈临时行程

`46844b4` 记录升降不可用，`f0efeb6` 记录升降模式完成。当前代码仍在用临时测试行程，正式结构必须重新记录编码器范围。

## 11. 编译

```powershell
UV4 -r task_up\MDK-ARM\My_C.uvprojx -t My_C -o build_up.log
```

当前保留日志：

```text
Code=71296
RO-data=2840
RW-data=2752
ZI-data=25272
0 Error(s), 0 Warning(s)
```

## 12. 日常调试观察点

建议加入 Keil Watch：

```text
Board_HeartBeat
Board_Rx_Info
Board_Tx_Info
Gimbal.gimbal_mode
Gimbal.base_info
Gimbal.feedforward
Gimbal.pid_info
gimbal_tune
launcher.state
launcher.fric_l_speed_rpm
launcher.fric_r_speed_rpm
launcher.dial_angle
launcher.dial_target_angle
lift.state
lift.fault_code
lift.top_zero
lift.top_target
lift.bottom_target
lift_debug
lift_tune
imu_dbg
```

排障优先级：

1. `Board_HeartBeat.status` 和 `car_state`。
2. DM/RM/KT 在线状态。
3. D2 目标和实际角度。
4. 发射 D1 位和状态机。
5. 升降 fault code 和原始电流/编码器。

## 13. 当前边界

- 正式 CAN 波特率仍需仪器确认。
- 升降行程仍是临时值。
- 卡弹恢复默认关闭。
- 云台机械中值、重力补偿和 D5 方向仍需实车标定。
- IWDG 未启用。
- 自瞄、视觉、裁判、UI 和正式功率限制未接入。
- 没有硬件验证时只能声明“编译通过”，不能声明“车辆可用”。

## 14. 目录

```text
task_up/
  Application/
    AlgorithmLayer/   PID、数学和滤波
    ConfigLayer/      外设、云台、发射、升降参数
    DeviceLayer/      IMU、电机、传感器抽象
    DriverLayer/      CAN、DWT、GPIO
    HardwareLayer/    DM/RM/KT 电机驱动
    ModuleLayer/      云台、发射、升降状态机
    ProtocolLayer/    板间协议和 CAN 分发
    TaskLayer/        Control、Monitor、Led
  Core/               CubeMX 主程序和中断
  Drivers/            STM32F4 HAL/CMSIS
  Middlewares/        FreeRTOS、USB
  MDK-ARM/            Keil 工程和产物
```

## 15. 验收清单

- `My_C` 全量 rebuild 为 0 errors。
- D1/D2/D5 和 C1/C2 在仪器抓包中周期稳定。
- 板间离线 100 ms 内云台卸力。
- 云台归中、机械模式、速控模式均完成空载和带载测试。
- 单发每次只转一圈，连发松触发后能制动停机。
- 升降找顶、上位、下位、回程、超程和堵转已实测。
- 机械中值、Pitch 限位、重力补偿和发射方向完成实车标定。
- 协议或 CAN ID 修改同步检查 `task_down`。
