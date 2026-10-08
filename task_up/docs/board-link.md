# 上板板间 CAN 协议与收发

[返回上板模块索引](README.md) · [返回根协议速查](../../README.md#protocol--tuning) · [下板通信实现](../../task_down/docs/board-link.md)

## 接口和接收流程

物理链路为上板 bxCAN CAN2（PB5/PB6）连接下板 FDCAN2；使用 11 位标准 ID、经典 CAN、8 字节。上板 `task_up/Application/ProtocolLayer/communicate.c` 按 D1–D5 分发解包，反馈由 `Send_To_Down_Board()` 发送 C1/C2。协议没有版本协商；任意字节定义变化必须同时改两板。

| ID | 方向 | 解码字段 | 上板实际使用 |
| --- | --- | --- | --- |
| `0xD1` | 下→上 | b0：car state、gimbal mode、vision mode、game start、color；b1–4：vx/vy 定点值；b5：launch/mode/trigger/is_hole | 车辆使能、云台模式、发射输入和升降请求 |
| `0xD2` | 下→上 | 4 个 uint16 大端：Pitch IMU、Yaw IMU、Pitch 机械、Yaw 机械 | 缓存模式目标；各控制模式选择使用对应目标 |
| `0xD3` | 下→上 | b0–1 热量上限、b2–3 当前热量、b4–5 冷却率；b6 序号；b7 bit0 参数有效、bit1 热量有效 | 发射热量数据及独立新鲜度检查 |
| `0xD4` | 下→上 | 保留 8 字节 | 当前只清心跳计数，不消费血量字段 |
| `0xD5` | 下→上 | b0 bit0 valid/bit1 source/bit2 cmd_type；b1 按键；b2–3 Yaw、b4–5 Pitch 有符号大端；b6–7 保留 | 来源为键鼠且 cmd_type=1 时取鼠标增量；否则按 0.1 deg/s/LSB 解释角速度 |
| `0xC1` | 上→下 | b0 bit0–6：Yaw、Pitch、lift、右/左摩擦轮、dial、vision 在线；b1：升降压缩状态；b2–5：当前编码的 0 deg 保留值；b6–7：0 | 下板用于设备状态显示、模式互锁和故障观察 |
| `0xC2` | 上→下 | 4 个 uint16 大端：Yaw 机械、Pitch 机械、Yaw IMU、Pitch IMU | 下板跟随/机械掉头使用机械 Yaw，调试显示四轴反馈 |

D1 b1–4 的速度字段映射范围为 [-8000,8000]；D2/C2 的 IMU 角映射 [-360,360] deg、机械角映射 [-4,4] rad。整型值为映射量，不是 IEEE 浮点数。具体编码/解码以 `communicate.c` 为准。

## 接收字段逐字节说明

| 帧/字节 | 位或编码 | 上板解析结果 | 消费边界 |
| --- | --- | --- | --- |
| D1 b0 | bit0–1 car_state；bit2 gimbal_mode；bit3–5 vision_mode；bit6 game_start；bit7 my_color | 上板解码车辆状态与模式 | 当前云台选择只按 `gimbal_mode` 的 0/非0 分支选择 MEC/RATE；vision 字段不会自动启用视觉控制 |
| D1 b5 | bit0 launch_state；bit1 shoot_mode；bit2 shoot_level；bit3 is_hole；bit4 r_turn_active；bit5 feed_permit | 摩擦轮使能/模式/触发/升降请求/R掉头状态/供弹许可 | bit4只在机械Yaw控制中选择R专用斜坡及最大速度；新旧发射固件不可混用，需同步更新上下板 |
| D2 b0–7 | 四个 uint16，大端 | Pitch IMU、Yaw IMU、Pitch mec、Yaw mec | IMU 区间 [-360,360] deg；机械角区间 [-4,4] rad；超范围编码会饱和 |
| D3 b0–5 | heat_limit、barrel_heat、cooling_rate 各 uint16 大端 | 裁判量与冷却率快照 | b7 bit0 参数有效、bit1 热量有效；收帧只刷新 `rx_tick`，不会替发送端保证其数据有效 |
| D5 b0 | bit0 valid；bit1 source；bit2 cmd_type | 角速度或鼠标增量类型选择 | 仅 `source=键鼠 && cmd_type=鼠标` 时按鼠标计数解释 b2–5；其他情况按 0.1 deg/s/LSB |
| D5 b2–5 | 两个有符号 int16，大端 | Yaw、Pitch 输入 | 原始值量化步长 0.1 deg/s/LSB；后续控制仍可能再次限幅 |

线性 16-bit 编码公式为 `raw=(value-min)/(max-min)*65535`，解码为 `value=raw*(max-min)/65535+min`。这是全区间量化而非浮点直传；范围端点、字节序或字段排列变化都属于两板协议变更。

## 心跳、有效位和调度语义

- D1/D2 分别重置心跳计数；任一关键控制帧离线会影响上板云台/机构安全判断。D3/D4/D5 的心跳计数不能替代各自字段有效位或接收时间。
- D3 收到后保存 `rx_tick`、`heat_seq`、`flags`；CAN 帧到达但 flags 未置有效，热量源仍可能未就绪。
- C1/C2 每类最短发送间隔 `BOARD_FEEDBACK_PERIOD_MS=5 ms`，每个 ControlTask 调用最多尝试一帧，轮流选择并竞争 CAN2 邮箱。
- 邮箱无空位会增加 `defer_count` 并延迟发送；HAL 接受入队才更新对应 `*_ok_count` 和时间戳。入队成功不代表总线上发送完成或下板已经收到。
- D4 当前关闭发送，但上板仍会处理接收 ID 的心跳；不要以“心跳在动”推断 D4 有血量内容。

## 数据排查

| 观察 | 检查方向 |
| --- | --- |
| `Board_Rx_Info.state_pkt` | D1 字节序、car_state 和 gimbal_mode 是否与下板模式对应 |
| `gimbal_target_pkt` | D2 映射区间、机械 rad / IMU deg 的单位 |
| `remote_cmd_pkt` | D5 valid/source/cmd_type；角速度与鼠标计数不能混用 |
| `heat_pkt` | D3 flags、seq、rx_tick 与裁判快照时间戳 |
| `Board_HeartBeat` | D1/D2 离线计数和 D3–D5 辅助计数 |
| `board_feedback_debug` | C1/C2 成功、失败、延后计数及最近入队时间 |

### 上板排查次序

1. 先看 CAN2 对应 ID 的接收/发送结果与 D1/D2 心跳，排除物理层、过滤器和总线速率问题。
2. 再解码 `state_pkt`、`gimbal_target_pkt`、`remote_cmd_pkt`、`heat_pkt`，确认字段范围/有效位/单位一致。
3. 进入消费模块看模式、状态机和使用的目标字段，确认数据没有被在线保护、超时或限幅拒绝。
4. 最后核对 C1/C2 入队结果、下板反馈 age 和解码值；本地发送成功不是端到端确认。

接收异常先核实双方标准 ID、过滤器、位时序和物理收发器，再核对字段定义。接口或位域调整需要同步[下板协议页](../../task_down/docs/board-link.md)和根 README。
