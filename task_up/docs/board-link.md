# 上板板间 CAN 协议与收发

[返回上板模块索引](README.md) · [返回根协议速查](../../README.md#protocol--tuning) · [下板通信实现](../../task_down/docs/board-link.md)

## 接口和接收流程

物理链路为上板 bxCAN CAN2（PB5/PB6）连接下板 FDCAN2；使用 11 位标准 ID、经典 CAN、8 字节。上板 `task_up/Application/ProtocolLayer/communicate.c` 按 D1–D5 分发解包，反馈由 `Send_To_Down_Board()` 发送 C1/C2。协议没有版本协商；任意字节定义变化必须同时改两板。

| ID | 方向 | 解码字段 | 上板实际使用 |
| --- | --- | --- | --- |
| `0xD1` | 下→上 | b0：车辆状态、云台模式、bit3–5 保留、比赛状态、颜色；b1–4 保留；b5：发射许可/模式/触发/过洞/R掉头 | 车辆使能、云台模式、发射输入和升降请求 |
| `0xD2` | 下→上 | 4 个 uint16 大端：Pitch IMU、Yaw IMU、Pitch 机械、Yaw 机械 | 缓存模式目标；各控制模式选择使用对应目标 |
| `0xD3` | 下→上 | b0–1 热量上限、b2–3 当前热量、b4–5 冷却率；b6 序号；b7 bit0 参数有效、bit1 热量有效 | 发射热量数据及独立新鲜度检查 |
| `0xD4` | 下→上 | 保留 8 字节 | 当前只清心跳计数，不消费血量字段 |
| `0xD5` | 下→上 | b0 bit0 valid/bit1 source/bit2 cmd_type；b1 按键；b2–3 Yaw、b4–5 Pitch 有符号大端；b6–7 保留 | 来源为键鼠且 cmd_type=1 时取鼠标增量；否则按 0.1 deg/s/LSB 解释角速度 |
| `0xD6` | 下→上 | b0–1 弹速0.01 m/s；b2–3 uint16逐发序号；b4–5 源年龄ms；b6 类型；b7 机构编号 | 原子弹速快照、单发修正及高弹速保护 |
| `0xC1` | 上→下 | b0 bit0–5：Yaw、Pitch、lift、右/左摩擦轮、dial 在线；bit6–7 保留；b1：升降压缩状态；b2–7 保留 | 下板用于模式互锁和故障观察 |
| `0xC2` | 上→下 | 4 个 uint16 大端：Yaw 机械、Pitch 机械、Yaw IMU、Pitch IMU | 下板跟随/机械掉头使用机械 Yaw，调试显示四轴反馈 |

D1 b1–4 为保留位；D2/C2 的 IMU 角映射 [-360,360] deg、机械角映射 [-4,4] rad。整型值为映射量，不是 IEEE 浮点数。具体编码/解码以 `communicate.c` 为准。

## 接收字段逐字节说明

| 帧/字节 | 位或编码 | 上板解析结果 | 消费边界 |
| --- | --- | --- | --- |
| D1 b0 | bit0–1 car_state；bit2 gimbal_mode；bit3–5 保留；bit6 game_start；bit7 my_color | 上板解码车辆状态与模式 | 云台模式选择 MEC/RATE |
| D1 b5 | bit0 launch_state；bit1 shoot_mode；bit2 shoot_level；bit3 is_hole；bit4 r_turn_active | 发射许可/模式/触发/升降请求及R掉头状态 | bit4只在机械Yaw控制中选择R专用斜坡及最大速度；需同步更新上下板 |
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

## D6 弹速有效性

- 仅分发标准数据帧、DLC=8 的 D6；通过 `Board_GetSpeedSnapshot()` 在临界区复制完整快照。
- D6 首发心跳为零弹速/零序号/零年龄、类型1/机构1，只证明裁判在线，不参与学习。源年龄65535表示离线；在线年龄最多65534。
- 记录每个事件首收时刻和首收年龄。相同序号重发仅刷新链路与源状态，不刷新样本采样时刻，避免旧数据被当作新发射。
- 弹速源不代替D1/D2整车心跳。D6超过300 ms未到达会禁止新发射；单发等待1000 ms、三发均值修正和保护条件见[发射模块](launcher.md#单发弹速闭环)。
- 两板需同步更新；D1/D3布局保持原样。模式切换及恢复隔离期间的旧事件不参与学习。

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

## 按函数讲解收发实现

| 顺序 | 文件与函数 | 说明 |
| --- | --- | --- |
| 1 | [drv_can.c](../Application/DriverLayer/drv_can.c) 接收回调 | 从FIFO取帧，进入协议分发 |
| 2 | [can_protocol.c](../Application/ProtocolLayer/can_protocol.c) `CAN2_rxDataHandler()` | 同一CAN2上区分Yaw电机与D帧标准ID |
| 3 | [communicate.c](../Application/ProtocolLayer/communicate.c) `Board_Rx_01/02/03/05()` | 解码目标对象，更新心跳/接收时间；D3另存seen/flags/seq |
| 4 | 同文件 `Board_GetHeatSnapshot()` | 在保存/恢复中断状态的临界区复制D3对象，供热量模块读取 |
| 5 | `Board_Tx_Update()` / `Board_Tx_Meg_01/02()` | 从电机/云台/升降状态形成C1/C2，定点编码 |
| 6 | `Send_To_Down_Board()` | 判断每类5 ms间隔及邮箱空位，单次最多尝试一帧 |
| 7 | `C_Board_Communicate_HeartBeat()` | D1/D2任一离线即判总体离线；其他帧计数不能替代关键帧 |

### 用一个数字说明定点编码

D5角速度编码步长为0.1 deg/s：目标+50 deg/s对应有符号整数500，即大端字节 `01 F4`；-50 deg/s对应int16的-500，即 `FE 0C`。这与D2/C2按范围映射到uint16的方法不同。机械角是rad、IMU角是deg，均不能把两字节直接当浮点数。

### 本协议没有整组原子接收保证

D1/D2/D5有独立CAN ID；即使下板预留三个FIFO空位，仍是三次独立发送和接收。当前没有共同控制组序号、版本协商或整组事务确认，上板可能短时组合不同更新时间的字段。D3有源序号，但它用于热量数据校准，不能当作整个控制组序号。

可用于讲解：“板间协议把模式、目标、手动速率和热量拆成经典CAN帧，上板按ID解码到共享对象；状态反馈单独回传，在线、字段有效和执行成功分别判断。”
