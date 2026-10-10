# 下板板间通信发送与反馈接收

[返回下板模块索引](README.md) · [返回根协议速查](../../README.md#protocol--tuning) · [上板通信实现](../../task_up/docs/board-link.md)

## 总线和任务入口

下板 FDCAN2 PB5/PB6 与上板 bxCAN CAN2 相连，11-bit 标准 ID、经典 CAN、8-byte 数据帧。发送调度位于 `task_down/Application/TaskLayer/connect_task.c`，协议编码和上板反馈解码位于 `ProtocolLayer/board_protocol.c`。当前 `BOARD_COMM_DEBUG=1`、`BOARD_COMM_TX_ENABLE=1`。

| 报文 | 方向 | 生产者/消费者 | 当前配置 |
| --- | --- | --- | --- |
| D1 `0xD1` | 下→上 | 车状态与发射/升降请求 | 与 D2、D5 成组尝试，每 1 ms 调度 |
| D2 `0xD2` | 下→上 | 云台目标/状态目标字段 | 同 D1；多字节值大端映射 |
| D3 `0xD3` | 下→上 | 裁判热量快照 | 开启，10 ms；快照可复制时发送，无裁判数据也可能发flags=0 |
| D4 `0xD4` | 下→上 | 裁判血量透传 | `BOARD_COMM_D4_ENABLE=0`，不发送 |
| D5 `0xD5` | 下→上 | 遥控/键鼠云台手动命令 | 开启，随 D1/D2 成组调度 |
| D6 `0xD6` | 下→上 | 裁判逐发弹速快照 | 开启，新事件立即尝试，100 ms 重发心跳 |
| C1 `0xC1` | 上→下 | 上板电机在线与升降压缩状态 | 下板接收/更新心跳 |
| C2 `0xC2` | 上→下 | 上板云台机械角和 IMU 角 | 下板接收，供跟随/掉头/显示使用 |

字段布局和映射范围详见[根 README 协议表](../../README.md#protocol--tuning)；上下板细分数据路径见[上板通信页](../../task_up/docs/board-link.md)。下板不定义第二套不同版本的字节格式。

## FIFO 策略和发送失败含义

ConnectTask 每轮先判断 D3 是否到期，再检查 FDCAN2 FIFO 至少有 `control_slots=3` 个空位（D1、D2、D5）。空间不足时控制组整体延到下一轮，增加 `control_tx_defer_count`，避免只发一部分控制帧。D3 独立排期；发送失败不更新时间基准，后续循环继续重试。

`board.tx_01/tx_02/tx_05` 的 HAL 入队返回只代表本地驱动接受该帧。诊断应结合 D1/D2/D3 各自结果、FIFO 延后计数、上板 D1/D2 心跳和 C1/C2 接收状态，判断链路是否端到端可用。D1/D2 发送标志与 D3 统计是不同通道。

## D6 弹速与源状态

8 字节大端布局：b0–1 弹速（0.01 m/s），b2–3 uint16 逐发序号，b4–5 源年龄（ms），b6 弹丸类型，b7 发射机构编号。第一17mm机构配置为类型1、编号1，两板配置需一致。

- 下板 `0x0207` 仅在完整 CRC 帧且负载长度为 7 时解析；正有限弹速及合法类型/编号更新原子快照，源序号每发加一并按 uint16 回绕。
- 裁判在线、尚无样本时发送零弹速/零序号/零年龄首发心跳。年龄 65535 为源离线；在线旧样本年龄饱和至 65534，不冒充新样本。
- 控制组入队后，FIFO 空位大于 `control_slots` 才尝试 D6；成功后记录序号/时刻，失败留待下轮。100 ms 重发增加源年龄，不能触发上板重复学习。
- 上板另检查300 ms链路超时和1000 ms单发关联窗口。旧上板不识别D6，新上板缺少D6会禁止新发射，因此必须同步更新两板。

## D1、D2、D5 的生产关系

| 报文 | 数据来源 | 主要打包步骤 | 本板侧观测 |
| --- | --- | --- | --- |
| D1 | `board.tx_pkt->car_pkt` 与 `shoot_pkt` | b0 拼接车状态/云台模式/视觉/比赛/颜色；b1–4 将 vx/vy 映射到 [-8000,8000]；b5 bit0~3 放许可/模式/触发/过洞，bit4 放R掉头状态 | `gimbal_d1_tx_ok` 与上板 D1 心跳 |
| D2 | `gimbal_target_pkt` | Pitch/Yaw IMU 映射 [-360,360] deg；Pitch/Yaw 机械映射 [-4,4] rad；均为 uint16 大端 | `gimbal_d2_tx_ok`、上板 `gimbal_target_pkt` |
| D5 | UART5 遥控及键鼠映射 | 遥控摇杆经死区和最大角速度映射；键鼠路径编码来源、鼠标键及轴命令 | 下板 TX 状态、上板 `remote_cmd_pkt.valid/source/cmd_type` |

D1、D2、D5 在调度层作为一个三帧控制组检查 FIFO 空位；协议层分别执行编码与发送。字段值从本地状态更新到对端模块消费之间还隔着发送、接收、解析和有效性判断。

## D3 有效位生成

`Board_Tx_Pkt_03()` 先复制裁判热量快照；复制接口失败才不发送。当前对有效快照指针会返回复制成功，未收到裁判数据时也可发送flags=0。bit0仅当热量上限已见、非0且年龄小于1500 ms时置1；bit1仅当当前热量已见且年龄小于300 ms时置1。帧ID周期10 ms不等于其字段一定有效；有效flags是上板热量状态机的输入。

D4 关闭时，即便裁判 HP 数据更新，也不会通过 ConnectTask 透传。D4 当前上板代码只清心跳，因此启用 D4 也需明确同步消费逻辑和两端兼容性。

## 调试与故障定位

| 观察量 | 说明 |
| --- | --- |
| `board.status->gimbal_d1_tx_ok/gimbal_d2_tx_ok` | 最近一轮 D1/D2 HAL 发送结果 |
| `board.status->heat_d3_tx_ok`, `heat_d3_tx_ok_count` | 最近一帧及累计 D3 发送结果 |
| `heat_d3_tx_fail_count`, `heat_d3_tx_gap_ms/max_gap_ms` | D3 发送失败/成功间隔 |
| `control_tx_defer_count` | D1/D2/D5 因 FIFO 容量不足整体延后次数 |
| `board.rx_meg` / `board.status` | C1/C2 解码数据和上板状态 |
| `Board_HeartBeat`（上板） | 上板是否实际收到关键 D 帧 |

### 端到端链路判定

```text
本地源数据更新 → 编码函数返回 → FDCAN FIFO 入队 → 总线仲裁/发送
             → 上板 ID 分发 → 解码字段与 flags → 模块状态实际消费
```

每一步对应不同观测量：本地下板看 `tx_pkt` 与 TX 结果；总线可观测帧 ID、长度和原始字节；上板看 `Board_Rx_Info`；控制行为看消费模块 Watch。单看 `gimbal_d1_tx_ok=1` 只能确认本地 HAL 接受发送请求，不能跳过后续链路步骤。

排查顺序：核实两端总线引脚/收发器与终端，检查 FDCAN2/CAN2 的标准 ID 和位时序，再检查 filter 命中、发送 FIFO、D3 snapshot flags，最后检查对端回传与心跳。任何协议字段调整需同步上板解析、调试 Watch 和根 README 表。

## 按函数讲解发送和反馈

| 文件与函数 | 作用 | 下游 |
| --- | --- | --- |
| [control_task.c](../Application/TaskLayer/control_task.c) `Board_Debug_Gimbal_Command()` / `Board_Debug_Hole_Command()` | 更新云台机械目标、R状态和过洞请求 | D1/D2 |
| [launch.c](../Application/ModuleLayer/launch.c) `Launch_Cmd_Transmit()` | 写发射许可、模式和触发 | D1 |
| [board_protocol.c](../Application/ProtocolLayer/board_protocol.c) `Board_Tx_Pkt_01/02/03/05()` | 各帧独立打包与发送 | 上板共享解码对象 |
| [connect_task.c](../Application/TaskLayer/connect_task.c) `StartConnectTask()` | D3优先排期、控制组容量判断、延后/重试 | FDCAN2 FIFO |
| [can_protocol.c](../Application/ProtocolLayer/can_protocol.c) `CAN2_rxDataHandler()` | C1/C2标准ID分发 | `Board_Rx_Meg_01/02()` |
| `Board_Rx_Meg_01/02()` | 解码电机/升降状态与角度 | `board.rx_meg`、C2有效位/时间戳 |
| [chassis_input.c](../Application/ModuleLayer/chassis_input.c) `Chassis_Input_NotifyUturnTxCycle()` | 记录掉头阶段的成功控制交接 | PREPARE/RESTORE的转移条件 |

### D5当前编码格式

当前下板键鼠路径把鼠标X/Y映射为角速度，再编码有符号int16；增益Yaw=5、Pitch=-3 (deg/s)/count，上限分别200、150 deg/s，量化步长0.1 deg/s/LSB。机械键鼠模式中鼠标X主要交给底盘，掉头动作也会影响Yaw透传；不能把所有档位都解释为同一个鼠标到云台映射。

协议支持 `cmd_type=鼠标增量`，但功能分支存在不代表当前发送该格式。讲解时以 `Board_Tx_Pkt_05()`实际写入的source/type和值为准。

### 成组容量检查不等于事务提交

预留3个空位减少控制组因FIFO容量不足而拆分的机会，但三帧分别调用HAL，不存在原子组提交或整组确认。某帧HAL失败时可能仍有其他帧入队；D1/D2成功标志用于下板交接逻辑，也不证明上板已消费。D3失败则保持旧成功时间并在后续任务轮次重试，统计成功间隔反映入队间隔而非裁判数据更新间隔。

可用于讲解：“下板独立调度控制组和热量帧，分别处理容量不足与发送失败；上板反馈用于状态和闭环，但每个字段仍需检查来源、有效位和年龄。”
