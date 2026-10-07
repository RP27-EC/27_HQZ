# 下板板间通信发送与反馈接收

[返回下板模块索引](README.md) · [返回根协议速查](../../README.md#protocol--tuning) · [上板通信实现](../../task_up/docs/board-link.md)

## 总线和任务入口

下板 FDCAN2 PB5/PB6 与上板 bxCAN CAN2 相连，11-bit 标准 ID、经典 CAN、8-byte 数据帧。发送调度位于 `task_down/Application/TaskLayer/connect_task.c`，协议编码和上板反馈解码位于 `ProtocolLayer/board_protocol.c`。当前 `BOARD_COMM_DEBUG=1`、`BOARD_COMM_TX_ENABLE=1`。

| 报文 | 方向 | 生产者/消费者 | 当前配置 |
| --- | --- | --- | --- |
| D1 `0xD1` | 下→上 | 车状态与发射/升降请求 | 与 D2、D5 成组尝试，每 1 ms 调度 |
| D2 `0xD2` | 下→上 | 云台目标/状态目标字段 | 同 D1；多字节值大端映射 |
| D3 `0xD3` | 下→上 | 裁判热量快照 | 开启，10 ms；无有效裁判快照时本轮不发 |
| D4 `0xD4` | 下→上 | 裁判血量透传 | `BOARD_COMM_D4_ENABLE=0`，不发送 |
| D5 `0xD5` | 下→上 | 遥控/键鼠云台手动命令 | 开启，随 D1/D2 成组调度 |
| C1 `0xC1` | 上→下 | 上板电机在线与升降压缩状态 | 下板记录有效标志/接收时刻；许可有效期50 ms |
| C2 `0xC2` | 上→下 | 上板云台机械角和 IMU 角 | 下板接收，供跟随/掉头/显示使用 |

字段布局和映射范围详见[根 README 协议表](../../README.md#protocol--tuning)；上下板细分数据路径见[上板通信页](../../task_up/docs/board-link.md)。下板不定义第二套不同版本的字节格式。

## FIFO 策略和发送失败含义

ConnectTask 每轮先判断 D3 是否到期，再检查 FDCAN2 FIFO 至少有 `control_slots=3` 个空位（D1、D2、D5）。空间不足时控制组整体延到下一轮，增加 `control_tx_defer_count`，避免只发一部分控制帧。D3 独立排期；发送失败不更新时间基准，后续循环继续重试。

`board.tx_01/tx_02/tx_05` 的 HAL 入队返回只代表本地驱动接受该帧。诊断应结合 D1/D2/D3 各自结果、FIFO 延后计数、上板 D1/D2 心跳和 C1/C2 接收状态，判断链路是否端到端可用。D1/D2 发送标志与 D3 统计是不同通道。

## D1、D2、D5 的生产关系

| 报文 | 数据来源 | 主要打包步骤 | 本板侧观测 |
| --- | --- | --- | --- |
| D1 | `board.tx_pkt->car_pkt` 与 `shoot_pkt` | b0 拼接车状态/云台模式/视觉/比赛/颜色；b1–4 将 vx/vy 映射到 [-8000,8000]；b5 放许可/模式/触发/过洞 | `gimbal_d1_tx_ok` 与上板 D1 心跳 |
| D2 | `gimbal_target_pkt` | Pitch/Yaw IMU 映射 [-360,360] deg；Pitch/Yaw 机械映射 [-4,4] rad；均为 uint16 大端 | `gimbal_d2_tx_ok`、上板 `gimbal_target_pkt` |
| D5 | UART5 遥控及键鼠映射 | 遥控摇杆经死区和最大角速度映射；键鼠路径编码来源、鼠标键及轴命令 | 下板 TX 状态、上板 `remote_cmd_pkt.valid/source/cmd_type` |

D1、D2、D5 在调度层作为一个三帧控制组检查 FIFO 空位；协议层分别执行编码与发送。字段值从本地状态更新到对端模块消费之间还隔着发送、接收、解析和有效性判断。

## 升降许可与掉头交接

- C1 byte1编码不变：0下端/堵转停止，1等待/运动/顶部许可未满足，2顶部就绪，3故障。上板状态码2要求 `Lift_IsReadyUp()` 成立，不再将初始化 `LIFT_WAIT` 视为顶部。
- 下板 `state_data_valid/state_rx_time_ms` 独立记录C1接收；发射、小陀螺和普通掉头要求C1年龄≤50 ms、升降电机在线且状态码2。C2到达不能刷新C1许可。
- 遥控和键鼠跟随掉头共用D1/D2/D5完整入队计数；掉头期间D5 Yaw为0。升降接管期间也屏蔽D5 Yaw，避免手动输入与回正冲突。
- CAN ID、8字节布局及角度映射不变；上下板需配套更新顶部状态语义和本地互锁。

升降目标变化通过本地序号保留一次非顶部C1回报，成功入队后才允许回报顶部。下板取消已入队的下降命令后，先观察非顶部C1计数变化，再接受后续新顶部C1；从未入队的下降可直接取消。这个确认过程不新增报文字段，也不把HAL入队成功等同于对端已收到。

## D3 有效位生成

`Board_Tx_Pkt_03()` 先取得裁判热量快照；没有快照就不发送。bit0 仅当热量上限已见、非 0 且年龄小于 1500 ms 时置 1；bit1 仅当当前热量已见且年龄小于 300 ms 时置 1。帧 ID 周期 10 ms 不等于其字段一定有效；有效 flags 是上板发射状态机的关键输入。

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
