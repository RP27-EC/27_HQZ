# 下板裁判系统数据链

[返回下板模块索引](README.md) · [底盘功率限制](chassis.md) · [发射热量控制](../../task_up/docs/launcher.md) · [板间协议](board-link.md)

## 硬件接收和解析

USART1 RX/TX 为 PA10/PA9，115200 baud、8N1；接收使用 DMA 和空闲线回调。驱动回调进入 `task_down/Application/ProtocolLayer/judge_protocol.c`：扫描帧头 `0xA5`，校验 5-byte 帧头 CRC8、按帧长校验 CRC16，再按 command ID 调 `Judge_Data_Update()`。仅通过完整校验的有效命令更新设备数据与时间戳。

| 文件 | 职责 |
| --- | --- |
| `task_down/Core/Src/usart.c` | USART1 引脚、波特率、DMA/IRQ 硬件初始化 |
| `task_down/Application/DriverLayer/drv_uart.c` | RX DMA 缓冲与 IDLE 后的回调分发 |
| `task_down/Application/ProtocolLayer/judge_protocol.c` | 帧同步、长度、CRC8/CRC16 与命令 ID 解析 |
| `task_down/Application/DeviceLayer/judge.c/.h` | 结构体更新、在线状态、热量/功率快照接口 |
| `task_down/Application/ProtocolLayer/board_protocol.c` | D3 热量帧、D4 血量帧打包 |
| `task_down/Application/TaskLayer/connect_task.c` | 定时发送 D3/D4 |

## 本工程读取的数据

| 命令 ID | 解析来源 | 使用字段 | 下游 |
| --- | --- | --- | --- |
| `0x0201` | robot status | 机器人 ID、17 mm 枪管热量上限/冷却率、底盘功率上限 | D3 参数有效性/热量预算；底盘功率预算；车队颜色状态 |
| `0x0202` | power/heat | 17 mm 1 号枪管热量、buffer energy | D3 当前热量；`Judge_GetPowerSnapshot()` 的缓冲能量 |
| `0x0003` | robot HP | 队伍血量字段 | D4 当前默认关闭；启用时用于血量透传 |

裁判设备 `judge.status` 只是最近有效帧更新的在线状态。底盘功率快照要求 `judge_online`、功率上限已见、buffer 帧已见，且两个时间戳均小于 `JUDGE_OFFLINE_CNT_MAX=1000 ms`。仅看到设备在线不代表 limit/buffer 两类数据都新鲜。

### 两个核心命令的数据消费表

| command ID | 本工程关心的字段 | 写入的快照 | 失效判断/消费者 |
| --- | --- | --- | --- |
| `0x0201` | chassis power limit、17 mm 1 号枪管 cooling rate 与 heat limit、robot ID/color | `judge_heat_data` 参数字段和 `judge_power_data` limit 字段 | D3 bit0 有独立 1500 ms 有效期；功率路径同时要求 judge online 与 limit 时间戳新鲜 |
| `0x0202` | chassis buffer energy、17 mm 1 号枪管 heat | `judge_power_data` buffer 与 `judge_heat_data` 当前热量 | D3 bit1 使用 300 ms 有效期；功率快照还要求 buffer tick 小于 1000 ms |

即使 CRC 正确，字段若未见过、为 0 或更新时间过旧也不能视为有效。定位 D3 时分别跟踪裁判字段更新、快照 `seen/tick`、打包 flags 和下板发送计数；定位功率回退时单独跟踪 limit 与 buffer 两个来源。

## 热量与功率是两条分开的消费路径

热量：`Judge_GetHeatSnapshot()` → `Board_Tx_Pkt_03()` → 下板 10 ms 发送 → 上板验证 D3 flags/接收年龄后更新热量源。当前 `BOARD_COMM_D3_ENABLE=1`；`BOARD_HEAT_LIMIT_TIMEOUT_MS=1500 ms`、`BOARD_HEAT_VALUE_TIMEOUT_MS=300 ms` 用于下板 D3 有效性。

底盘功率：`Judge_GetPowerSnapshot()` → `Power_Limit_GetTarget()` → 四轮候选力矩经比例限幅。当前开关 `CHASSIS_POWER_LIMIT_ENABLE=1`；快照不满足新鲜度/范围条件时使用 45 W 固定回退预算。详细 PI/缓冲能量机制见[底盘功率限制](chassis.md)。D3 的发射热量字段不能替代底盘 `power_limit_state`。

配置宏 `BOARD_JUDGE_ENABLE=0` 当前没有守护 USART1 初始化或裁判解析调用；它不能代表本配置下裁判 UART 不运行。以 `DRIVER_Init()`、`USART1_Init()` 和 RX 回调实际执行路径为准。

## 数据和故障定位

| 现象 | 观察点 |
| --- | --- |
| `judge.status` 离线 | USART1 配置、RX DMA 状态、idle 回调、接收缓冲和帧头同步 |
| 帧进入但值不更新 | frame length、CRC8/CRC16 计数、command ID、裁判数据版本与字段布局 |
| D3 flags 不完整 | `judge_heat_data.limit_seen/heat_seen`、`limit_tick/heat_tick`、`BOARD_HEAT_*_TIMEOUT_MS` |
| 功率限幅回退 | `judge_power_data.limit_seen/buffer_seen`、两类更新时间戳、`power_limit_state.fallback_used` |
| D3 未发出 | `BOARD_COMM_D3_ENABLE`、ConnectTask 排期、CAN TX 结果计数 |

裁判 UART 侧通常为裁判系统电气接口，不能与 CAN 收发器或普通 TTL 线序混用；依据实际裁判系统接口标准核实电平、地线与供电。CRC 通过只是帧格式正确，不证明字段值符合比赛场景或功率测量准确。

## 按函数讲解解析与快照

| 文件与函数 | 作用 | 消费者 |
| --- | --- | --- |
| [judge_protocol.c](../Application/ProtocolLayer/judge_protocol.c) `USART1_rxDataHandler()` / `judge_receive()` | 处理接收缓冲、寻找帧头、检查CRC和命令 | `judge.rx`绑定的更新函数 |
| [judge.c](../Application/DeviceLayer/judge.c) `Judge_Data_Update()` | 按ID更新结构、seen、tick与序号 | 热量/功率快照、车状态 |
| 同文件 `Judge_GetHeatSnapshot()` | 临界区复制热量结构 | D3打包；返回成功表示复制完成，不等于字段有效 |
| 同文件 `Judge_GetPowerSnapshot()` | 临界区复制并校验在线和两个更新时间 | 下板功率预算 |
| [board_protocol.c](../Application/ProtocolLayer/board_protocol.c) `Board_Tx_Pkt_03()` | 按不同有效期生成flags，打包热量 | 上板 `Launcher_HeatUpdate()` |

### 为什么用快照而不是逐字段读取

裁判字段由接收路径更新，控制任务读取时若跨越一次更新，可能组合出旧上限、新缓冲或不同序号的数据。两个快照接口保存原PRIMASK、短暂关中断完成复制后恢复原中断状态，避免单次复制被接收中断撕裂。它保证读取时的一致性，不保证不同裁判命令原本就来自同一时刻。

### 零值、在线和有效性的区别

- 枪管热量为0可能合法；D3热量有效位检查 `heat_seen` 与年龄，不能因热量为0判失效。
- 热量上限为0则不建立参数有效位；功率上限还由功率模块检查20～120 W范围。
- `Judge_GetHeatSnapshot()` 对有效指针返回复制成功，即便没有裁判数据也可得到全零初值；D3当前可以发送flags=0的帧。它不代表裁判有效，只能保证对端得到明确无效标志。
- `heat_seq`区别新的裁判热量更新与10 ms重复转发；`buffer_seq`控制功率积分只随新缓冲帧更新。序号不代表CAN发送次数。

可用于讲解：“裁判模块先校验帧格式，再维护带时间戳和序号的字段快照；热量和底盘功率分别判断有效性，避免用设备在线代替数据新鲜。”
