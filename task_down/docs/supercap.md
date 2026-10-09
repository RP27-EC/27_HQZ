# 下板超电通信与当前运行边界

[返回下板模块索引](README.md) · [底盘功率限制](chassis.md) · [通信总线](board-link.md)

## 当前用途和拓扑

当前只保活并读取超电反馈，不把超电作为底盘功率闭环执行器。`SUPERCAP_BRINGUP_ENABLE=1` 时设备对象在 `DeviceLayer/supercap.c` 初始化；CtrlTask 调 `SuperCap_Tx()`，FDCAN1 接收 `0x211` 更新反馈；发送控制帧 ID 为 `0x222`。超电与 RM3508 底盘电机共用 FDCAN1。

| 宏（`task_down/Application/ConfigLayer/supercap_config.h`） | 当前值 | 效果 |
| --- | ---: | --- |
| `SUPERCAP_BRINGUP_ENABLE` | 1 | 初始化/保活通信路径 |
| `SUPERCAP_OFFLINE_TIMEOUT_MS` | 100 ms | 超过时间未收反馈转为 OFFLINE |
| `SUPERCAP_CAP_SWITCH` | 0 | 超电开关关闭 |
| `SUPERCAP_TURBO_MODE` | 0 | 增强模式关闭 |
| `SUPERCAP_PRE_CHARGE_ENABLE` | 0 | 预充命令关闭 |
| `SUPERCAP_POWER_BUFFER` | 0 | 控制帧缓冲字段为 0 |
| `SUPERCAP_POWER_LIMIT` | 0 W | 发送功率限制字段为 0 |
| `SUPERCAP_POWER_OUT_LIMIT/IN_LIMIT` | 0 | 输出/输入限制值为 0 |

## CAN 字节布局

经典 8-byte 标准帧，协议声明多字节字段为小端。解码与编码见 `task_down/Application/ProtocolLayer/supercap_protocol.c`。

| ID/方向 | Byte | 定义 |
| --- | --- | --- |
| `0x211` 反馈 | 0–1 | `chassis_power` int16，小端；工程注释标“计数，量纲待确认” |
| | 2–3 | `voltage_raw` int16，小端；线性映射到 0–25 V |
| | 4–5 | `current_raw` int16，小端；线性映射到 -16–16 A |
| | 6 bit0/bit1 | `ability` / `pre_charge_mode`；byte 7 未消费 |
| `0x222` 控制 | 0 | `power_buffer` uint8，J 字段 |
| | 1–2 | `power_limit` uint16，小端，W |
| | 3–4 | `power_out_limit` int16，小端，驱动计数域 |
| | 5–6 | `power_in_limit` uint16，小端，驱动计数域 |
| | 7 bit0–2 | cap switch、turbo mode、pre-charge enable |

接收值 `voltage_raw/current_raw` 通过 `SuperCap_Scale()` 映射；底盘功率原始字段保留为计数，不能直接写成瓦特。具体换算范围是当前固件软件映射，不是已校准的模块精度指标。

线性缩放按固件映射将电压原始域转换到 0–25 V、将电流原始域转换到 -16–16 A；该映射只说明程序如何显示/使用数据，不代表模块的 ADC 精度、校准系数或供电边界。`ability` 与 `pre_charge_mode` 来自 byte 6 的 bit0/bit1，byte 7 当前未被解码。

## 与底盘功率限制的关系

`Chassis_Control_Update()` 将超电功率、电压、电流、ability 和在线标志写入 `Power_Limit_SetCapFeedback()`，用于 `power_limit_state` 观察。当前 `Power_Limit_GetTarget()` 使用裁判上限、buffer energy 与 45 W 回退配置；`Power_Limit_Apply()` 按电机模型缩放四轮输出。超电反馈不会参与目标预算或闭环计算。

所以超电收到 `0x211`、状态显示 ONLINE、或发送 `0x222` 成功，都不能说明电容已使能或提供底盘功率。这里不提供超电控制启用步骤；任何启动/预充/输出配置变更都要先核对模块协议、电气条件和人工批准的系统设计。

## 观察和排查

查看 `supercap.state`、`rx_count`、`tx_count`、`last_rx_ms`、`offline_count`、`feedback.ability/pre_charge_mode` 和映射后的 `cap_voltage/cap_current`。掉线判据为最近一次反馈年龄达到 100 ms；心跳状态恢复仍需后续有效 RX 帧。

若没有反馈，按 FDCAN1 时钟/位时序、滤波器、线序/终端和标准 ID 排查；若仅超电节点异常，不要先改底盘 0x200 电机协议。使用示波器/电流探头的物理量验证由台架人员完成，软件映射值只能作为诊断记录。

### 控制帧当前有效字段

| 字节 | 编码源 | 当前配置结果 |
| --- | --- | --- |
| b0 | `power_buffer` | 配置为 0 J 字段值 |
| b1–2 | uint16 little-endian `power_limit` | 当前配置为 0 W |
| b3–4 | int16 little-endian `power_out_limit` | 当前配置为 0 raw |
| b5–6 | uint16 little-endian `power_in_limit` | 当前配置为 0 raw |
| b7 | bit0 cap switch、bit1 turbo、bit2 pre-charge | 三个位当前全为 0 |

因此现有 0x222 是关闭输出项的配置帧；确认总线有 0x222 只能证明通信尝试，不能推断超电开关或预充已经打开。

## 按函数讲解当前实现

| 文件与函数 | 输入 → 输出 | 当前意义 |
| --- | --- | --- |
| [supercap.c](../Application/DeviceLayer/supercap.c) `SuperCap_Init()` | 零初值 → 设备/统计对象 | 初始OFFLINE，等待反馈 |
| 同文件 `SuperCap_Tx()` | 配置宏 → 0x222控制帧 | CtrlTask每轮尝试发送；当前控制字段全零 |
| `SuperCap_Rx()` | 0x211 → raw反馈、缩放值、时间戳 | 收帧更新 `rx_count/last_rx_ms` 并置ONLINE |
| `SuperCap_Heartbeat()` | 最近接收年龄 → 状态 | 年龄达到100 ms转OFFLINE；不是放电状态判定 |
| [supercap_protocol.c](../Application/ProtocolLayer/supercap_protocol.c) `SuperCap_Protocol_Decode/Encode()` | 字节 ↔ 协议结构 | 小端字段，与板间大端协议不同 |
| [power_limit.c](../Application/AlgorithmLayer/power_limit.c) `Power_Limit_SetCapFeedback()` | 超电观测量 → 功率状态 | 记录观测，不参与目标预算/公共比例计算 |

### 统计值应该怎样解释

`SuperCap_Tx()` 调用发送后直接增加 `tx_count`，没有根据HAL返回过滤，因此该计数表示发送尝试次数，不能称为成功次数。`rx_count`和`last_rx_ms`表示程序收到并解码对应反馈，也不能确认电容已输出能量。

`offline_count`有辅助计数，但真实离线转移依据 `HAL_GetTick()-last_rx_ms>=100` ms。上电OFFLINE时心跳函数不会凭计数自行转ONLINE，恢复由接收路径触发。

可用于讲解：“超电模块当前只完成协议收发、在线维护和观测，输出功能宏关闭；功率限制使用裁判缓冲和电机模型，尚未把超电接入执行闭环。”
