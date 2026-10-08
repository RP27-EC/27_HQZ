# 上板发射机构与热量预算

[返回上板模块索引](README.md) · [返回上板工程说明](../Readme.md) · [裁判链路](../../task_down/docs/referee.md) · [板间协议](board-link.md)

## 职责与许可

- 下板读取遥控/键鼠，D1 传递发射许可、模式、触发与过洞请求；S2 消抖及档位映射不变。
- 上板控制两台 RM3508 摩擦轮和 KT4005 拨盘；裁判热量通过 D3 同步。D1/D3 字节布局不变。
- `launcher_dial.hold_allowed`：板间在线、整车使能、拨盘在线、编码器基准有效、无发射机构故障，且过洞暂停解除时保持。
- `feed_allowed`：另需摩擦轮总使能、两轮在线、D1 发射许可有效且过洞请求为零。热量在请求接受时另行检查，不能凭该字段为 1 判断一定会供弹。
- 摩擦轮关闭、S2 中位、热量不足时继续保持；`is_hole=1` 时拨盘停机，退出请求后等待 2000 ms 恢复保持，不依赖升降到位或 `lift.fault_code`。下板发射许可仍独立约束供弹。
- 失能后重新接管当前位置，不追赶旧目标；正常回中位、摩擦轮切换不重采样。

`launcher` 保留原观测入口；`launcher_dial` 为上板本地独立状态，不发送到 CAN。摩擦轮休眠时 `launcher.state` 可为 `SLEEP`，但拨盘仍可能为 `launcher_dial.state=READY`。

保持是有限力矩闭环，允许 100 count 死区，不能保证完全拨不动；死区内速度目标为零，速度环仍提供阻尼。

待发保持的 PID 仍每个控制周期计算。进入保持时立即发送首帧，位置误差不在 100 count 死区内或速度超过 20°/s 时，力矩按 1 ms 间隔发送；误差进入死区且速度不超过 20°/s，连续 100 ms 后才降为 10 ms。单发、连发、制动和摩擦轮/升降 `0x200` 发送周期不变。

## 拨盘阶段

| `launcher_dial.state` | 行为 |
| --- | --- |
| `SLEEP` | 卸力；有效保持许可恢复后接管当前位置 |
| `READY` | 独立待发 PID 保持累计目标 |
| `SINGLE` | 原目标增加 65536 count，释放触发仍完成本发 |
| `REPEAT` | 原连发速度环；释放或许可/热量失效后制动 |
| `STOPPING` | 零速制动；速度不超过 20°/s 或达到 120 ms 后锁定当前位置 |
| `REVERSE/RELOAD` | 可选堵转退让，当前宏关闭 |
| `FAULT` | 故障卸力，禁止保持与供弹 |

- KT 首个反馈建立累计编码器基准；65536 count 为电机编码器一圈，机构位移仍取决于传动。
- 单发完成需误差不超过 500 count、速度不超过 20°/s，连续 20 ms；保持原目标，清除供弹积分。
- 500 ms 未完成：增加 `timeout_count`，记录原因 6，停止追旧目标，制动后保持停止位置，不自动补射。
- 总发射许可撤销、热量基准失效或热量超过上限会中止供弹；单发预占引起的普通热停发锁不打断本发。
- 启动发送失败只保留一个已通过许可检查且预占热量的请求，最长 50 ms；启动成功才增加目标，超时或许可撤销取消。
- 忙碌、热量不足时拒绝新升沿，不排队；拒绝、超时、中断以及连发热停发后均需释放触发再供弹。
- 未执行或中止单发的热量预占不退回，等待裁判校准或冷却。
- `SPINUP/INIT` 保留枚举兼容值，主路径不进入；无效的固定归零与待发停机配置已删除。

## 配置

入口：`task_up/Application/ConfigLayer/launcher_config.h`。公共 PID 算法未修改。

| 参数 | 值 | 含义 |
| --- | ---: | --- |
| `LAUNCHER_FRIC_TARGET_RPM` | 1500 | 摩擦轮目标 rpm |
| `LAUNCHER_FRIC_READY_TOL_RPM / READY_TIME_MS` | 500 / 100 | 达速误差 rpm / 确认 ms |
| `LAUNCHER_DIAL_ANGLE_KP` | 0.08 | 单发位置增益，(deg/s)/count |
| `LAUNCHER_DIAL_SPEED_KP / KI / KD` | 0.15 / 0.05 / 0 | 单发速度环，原参数 |
| `LAUNCHER_DIAL_HOLD_ANGLE_KP / HOLD_DEADBAND` | 0.04 / 100 | 待发位置增益 / count 死区 |
| `LAUNCHER_DIAL_HOLD_SPEED_KP / KI / KD` | 0.15 / 0 / 0 | 待发速度阻尼，无积分 |
| `LAUNCHER_DIAL_SETTLE_SPEED_DPS / SETTLE_TIME_MS` | 20 / 20 | 到位速度 deg/s / 确认 ms |
| `LAUNCHER_DIAL_START_TIMEOUT_MS / SINGLE_TIMEOUT_MS` | 50 / 500 | 启动等待 / 单发时限 ms |
| `LAUNCHER_DIAL_REPEAT_SPEED_DPS` | 6000 | 连发机械速度上限 deg/s，另受热量限制 |
| `LAUNCHER_DIAL_SPEED_OUT_MAX / REPEAT_OUT_MAX` | 1500 / 1500 | 单发/连发电流原始量限幅 |
| `LAUNCHER_DIAL_CURRENT_LIMIT` | 2000 | 总电流原始量限幅，不是 A |
| `LAUNCHER_DIAL_BRAKE_KP / BRAKE_TIMEOUT_MS` | 0.1 / 120 | 制动增益 / ms |
| `LAUNCHER_DIAL_JAM_ENABLE` | 0 | 堵转退让关闭 |
| `LAUNCHER_DIAL_HOLE_RELEASE_DELAY_MS` | 2000 | `is_hole` 从 1 变 0 后恢复保持的延时，ms |
| `LAUNCHER_DIAL_HOLD_TX_INTERVAL_MS` | 10 | 稳态保持力矩发送间隔，ms，至少 1 |
| `LAUNCHER_DIAL_HOLD_ACTIVE_TX_MS` | 1 | 保持运动或纠偏时发送间隔，ms，至少 1 |
| `LAUNCHER_DIAL_HOLD_IDLE_CONFIRM_MS` | 100 | 误差与速度满足稳态条件的确认时间，ms |

`fric_ready` 仍为观测值，未新增达速硬门槛；保持参数是人工验证起点，单发/连发力度未增加。

## 热量预算

- D3 保留上限、枪管热量、冷却率、源序号及有效位；新源序号校准本地估计。
- 初始预算未建立时禁发；D3 失联后沿用已有参数估算，不将失联解释为零热量。
- 每发预占 10 热量单位，安全余量 20，恢复余量 30，最高射频 15 发/s。
- 降速起点 200、平衡区起点 50；D3 新鲜度为 100 ms；训练开关及训练上限/冷却率均为 0。
- 查看 `launcher_heat.source/ready/blocked/remaining/target_rate` 区分预算拒绝与机械无响应。

## 无响应观测

| `launcher_dial` 字段 | 含义 |
| --- | --- |
| `state / hold_allowed / feed_allowed / trigger_ready` | 独立阶段、许可、是否已释放触发 |
| `target_angle / target_error / speed_target_dps` | 目标 count、残差 count、速度目标 deg/s |
| `output_current_raw` | 最近力矩指令原始量；失能时显示零指令 |
| `run_tx_status / torque_tx_status / stop_tx_status` | 最近发送结果：HAL 0 成功、1 错误、2 忙、3 超时 |
| `accepted_count / rejected_count / completed_count` | 接纳/拒绝/到位单发数；接纳包含等待启动 |
| `timeout_count / start_timeout_count` | 单发未完成 / 启动超时次数 |
| `pending_single / pending_tick` | 等待启动标志及起点 ms |

`reject_reason`：0 无拒绝，1 热量，2 忙碌，3 通信/电机离线，4 失能/互锁/未释放触发，5 启动超时，6 单发未完成。原因保留到下次接纳；原因 6 计入 `timeout_count`，不增加拒绝请求计数。

HAL 成功仅表示 CAN 驱动接纳发送，不能证明电机执行。目标未变化且拒绝数增长先查许可；目标变化但反馈位移很小，再由人工检查发送结果、电流与机械阻力。

## 人工验收

1. 确认 S2 中位、摩擦轮关闭时保持；过洞请求期间停机，退出请求 2000 ms 后恢复保持，即使升降故障码为 5；失能或断联时卸力。
2. 确认单发累计目标、释放后完成本发，观察到位振荡是否减轻。
3. 确认启动等待 50 ms、单发超时 500 ms 后不补射、不追旧目标。
4. 确认许可或冷却恢复不自动补射，释放并重新触发后才供弹。
5. 无响应时记录拨盘观测、热量状态及电机位置/速度/电流反馈。

本次未编译、未烧录、未做实机验证；静态检查不证明硬件效果。
