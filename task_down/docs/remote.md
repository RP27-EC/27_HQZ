# 下板遥控器与键鼠输入

[返回下板模块索引](README.md) · [底盘控制](chassis.md) · [下板通信](board-link.md)

## 接收与解析链

接收端是 UART5 RX PD2，DBUS 配置为 100000 baud、8E2；DMA 与空闲线处理由 `task_down/Application/DriverLayer/drv_uart.c` 完成。`ProtocolLayer/rc_protocol.c` 解码 18-byte DBUS 数据，`DeviceLayer/Sensor/rc_sensor.c` 更新 `rc_dev`、拨杆边沿及在线状态。CommandTask 每轮调用解析和键鼠状态更新，底盘任务消费统一的 `rc_data_t`。

| DBUS 输入 | 用途 |
| --- | --- |
| `ch0` | 右摇杆左右；底盘转向/小陀螺调节 |
| `ch1` | 右摇杆上下；云台/遥控相关通路 |
| `ch2` | 左摇杆左右；底盘横移 |
| `ch3` | 左摇杆上下；底盘前后 |
| `s1`, `s2` | 模式选择和输入源限制；键鼠不绕过遥控在线条件 |
| `mouse_x/y`, 鼠标键 | 云台手动输入及机械底盘转向等相应模式功能 |
| `key_v` | WASD/QE、Shift/Ctrl、F/Z/X/C/R/G 等按键位图 |

DBUS 通道原始中心值由协议层换算；上层死区/满量程参数位于 `task_down/Application/ConfigLayer/chassis_config.h`。当前 `CHASSIS_RC_INPUT_ENABLE=1`、`CHASSIS_KEYBOARD_INPUT_ENABLE=1`、`CHASSIS_OWNS_RC_YAW=1`。

## DBUS 解码值的使用边界

DBUS 的 18 字节帧先由协议层还原四个摇杆通道和两个拨杆，再由设备层维护在线/边沿状态；底盘模块消费的是处理后的通道而不是 UART 字节。调试时可以按以下顺序检查：

1. UART5 DMA 缓冲是否持续被 IDLE 回调更新，解析器是否识别完整 18 字节帧。
2. `rc_dev.work_state` 是否在线，通道是否接近中位且在预期方向变化。
3. S1/S2 的枚举值是否与当前输入源条件匹配；F 必须在允许档位产生有效按下沿。
4. `keyboard_source_active` 是否切换；再检查 `chassis_input_cmd` 的来源、限幅和 valid。

遥控离线处理会复位键鼠选择和 R 掉头状态；键鼠不是独立无线链路，即便按键位图仍残留，也不应被解释成新的有效运动命令。

## 输入源与操作模式

F 键仅在遥控在线且 S1 上位时以按下沿翻转键鼠输入源。遥控掉线会清零命令、复位键鼠选择和 R 掉头状态；不能把键鼠按键缓冲当作独立于 DBUS 的安全控制器。

| 键/拨杆 | 当前代码行为 | 生效限制 |
| --- | --- | --- |
| W/S、A/D | 前后/左右平移 | `CHASSIS_MAX_VX/VY` 控制域限幅 |
| Q/E | 旋转方向输入 | 按 `CHASSIS_MAX_WZ` 与速度倍率 |
| Shift / Ctrl | 速度倍率 1.5 / 0.5 | Shift 优先于 Ctrl |
| Z / X / C | 跟随 / 机械 / 小陀螺 | 仅键鼠源；由上板 C2/Yaw 反馈影响特定模式 |
| R | 机械模式切换前/后基准；跟随模式触发云台掉头状态机 | 依赖有效 C2 反馈和通信交接 |
| 鼠标 X（机械模式） | 底盘角速度附加项 | gain=10，输出裁剪至 ±15 控制域单位 |

普通遥控摇杆模式由 `S1` 档位和 `CHASSIS_OWNS_RC_YAW` 决定 vx/vy/wz。跟随与小陀螺是否接管旋转在各自模块中仲裁；请看 `chassis_cmd_t.source`，不要仅凭拨杆位置判断最终命令来源。

## 关键判据和观察量

| 变量 | 检查内容 |
| --- | --- |
| `rc_dev.work_state` | DBUS 设备是否在线 |
| `rc_dev.info->ch0..ch3` | 摇杆中心、方向和限幅前输入 |
| `rc_dev.info->s1/s2` | 档位原始值和边沿状态 |
| `rc_dev.info->key_v` | 当前按键位图；核对 F/Z/X/C/R 的上升沿和按住状态 |
| `keyboard_source_active` / keyboard chassis mode | 输入源切换及 Z/X/C 模式选择 |
| `chassis_input_cmd` | 经映射、限幅后的统一底盘指令及有效标志 |
| `uturn_state/result` | R 掉头阶段、反馈时间戳、稳定时长和超时结果 |

先检查 UART5 DMA/IDLE 收到连续帧，再检查 DBUS 通道中心与拨杆编码，最后检查键鼠映射与 chassis input 输出。若遥控在线但所有按键无效，优先核实 F 是否在允许档位产生边沿；若命令突然归零，检查在线状态、C2 超时、跟随故障锁存和源仲裁。

遥控接收失效会使底盘指令失效。初次通电测试前架空底盘，先单独确认摇杆方向与键位；不要把 Shift 倍速、旧键盘状态或心跳在线视为底盘允许运动的充分条件。

### D5 与本地底盘输入的分流

遥控/键鼠数据在下板既参与 chassis input，也通过 D5 提供上板云台输入。D5 的键鼠来源位用于区分鼠标增量和角速度格式；下板底盘键位（WASD/QE、Z/X/C、R）不应误认为都由 D5 透传。追云台输入问题时走 [D5 通信页](board-link.md)；追底盘运动问题时继续看 [底盘控制页](chassis.md)。

## 摩擦轮和发射

- 键鼠模式按 G 切换摩擦轮开/关，长按只切换一次；F 仍负责输入源切换。
- 遥控 S1 中位默认转轮；S2 下位只禁止供弹，S2 回中后再上拨触发单发。
- 转轮关闭、断联或升降互锁时禁止供弹；解除后左键先释放、遥控 S2 先回中，再重新触发。
- 底部和下降期间禁止转轮/发射；正常上升期间即可操作 G 和发射，不等待顶部。
- 同一键鼠模式内保持 G 的关闭选择；重新进入键鼠模式或重连进入 S1 中位恢复默认开启。断联退出键鼠模式的现有行为不变。
- 发射请求通过 D1 b5 bit0 转轮使能、bit5 供弹许可及原模式/触发位发送；需同时更新上下板。上板达速和热量判定见[发射说明](../../task_up/docs/launcher.md)。
