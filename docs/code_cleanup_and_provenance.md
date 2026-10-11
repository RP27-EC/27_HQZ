# 死代码清理与模板适配依据

[项目总览](../README.md) · [上板入口](../task_up/Readme.md) · [下板入口](../task_down/Readme.md)

## 范围与证据边界

- 执行日期：2026-10-11；对象：Train_code_plus 当前上下板工作区。
- 固定原 `BOARD_COMM_DEBUG=1`、`CHASSIS_BRINGUP_ENABLE=1` 的现用控制路径；移除旧业务及停用外设实现。
- 对照实施前的工作区内容，保留既有修改；不重置 Git，不删除既有临时文件。
- 本次仅静态审查；未编译、未烧录、未台架/实车验证。编译和功能验证由人工完成。
- 保留 PID、限幅、模型系数、电机映射、帧布局、任务节拍、安全联锁及现用 Watch 字段。
- `docs/superpowers/plans/` 下的旧方案属于历史记录，其中的已删除路径和宏不能作为当前接口使用。

## 删除清单与原引用关系

`.c/.h` 成对删除时，下表分别列出实际文件。路径均相对于仓库根目录；Keil 仅移除相应源文件条目与失效包含目录。

| 删除文件 | 原引用关系 / 判断依据 | 处理 |
| --- | --- | --- |
| `task_up/Application/TaskLayer/community_task.c` | 上板 CommunityTask 未创建，弱任务与业务实现无有效业务 | 删除实现及失效声明/包含 |
| `task_up/Application/TaskLayer/community_task.h` | 上板 CommunityTask 未创建，弱任务与业务实现无有效业务 | 删除实现及失效声明/包含 |
| `task_up/Application/AlgorithmLayer/power_limit.c` | 上板旧模型无调用并依赖过时裁判对象；现用功率限制在下板 AlgorithmLayer | 删除实现及失效声明/包含 |
| `task_up/Application/AlgorithmLayer/power_limit.h` | 上板旧模型无调用并依赖过时裁判对象；现用功率限制在下板 AlgorithmLayer | 删除实现及失效声明/包含 |
| `task_up/Application/AlgorithmLayer/low_pass_filter.c` | 无调用低通模块；现用控制/IMU 自身滤波未改 | 删除实现及失效声明/包含 |
| `task_up/Application/AlgorithmLayer/low_pass_filter.h` | 无调用低通模块；现用控制/IMU 自身滤波未改 | 删除实现及失效声明/包含 |
| `task_up/Application/HardwareLayer/HT_Motor.c` | 驱动/旧对象禁用；DM 曾经只借用 MIT 命令类型，公共枚举迁至 motor_def.h | 删除实现及失效声明/包含 |
| `task_up/Application/HardwareLayer/HT_Motor.h` | 驱动/旧对象禁用；DM 曾经只借用 MIT 命令类型，公共枚举迁至 motor_def.h | 删除实现及失效声明/包含 |
| `task_up/Application/DeviceLayer/imu_xrobot.c` | 外置 IMU 解析无现用调用，上板 USART1 接收未启用，保留通用弱回调 | 删除实现及失效声明/包含 |
| `task_up/Application/DeviceLayer/imu_xrobot.h` | 外置 IMU 解析无现用调用，上板 USART1 接收未启用，保留通用弱回调 | 删除实现及失效声明/包含 |
| `task_up/Application/DriverLayer/drv_flash.c` | 包装接口无现用消费者；Core 外设初始化、BMI088 原有 SPI 与 IMU 温控直接调用 HAL 保留 | 删除实现及失效声明/包含 |
| `task_up/Application/DriverLayer/drv_flash.h` | 包装接口无现用消费者；Core 外设初始化、BMI088 原有 SPI 与 IMU 温控直接调用 HAL 保留 | 删除实现及失效声明/包含 |
| `task_up/Application/DriverLayer/drv_spi.c` | 包装接口无现用消费者；Core 外设初始化、BMI088 原有 SPI 与 IMU 温控直接调用 HAL 保留 | 删除实现及失效声明/包含 |
| `task_up/Application/DriverLayer/drv_spi.h` | 包装接口无现用消费者；Core 外设初始化、BMI088 原有 SPI 与 IMU 温控直接调用 HAL 保留 | 删除实现及失效声明/包含 |
| `task_up/Application/ConfigLayer/rp_user_config.h` | 旧状态框架无消费者；仅由 rp_config.h 包含，删除其包含关系 | 删除实现及失效声明/包含 |
| `task_down/Application/ControlLayer/infantry.c` | 旧 infantry 调用旧 Chassis/gimbal；仅位于未选用业务分支，现用 chassis_control 独立闭环 | 删除实现及失效声明/包含 |
| `task_down/Application/ControlLayer/infantry.h` | 旧 infantry 调用旧 Chassis/gimbal；仅位于未选用业务分支，现用 chassis_control 独立闭环 | 删除实现及失效声明/包含 |
| `task_down/Application/ModuleLayer/Chassis.c` | 旧 infantry 调用旧 Chassis/gimbal；仅位于未选用业务分支，现用 chassis_control 独立闭环 | 删除实现及失效声明/包含 |
| `task_down/Application/ModuleLayer/Chassis.h` | 旧 infantry 调用旧 Chassis/gimbal；仅位于未选用业务分支，现用 chassis_control 独立闭环 | 删除实现及失效声明/包含 |
| `task_down/Application/ModuleLayer/gimbal.c` | 旧 infantry 调用旧 Chassis/gimbal；仅位于未选用业务分支，现用 chassis_control 独立闭环 | 删除实现及失效声明/包含 |
| `task_down/Application/ModuleLayer/gimbal.h` | 旧 infantry 调用旧 Chassis/gimbal；仅位于未选用业务分支，现用 chassis_control 独立闭环 | 删除实现及失效声明/包含 |
| `task_down/Application/ConfigLayer/config_chassis.h` | 旧 infantry 调用旧 Chassis/gimbal；仅位于未选用业务分支，现用 chassis_control 独立闭环 | 删除实现及失效声明/包含 |
| `task_down/Application/SolvingLayer/Flag_Handle.c` | 轮腿旧 Command/姿态/标志依赖簇；未接入当前四轮控制，保留 TaskLayer/Command_Task 遥控解析 | 删除实现及失效声明/包含 |
| `task_down/Application/SolvingLayer/Flag_Handle.h` | 轮腿旧 Command/姿态/标志依赖簇；未接入当前四轮控制，保留 TaskLayer/Command_Task 遥控解析 | 删除实现及失效声明/包含 |
| `task_down/Application/ServiceLayer/Command.c` | 轮腿旧 Command/姿态/标志依赖簇；未接入当前四轮控制，保留 TaskLayer/Command_Task 遥控解析 | 删除实现及失效声明/包含 |
| `task_down/Application/ServiceLayer/Command.h` | 轮腿旧 Command/姿态/标志依赖簇；未接入当前四轮控制，保留 TaskLayer/Command_Task 遥控解析 | 删除实现及失效声明/包含 |
| `task_down/Application/ServiceLayer/Command_Instance.c` | 轮腿旧 Command/姿态/标志依赖簇；未接入当前四轮控制，保留 TaskLayer/Command_Task 遥控解析 | 删除实现及失效声明/包含 |
| `task_down/Application/ServiceLayer/Command_Instance.h` | 轮腿旧 Command/姿态/标志依赖簇；未接入当前四轮控制，保留 TaskLayer/Command_Task 遥控解析 | 删除实现及失效声明/包含 |
| `task_down/Application/DeviceLayer/Chassis_Posture.c` | 轮腿旧 Command/姿态/标志依赖簇；未接入当前四轮控制，保留 TaskLayer/Command_Task 遥控解析 | 删除实现及失效声明/包含 |
| `task_down/Application/DeviceLayer/Chassis_Posture.h` | 轮腿旧 Command/姿态/标志依赖簇；未接入当前四轮控制，保留 TaskLayer/Command_Task 遥控解析 | 删除实现及失效声明/包含 |
| `task_down/Application/SolvingLayer/Chassis_Posture.c` | 轮腿旧 Command/姿态/标志依赖簇；未接入当前四轮控制，保留 TaskLayer/Command_Task 遥控解析 | 删除实现及失效声明/包含 |
| `task_down/Application/SolvingLayer/Chassis_Posture.h` | 轮腿旧 Command/姿态/标志依赖簇；未接入当前四轮控制，保留 TaskLayer/Command_Task 遥控解析 | 删除实现及失效声明/包含 |
| `task_down/Application/ParameterLayer/Filter.c` | 轮腿旧 Command/姿态/标志依赖簇；未接入当前四轮控制，保留 TaskLayer/Command_Task 遥控解析 | 删除实现及失效声明/包含 |
| `task_down/Application/ParameterLayer/Filter.h` | 轮腿旧 Command/姿态/标志依赖簇；未接入当前四轮控制，保留 TaskLayer/Command_Task 遥控解析 | 删除实现及失效声明/包含 |
| `task_down/Application/HardwareLayer/BRT_code.c` | 轮腿旧 Command/姿态/标志依赖簇；未接入当前四轮控制，保留 TaskLayer/Command_Task 遥控解析 | 删除实现及失效声明/包含 |
| `task_down/Application/HardwareLayer/BRT_code.h` | 轮腿旧 Command/姿态/标志依赖簇；未接入当前四轮控制，保留 TaskLayer/Command_Task 遥控解析 | 删除实现及失效声明/包含 |
| `task_down/Application/TaskLayer/observe_task.c` | 下板旧 UpdataTask 未创建，旧 IMU 更新循环随旧链路退出 | 删除实现及失效声明/包含 |
| `task_down/Application/TaskLayer/observe_task.h` | 下板旧 UpdataTask 未创建，旧 IMU 更新循环随旧链路退出 | 删除实现及失效声明/包含 |
| `task_down/Application/AlgorithmLayer/DWT.c` | 与 drv_tick.c 重复的实现；有效 DWT 定义原已位于 drv_tick.c，声明迁入 drv_tick.h | 删除实现及失效声明/包含 |
| `task_down/Application/AlgorithmLayer/DWT.h` | 与 drv_tick.c 重复的实现；有效 DWT 定义原已位于 drv_tick.c，声明迁入 drv_tick.h | 删除实现及失效声明/包含 |
| `task_down/Application/DeviceLayer/cap.c` | 旧 cap 初始化/心跳/收发停用；无线充电 CAN 分支无现用控制消费者，现用 supercap 独立保留 | 删除实现及失效声明/包含 |
| `task_down/Application/DeviceLayer/cap.h` | 旧 cap 初始化/心跳/收发停用；无线充电 CAN 分支无现用控制消费者，现用 supercap 独立保留 | 删除实现及失效声明/包含 |
| `task_down/Application/ProtocolLayer/cap_protocol.c` | 旧 cap 初始化/心跳/收发停用；无线充电 CAN 分支无现用控制消费者，现用 supercap 独立保留 | 删除实现及失效声明/包含 |
| `task_down/Application/ProtocolLayer/cap_protocol.h` | 旧 cap 初始化/心跳/收发停用；无线充电 CAN 分支无现用控制消费者，现用 supercap 独立保留 | 删除实现及失效声明/包含 |
| `task_down/Application/DriverLayer/drv_flash.c` | 包装接口无现用消费者；Core 外设初始化、BMI088 原有 SPI 与 IMU 温控直接调用 HAL 保留 | 删除实现及失效声明/包含 |
| `task_down/Application/DriverLayer/drv_flash.h` | 包装接口无现用消费者；Core 外设初始化、BMI088 原有 SPI 与 IMU 温控直接调用 HAL 保留 | 删除实现及失效声明/包含 |
| `task_down/Application/DriverLayer/drv_spi.c` | 包装接口无现用消费者；Core 外设初始化、BMI088 原有 SPI 与 IMU 温控直接调用 HAL 保留 | 删除实现及失效声明/包含 |
| `task_down/Application/DriverLayer/drv_spi.h` | 包装接口无现用消费者；Core 外设初始化、BMI088 原有 SPI 与 IMU 温控直接调用 HAL 保留 | 删除实现及失效声明/包含 |
| `task_down/Application/ConfigLayer/rp_user_config.h` | 旧状态框架无消费者；仅由 rp_config.h 包含，删除其包含关系 | 删除实现及失效声明/包含 |
| `task_up/Application/ConfigLayer/rp_config.c` | 空实现，无初始化或业务定义 | 删除实现及失效声明/包含 |
| `task_down/Application/ConfigLayer/rp_config.c` | 空实现，无初始化或业务定义 | 删除实现及失效声明/包含 |
| `task_down/Application/TaskLayer/connnect_task.h` | 错拼且未被包含的空头；现用 connect_task.c/.h 保留 | 删除实现及失效声明/包含 |
| `task_up/Application/DriverLayer/drv_tim.c` | 包装接口无现用消费者；Core 外设初始化、BMI088 原有 SPI 与 IMU 温控直接调用 HAL 保留 | 删除实现及失效声明/包含 |
| `task_up/Application/DriverLayer/drv_tim.h` | 包装接口无现用消费者；Core 外设初始化、BMI088 原有 SPI 与 IMU 温控直接调用 HAL 保留 | 删除实现及失效声明/包含 |
| `task_down/Application/DriverLayer/drv_tim.c` | 包装接口无现用消费者；Core 外设初始化、BMI088 原有 SPI 与 IMU 温控直接调用 HAL 保留 | 删除实现及失效声明/包含 |
| `task_down/Application/DriverLayer/drv_tim.h` | 包装接口无现用消费者；Core 外设初始化、BMI088 原有 SPI 与 IMU 温控直接调用 HAL 保留 | 删除实现及失效声明/包含 |

### 文件内同步清理

| 位置 | 已删除 / 已迁移内容 | 依据与保留边界 |
| --- | --- | --- |
| 下板 device.c / driver.c / control_task.c / monitor_task.c | 新旧链路切换分支、旧 infantry/IMU/cap 调用与包含 | 固定原有效分支；顺序见下节 |
| 两板 Core/Src/freertos.c | Community/Updata 停用任务的句柄、属性、声明、创建残留与弱实现 | 上板 Monitor/Control/Led，下板 Command/Ctrl/Monitor/Connect 的优先级与栈保留 |
| 上板 DeviceLayer/motor.c/.h | rm_motor_driver、ht_motor_drive、GIMB_P_mec、独立 Yaw_Motor/Pitch_Motor、HT 初始化和旧卸力接口 | 保留 dm_motor、DM_Group、rm_motor、RM_Group、lift_motor、dail_motor 与反馈对象 |
| 上板 motor_def.h / DM_Motor.h | mit_cmd_t 迁入公共电机头，DM 不再包含 HT | Enter/Exit/Zero 编号保持 0/1/2，MIT 帧编码未改 |
| 两板 rp_math.c/.h | Time_Trigger_inloop 与 Time_trigger_t | 无消费者；其他数学函数保持 |
| 上板 drv_can.c/.h | HAL_CAN_TxHeadeInit、CAN_TxHeadeType | 无消费者的旧发送头包装；现用 CAN_SendData 未改 |
| 两板 drv_uart.c/.h | 上板 UART_printf，下板 WL_UART_printf / USART7_Init，以及闲置缓冲 | 无消费者；现用 UART 初始化、DMA/IDLE、弱回调保留 |
| 下板 PID.c/.h / judge.c/.h | feedforward_pid_calc、Judge_GetPowerData、旧 cap 包含 | 均无消费者；single_pid_ctrl 与现用热量/弹速/功率快照保留 |
| 两板 can_protocol.h；下板 can_protocol.c | 旧超电、无线充电对象/转换/ID、空 SLAVE ID、失效旧底盘 ID 列表 | 真实轮序及 ID 位于 motor.h；四轮、现用 supercap、板间回调保持 |
| 两板 MDK-ARM/*.uvprojx | 被删源文件条目、失效 IncludePath | Target、编译器、优化及链接配置不变 |

### 配置接口迁移

- 删除 `BOARD_COMM_DEBUG`、`CHASSIS_BRINGUP_ENABLE`：不再提供旧业务选择分支。
- 删除 `BOARD_CAP_ENABLE`、`BOARD_JUDGE_ENABLE`、`BOARD_HOLE_WHEEL_REVERSE`；后两项原无消费者，裁判接收实际由 USART1 驱动与有效帧决定。
- 删除下板 `CAP_SWITCH`、`POWER_LIMIT_SWITCH`、`CHASSIS_SWITCH`、`GIMBAL_SWITCH`、`LAUNCH_SWITCH`、`SLIP_SWITCH`、`TURN_MODE` 旧框架配置。
- 删除 `CHASSIS_PLANNER_ENABLE`、`CHASSIS_FEEDFORWARD_ENABLE`、`CHASSIS_TURN_CYCLE_SPEED`、固定限流/限矩空配置与未被使用的几何/质量参数。
- 删除上板 `LAUNCHER_FRIC_KFF`、两板 `configDRV_CAN_USE_MAIL` 与上板过时 RM 电机列表。
- 保留板间发送、D3/D5/D6、热量/弹速、遥控/键鼠、跟随/小陀螺及 `SUPERCAP_BRINGUP_ENABLE` 等有消费者开关。
- 不通过重命名 `Board_Debug_*` 接口或删 Watch 字段伪装改写；现用公开接口与报文结构保留。

## 模板、国赛参考与本车适配

对照目录：

| 参考工程 | 本地位置 |
| --- | --- |
| 上板模板 | `G:/STM32_ALL/RM_code/temple/RP27-Train_Template-main_up` |
| 下板模板 | `G:/STM32_ALL/RM_code/temple/RP27-Train_Template-DM-down/RP2026_Templete` |
| 国赛上板 | `G:/STM32_ALL/RM_code/temple/hole_infantry-hole_infantry_up` |
| 国赛下板 | `G:/STM32_ALL/RM_code/temple/hole_infantry-hole_infantry_down` |

| 保留模块 | 来源与对应关系 | 本车需要说明的适配 |
| --- | --- | --- |
| Core / HAL / CMSIS / FreeRTOS | ST 与 RTOS 基础层；保留原版权和工程配置 | 芯片、引脚、时钟、总线与任务入口以本工程为准，不把基础驱动宣称为原创 |
| HardwareLayer/RM_motor、DM_Motor、KT_motor；DriverLayer | 模板/参考工程的通用电机、CAN、UART 基础实现 | motor.c 装配实际电机/总线/槽位；控制层区分 raw、N·m、rpm、rad/s |
| 上板 Sensor/Imu | BMI088 采样与 EKF 基础实现 | 当前用板载 BMI088；停用外置 IMU 已删，不能把 XRobot 描述为当前姿态来源 |
| 下板 chassis_input/control/follow/spin | 从基础驱动与参考运动控制发展出的现用业务层 | 遥控/键鼠仲裁、轮速闭环、云台跟随、小陀螺、掉头及失联保护均沿当前源码解释 |
| 上板 gimbal/lift/launcher | 国赛工程中的机构思路是对照对象；当前实现必须按实际调用链评审 | DM 云台、RM2006 升降、RM3508 摩擦轮与 KT 拨盘；讲清单位、状态跳转、互锁和恢复条件 |
| 下板 power_limit.c | 六项模型系数与国赛下板 Application/ModuleLayer/Chassis.c 四组数据一致 | 当前独立预算、量化预测及公共比例限制；系数未做本车标定 |
| 板间 board_protocol / communicate；supercap | 协议/设备基础接口与本车双板协作逻辑 | 保留现用 D/C 帧及新超电收发心跳；旧 cap/无线充电不再作为当前功能描述 |

相似代码只能支持复用关系判断，不能据此认定抄袭。审车应主动说明复用范围，并能够从需求推导目标、单位、输出和保护条件；改名、改注释、改变排版不构成自主适配证据。

## 当前有效控制链路

### 初始化与调度

```text
下板 DEVICE_Init:
  SuperCap_Init（通信开关允许） → judge.init → rc_dev.init → board.init
  → launch.init → rm_motor_list_init → Follow_Init → Spin_Init → Input_Init → Control_Init
下板 DRIVER_Init:
  USART1_Init → USART5_Init → CAN2_Filter_Init → CAN1_Filter_Init
下板 CtrlTask:
  Input_Update → Board_Debug_Gimbal_Command → Follow_UpdateMode → Spin_UpdateMode
  → Follow_Update → Spin_Update → Control_Update → launch.work → SuperCap_Tx（允许时） → osDelay(1)
下板 MonitorTask:
  四轮心跳 → RC心跳 → 发射心跳 → 板间心跳 → SuperCap心跳（允许时） → 裁判心跳 → osDelay(1)
上板 ControlTask:
  IMU更新（错误状态允许时） → 调试快照 → 本地RC（配置允许）
  → Module_Work（Gimbal → Lift） → 云台CAN输出 → Launcher_Work → C帧反馈 → osDelayUntil(1)
```

- 下板 CommandTask 解析 DBUS/键鼠；ConnectTask 按原周期发送 D1/D2/D5、D3/D6，D4默认关闭。
- 下板 `osDelay(1)` 是让出一 tick，循环还包括执行耗时；上板控制任务对齐 1 ms。清理未改变这些节拍。
- 输入 → 目标：UART5 DMA/IDLE → rc_dev → chassis_input_cmd；Yaw/Pitch 操作经 D1/D2/D5 进入上板。
- 计算 → 输出：跟随/小陀螺修改轮速分量 → 四轮目标 → 速度 P 环 → 模式限矩 → 公共功率缩放 → RM 0x200；云台与机构使用各自现用闭环。
- 反馈：四轮 rpm/电流/在线状态、C2 云台角及时间戳、BMI088/DM/RM/KT 反馈、裁判有效快照分别进入对应控制模块。
- 失联：任意轮离线停止四轮；RC、板间和 C2 新鲜度按原分支处理；裁判无效回退功率预算，热量/弹速仍受独立有效位和时效检查。
- 升降过洞、发射许可、重新解锁与松键再按等安全条件沿现用代码保留；`is_down` 报告状态不能独立证明物理到顶。

## 单位关系与运动学边界

| 量 | 单位 / 换算 | 源码含义 |
| --- | --- | --- |
| chassis_cmd.vx / vy / wz | 输出轴轮速分量 rad/s | 直接叠加为轮速，不能把 vx/vy 当车体 m/s，不能把 wz 当车体自转角速度 |
| 四轮目标 LF/LB/RF/RB | `-vx+vy+wz` / `-vx-vy+wz` / `vx+vy+wz` / `vx-vy+wz` | 轮序0/1/2/3；未使用轮半径、轮距作 SI 运动学转换 |
| RM encoder_speed | 转子 rpm | CAN 原始反馈，功率模型使用此量 |
| 下板 wheel_speed | 输出轴 rad/s = rpm × 2π / 60 / 19 | `_3508_Reduction`，减速比按 `_3508_REDUCT_RATIO=19` |
| 下板轮控输出 torque | N·m | `I_A = torque / 0.3`；0.3 N·m/A 为沿用的减速电机常数 |
| 下板发送原始电流 raw | `raw = (int16_t)(clamp(I_A, -18, 18) / 20 × 16384)` | 先±90%额定电流限幅再截断，模型与驱动相同；raw不是 A |
| 上板摩擦轮/升降输出 | 原始控制计数 | 当前 Single 分支将 torque 字段按 raw 裁剪发送，不能套用下板 N·m 换算 |
| DM 云台 | 位置 rad、速度 rad/s、力矩 N·m；业务部分转换 deg/deg/s | 看具体环路输入，不能直接比较不同单位 PID |
| KT 拨盘 | 位置 count、速度 deg/s | 单发累计位移与实际供弹传动对应；不能套用 RM 编码器单位 |

若后续需要车体 SI 运动学，应由人工给出实测轮半径、轮距、符号和传动比，再独立设计换算；本次不改变当前运动学和参数。

## 功率模型与系数来源

```text
P_i = k0 + k1*I + k2*rpm + k3*I*rpm + k4*I^2 + k5*rpm^2
I = 当前候选力矩按驱动规则换算、限幅、截断后的原始电流计数
P_sum = sum(max(P_i, 0))
```

- 系数索引对应 LF、LB、RF、RB，即左前、左后、右前、右后。
- 第三、第四组六个系数完全相同；不能描述成四轮分别完成本车标定。
- 来源：国赛下板 `Application/ModuleLayer/Chassis.c` 的四组六项系数；当前保留于下板 `AlgorithmLayer/power_limit.c` 的 `power_coeff`。
- **沿用参考工程，尚需人工确认本车适用性。** 本次不修改系数，不宣称已完成本车标定。
- 当前功率预算来自裁判上限与缓冲快照；无效时回退45 W。超预算时寻找公共比例，四轮方向/配比保持，最终按量化发送值复算。
- 单轮负预测不用于抵消其它轮耗电；零力矩预测仍超预算时保留 `target_unreachable` 提示。
- 新超电通信/反馈/心跳保留；放电、预充、Turbo 与功率缓冲默认关闭，不能宣称已启用超电增功率。
- 后续标定由人工提供各轮 raw/rpm/电气功率记录、轮序、供电/负载与拟合结果，再评审替换；不填虚构参数。

## 静态验收与人工验收

静态检查范围：删除符号在现用业务中的引用、头文件/Keil路径、MIT/DWT唯一有效定义、保留函数与配置相对实施前的差异、任务顺序及节拍。静态检查不能证明实车控制效果。

人工验收：

| 场景 | 重点核对 |
| --- | --- |
| 上下板编译 | 原有 Keil 环境，两板0 errors；记录工具链/Pack和告警 |
| 遥控/键鼠切换 | 来源/valid、切换后目标、失联卸力及恢复许可 |
| 跟随/小陀螺/掉头 | C2年龄、模式交接、目标与输出符号、转向限制 |
| 升降过洞及恢复 | 对齐、编码器目标、下降动作锁、顶部确认、恢复后重新许可 |
| 发射联锁 | 摩擦轮达速、热量、弹速、供弹许可、松键再按条件 |
| 裁判超时 | 快照年龄/flags、45 W回退、热量限制及恢复 |
| 四轮掉线 | 任一轮离线，四轮输出停止 |
| 新超电通信 | 收发/心跳/反馈单位，默认关闭输出保持 |

审车建议按“需求 → 输入与单位 → 控制公式 → 参数依据 → 输出限幅 → 反馈与失联 → 人工验证记录”讲解；先指出复用驱动和未经本车标定的系数，再说明本车业务适配，避免以更换命名代替原理理解。

### 本次静态验收记录

- 56 个目标文件已删除；剩余现用业务未检出对应旧头、对象、函数或选择宏消费者。
- 下板初始化、控制、监控、D5 打包与两板任务创建有效语句保持原选中路径；优先级、栈与节拍保持。
- 16 个关键控制、功率、超电和协议结构文件的有效代码与实施前一致；保留配置项值一致。
- 两板 Keil 的 FilePath 和 IncludePath 均能解析到现有路径；除源文件条目与 IncludePath 外的工程配置一致。
- 上板共享 MIT 枚举只有一份；下板 DWT_Init 的有效实现只有 drv_tick.c 一处。
- 独立静态复核发现的 CommunityTask 删除残片已修正并回读；未以编译验证替代源码证据。
- 静态限制：下板保留源码引用 arm_math.h，仓库内未找到该头；是否由既有 Keil 工具链提供需人工编译确认。未新增依赖或更改环境。
- 未编写测试/Mock/测试脚本，未自动编译、烧录或执行排错循环；按用户规范跳过。

## 关键完整交付单元

以下是交付时的完整函数/公共枚举，实际编译以对应源码文件为准。

### DEVICE_Init

源码：`task_down/Application/DeviceLayer/device.c`。

```c
void DEVICE_Init(void)
{
#if SUPERCAP_BRINGUP_ENABLE
    SuperCap_Init();
#endif
    /* 裁判心跳供功率预算使用 */
    judge.init(&judge);
    rc_dev.init(&rc_dev);
    board.init(&board);
    launch.init(&launch);
    rm_motor_list_init();
    Chassis_Follow_Init();
    Chassis_Spin_Init();
    Chassis_Input_Init();
    Chassis_Control_Init();
}
```

### DRIVER_Init

源码：`task_down/Application/DriverLayer/driver.c`。

```c
void DRIVER_Init(void)
{
    /* 裁判数据供功率预算使用 */
    USART1_Init();

    USART5_Init();
    CAN2_Filter_Init();
    CAN1_Filter_Init();
}
```

### StartCtrlTask

源码：`task_down/Application/TaskLayer/control_task.c`。

```c
void StartCtrlTask(void const *argument)
{
    (void)argument;

    for (;;)
    {
        Chassis_Input_Update();

        Board_Debug_Gimbal_Command();

        Chassis_Follow_UpdateMode();
        Chassis_Spin_UpdateMode();
        Chassis_Follow_Update(&chassis_input_cmd);
        Chassis_Spin_Update(&chassis_input_cmd);
        Chassis_Control_Update(&chassis_input_cmd);
        launch.work(&launch);

#if SUPERCAP_BRINGUP_ENABLE
        SuperCap_Tx();
#endif
        osDelay(1);
    }
}
```

### StartMonitorTask

源码：`task_down/Application/TaskLayer/monitor_task.c`。

```c
void StartMonitorTask(void const *argument)
{
    (void)argument;

    for (;;)
    {
        rm_motor_list_heart_beat();
        rc_dev.heart_beat(&rc_dev);
        launch.heart_beat(&launch);
        board.heartbeat(&board);

#if SUPERCAP_BRINGUP_ENABLE
        SuperCap_Heartbeat();
#endif


        /* 裁判离线使功率预算回退 */
        judge.heartbeat(&judge);



        osDelay(1);
    }
}
```

### mit_cmd_t

源码：`task_up/Application/HardwareLayer/motor_def.h`。

```c
typedef enum mit_cmd_enum_t
{
    Enter_Motor_Mode = 0, // 使能命令，编号0
    Exit_Motor_Mode = 1, // 失能命令，编号1
    Zero_Position_Sensor = 2, // 零点设定命令，编号2
} mit_cmd_t;
```
