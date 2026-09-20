# 步兵上下板分阶段调试与模板清理计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在暂时不启用自瞄、裁判、超电和功率限制的前提下，先完成上板云台调试并冻结上下板接口，再逐阶段接入底盘、发射和升降，最后清理不再使用的旧架构代码。

**Architecture:** 下板作为遥控器、整车模式、底盘和安全策略主机；上板作为云台执行主机，负责 IMU、云台 PID、前馈、重力补偿和电机输出。上下板各自保留自己的算法与硬件驱动副本，但职责不重复；旧模块先通过编译开关旁路，确认失效后再删除。

**Tech Stack:** STM32H723 下板 Keil 工程 `DM-MC02`、STM32F407 上板 Keil 工程 `My_C`、FreeRTOS、FDCAN/CAN、现有 `board_protocol/communicate` 协议。

**Spec:** 当前上下板代码、`03_yuntai_down/docs/superpowers/plans/2026-09-17-yuntai-down-comm-bringup.md` 以及本计划。

## Global Constraints

- 当前不启用自瞄、视觉串级、裁判系统、超电和底盘功率限制。
- 当前只调试云台，底盘、发射、升降保持旁路。
- 不允许通过大规模注释源码来临时改功能，优先使用配置宏和任务隔离。
- 旧模块未经引用检查，不直接删除。
- 每次只恢复一个子系统，并单独完成上电、方向和闭环验收。
- 双板通信包一旦冻结，任何一侧修改必须同步修改另一侧。
- `0xD1/0xD2/0xD5` 和 `0xC1/0xC2` 在云台阶段保持稳定。
- 电机调试必须拆桨或空载，云台必须可靠固定，急停准备完成。
- 每个阶段必须达到 `0 errors`，本轮修改文件不得新增警告。

---

## 一、模板代码现状判断

当前模板“能编译、能工作”，但不能称为干净架构。主要问题如下：

- `infantry.c` 中大量模式和按键组合使用连续 `if/else`，决策、模式、发射和视觉标志混在一起。
- `Chassis.c` 同时包含底盘解算、功率预测、超电交互、裁判功率读取和电机输出，职责过多。
- 下板仍有旧 `gimbal.c` 目标生成逻辑，与上板云台控制存在重复。
- 下板 `launch.c` 目前主要转发发射状态，真实发射执行链路不完整。
- `vision.c`、裁判、超电和 UI 代码仍参与编译，但当前运行路径已经旁路。
- 工程仍编译大量当前不使用的模块，产生警告，但不一定增加最终固件体积。
- 上下板存在同名 PID、数学、IMU、遥控器和电机驱动文件，这是双 MCU 工程的正常副本，但必须同步维护。
- `connnect_task.h` 是拼写错误残留文件，目前没有源码引用。
- `power_limit.c` 基本处于注释和旧实现状态，当前不应作为底盘正式功率限制依赖。

结论：

- 当前模板可以作为分阶段调试基础。
- 不建议现在直接把 `chassis`、`launch`、`vision`、`judge` 等文件删除。
- 推荐先把它们从“调用路径”旁路，再从“Keil 编译路径”排除，最后才考虑物理删除。
- 真正删除前必须经过“全局引用检查、工程移除、全量构建、实车验证”四步。

---

## 二、目标职责划分

### 下板职责

- 接收并解析遥控器、键鼠。
- 管理整车控制源 `RC_CTRL / KEY_CTRL`。
- 管理整车安全状态和安全开关。
- 通过 `0xD1/0xD2/0xD5` 向上板发送状态、目标或手动角速度。
- 接收 `0xC1/0xC2` 获取上板云台状态与反馈。
- 下一阶段负责底盘控制。
- 后续负责发射和升降中物理连接在下板的执行器。
- 后续恢复裁判、超电和功率限制。
- 当前不负责云台电机 PID。

### 上板职责

- 接收 `0xD1/0xD2/0xD5`。
- 管理 IMU、云台电机和云台状态。
- 执行云台模式状态机。
- 执行 Yaw/Pitch 串级或速控 PID。
- 计算重力、前馈和摩擦补偿。
- 通过 `0xC1/0xC2` 返回云台状态与反馈。
- 当前不读取本机遥控器，遥控输入由下板 D5 提供。
- 不负责底盘。

### 发射与升降职责

发射和升降最终归属必须按实际接线确定：

- 电机 CAN 连接在上板：上板负责执行。
- 电机 CAN 连接在下板：下板负责执行。
- 命令决策始终建议由下板统一管理。
- 不允许一边读遥控器、另一边也读遥控器控制同一执行器。

### 同名文件处理原则

必须保留的上下板副本：

- `PID.c/.h`
- `rp_math.c/.h`
- `motor.c/.h`
- `DM_Motor.c/.h`
- `RM_motor.c/.h`
- IMU 驱动
- 遥控器驱动
- CAN/UART 驱动

原因：两个 MCU 不能直接共享编译目标，副本是必要的。

需要同步的内容：

- PID 公式和单位。
- 协议字段布局。
- 电机反馈解析。
- 云台角度方向定义。
- 公共限幅和保护逻辑。

---

## 三、阶段 A：完成并冻结云台

### Task A1: 固化云台接口和调试入口

**Files:**
- Read: `03_yuntai_down/Application/ConfigLayer/board_comm_config.h`
- Read: `03_yuntai_down/Application/ModuleLayer/board_protocol.c`
- Read: `03_yuntai_up/Application/ConfigLayer/board_remote_config.h`
- Read: `03_yuntai_up/Application/ModuleLayer/gimbal.c`
- Read: `03_yuntai_up/Application/ModuleLayer/gimbal.h`

**Interfaces:**
- Produces: 固定 D5 字段、G_RATE 输入路径和云台调试变量清单。

- [ ] **Step 1: 固定 D5 字节定义**

保持：

```text
Byte 0: valid[0], ctrl_source[1], reserved[7:2]
Byte 1: button_bits
Byte 2-3: yaw_rate_deg_s int16, 0.1 deg/s
Byte 4-5: pitch_rate_deg_s int16, 0.1 deg/s
Byte 6-7: reserved
```

- [ ] **Step 2: 禁止云台阶段修改 D5 字段含义**

任何新增按键、键鼠或模式都只能使用保留位或新包，不能重解释 `Yaw/Pitch` 角速度字段。

- [ ] **Step 3: 记录实车通道方向**

记录并确认：

```text
右摇杆向左 -> Yaw 正或负
右摇杆向上 -> Pitch 正或负
```

- [ ] **Step 4: 记录上电条件**

必须满足：

```text
下板 rc_sensor.work_state == DEV_ONLINE
下板 car_state == 1
下板 gimbal_mode == 1
上板 Board_HeartBeat.status == DEV_ONLINE
上板 offline_cnt_5 < offline_cnt_max
```

### Task A2: 调试 G_RATE 内环

**Files:**
- Modify: `03_yuntai_up/Application/ModuleLayer/gimbal.c:193-206`
- Modify: `03_yuntai_up/Application/ModuleLayer/gimbal.h:51-68`

**Interfaces:**
- Consumes: D5 手动角速度。
- Produces: 稳定的 Yaw/Pitch 角速度内环。

- [ ] **Step 1: 关闭底盘和发射**

保持：

```c
#define BOARD_COMM_DEBUG 1u
```

- [ ] **Step 2: 先调 Yaw 内环**

```c
Gimbal.pid_info.yaw_gyro_inner.kp
Gimbal.pid_info.yaw_gyro_inner.ki = 0
Gimbal.pid_info.yaw_gyro_inner.kd = 0
```

从较小值开始，逐步增大 `kp`，出现高频抖动时回调 20% 到 30%。

- [ ] **Step 3: 再调 Pitch 内环**

```c
Gimbal.pid_info.pitch_gyro_inner.kp
Gimbal.pid_info.pitch_gyro_inner.ki = 0
Gimbal.pid_info.pitch_gyro_inner.kd = 0
```

- [ ] **Step 4: 观察关键变量**

```text
Gimbal.feedforward.yaw_rate_cmd_deg_s
Gimbal.feedforward.yaw_rate_target_deg_s
Gimbal.base_info.yaw_imu_speed
Gimbal.base_info.output_gimbal_y

Gimbal.feedforward.pitch_rate_cmd_deg_s
Gimbal.feedforward.pitch_rate_target_deg_s
Gimbal.base_info.pitch_imu_speed
Gimbal.base_info.output_gimbal_p
```

- [ ] **Step 5: 验收**

要求：

```text
低速无爬行
中速无持续超调
高速无高频振荡
松杆后无持续单向漂移
```

### Task A3: 调试遥控器速度映射

**Files:**
- Modify: `03_yuntai_down/Application/ProtocolLayer/board_protocol.h`
- Modify: `03_yuntai_up/Application/ModuleLayer/gimbal.h`

- [ ] **Step 1: 先降低最大速度**

建议先使用：

```text
Yaw 最大 60 deg/s
Pitch 最大 30 deg/s
```

- [ ] **Step 2: 确认死区**

当前死区为 `20`。若中位仍然漂，适当增大；若小杆量不响应，适当减小。

- [ ] **Step 3: 调试速度斜坡**

```c
GIMBAL_RATE_CMD_RAMP_DEG_S_PER_MS
```

增大斜坡值：响应更快，但更容易冲击。
减小斜坡值：手感更柔，但响应更慢。

- [ ] **Step 4: 调试小角度保持**

```c
GIMBAL_RATE_HOLD_KP
GIMBAL_RATE_HOLD_DEADBAND_DEG_S
```

松杆后如果缓慢漂，增大保持 Kp。
如果回中时来回晃，减小保持 Kp。

### Task A4: 调试重力补偿

**Files:**
- Modify: `03_yuntai_up/Application/ModuleLayer/gimbal.h`
- Read: `03_yuntai_up/Application/ModuleLayer/gimbal.c`

- [ ] **Step 1: 保持重力补偿开启**

```c
#define GIMBAL_GRAVITY_ENABLE 1
```

- [ ] **Step 2: 从保守值开始**

```c
#define GIMBAL_GRAVITY_K_NM 0.2f
```

- [ ] **Step 3: 观察不同 Pitch 角度**

检查：

```text
水平附近是否下垂
低位是否向一侧压
高位是否回弹困难
```

- [ ] **Step 4: 只增大到刚好抵消重力**

如果电机静止时持续顶向一侧，说明补偿过大。
如果不同角度差异仍大，再考虑分段补偿或余弦模型。

### Task A5: 云台阶段冻结条件

- [ ] **Step 1: 连续运行 10 分钟**

要求：

```text
无 CAN 离线
无 D5 超时
无电机过热
无持续振荡
无方向错误
```

- [ ] **Step 2: 记录最终参数**

保存：

```text
Yaw/Pitch 内环 Kp/Ki/Kd
最大速度
死区
斜坡
保持 Kp
重力补偿
```

- [ ] **Step 3: 冻结接口**

云台阶段通过后，不再随意修改 D1/D2/D5 和云台变量名；进入底盘阶段。

---

## 四、阶段 B：清理云台阶段残留并冻结职责

### Task B1: 清理错误和确定不用的文件

**Files:**
- Delete later: `03_yuntai_down/Application/TaskLayer/connnect_task.h`
- Review: `03_yuntai_down/Application/ModuleLayer/gimbal.c/.h`
- Review: `03_yuntai_down/Application/ModuleLayer/vision.c/.h`
- Review: `03_yuntai_up/Application/AlgorithmLayer/power_limit.c/.h`

- [ ] **Step 1: 全局搜索引用**

```powershell
rg -n "connnect_task|vision\.|power_limit|Gimbal_|gimbal\." Application Core
```

- [ ] **Step 2: 先从 Keil 工程排除，不立即删除**

确认全量构建通过后，再删除物理文件。

- [ ] **Step 3: 删除拼写错误头文件**

`connnect_task.h` 目前只有自身定义，没有其他源码引用，可以删除。

### Task B2: 废弃下板旧云台目标生成

**Files:**
- Review: `03_yuntai_down/Application/ModuleLayer/gimbal.c/.h`
- Review: `03_yuntai_down/Application/ControlLayer/infantry.c`

- [ ] **Step 1: 确认上板已独立完成云台控制**

只有云台阶段验收完成后才能执行。

- [ ] **Step 2: 从下板活动路径移除下板 `gimbal.work()`**

保留文件到稳定期结束，不立即删除。

- [ ] **Step 3: 决定 D2 是否继续使用**

当前 D2 是角度目标包：

- 若以后可能使用位置模式或自动目标，保留。
- 若长期只使用 G_RATE，可改成语义更明确的辅助控制包。

### Task B3: 将不用的模块从编译中隔离

**Files:**
- Modify: `03_yuntai_down/MDK-ARM/DM-MC02.uvprojx`
- Modify: `03_yuntai_up/MDK-ARM/My_C.uvprojx`

- [ ] **Step 1: 云台阶段保持下板模块旁路**

```text
chassis
launch
vision
ui
judge
cap
power_limit
```

- [ ] **Step 2: 不从磁盘删除，只在 Keil 工程中排除**

这样可以保留以后恢复底盘和发射的代码。

- [ ] **Step 3: 每次排除一组后执行全量 rebuild**

确保：

```text
0 errors
不出现新的未定义符号
```

---

## 五、阶段 C：接入底盘

### Task C1: 冻结底盘控制接口

**Files:**
- Modify: `03_yuntai_down/Application/ModuleLayer/Chassis.c/.h`
- Modify: `03_yuntai_down/Application/ControlLayer/infantry.c`
- Modify: `03_yuntai_down/Application/TaskLayer/control_task.c`

**Interfaces:**
- Consumes: 遥控器左摇杆和键盘 WASD。
- Produces: 底盘目标速度、轮速 PID 输出和底盘状态。

- [ ] **Step 1: 固定输入映射**

```text
左摇杆上下：前后
左摇杆左右：左右平移
鼠标 X 或底盘转向输入：旋转
```

- [ ] **Step 2: 固定底盘模式**

建议：

```text
C_SLEEP
C_OPEN_LOOP_TEST
C_SPEED
C_FULL
```

- [ ] **Step 3: 禁止上板参与底盘控制**

所有底盘电机、解算、功率保护只在下板。

### Task C2: 单轮方向与闭环验收

- [ ] **Step 1: 单轮低速正转**
- [ ] **Step 2: 单轮低速反转**
- [ ] **Step 3: 确认编码器方向**
- [ ] **Step 4: 加入轮速 PID**
- [ ] **Step 5: 四轮同时同向**
- [ ] **Step 6: 四轮平移**
- [ ] **Step 7: 旋转和斜向运动**

每一步必须先通过再进入下一步。

### Task C3: 无裁判、无超电的功率保护

- [ ] **Step 1: 使用固定安全功率或电流上限**

当前不能依赖裁判：

```text
judge.chassis_power_limit
```

也不能依赖超电：

```text
cap.info
```

- [ ] **Step 2: 先限制电机最大输出**

使用保守电流或力矩限幅。

- [ ] **Step 3: 以后恢复裁判和超电**

恢复顺序：

```text
裁判数据 -> 固定功率限制 -> 超电反馈 -> 超电协同
```

---

## 六、阶段 D：发射和升降

### Task D1: 先确定物理归属

- [ ] **Step 1: 列出电机和 CAN 总线**

记录：

```text
摩擦轮左/右在哪个 CAN
拨盘在哪个 CAN
升降电机在哪个 CAN
每个电机属于下板还是上板
```

- [ ] **Step 2: 确定唯一命令所有者**

推荐：

```text
下板：开关、模式、安全锁和命令决策
执行板：电机 PID、CAN 发送和状态反馈
```

- [ ] **Step 3: 先空载测试**

发射先不装弹，升降先不接机械负载。

---

## 七、阶段 E：代码结构优化

### Task E1: 拆分步兵状态机

**Files:**
- Modify: `03_yuntai_down/Application/ControlLayer/infantry.c`
- Create later: `Application/ControlLayer/Input_Map.c/.h`
- Create later: `Application/ControlLayer/Vehicle_Mode.c/.h`

- [ ] **Step 1: 将遥控器原始输入转为统一命令结构**

```c
typedef struct
{
    float chassis_front;
    float chassis_left;
    float chassis_rotate;
    float gimbal_yaw_rate;
    float gimbal_pitch_rate;
    uint8_t shoot_mode;
    uint8_t control_source;
} Vehicle_Command_t;
```

- [ ] **Step 2: 将拨杆组合从巨大 if/else 中移出**

- [ ] **Step 3: 各模块只读取命令结构，不直接读遥控器**

### Task E2: 拆分底盘功率和电机控制

- [ ] **Step 1: 将功率预测从 `Chassis.c` 中拆出**
- [ ] **Step 2: 将超电和裁判适配拆成独立接口**
- [ ] **Step 3: 底盘只接收最终限幅后的力矩**

### Task E3: 删除流程

- [ ] **Step 1: 全局搜索符号引用**
- [ ] **Step 2: 从 Keil 工程移除文件**
- [ ] **Step 3: 全量 rebuild**
- [ ] **Step 4: 实车验证**
- [ ] **Step 5: 再删除物理文件**

---

## 八、推荐执行顺序

1. 完成云台 G_RATE 内环调试。
2. 完成遥控器速度映射和重力补偿。
3. 冻结 D1/D2/D5 与 C1/C2。
4. 删除 `connnect_task.h` 拼写残留。
5. 确认下板旧云台模块不再调用。
6. 从 Keil 工程排除视觉、裁判、超电、UI、发射和功率限制模块。
7. 开始底盘单轮、四轮和速度环。
8. 接入底盘固定安全功率限制。
9. 再确定发射和升降的执行板。
10. 最后恢复裁判、超电和正式功率限制。
11. 最后重构 `infantry.c`、`Chassis.c` 和旧协议残留。

## 九、短期明确不做

- 不处理自瞄。
- 不处理视觉串级。
- 不恢复裁判系统。
- 不恢复超电。
- 不做正式功率限制。
- 不删除底盘、发射和升降代码。
- 不重写全部上下板架构。
- 不修改已经稳定的云台协议字段。
