# 云台下板通信与遥控联调实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让 `03_yuntai_down` 先以最小安全集合稳定编译、周期发送 `0xD1/0xD2`、可靠接收 `0xC1/0xC2`，再完成遥控器单主机方案和上板速控联调，最后才恢复底盘、发射、视觉和 UI 的完整整车逻辑。

**Architecture:** 下板保留现有 FreeRTOS 任务结构，但把板间通信从 `ConnectTask` 中独立出来；联调阶段通过 `BOARD_COMM_DEBUG` 只初始化 IMU、遥控器、板间协议和必要的目标生成模块。通信第一阶段维持现有协议，不新增包；确认链路稳定后，再决定是否新增 `0xD5` 作为“下板遥控器角速度命令包”，从而让上板真正使用下板遥控器。

**Tech Stack:** STM32H723、Keil MDK 工程 `MDK-ARM/DM-MC02.uvprojx`、CMSIS-RTOS2/FreeRTOS、STM32 HAL FDCAN、现有 `board_protocol`、`rc_sensor`、`gimbal` 模块。

**Spec:** 当前下板代码与上板 `03_yuntai_up` 中 `Application/ProtocolLayer/communicate.c/.h`、`Application/ModuleLayer/gimbal.c/.h` 的实际接口。

## Global Constraints

- 所有上机测试默认拆桨，云台先空载或可靠固定，CAN 分析仪与急停手段准备完成后再使能电机。
- 第一阶段通信测试必须让下板发送的 `car_pkt.car_state = 0`，上板因此保持卸力。
- 第二阶段确认 CAN 帧、ID、字节序、数值符号和心跳全部正确后，才允许 `car_state` 非零。
- 现有包 ID 固定为：下板发 `0xD1/0xD2/0xD3/0xD4`，上板发 `0xC1/0xC2`。
- `0xD1/0xD2` 优先保证 1 ms 周期；`0xD3/0xD4` 先按 10 ms 周期发送。
- `ConnectTask` 只负责板间通信，不负责整车决策、云台控制或 UI。
- 联调阶段不允许同时运行底盘速度环、发射摩擦轮、拨盘、视觉串级和 UI 全量刷新。
- 下板不再新增未定义的 CAN 包；若需要下板遥控器控制上板 `G_RATE`，必须新增并同步实现 `0xD5`。
- 上板和下板任何协议字段改动必须同一次提交完成，不允许单边修改。
- 所有改动完成后保留题目 `DM-MC02`，要求 Keil 编译结果为 `0 errors`；警告必须逐条确认，不允许用关闭警告掩盖问题。

---

## 一、当前结论与冻结边界

### 上板当前状态

上板可以作为“云台功能验证版”暂时冻结，原因如下：

- 云台控制、PID、前馈、模式、在线互锁和板间收发已经形成闭环。
- 上板 `Send_To_Down_Board()` 每 1 ms 发送 `0xC1/0xC2`。
- 上板使用 `Board_HeartBeat` 判断下板在线，并使用 `Board_Rx_Info.state_pkt.car_state` 判断是否允许云台出力。
- 上板 `0xD1` 解析模式字，`0xD2` 解析四个角度目标，字段顺序与下板发送结构一致。
- 上板仍直接读取本机 `rc_sensor` 生成 `G_RATE` 的角速度指令，因此目前还不能由下板遥控器直接控制上板速控。

上板暂不继续修改的条件：

- 先验证下板能稳定发送 `0xD1/0xD2`。
- 先验证上板 `Board_HeartBeat.status` 能从 `DEV_OFFLINE` 变为 `DEV_ONLINE`。
- 先验证上板 `Board_Rx_Info.gimbal_target_pkt` 的四个角度与下板发送值一致。
- 在单主机遥控器方案确定前，不把上板 `rc_sensor` 删除。

### 下板当前阻塞

已确认的阻塞与风险：

1. `Application/TaskLayer/connect_task.c` 中函数名误写为 `StartUITask()`，与 `UI_Task.c` 重复定义，产生 `L6200E: Symbol StartUITask multiply defined`。
2. 下板没有任何代码调用 `board.tx_01()`、`board.tx_02()`、`board.tx_03()`、`board.tx_04()`。
3. `Core/Src/freertos.c` 中 `StartConnectTask()` 是 weak 空循环，需要由 `connect_task.c` 的强定义覆盖。
4. `DEVICE_Init()` 当前一次性初始化全部整车模块，`StartCtrlTask()` 当前每 1 ms 调用完整 `infantry.work()`，联调时风险过大。
5. 上板速控依赖本机遥控器；若把遥控器移到下板，上板当前没有角速度指令来源。
6. `0xC1/0xC2` 的接收已经接入 `CAN2_rxDataHandler()`，但需要在真实双板环境下确认 FDCAN2 过滤器、总线端接和 ID 无冲突。

---

## 二、文件职责与计划边界

### 预计修改文件

- Modify: `Application/TaskLayer/connect_task.c`
  - 修复强定义函数名。
  - 改成周期发送 `0xD1~0xD4` 的通信任务。
- Create: `Application/TaskLayer/connect_task.h`
  - 声明 `StartConnectTask()`，避免依赖其他任务头文件。
- Modify: `Core/Src/freertos.c`
  - 保留 weak `StartConnectTask()`，不删除。
  - 必要时调整 `ConnectTask` 优先级和栈大小。
- Modify: `Application/ConfigLayer/rp_config.h`
  - 增加 `BOARD_COMM_DEBUG` 和周期/安全默认值宏。
- Modify: `Application/DeviceLayer/device.c`
  - 在 `BOARD_COMM_DEBUG` 下只初始化通信联调需要的模块。
- Modify: `Application/TaskLayer/control_task.c`
  - 在联调模式下不运行完整 `infantry.work()`。
- Modify: `Application/TaskLayer/monitor_task.c`
  - 在联调模式下只维护必要心跳。
- Modify: `Application/ModuleLayer/gimbal.h`
  - 给 Yaw/Pitch 机械中值补宏定义与注释，后续实测后填值。
- Modify: `Application/ProtocolLayer/board_protocol.c/.h`
  - 阶段一不改包结构。
  - 只有进入“下板遥控器控制上板速控”时，才增加 `0xD5` 编解码。
- Modify: `..\03_yuntai_up\Application\ProtocolLayer\communicate.c/.h`
  - 只有确定使用 `0xD5` 时同步修改。
- Modify: `..\03_yuntai_up\Application\ModuleLayer\gimbal.c`
  - 只有确定使用 `0xD5` 时，把上板手动输入来源从本机 `rc_sensor` 切换到下板命令。

### 不在本计划内

- 底盘四轮闭环、麦轮/全向轮运动学、功率限制和超电策略。
- 发射摩擦轮、拨盘、弹道和热量控制。
- 自瞄、视觉串级和 `0xD5` 明确模式包。
- UI 页面重构。
- 上板云台 PID 二次整定。
- Git 上传操作；下板稳定后再单独整理提交。

---

## 三、阶段 0：建立安全基线与可回退点

### Task 0: 记录当前状态与构建基线

**Files:**
- Read: `MDK-ARM/DM-MC02.uvprojx`
- Read: `Application/TaskLayer/connect_task.c`
- Read: `Application/ProtocolLayer/board_protocol.c`
- Read: `Core/Src/freertos.c`

**Interfaces:**
- Consumes: 当前工作区代码。
- Produces: 一张下板当前状态记录，后续每阶段完成后复用。

- [ ] **Step 1: 在 Keil 打开工程**

打开：

```text
G:\STM32_ALL\RM_code\Train_code_plus\03_yuntai_down\MDK-ARM\DM-MC02.uvprojx
```

- [ ] **Step 2: 选择目标并执行一次 Build**

Keil 目标：

```text
DM-MC02
```

预期当前结果：

```text
L6200E: Symbol StartUITask multiply defined
```

- [ ] **Step 3: 保存完整构建日志**

记录错误文件、函数名、目标名和编译时间。不要在这个阶段修改其他代码。

- [ ] **Step 4: 建立改前备份边界**

只允许创建一份明确命名的备份，例如：

```text
Application/TaskLayer/connect_task.c.bak
```

如果使用 Git，则优先使用 Git diff/commit 作为回退点，不再额外创建备份文件；阶段结束后删除 `.bak`。

---

## 四、阶段 1：先让下板恢复可编译

### Task 1: 修复 `connect_task.c` 重复符号

**Files:**
- Modify: `Application/TaskLayer/connect_task.c`
- Create: `Application/TaskLayer/connect_task.h`

**Interfaces:**
- Consumes: `board.init(&board)` 已在 `DEVICE_Init()` 中完成，且 `board.tx_01` 到 `board.tx_04` 已绑定。
- Produces: `void StartConnectTask(void const *argument)` 强定义，覆盖 `Core/Src/freertos.c` 中的 weak 定义。

- [ ] **Step 1: 创建 `connect_task.h`**

```c
#ifndef __CONNECT_TASK_H
#define __CONNECT_TASK_H

#include "main.h"
#include "cmsis_os.h"

void StartConnectTask(void const *argument);

#endif
```

- [ ] **Step 2: 将 `connect_task.c` 改为正确函数名**

先只做最小修复，不加入发送逻辑：

```c
#include "connect_task.h"

void StartConnectTask(void const *argument)
{
    (void)argument;

    for (;;)
    {
        osDelay(1);
    }
}
```

- [ ] **Step 3: 构建验证**

在 Keil 中重新 Build `DM-MC02`。

预期结果：

```text
0 errors
```

- [ ] **Step 4: 检查重复符号是否消失**

确认 `UI_Task.c` 中只有一个：

```c
void StartUITask(void const * argument)
```

确认 `connect_task.c` 中只有一个：

```c
void StartConnectTask(void const *argument)
```

- [ ] **Step 5: 阶段提交点**

提交信息示例：

```text
fix(down): resolve duplicate FreeRTOS task symbol
```

---

## 五、阶段 2：实现板间 CAN 通信任务

### Task 2: `ConnectTask` 周期发送 `0xD1/0xD2`

**Files:**
- Modify: `Application/TaskLayer/connect_task.c`
- Modify: `Application/ConfigLayer/rp_config.h`
- Read: `Application/ProtocolLayer/board_protocol.c`

**Interfaces:**
- Consumes:
  - `board.tx_01(Board_t *board)` 发送 `0xD1`。
  - `board.tx_02(Board_t *board)` 发送 `0xD2`。
- Produces:
  - 下板每 1 ms 发送一帧 `0xD1` 和一帧 `0xD2`。
  - 下板每 10 ms 发送一帧 `0xD3` 和一帧 `0xD4`。
  - 调试模式下 `0xD3/0xD4` 可以独立关闭。

- [ ] **Step 1: 在 `rp_config.h` 增加通信参数**

```c
#define BOARD_COMM_D1D2_PERIOD_MS       1u
#define BOARD_COMM_D3D4_PERIOD_MS       10u
#define BOARD_COMM_TX_ENABLE            1u
#define BOARD_COMM_D3D4_ENABLE          0u
```

调试模式默认 `BOARD_COMM_D3D4_ENABLE = 0`。
只有完整初始化 `judge` 后，才允许把它改为 1。

- [ ] **Step 2: 实现最小周期发送**

```c
#include "connect_task.h"
#include "board_protocol.h"
#include "rp_config.h"

void StartConnectTask(void const *argument)
{
    uint16_t slow_div = 0u;

    (void)argument;

    for (;;)
    {
#if BOARD_COMM_TX_ENABLE
        board.tx_01(&board);
        board.tx_02(&board);

#if BOARD_COMM_D3D4_ENABLE
        slow_div++;
        if (slow_div >= BOARD_COMM_D3D4_PERIOD_MS)
        {
            slow_div = 0u;
            board.tx_03(&board);
            board.tx_04(&board);
        }
#else
        (void)slow_div;
#endif
#endif

        osDelay(BOARD_COMM_D1D2_PERIOD_MS);
    }
}
```

- [ ] **Step 3: 暂时不加入复杂错误处理**

理由：先用逻辑分析仪或 CAN 分析仪确认帧确实发出，再加入发送失败计数。不要在这一步引入队列、互斥锁或动态内存。

- [ ] **Step 4: 编译验证**

预期结果：

```text
0 errors
```

- [ ] **Step 5: 构建后不连接电机**

烧录后先只连接 CAN 分析仪或上板，不连接云台电机动力。

### Task 3: 增加发送状态计数，定位 CAN 队列问题

**Files:**
- Modify: `Application/ProtocolLayer/board_protocol.c`
- Modify: `Application/ProtocolLayer/board_protocol.h`
- Modify: `Application/TaskLayer/connect_task.c`

**Interfaces:**
- Consumes: `CAN_SendData(FDCAN_HandleTypeDef *hcan, uint32_t stdId, uint8_t *dat)`。
- Produces:
  - `board.tx_ok_cnt`
  - `board.tx_fail_cnt`
  - `board.tx_d1_cnt`
  - `board.tx_d2_cnt`

- [ ] **Step 1: 在 `Board_Status_t` 增加计数**

```c
uint32_t tx_ok_cnt;
uint32_t tx_fail_cnt;
uint32_t tx_d1_cnt;
uint32_t tx_d2_cnt;
```

- [ ] **Step 2: 让 `CAN_SendData()` 的返回值参与判断**

`Board_Tx_Pkt_01()` 和 `Board_Tx_Pkt_02()` 中保存返回值：

```c
if (CAN_SendData(&hfdcan2, ID_PKT_01, pkt_01) == HAL_OK)
{
    board->status->tx_ok_cnt++;
    board->status->tx_d1_cnt++;
}
else
{
    board->status->tx_fail_cnt++;
}
```

- [ ] **Step 3: 编译并在调试器观察计数**

连续运行 10 秒，要求：

```text
tx_d1_cnt 大约为 10000
tx_d2_cnt 大约为 10000
tx_fail_cnt 不持续增长
```

- [ ] **Step 4: 若 `tx_fail_cnt` 增长，停止联调**

优先检查：

- FDCAN2 是否已 `HAL_FDCAN_Start()`。
- `CAN2_Filter_Init()` 是否在 `DRIVER_Init()` 中执行。
- 总线是否只有一台发送器、是否正确端接。
- 是否与 FDCAN2 上其他电机帧发生 ID 冲突。
- TX FIFO 是否被高频发送打满。

---

## 六、阶段 3：联调模式隔离整车逻辑

### Task 4: 增加 `BOARD_COMM_DEBUG`

**Files:**
- Modify: `Application/ConfigLayer/rp_config.h`
- Modify: `Application/DeviceLayer/device.c`
- Modify: `Application/TaskLayer/control_task.c`
- Modify: `Application/TaskLayer/monitor_task.c`

**Interfaces:**
- Consumes: 现有 `DEVICE_Init()`、`StartCtrlTask()`、`StartMonitorTask()`。
- Produces: `BOARD_COMM_DEBUG=1` 时只初始化通信、IMU、遥控器和板间协议，不运行完整步兵控制。

- [ ] **Step 1: 增加调试宏**

```c
#define BOARD_COMM_DEBUG                 1u
#define BOARD_COMM_DEBUG_CAR_STATE       0u
#define BOARD_COMM_DEBUG_ALLOW_MOTION    0u
```

阶段 3 中：

```text
BOARD_COMM_DEBUG = 1
BOARD_COMM_DEBUG_CAR_STATE = 0
BOARD_COMM_DEBUG_ALLOW_MOTION = 0
```

- [ ] **Step 2: 修改 `DEVICE_Init()`**

调试模式下最少初始化：

```c
imu_sensor.init(&imu_sensor);
rc_sensor.init(&rc_sensor);
board.init(&board);
```

调试模式下暂时不要初始化：

```text
chassis
launch
vision
My_Ui_Init
infantry
supercap
judge
```

调试模式下必须设置 `BOARD_COMM_D3D4_ENABLE = 0`，只发送 `0xD1/0xD2`。
如果需要发送 `0xD3`，则必须同时保留 `judge.init(&judge)`，并确认裁判离线不会阻塞基础通信。

- [ ] **Step 3: 修改 `StartCtrlTask()`**

调试模式只更新遥控器和安全场：

```c
#if BOARD_COMM_DEBUG
    board.tx_pkt->car_pkt.car_state = BOARD_COMM_DEBUG_CAR_STATE;

    if (BOARD_COMM_DEBUG_ALLOW_MOTION == 0u)
    {
        board.tx_pkt->gimbal_target_pkt.yaw_mec_tar = 0.0f;
        board.tx_pkt->gimbal_target_pkt.pitch_mec_tar = 0.0f;
        board.tx_pkt->gimbal_target_pkt.yaw_imu_tar = 0.0f;
        board.tx_pkt->gimbal_target_pkt.pitch_imu_tar = 0.0f;
    }
#else
    infantry.work(&infantry);
#endif
```

- [ ] **Step 4: 修改 `StartMonitorTask()`**

调试模式至少保留：

```c
rc_sensor.heart_beat(&rc_sensor);
board.heartbeat(&board);
imu_sensor.heart_beat(&imu_sensor.work_state);
```

调试模式暂时不要运行：

```text
cap.heartbeat
judge.heartbeat
infantry.heart_beat
rm_motor_list_heart_beat
```

- [ ] **Step 5: 编译并检查未使用变量警告**

若某些模块只在非调试模式使用，必须用 `#if` 包住相关变量或调用，不能留下未使用警告。

### Task 5: 记录双板心跳

**Files:**
- Read: `Application/TaskLayer/monitor_task.c`
- Read: `Application/ProtocolLayer/board_protocol.c`
- Read: `..\03_yuntai_up\Application\ModuleLayer\gimbal.c`

**Interfaces:**
- Consumes:
  - 下板 `board.heartbeat(&board)`。
  - 上板 `C_Board_Communicate_HeartBeat()`。
- Produces:
  - 可在调试器观察的在线状态和离线计数。

- [ ] **Step 1: 下板观察点**

```text
board.status->offline_cnt
board.status->status
board.status->tx_d1_cnt
board.status->tx_d2_cnt
board.status->tx_fail_cnt
```

- [ ] **Step 2: 上板观察点**

```text
Board_HeartBeat.status
Board_HeartBeat.offline_cnt_1
Board_HeartBeat.offline_cnt_2
Board_Rx_Info.state_pkt.car_state
Board_Rx_Info.gimbal_target_pkt
```

- [ ] **Step 3: 判定标准**

下板连续发送时：

```text
board.status->offline_cnt 必须能回到 0
board.status->status 必须为 DEV_ONLINE
```

上板同时发送 `0xC1/0xC2` 时：

```text
Board_HeartBeat.offline_cnt_1 必须能回到 0
Board_HeartBeat.offline_cnt_2 必须能回到 0
Board_HeartBeat.status 必须为 DEV_ONLINE
```

---

## 七、阶段 4：协议和数值闭环验证

### Task 6: 逐字段验证 `0xD2 -> 0xC2` 数据路径

**Files:**
- Read: `Application/ProtocolLayer/board_protocol.c`
- Read: `..\03_yuntai_up\Application\ProtocolLayer\communicate.c`

**Interfaces:**
- Consumes:
  - 下板发送角度目标。
  - 上板接收并解码到 `Board_Rx_Info.gimbal_target_pkt`。
- Produces: 一份字段对应关系表和实测记录。

- [ ] **Step 1: 写入固定测试值**

在 `BOARD_COMM_DEBUG` 下临时写入：

```c
board.tx_pkt->gimbal_target_pkt.pitch_imu_tar = -30.0f;
board.tx_pkt->gimbal_target_pkt.yaw_imu_tar = 45.0f;
board.tx_pkt->gimbal_target_pkt.pitch_mec_tar = 0.10f;
board.tx_pkt->gimbal_target_pkt.yaw_mec_tar = -0.20f;
```

单位：

```text
pitch_imu_tar: deg
yaw_imu_tar: deg
pitch_mec_tar: rad
yaw_mec_tar: rad
```

- [ ] **Step 2: 上板调试器读取**

预期上板接收值：

```text
Board_Rx_Info.gimbal_target_pkt.pitch_imu_tar = -30.0f
Board_Rx_Info.gimbal_target_pkt.yaw_imu_tar = 45.0f
Board_Rx_Info.gimbal_target_pkt.pitch_mec_tar = 0.10f
Board_Rx_Info.gimbal_target_pkt.yaw_mec_tar = -0.20f
```

允许误差：

```text
IMU 角误差 <= 0.02 deg
机械角误差 <= 0.001 rad
```

- [ ] **Step 3: 改变符号再测一次**

写入负值：

```c
board.tx_pkt->gimbal_target_pkt.pitch_imu_tar = 30.0f;
board.tx_pkt->gimbal_target_pkt.yaw_imu_tar = -45.0f;
board.tx_pkt->gimbal_target_pkt.pitch_mec_tar = -0.10f;
board.tx_pkt->gimbal_target_pkt.yaw_mec_tar = 0.20f;
```

确认上板接收符号与发送符号一致。

- [ ] **Step 4: 验证范围边界**

分别测试：

```text
-360 deg
+360 deg
-4 rad
+4 rad
```

确认编码值不是 `NaN`、不是反向饱和、不是零点偏置。

- [ ] **Step 5: 恢复安全值**

测试完成后恢复：

```c
board.tx_pkt->gimbal_target_pkt.pitch_imu_tar = 0.0f;
board.tx_pkt->gimbal_target_pkt.yaw_imu_tar = 0.0f;
board.tx_pkt->gimbal_target_pkt.pitch_mec_tar = 0.0f;
board.tx_pkt->gimbal_target_pkt.yaw_mec_tar = 0.0f;
```

### Task 7: 逐字段验证 `0xC2 -> 下板` 数据路径

**Files:**
- Read: `..\03_yuntai_up\Application\ProtocolLayer\communicate.c`
- Read: `Application\ProtocolLayer\board_protocol.c`

**Interfaces:**
- Consumes: 上板 `Board_Tx_Info.gimbal_meg`。
- Produces: 下板 `board.rx_meg->gimbal_meg` 的实测值。

- [ ] **Step 1: 上板注入固定反馈值**

临时设置：

```c
Board_Tx_Info.gimbal_meg.yaw_mec = 0.20f;
Board_Tx_Info.gimbal_meg.pitch_mec = -0.10f;
Board_Tx_Info.gimbal_meg.yaw_imu = 45.0f;
Board_Tx_Info.gimbal_meg.pitch_imu = -30.0f;
```

- [ ] **Step 2: 下板观察**

预期：

```text
board.rx_meg->gimbal_meg.yaw_mec = 0.20f
board.rx_meg->gimbal_meg.pitch_mec = -0.10f
board.rx_meg->gimbal_meg.yaw_imu = 45.0f
board.rx_meg->gimbal_meg.pitch_imu = -30.0f
```

- [ ] **Step 3: 验证 `0xC1` 状态位**

上板状态位：

```text
yaw_motor_state
pitch_motor_state
```

下板应正确映射到：

```text
gimbal.state.yaw_heart
gimbal.state.pitch_heart
```

---

## 八、阶段 5：第一次上电联调

### Task 8: CAN 分析仪全链路测试

**Files:**
- Read: `Application/ProtocolLayer/board_protocol.c`
- Read: `..\03_yuntai_up\Application\ProtocolLayer\communicate.c`

**Interfaces:**
- Consumes: 两块控制板、CAN 总线、分析仪、稳压电源。
- Produces: 一份 CAN 帧记录。

- [ ] **Step 1: 只接通信电源**

不接电机动力，不接发射机构。

- [ ] **Step 2: 确认总线配置**

检查：

```text
CAN 波特率一致
CANH/CANL 不反接
总线两端 120 欧姆端接正确
所有节点共地
D1~D4 和 C1/C2 没有其他节点重复发送
```

- [ ] **Step 3: 抓取 10 秒数据**

确认帧存在：

```text
0xD1: 1 kHz
0xD2: 1 kHz
0xD3: 100 Hz
0xD4: 100 Hz
0xC1: 1 kHz
0xC2: 1 kHz
```

- [ ] **Step 4: 检查错误帧**

要求：

```text
Bus Off = 0
Error Warning 不持续增长
ACK Error = 0
Form Error = 0
```

- [ ] **Step 5: 记录 CPU 负载**

若 FreeRTOS 有运行时统计，记录通信任务加入前后的 CPU 占用；若没有统计，用调试 GPIO 翻转测量 `ConnectTask` 执行时间。

### Task 9: 云台第一次受控出力

**Files:**
- Read: `..\03_yuntai_up\Application\ModuleLayer\gimbal.c`
- Read: `..\03_yuntai_up\Application\TaskLayer\control_task.c`

**Interfaces:**
- Consumes: 下板 `car_state`、上板 `Board_HeartBeat`、IMU 和电机在线状态。
- Produces: 安全的第一动作记录。

- [ ] **Step 1: 仍然保持下板 `car_state = 0`**

确认上板执行卸力分支：

```c
DM_Group.group_sleep(&DM_Group);
DM_Group.group_set_torque(&DM_Group);
```

- [ ] **Step 2: 确认上板所有安全条件**

要求：

```text
Board_HeartBeat.status == DEV_ONLINE
Board_Rx_Info.state_pkt.car_state != 0
IMU online
IMU cali_end != 0
Yaw DM motor online
Pitch DM motor online
```

- [ ] **Step 3: 只把 `car_state` 改为 1，目标仍为零**

观察：

```text
电机是否只执行初始化归中
是否有异常抖动
CAN 是否出现发送失败
板间心跳是否保持在线
```

- [ ] **Step 4: 出现任一异常立即回到 `car_state = 0`**

异常定义：

```text
电机持续堵转
云台单方向冲
CAN 丢帧大量增加
IMU 角度跳变
板间心跳反复离线
```

- [ ] **Step 5: 记录第一次成功上电条件**

将以下内容记录到 README 或调试日志：

```text
上电顺序
急停位置
初始模式
初始 car_state
电机安装方向
机械限位
观察到的最大误差
```

---

## 九、阶段 6：遥控器单主机方案

### 方案 A：临时方案，上板继续作为遥控器主机

适用场景：

- 只验证上板云台控制。
- 下板只需要提供 `car_state` 和协议心跳。
- 暂时不想扩展协议。

限制：

- 下板 `car_state` 不能由自己的遥控器离线状态直接决定，否则上板会卸力。
- 上板仍然使用本机遥控器。
- 该方案不能用于最终整车架构。

只允许在调试条件下实现：

```c
#define BOARD_COMM_DEBUG_CAR_STATE 1u
```

正式上车前必须删除或改为真实遥控器状态。

### 方案 B：最终方案，下板作为唯一遥控器主机

适用场景：

- 遥控器统一接在下板。
- 下板解析遥控器后，通过 CAN 把合法角速度命令发送给上板。
- 上板不再直接读取遥控器。

所需协议：

```text
新增 0xD5：下板 -> 上板 遥控速度命令包
```

建议字节布局：

```text
Byte 0: car_state[1:0], ctrl_source[2], gimbal_mode[3], shoot_enable[4], reserved[7:5]
Byte 1: button bitfield
Byte 2-3: yaw_rate_cmd_deg_s，int16，单位 0.1 deg/s
Byte 4-5: pitch_rate_cmd_deg_s，int16，单位 0.1 deg/s
Byte 6: shoot_mode
Byte 7: shoot_level
```

编码示例：

```c
int16_t yaw_raw = (int16_t)(yaw_rate_cmd_deg_s / 0.1f);
int16_t pitch_raw = (int16_t)(pitch_rate_cmd_deg_s / 0.1f);
```

下板发送 `0xD5`：

```c
static void Board_Tx_Pkt_05(Board_t *board)
{
    uint8_t txbuf[8] = {0};
    int16_t yaw_raw = (int16_t)(board->tx_pkt->remote_pkt.yaw_rate_cmd_deg_s / 0.1f);
    int16_t pitch_raw = (int16_t)(board->tx_pkt->remote_pkt.pitch_rate_cmd_deg_s / 0.1f);

    txbuf[0] = (uint8_t)((board->tx_pkt->car_pkt.car_state & 0x03u) |
                         ((board->tx_pkt->remote_pkt.ctrl_source & 0x01u) << 2) |
                         ((board->tx_pkt->car_pkt.gimbal_mode & 0x01u) << 3) |
                         ((board->tx_pkt->remote_pkt.shoot_enable & 0x01u) << 4));
    txbuf[1] = board->tx_pkt->remote_pkt.button_bits;
    txbuf[2] = (uint8_t)((uint16_t)yaw_raw >> 8);
    txbuf[3] = (uint8_t)yaw_raw;
    txbuf[4] = (uint8_t)((uint16_t)pitch_raw >> 8);
    txbuf[5] = (uint8_t)pitch_raw;
    txbuf[6] = board->tx_pkt->shoot_pkt.shoot_mode;
    txbuf[7] = board->tx_pkt->shoot_pkt.shoot_level;

    CAN_SendData(&hfdcan2, ID_PKT_05, txbuf);
}
```

上板接收并替换本机遥控器来源：

```c
if (Board_Rx_Info.state_pkt.car_state == 1u)
{
    yaw_rate = Board_Rx_Info.remote_pkt.yaw_rate_cmd_deg_s;
    pitch_rate = Board_Rx_Info.remote_pkt.pitch_rate_cmd_deg_s;
}
else if (Board_Rx_Info.state_pkt.car_state == 2u)
{
    yaw_rate = Board_Rx_Info.remote_pkt.yaw_rate_cmd_deg_s;
    pitch_rate = Board_Rx_Info.remote_pkt.pitch_rate_cmd_deg_s;
}
```

上板删除本机遥控器依赖前必须满足：

```text
0xD5 连续接收 10 秒无丢包
角速度符号与上板定义一致
上板本机 rc_sensor 离线时 G_RATE 仍能正常控制
下板遥控器离线时 car_state 自动回 0
```

### Task 10: 遥控器输入链路规范化

**Files:**
- Modify: `Application/ProtocolLayer/rc_protocol.c`
- Modify: `Application/TaskLayer/Command_Task.c`
- Modify: `Application/ControlLayer/infantry.c`
- Modify: `..\03_yuntai_up\Application\ModuleLayer\gimbal.c`

**Interfaces:**
- Consumes: `rc_sensor.info->ch0/ch1/ch2/ch3`、鼠标值、键位值。
- Produces: `yaw_rate_cmd_deg_s`、`pitch_rate_cmd_deg_s`、`car_state`、模式标志。

- [ ] **Step 1: 固定遥控器轴定义**

建议映射：

```text
右摇杆左右：Yaw 角速度
右摇杆上下：Pitch 角速度
左摇杆：底盘移动
拨轮：模式/发射辅助
S1/S2：安全开关和模式选择
```

- [ ] **Step 2: 加入死区**

建议：

```c
#define RC_GIMBAL_AXIS_DEADBAND       30
#define RC_GIMBAL_AXIS_MAX            660
```

死区内直接输出 0，避免云台静止时慢慢漂。

- [ ] **Step 3: 归一化**

```c
float axis_to_norm(int16_t axis, int16_t deadband, int16_t max_value)
{
    if (axis > -deadband && axis < deadband)
    {
        return 0.0f;
    }

    if (axis > 0)
    {
        return (float)(axis - deadband) / (float)(max_value - deadband);
    }

    return (float)(axis + deadband) / (float)(max_value - deadband);
}
```

- [ ] **Step 4: 转换到角速度**

建议初值：

```c
#define GIMBAL_RC_YAW_MAX_RATE_DEG_S       180.0f
#define GIMBAL_RC_PITCH_MAX_RATE_DEG_S     120.0f
```

```c
yaw_rate_cmd_deg_s = axis_to_norm(rc_sensor.info->ch0, 30, 660) *
                     GIMBAL_RC_YAW_MAX_RATE_DEG_S;
pitch_rate_cmd_deg_s = axis_to_norm(rc_sensor.info->ch1, 30, 660) *
                       GIMBAL_RC_PITCH_MAX_RATE_DEG_S;
```

- [ ] **Step 5: 增加指令斜坡**

建议：

```c
#define GIMBAL_RC_RATE_RAMP_DEG_S2     1200.0f
```

每 1 ms 最大变化：

```c
float max_step = GIMBAL_RC_RATE_RAMP_DEG_S2 * 0.001f;
yaw_rate_cmd_deg_s = rate_limit(yaw_rate_cmd_deg_s, target, max_step);
```

- [ ] **Step 6: 离线安全**

`rc_sensor.work_state == DEV_OFFLINE` 时：

```text
yaw_rate_cmd_deg_s = 0
pitch_rate_cmd_deg_s = 0
car_state = 0
shoot_enable = 0
```

- [ ] **Step 7: 模式切换清前馈**

进入或退出 `G_RATE` 时：

```text
yaw_rate_cmd_last_deg_s = 0
pitch_rate_cmd_last_deg_s = 0
yaw_rate_target_deg_s = 0
pitch_rate_target_deg_s = 0
hold_angle = 当前角度
```

---

## 十、阶段 7：机械中值和标定准备

### Task 11: 给 Yaw/Pitch 机械中值增加明确宏

**Files:**
- Modify: `Application/ModuleLayer/gimbal.h`
- Modify: `..\03_yuntai_up\Application\ModuleLayer\gimbal.h`

**Interfaces:**
- Consumes: 当前下板 `YAW_MEC_ZERO_ANGLE`、`PITCH_MEC_ZERO_ANGLE` 与上板 `GIMBAL_YAW_MIDDLE_DEG`、`GIMBAL_PITCH_MIDDLE_DEG`。
- Produces: 可实车标定的宏。

- [ ] **Step 1: 下板宏统一命名**

```c
#define YAW_MEC_ZERO_ANGLE_RAD          (-0.387884378f)
#define PITCH_MEC_ZERO_ANGLE_RAD        (0.0f)
```

保留兼容别名：

```c
#define YAW_MEC_ZERO_ANGLE              YAW_MEC_ZERO_ANGLE_RAD
#define PITCH_MEC_ZERO_ANGLE            PITCH_MEC_ZERO_ANGLE_RAD
```

- [ ] **Step 2: 上板宏统一命名**

```c
#define GIMBAL_YAW_MIDDLE_DEG           (0.0f)
#define GIMBAL_PITCH_MIDDLE_DEG        (0.0f)
```

当前值未实测，不允许假装已经标定。

- [ ] **Step 3: 标定方法**

机械中值标定步骤：

1. 云台断电，手动把 Yaw 推到机械前向中心。
2. 上电但保持 `car_state = 0`，只读取编码器角度。
3. 记录 Yaw 原始机械角。
4. 将记录的弧度值写入 `GIMBAL_YAW_MIDDLE_DEG` 对应换算位置或下板 `YAW_MEC_ZERO_ANGLE_RAD`。
5. Pitch 用同样方法记录水平安全角。
6. 重新编译，确认在机械中心时 `yaw_mec_angle` 和 `pitch_mec_angle` 接近 0。
7. 再测试正负 30 度，确认符号方向。

- [ ] **Step 4: 禁止未标定时启用大范围运动**

未标定期间：

```text
Pitch 限位保持保守
Yaw 机械目标范围保持保守
car_state 只允许做小角度测试
```

---

## 十一、阶段 8：下板通信稳定后的验收门槛

### Task 12: 通信阶段验收

**Files:**
- Read: `Application/TaskLayer/connect_task.c`
- Read: `Application/ProtocolLayer/board_protocol.c`
- Read: `..\03_yuntai_up\Application\ProtocolLayer\communicate.c`

**Interfaces:**
- Consumes: 联调记录和 CAN 抓包。
- Produces: 是否进入底盘阶段的结论。

- [ ] **Step 1: 编译验收**

```text
0 errors
无新增未确认警告
```

- [ ] **Step 2: 通信验收**

连续 10 分钟：

```text
下方发送 D1/D2 无持续失败
上板收到 D1/D2，status 保持 DEV_ONLINE
下板收到 C1/C2，offline_cnt 保持 0
CAN 错误计数不增长
```

- [ ] **Step 3: 安全验收**

```text
遥控器离线时 car_state = 0
板间离线时上板卸力
下板复位时上板不保持上一帧出力
上板复位时下板不继续给危险目标
```

- [ ] **Step 4: 遥控器验收**

```text
死区内云台不漂
正负方向正确
最大角速度可控
模式切换不跳变
遥控器重新连接后不瞬间满输出
```

- [ ] **Step 5: 只有全部通过才进入底盘阶段**

未全部通过时，不开始底盘四轮闭环，不接入功率限制和超电策略。

---

## 十二、阶段 9：进入底盘前的接口准备

### Task 13: 为底盘阶段预留接口，但暂时不实现

**Files:**
- Read: `Application/ProtocolLayer/board_protocol.c`
- Read: `Application/ControlLayer/infantry.c`
- Read: `Application/ModuleLayer/Chassis.c`

**Interfaces:**
- Consumes: 当前 `car_pkt.v_x`、`car_pkt.v_y`。
- Produces: 底盘阶段可直接接入的速度目标和模式来源。

- [ ] **Step 1: 确认现有下板发送速度字段**

`Board_Tx_Pkt_01()` 已经发送：

```text
v_x
v_y
```

上板 `Board_Rx_Pkt_01()` 当前没有解析这两个字段。

- [ ] **Step 2: 底盘阶段再决定是否扩展**

底盘阶段需要回答：

```text
上板是否使用底盘速度做前馈
上板是否只使用底盘角速度做自稳
底盘速度是否需要同步给视觉
```

- [ ] **Step 3: 预留但不实现底盘角速度**

用户已决定底盘角速度前馈后续再处理，因此本阶段不改 `0xD1` 布局。

- [ ] **Step 4: 启动底盘阶段前先完成独立验收**

底盘阶段建议从以下顺序开始：

```text
单轮方向测试
四轮同向测试
正反转死区确认
编码器方向确认
速度环阶跃测试
麦轮/全向轮运动学测试
陀螺仪转向测试
功率限制测试
超电联调
```

---

## 十三、推荐执行顺序

按以下顺序做，不跳步：

1. 修复 `connect_task.c` 重复符号。
2. 编译到 `0 errors`。
3. 实现 `Board_Tx_Pkt_01/02` 1 ms 周期发送。
4. 加 D3/D4 的 10 ms 周期发送。
5. 增加发送计数和失败计数。
6. 加 `BOARD_COMM_DEBUG`，隔离底盘、发射、视觉、UI、超电。
7. 用 CAN 分析仪验证帧和 ID。
8. 验证下板收到 C1/C2。
9. 验证上板收到 D1/D2。
10. 验证 D2/C2 四角度的符号、单位、范围。
11. `car_state = 0` 下确认上板卸力。
12. 只允许第一次 `car_state = 1` 做归中测试。
13. 确定遥控器主机：临时上板或正式下板。
14. 若下板做主机，新增 `0xD5` 并同步修改上板。
15. 整理遥控器死区、归一化、最大角速度和离线安全。
16. 标定 Yaw/Pitch 机械中值。
17. 连续运行 10 分钟，完成通信验收。
18. 再进入底盘阶段。

---

## 十四、每一步的完成标准

### 编译

```text
0 errors
目标 DM-MC02
没有未处理的重复符号
没有新增未确认警告
```

### 发送

```text
D1 周期 1 ms
D2 周期 1 ms
D3 周期 10 ms
D4 周期 10 ms
发送失败计数不持续增长
```

### 接收

```text
C1 在线计数回到 0
C2 在线计数回到 0
下板 status = DEV_ONLINE
上板 Board_HeartBeat.status = DEV_ONLINE
```

### 数据

```text
IMU 角误差 <= 0.02 deg
机械角误差 <= 0.001 rad
符号与方向一致
边界值不饱和错误
```

### 安全

```text
遥控器离线 -> car_state = 0
板间离线 -> 上板卸力
初始化未完成 -> 上板不出力
模式切换 -> 无跳变
```

### 遥控器

```text
死区有效
方向正确
最大角速度可控
回中不漂
重连不冲
```

### 进入底盘阶段的条件

```text
以上所有通信、遥控器、安全条件全部连续通过
另一人复核 CAN 抓包和上电步骤
保留一份成功配置的参数记录
```
