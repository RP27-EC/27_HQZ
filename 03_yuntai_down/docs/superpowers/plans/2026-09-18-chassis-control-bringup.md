# 下板底盘控制实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 `03_yuntai_down` 中新增一套干净的固定轮 RM3508 底盘控制模块，先完成遥控器前后/左右/旋转控制、四轮速度闭环和固定限流，再逐步加入速度规划、力前馈、坡面补偿、小陀螺和底盘跟随。

**Architecture:** 不改写旧 `infantry.c` 和旧 `Chassis.c`，新增独立的 `chassis_input`、`chassis_control`、`chassis_planner` 和 `chassis_config`。底盘调试阶段只运行新模块，旧 `infantry`、视觉、裁判、超电、发射和旧底盘逻辑全部旁路。后续整车稳定后，再让 `infantry` 作为薄调度层调用新底盘模块。

**Tech Stack:** STM32H723VGTx、Keil MDK、CMSIS-RTOS2/FreeRTOS、FDCAN1、四个 RM3508、现有 `motor.c/.h`、`RM_motor.c/.h`、`can_protocol.c/.h`、`rc_sensor`。

**Spec:** 当前 `03_yuntai_down` 的四轮固定轮结构、`03_yuntai_up` 的 C2 云台反馈，以及开源教程中的 `crt_chassis.cpp`、`crt_chassis.h`、`alg_slope.cpp` 和 `ita_robot.cpp`。

## Global Constraints

- 当前分支固定使用 `chassis`，不要把底盘代码写到 `yuntai` 分支。
- 不覆盖、不撤销用户现有未提交修改，尤其是 `Application/AlgorithmLayer/PID.c` 和 `Application/DeviceLayer/Chassis_Posture.c`。
- 第一阶段只运行遥控器、底盘运动学、四轮速度环和固定限流。
- 第一阶段不运行视觉、裁判、超电、UI、发射、完整 `infantry.work()`、下板 `gimbal.work()` 和旧 `Chassis.c`。
- 底盘输出固定使用 CAN1、标准帧 ID `0x200`，反馈 ID 使用现有 `RM3508_CAN_ID_201~204`。
- `ch0` 同一时刻只能归底盘旋转或上板云台 Yaw，二者不能同时抢占。
- 第一阶段不使用裁判系统功率限制，使用单独的固定电流或力矩限幅。
- 任意电机离线、遥控离线、输出 NaN 或控制器失能时，四个轮子必须输出 0。
- 每次只增加一个可独立验证的功能，完成后编译并提交。
- Keil 目标固定为 `DM-MC02`，最终要求 `0 errors`。

---

## 文件结构

新增：

```text
Application/ConfigLayer/chassis_config.h
Application/ModuleLayer/chassis_control.h
Application/ModuleLayer/chassis_control.c
Application/ModuleLayer/chassis_input.h
Application/ModuleLayer/chassis_input.c
Application/ModuleLayer/chassis_planner.h
Application/ModuleLayer/chassis_planner.c
```

修改：

```text
Application/DeviceLayer/device.c
Application/TaskLayer/control_task.c
Application/TaskLayer/monitor_task.c
Application/ProtocolLayer/board_protocol.c
MDK-ARM/DM-MC02.uvprojx
```

保留不调用：

```text
Application/ModuleLayer/Chassis.c
Application/ModuleLayer/Chassis.h
Application/ControlLayer/infantry.c
Application/ModuleLayer/launch.c
Application/ModuleLayer/vision.c
Application/DeviceLayer/judge.c
Application/DeviceLayer/cap.c
Application/ModuleLayer/ui.c
```

---

## Task 0: 建立基线和保留现有修改

**Files:**
- Read: `Application/AlgorithmLayer/PID.c`
- Read: `Application/DeviceLayer/Chassis_Posture.c`
- Read: `Application/DeviceLayer/motor.c`
- Read: `Application/HardwareLayer/RM_motor.c`
- Read: `Application/TaskLayer/control_task.c`
- Read: `Application/TaskLayer/monitor_task.c`
- Read: `Application/DeviceLayer/device.c`

**Interfaces:**
- Consumes: 当前 `chassis` 分支状态。
- Produces: 未提交修改清单和可编译基线。

- [ ] **Step 1: 确认分支和未提交文件**

```powershell
git branch --show-current
git status --short
```

Expected:

```text
chassis
 M 03_yuntai_down/Application/AlgorithmLayer/PID.c
 M 03_yuntai_down/Application/DeviceLayer/Chassis_Posture.c
```

- [ ] **Step 2: 编译当前基线**

运行：

```powershell
& 'G:\Keil 5\Core\UV4\UV4.exe' -r 'G:\STM32_ALL\RM_code\Train_code_plus\03_yuntai_down\MDK-ARM\DM-MC02.uvprojx' -t 'DM-MC02' -j0
```

Expected: `0 Error(s)`。

- [ ] **Step 3: 记录编译警告数量**

读取：

```text
MDK-ARM/DM-MC02/DM-MC02.build_log.htm
```

记录 `Error(s)` 和 `Warning(s)` 数量，后续作为新增警告基线。

- [ ] **Step 4: 提交现有用户修改前先确认**

不要自动提交 `PID.c` 和 `Chassis_Posture.c`。只有用户明确要求时才提交这两个文件。

---

## Task 1: 新增底盘配置和公共类型

**Files:**
- Create: `Application/ConfigLayer/chassis_config.h`
- Create: `Application/ModuleLayer/chassis_control.h`

**Interfaces:**
- Consumes: `WHEEL_CNT`、`Motor_RM_t`、`RM3508` 类型。
- Produces: `chassis_cmd_t`、`chassis_source_e`、`chassis_control_state_t`、`chassis_control_t`。

- [ ] **Step 1: 新建配置文件**

```c
#ifndef __CHASSIS_CONFIG_H
#define __CHASSIS_CONFIG_H

#define CHASSIS_BRINGUP_ENABLE          1u
#define CHASSIS_RC_INPUT_ENABLE         1u
#define CHASSIS_KEYBOARD_INPUT_ENABLE   0u
#define CHASSIS_OWNS_RC_YAW             1u

#define CHASSIS_POWER_LIMIT_ENABLE      0u
#define CHASSIS_PLANNER_ENABLE          0u
#define CHASSIS_FEEDFORWARD_ENABLE      0u
#define CHASSIS_GIMBAL_FOLLOW_ENABLE    0u

#define CHASSIS_CONTROL_PERIOD_MS       1u

#define CHASSIS_CTRL_MAX_SPEED          80.0f
#define CHASSIS_MAX_VX                  50.0f
#define CHASSIS_MAX_VY                  50.0f
#define CHASSIS_MAX_WZ                  40.0f
#define CHASSIS_TURN_CYCLE_SPEED         55.0f
#define CHASSIS_RC_DEADBAND              0.0f
#define CHASSIS_RC_AXIS_MAX             660.0f

#define CHASSIS_SPEED_KP                1.0f
#define CHASSIS_SPEED_KI                0.0f
#define CHASSIS_SPEED_KD                0.0f
#define CHASSIS_TEST_TORQUE_LIMIT_NM    5.4f

#define CHASSIS_LENGTH_M                0.39994f
#define CHASSIS_WIDTH_M                 0.39990f
#define CHASSIS_DIAGONAL_LENGTH_M       0.56558f
#define CHASSIS_MASS_KG                 11.85f
#define CHASSIS_WHEEL_RADIUS_M          0.154f
#define CHASSIS_GRAVITY_MPS2            9.81f

#endif
```

- [ ] **Step 2: 定义公共底盘命令和状态类型**

```c
typedef enum
{
    CHASSIS_SRC_NONE = 0,
    CHASSIS_SRC_RC,
    CHASSIS_SRC_KEYBOARD,
} chassis_source_e;

typedef struct
{
    float vx;
    float vy;
    float wz;
    uint8_t valid;
    chassis_source_e source;
} chassis_cmd_t;

typedef struct
{
    float wheel_target[WHEEL_CNT];
    float wheel_speed[WHEEL_CNT];
    float wheel_torque_out[WHEEL_CNT];

    uint8_t wheel_online[WHEEL_CNT];
    uint8_t all_online;

    chassis_cmd_t cmd;
    uint8_t enabled;
    uint8_t fault;
} chassis_control_state_t;
```

- [ ] **Step 3: 编译验证**

Expected: 新头文件可被现有工程包含，`0 errors`。

---

## Task 2: 实现遥控器和键鼠输入层

**Files:**
- Create: `Application/ModuleLayer/chassis_input.h`
- Create: `Application/ModuleLayer/chassis_input.c`
- Modify: `Application/TaskLayer/control_task.c`

**Interfaces:**
- Consumes: `rc_sensor.info->ch0/ch1/ch2/ch3`。
- Produces: `Chassis_Input_Init()`、`Chassis_Input_Update()`、`chassis_input_cmd`。

- [ ] **Step 1: 定义输入接口**

```c
void Chassis_Input_Init(void);
void Chassis_Input_Update(void);
void Chassis_Input_SetSource(chassis_source_e source);
extern chassis_cmd_t chassis_input_cmd;
```

- [ ] **Step 2: 实现遥控器通道映射**

```c
static float Chassis_Rc_Axis_Value(int16_t axis)
{
    float value = (float)axis;

    if ((value > -CHASSIS_RC_DEADBAND) && (value < CHASSIS_RC_DEADBAND))
    {
        return 0.0f;
    }

    value -= (value > 0.0f) ? CHASSIS_RC_DEADBAND : -CHASSIS_RC_DEADBAND;
    value /= (CHASSIS_RC_AXIS_MAX - CHASSIS_RC_DEADBAND);
    return constrain(value, -1.0f, 1.0f);
}
```

```c
chassis_input_cmd.vx = Chassis_Rc_Axis_Value(rc_sensor.info->ch3) * CHASSIS_MAX_VX;
chassis_input_cmd.vy = Chassis_Rc_Axis_Value(rc_sensor.info->ch2) * CHASSIS_MAX_VY;
chassis_input_cmd.wz = Chassis_Rc_Axis_Value(rc_sensor.info->ch0) * CHASSIS_MAX_WZ;
chassis_input_cmd.valid = (rc_sensor.work_state == DEV_ONLINE);
chassis_input_cmd.source = CHASSIS_SRC_RC;
```

- [ ] **Step 3: 遥控离线清零**

```c
if (rc_sensor.work_state != DEV_ONLINE)
{
    chassis_input_cmd.vx = 0.0f;
    chassis_input_cmd.vy = 0.0f;
    chassis_input_cmd.wz = 0.0f;
    chassis_input_cmd.valid = 0u;
}
```

- [ ] **Step 4: 预留键鼠输入**

```c
#if CHASSIS_KEYBOARD_INPUT_ENABLE
chassis_input_cmd.vx =
    ((rc_sensor.info->W.status == press_to_release) ? CHASSIS_MAX_VX : 0.0f) -
    ((rc_sensor.info->S.status == press_to_release) ? CHASSIS_MAX_VX : 0.0f);
chassis_input_cmd.vy =
    ((rc_sensor.info->A.status == press_to_release) ? CHASSIS_MAX_VY : 0.0f) -
    ((rc_sensor.info->D.status == press_to_release) ? CHASSIS_MAX_VY : 0.0f);
chassis_input_cmd.wz = constrain(rc_sensor.info->mouse_vx, -CHASSIS_MAX_WZ, CHASSIS_MAX_WZ);
chassis_input_cmd.source = CHASSIS_SRC_KEYBOARD;
#endif
```

- [ ] **Step 5: 控制任务中先只更新输入，不输出电机**

```c
#if CHASSIS_BRINGUP_ENABLE
Chassis_Input_Update();
#endif
```

- [ ] **Step 6: Keil Watch 验证**

观察：

```text
chassis_input_cmd.vx
chassis_input_cmd.vy
chassis_input_cmd.wz
chassis_input_cmd.valid
chassis_input_cmd.source
```

拨动左/右摇杆，确认数值变化方向正确，电机保持不输出。

---

## Task 3: 建立底盘控制对象和安全停止

**Files:**
- Create: `Application/ModuleLayer/chassis_control.c`
- Modify: `Application/ModuleLayer/chassis_control.h`
- Modify: `Application/DeviceLayer/device.c`
- Modify: `Application/TaskLayer/control_task.c`

**Interfaces:**
- Consumes: `wheel_group`、`wheel_motor[4]`。
- Produces: `Chassis_Control_Init()`、`Chassis_Control_SetEnable()`、`Chassis_Control_Stop()`、`Chassis_Control_Update()`。

- [ ] **Step 1: 定义控制对象**

```c
typedef struct
{
    Motor_RM_Group_t *wheel;
    chassis_control_state_t state;

    void (*init)(void);
    void (*update)(const chassis_cmd_t *cmd);
    void (*stop)(void);
} chassis_control_t;

extern chassis_control_t chassis_ctrl;
```

- [ ] **Step 2: 实现初始化和 PID 参数写入**

```c
for (uint8_t i = 0u; i < WHEEL_CNT; i++)
{
    pid_ctrl_t *pid = chassis_ctrl.wheel->motor[i]->ctrl->speed_ctrl;
    pid->kp = CHASSIS_SPEED_KP;
    pid->ki = CHASSIS_SPEED_KI;
    pid->kd = CHASSIS_SPEED_KD;
    pid->integral_max = 0.0f;
    pid->out_max = CHASSIS_TEST_TORQUE_LIMIT_NM;
}
```

- [ ] **Step 3: 实现安全停止**

```c
for (uint8_t i = 0u; i < WHEEL_CNT; i++)
{
    chassis_ctrl.wheel->motor[i]->tx_info->torque = 0.0f;
}
chassis_ctrl.wheel->group_set_torque(chassis_ctrl.wheel);
```

- [ ] **Step 4: 在设备初始化和心跳中接入**

`device.c` 调试分支加入：

```c
rm_motor_list_init();
Chassis_Control_Init();
```

`monitor_task.c` 调试模式下仍执行：

```c
rm_motor_list_heart_beat();
```

如果当前仍使用 `BOARD_COMM_DEBUG`，把心跳条件改成：

```c
#if !BOARD_COMM_DEBUG || CHASSIS_BRINGUP_ENABLE
    rm_motor_list_heart_beat();
#endif
```

- [ ] **Step 5: 在控制任务中禁止输出**

只执行：

```c
Chassis_Control_Stop();
```

确认四个轮子输出为 0。

---

## Task 4: 实现四轮运动学逆解

**Files:**
- Modify: `Application/ModuleLayer/chassis_control.c`
- Modify: `Application/ModuleLayer/chassis_control.h`

**Interfaces:**
- Consumes: `chassis_cmd_t.vx/vy/wz`。
- Produces: `state.wheel_target[4]`。

- [ ] **Step 1: 实现轮速目标分配**

```c
static void Chassis_Kinematics_Inverse(const chassis_cmd_t *cmd)
{
    float front = cmd->vx;
    float left = cmd->vy;
    float cycle = cmd->wz;

    chassis_ctrl.state.wheel_target[WHEEL_LF] = -front + left + cycle;
    chassis_ctrl.state.wheel_target[WHEEL_LB] = -front - left + cycle;
    chassis_ctrl.state.wheel_target[WHEEL_RF] =  front + left + cycle;
    chassis_ctrl.state.wheel_target[WHEEL_RB] =  front - left + cycle;
}
```

- [ ] **Step 2: 增加总速度限制**

```c
float trans = fabsf(front) + fabsf(left);
float rotate = fabsf(cycle);
float total = trans + rotate;

if (total > CHASSIS_CTRL_MAX_SPEED)
{
    float k = CHASSIS_CTRL_MAX_SPEED / total;
    front *= k;
    left *= k;
    cycle *= k;
}
```

- [ ] **Step 3: Watch 验证**

分别拨动：

```text
左摇杆上下 -> LF/LB/RF/RB 应同时反向变化
左摇杆左右 -> 四轮左右差速
右摇杆左右 -> 四轮旋转差速
```

电机仍不输出。

---

## Task 5: 加入四轮速度 P 控制和固定限流

**Files:**
- Modify: `Application/ModuleLayer/chassis_control.c`

**Interfaces:**
- Consumes: `wheel_target[4]`、`wheel_motor[i].rx_info->speed`。
- Produces: `wheel_torque_out[4]`。

- [ ] **Step 1: 更新轮速反馈**

```c
chassis_ctrl.state.wheel_speed[i] = chassis_ctrl.wheel->motor[i]->rx_info->speed;
```

- [ ] **Step 2: 执行速度 P 控制**

```c
pid_ctrl_t *pid = chassis_ctrl.wheel->motor[i]->ctrl->speed_ctrl;
pid->target = chassis_ctrl.state.wheel_target[i];
pid->measure = chassis_ctrl.state.wheel_speed[i];
pid->err = pid->target - pid->measure;
single_pid_ctrl(pid);
chassis_ctrl.state.wheel_torque_out[i] = pid->out;
```

- [ ] **Step 3: 输出力矩限幅**

```c
chassis_ctrl.state.wheel_torque_out[i] =
    constrain(chassis_ctrl.state.wheel_torque_out[i],
              -CHASSIS_TEST_TORQUE_LIMIT_NM,
              CHASSIS_TEST_TORQUE_LIMIT_NM);
```

- [ ] **Step 4: 写入底层并发送**

```c
chassis_ctrl.wheel->motor[i]->tx_info->torque =
    chassis_ctrl.state.wheel_torque_out[i];

chassis_ctrl.wheel->group_set_torque(chassis_ctrl.wheel);
```

- [ ] **Step 5: 上架验证**

四个轮子空载，目标速度从 0 缓慢增加。确认：

```text
轮子转向正确
速度反馈符号正确
目标速度与反馈速度趋势一致
没有过流或突然反转
```

---

## Task 6: 处理 ch0 的底盘/云台归属冲突

**Files:**
- Modify: `Application/ProtocolLayer/board_protocol.c`
- Modify: `Application/ModuleLayer/chassis_input.c`
- Modify: `Application/ConfigLayer/chassis_config.h`

**Interfaces:**
- Consumes: `CHASSIS_OWNS_RC_YAW`。
- Produces: `ch0` 唯一归属规则。

- [ ] **Step 1: 底盘占用 ch0 时，上板 Yaw 清零**

```c
#if CHASSIS_BRINGUP_ENABLE && CHASSIS_OWNS_RC_YAW
    yaw_rate = 0.0f;
#else
    yaw_rate = Board_Remote_Axis_To_Rate(rc_sensor.info->ch0,
                                         BOARD_D5_YAW_RATE_MAX_DEG_S);
#endif
```

- [ ] **Step 2: 底盘不占用 ch0 时，底盘 wz 清零**

```c
#if CHASSIS_OWNS_RC_YAW
    chassis_input_cmd.wz = Chassis_Rc_Axis_Value(rc_sensor.info->ch0) * CHASSIS_MAX_WZ;
#else
    chassis_input_cmd.wz = 0.0f;
#endif
```

- [ ] **Step 3: 验证互斥**

拨动 `ch0`：

```text
CHASSIS_OWNS_RC_YAW = 1：
    底盘 wz 变化
    D5 yaw_rate = 0

CHASSIS_OWNS_RC_YAW = 0：
    底盘 wz = 0
    D5 yaw_rate 变化
```

---

## Task 7: 增加速度规划和输入平滑

**Files:**
- Create: `Application/ModuleLayer/chassis_planner.h`
- Create: `Application/ModuleLayer/chassis_planner.c`
- Modify: `Application/ModuleLayer/chassis_control.c`

原底盘代码没有速度规划器参数，因此本任务默认关闭，只预留后续可标定的实现。

**Interfaces:**
- Consumes: `chassis_cmd_t`。
- Produces: `Chassis_Planner_Update(const chassis_cmd_t *in, chassis_cmd_t *out)` 和 `extern chassis_cmd_t chassis_planned_cmd;`。

- [ ] **Step 1: 定义规划参数**

```c
/* 这些不是原代码参数，默认关闭规划器，标定后才能使用。*/
#define CHASSIS_VX_ACCEL_PER_MS     0.05f
#define CHASSIS_VX_DECEL_PER_MS     0.08f
#define CHASSIS_VY_ACCEL_PER_MS     0.05f
#define CHASSIS_VY_DECEL_PER_MS     0.08f
#define CHASSIS_WZ_ACCEL_PER_MS     0.08f
#define CHASSIS_WZ_DECEL_PER_MS     0.12f
```

- [ ] **Step 2: 实现通用斜坡**

```c
static float Chassis_Ramp(float now, float target, float accel, float decel)
{
    float delta = target - now;
    float step;

    if (fabsf(target) > fabsf(now))
    {
        step = accel;
    }
    else
    {
        step = decel;
    }

    if (delta > step) return now + step;
    if (delta < -step) return now - step;
    return target;
}
```

- [ ] **Step 3: 接入控制链**

```c
chassis_cmd_t planned_cmd;
Chassis_Planner_Update(&chassis_input_cmd, &planned_cmd);
Chassis_Control_Update(&planned_cmd);
```

- [ ] **Step 4: 验证起步和刹车**

遥控器从中位快速推到满杆，观察输出逐步增加，不出现明显冲击。

---

## Task 8: 增加力前馈和阻力补偿接口

**Files:**
- Modify: `Application/ModuleLayer/chassis_control.c`
- Modify: `Application/ModuleLayer/chassis_control.h`
- Modify: `Application/ConfigLayer/chassis_config.h`

**Interfaces:**
- Consumes: `wheel_target[4]`、`wheel_speed[4]`。
- Produces: `wheel_torque_ff[4]`。

- [ ] **Step 1: 定义前馈结构**

```c
float wheel_torque_ff[WHEEL_CNT];
float wheel_torque_corr[WHEEL_CNT];
float wheel_torque_out[WHEEL_CNT];
```

- [ ] **Step 2: 计算阻力前馈**

```c
static float Chassis_Drag_Feedforward(float target_speed, float now_speed)
{
    const float coulomb = 0.0f;
    const float viscous = 0.0f;
    const float threshold = 1.0f;

    if (target_speed > threshold)
    {
        return coulomb + viscous * now_speed;
    }
    if (target_speed < -threshold)
    {
        return -coulomb + viscous * now_speed;
    }
    return 0.0f;
}
```

- [ ] **Step 3: 加入弱速度修正**

```c
wheel_torque_corr[i] =
    CHASSIS_SPEED_KP * (wheel_target[i] - wheel_speed[i]);

wheel_torque_out[i] =
    wheel_torque_ff[i] + wheel_torque_corr[i];
```

- [ ] **Step 4: 保持默认关闭**

```c
#if CHASSIS_FEEDFORWARD_ENABLE
    wheel_torque_ff[i] = Chassis_Drag_Feedforward(wheel_target[i], wheel_speed[i]);
#else
    wheel_torque_ff[i] = 0.0f;
#endif
```

- [ ] **Step 5: 调整速度修正系数**

先把：

```c
CHASSIS_SPEED_KP = 1.0f
```

调通后降低到：

```text
0.2f ~ 0.3f
```

观察前馈是否接管主要输出。

---

## Task 9: 增加坡面重力前馈基础

**Files:**
- Modify: `Application/ModuleLayer/chassis_control.c`
- Modify: `Application/ConfigLayer/chassis_config.h`
- Read: `Application/DeviceLayer/Chassis_Posture.c`

**Interfaces:**
- Consumes: 下板 IMU Pitch、Roll 或现有 `Chassis_Posture`。
- Produces: `slope_force_ff`。

- [ ] **Step 1: 定义参数**

不新增同名参数，直接使用 Task 1 中按原代码写入的值：

```text
CHASSIS_MASS_KG         = 11.85f
CHASSIS_WHEEL_RADIUS_M  = 0.154f
CHASSIS_GRAVITY_MPS2    = 9.81f
```

- [ ] **Step 2: 计算坡面力**

角度来源必须明确。若下板 IMU 已初始化，读取：

```c
float pitch_rad = imu_sensor.info->base_info.pitch * GIMBAL_DEG_TO_RAD;
float roll_rad  = imu_sensor.info->base_info.roll  * GIMBAL_DEG_TO_RAD;
```

```c
float force_x =
    CHASSIS_MASS_KG * CHASSIS_GRAVITY_MPS2 * sinf(-pitch_rad);
float force_y =
    CHASSIS_MASS_KG * CHASSIS_GRAVITY_MPS2 * sinf(roll_rad);
```

- [ ] **Step 3: 分配到四轮**

使用固定轮逆解方向分配：

```c
wheel_torque_ff[WHEEL_LF] += (-force_x + force_y) * CHASSIS_WHEEL_RADIUS_M / 4.0f;
wheel_torque_ff[WHEEL_LB] += (-force_x - force_y) * CHASSIS_WHEEL_RADIUS_M / 4.0f;
wheel_torque_ff[WHEEL_RF] += ( force_x + force_y) * CHASSIS_WHEEL_RADIUS_M / 4.0f;
wheel_torque_ff[WHEEL_RB] += ( force_x - force_y) * CHASSIS_WHEEL_RADIUS_M / 4.0f;
```

- [ ] **Step 4: 限幅和死区**

```c
wheel_torque_ff[i] = constrain(wheel_torque_ff[i], -0.5f, 0.5f);
if (fabsf(pitch_rad) < 0.02f && fabsf(roll_rad) < 0.02f)
{
    wheel_torque_ff[i] = 0.0f;
}
```

- [ ] **Step 5: 台架验证**

将底盘放在小坡上，先手动拨动轮子观察方向。确认补偿方向正确后，再开启自动补偿。

---

## Task 10: 增加固定功率/电流兜底限制

**Files:**
- Modify: `Application/ModuleLayer/chassis_control.c`
- Modify: `Application/ConfigLayer/chassis_config.h`

**Interfaces:**
- Consumes: 四轮目标力矩。
- Produces: 统一缩放系数。

- [ ] **Step 1: 定义训练限流参数**

```c
#define CHASSIS_FIXED_CURRENT_LIMIT_A   20.0f
#define CHASSIS_FIXED_TORQUE_LIMIT_NM   5.4f
```

- [ ] **Step 2: 单轮限幅**

```c
wheel_torque_out[i] =
    constrain(wheel_torque_out[i],
              -CHASSIS_FIXED_TORQUE_LIMIT_NM,
              CHASSIS_FIXED_TORQUE_LIMIT_NM);
```

- [ ] **Step 3: 总功率估算**

第一版使用固定总力矩限制，不接入裁判系统。

```c
float total_torque =
    fabsf(wheel_torque_out[0]) +
    fabsf(wheel_torque_out[1]) +
    fabsf(wheel_torque_out[2]) +
    fabsf(wheel_torque_out[3]);

if (total_torque > CHASSIS_FIXED_TORQUE_LIMIT_NM * 4.0f)
{
    float factor = (CHASSIS_FIXED_TORQUE_LIMIT_NM * 4.0f) / total_torque;
    for (uint8_t i = 0; i < WHEEL_CNT; i++)
    {
        wheel_torque_out[i] *= factor;
    }
}
```

- [ ] **Step 4: 验证**

在轮子堵转时，四轮总输出不应超过设定上限。

原工程宏为 `CHASSIS_SWITCH=1`、`POWER_LIMIT_SWITCH=1`。本训练阶段因为裁判和超电关闭，即使宏值保留，也不调用旧的 `New_Chassis_Power_Limit()`，而是使用本任务的固定限幅。

---

## Task 11: 预留小陀螺和底盘跟随

**Files:**
- Modify: `Application/ModuleLayer/chassis_control.c`
- Modify: `Application/ModuleLayer/chassis_input.c`
- Modify: `Application/ConfigLayer/chassis_config.h`

**Interfaces:**
- Consumes: 上板 C2 的 `board.rx_meg->gimbal_meg.yaw_imu` 或下板 IMU Yaw。
- Produces: `wz_cmd_ff`、`chassis_follow_error`。

- [ ] **Step 1: 定义小陀螺参数**

```c
#define CHASSIS_GYRO_OMEGA_RAD_S        CHASSIS_TURN_CYCLE_SPEED
#define CHASSIS_GYRO_DELAY_FORWARD      0.0f
```

- [ ] **Step 2: 小陀螺模式只叠加角速度前馈**

先定义角度来源和弧度包装函数：

```c
static float Chassis_Wrap_Pi(float angle)
{
    return atan2f(sinf(angle), cosf(angle));
}

float gimbal_yaw =
    board.rx_meg->gimbal_meg.yaw_imu * GIMBAL_DEG_TO_RAD;
```

```c
#if CHASSIS_GIMBAL_FOLLOW_ENABLE
    if (small_gyro_enable)
    {
        cmd.wz += CHASSIS_GYRO_OMEGA_RAD_S;
    }
#endif
```

- [ ] **Step 3: 增加底盘跟随**

```c
float follow_error = -Chassis_Wrap_Pi(gimbal_yaw);
float follow_omega = -300.0f * follow_error * fabsf(follow_error);
follow_omega = constrain(follow_omega, -CHASSIS_MAX_WZ, CHASSIS_MAX_WZ);
cmd.wz += follow_omega;
```

- [ ] **Step 4: 前馈补偿姿态延迟**

```c
float yaw_for_planning =
    gimbal_yaw - chassis_omega * CHASSIS_GYRO_DELAY_FORWARD;
```

- [ ] **Step 5: 只在基础底盘稳定后开启**

保持默认：

```c
#define CHASSIS_GIMBAL_FOLLOW_ENABLE 0u
```

---

## Task 12: 与旧 infantry 解耦和最终集成

**Files:**
- Modify: `Application/TaskLayer/control_task.c`
- Modify: `Application/DeviceLayer/device.c`
- Read: `Application/ControlLayer/infantry.c`

**Interfaces:**
- Consumes: 新的 `Chassis_Input_Update()`、`Chassis_Planner_Update()`、`Chassis_Control_Update()`。
- Produces: 底盘调试运行链和旧的整车运行链互斥。

- [ ] **Step 1: 底盘调试分支只运行底盘模块**

```c
#if CHASSIS_BRINGUP_ENABLE
    Chassis_Input_Update();
#if CHASSIS_PLANNER_ENABLE
    Chassis_Planner_Update(&chassis_input_cmd, &chassis_planned_cmd);
    Chassis_Control_Update(&chassis_planned_cmd);
#else
    Chassis_Control_Update(&chassis_input_cmd);
#endif
#else
    infantry.work(&infantry);
#endif
```

- [ ] **Step 2: 禁止调试模式调用旧整车模块**

确保调试分支不存在：

```c
infantry.work(&infantry);
chassis.work(&chassis);
gimbal.work(&gimbal);
launch.work(&launch);
vision.work(&vision);
```

- [ ] **Step 3: 保留旧的 `infantry.c` 不删除**

后续待新底盘稳定后，再把 `infantry.c` 精简为：

```text
模式切换
输入仲裁
调用 Chassis_Control_Update
调用 Gimbal_Bridge_Update
```

- [ ] **Step 4: 最终全量编译**

打开：

```text
03_yuntai_down/MDK-ARM/DM-MC02.uvprojx
```

Target：

```text
DM-MC02
```

Expected: `0 Error(s)`。

---

## 验收标准

### 第一阶段静态验收

- 新底盘模块不引用 `infantry`、`judge`、`cap`、`launch`、`vision`。
- 旧 `Chassis.c` 和 `infantry.c` 不参与调试运行链。
- 遥控器离线时四轮输出为 0。
- 任意 RM3508 离线时四轮输出为 0。
- 速度规划和力矩限幅都存在。
- D5 的 Yaw 和底盘 `wz` 不会同时使用 `ch0`。

### 第一阶段上架验收

- 左摇杆上下控制前后。
- 左摇杆左右控制左右平移。
- 右摇杆左右控制底盘旋转。
- 四轮方向一致。
- 遥控器回中后底盘停止。
- 遥控器失联后约 100 ms 内卸力。
- 最大输出不超过固定测试限幅。

### 第二阶段力控验收

- 速度误差较小时，小 Kp 能修正稳态误差。
- 阻力前馈不会引起零点抖动。
- 坡面补偿方向和符号正确。
- 总输出不超过固定功率或电流限制。

---

## 调试变量

Keil Watch 重点观察：

```text
chassis_input_cmd.vx
chassis_input_cmd.vy
chassis_input_cmd.wz
chassis_input_cmd.valid

chassis_planned_cmd.vx
chassis_planned_cmd.vy
chassis_planned_cmd.wz

chassis_ctrl.state.wheel_target[0..3]
chassis_ctrl.state.wheel_speed[0..3]
chassis_ctrl.state.wheel_torque_ff[0..3]
chassis_ctrl.state.wheel_torque_out[0..3]
chassis_ctrl.state.all_online
chassis_ctrl.state.enabled
chassis_ctrl.state.fault
```

---

## 禁止事项

- 不在 `infantry.c` 中新增底盘业务逻辑。
- 不在 `gimbal.c` 中写底盘代码。
- 不把视觉、裁判、超电、发射作为底盘基础控制的前置条件。
- 不直接启用旧 `New_Chassis_Power_Limit()`，因为它依赖 `judge` 和 `cap`。
- 不把键盘和鼠标数据硬塞进现有 D5，后续需要独立输入结构或独立协议。
- 不在未确认四轮方向前开启较大速度目标。
- 不在未编译和未上架验证的情况下连续提交多个阶段。
