# 速控模式 Pitch 惯性自稳实现计划（适配 Train_code_plus 上板）

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 `G_RATE` 模式下，用 EKF 姿态（机体→世界四元数）与本车 pitch 关节编码器角/角速度重建“枪管相对水平面的仰角及仰角变化率”，使速控 Pitch 的保持环与速率环工作在该惯性量上，实现车体倾斜补偿、操作输入控制与松杆保持。

**Architecture:** 只新增一条 `G_RATE` 专用 Pitch 通路，`G_MEC`、上电归中、升降机械锁零逐字保持不变。`G_RATE` 的 Pitch 目标域由 `gimbal_feedforward_t.pitch_inertial_domain` 决定，形成三分支：升降锁零 → 机械零位环；IMU 有效且开关打开 → 惯性环；否则 → 原机械环。设备层向上发布机体→世界四元数与去偏机体系角速度，几何重建放在 `gimbal.c` 内，用局部 3×3 矩阵手算，不复用 EKF 的全局矩阵句柄。

**Tech Stack:** STM32F407 上板 Keil 工程 `My_C`、FreeRTOS 1 kHz 控制任务、BMI088 + 四元数 EKF、DM4310 MIT 力矩模式、libm `cosf`/`sinf`/`atan2f`/`sqrtf`（与 `gimbal.c` 现有 `cosf` 风格一致；如需削减耗时可在 Task 4 换 `arm_atan2_f32`/`arm_sqrt_f32`）。

**Spec:** 用户提供的《速控模式 pitch 惯性自稳修改计划》，以及针对本仓库的可行性复核结论。关键结论：参考工程 `zip/infantry_up/user/gimbal/cloud_terrace.c:601` 的 Pitch 惯性环建立在“IMU 随 pitch 转动”前提上（`angle = imu.pitch_deg` 直接当枪管仰角），与本车“IMU 固定在偏航部分”的前提相反，不可照搬。

---

## Global Constraints

- 只改 `G_RATE` 的 Pitch。`G_MEC`、`G_INIT`、`G_SLEEP` 的 Pitch 行为不得变化；`G_MEC` 与 `G_RATE` 现有的机械 Pitch 通路（共用 `pitch_hold` + `pitch_gyro_inner`）保留原样。
- 安装前提：**IMU 固定在偏航部分、不随 pitch 转动**。依据 `gimbal.c:763-767` 现有注释。启用前必须由人工用验收第 1 步实测坐实。
- 不改板间协议：`communicate.c:128-130` 的 `pitch_mec` / `pitch_imu` 字段含义与字节布局不变；不改 CAN ID、`D2` 目标包、`D5` 输入包。
- 不改 IMU 安装矩阵：`bmi_EKF.c:25-30` 的 `ekf_imu_frame`（`arz=90°`）与 `imu_frame_rotate()` 的行向量右乘语义保持不变。
- **禁止复用** `bmi_EKF.c:9-11` 的全局 `ekf_trans` / `ekf_src` / `ekf_dst`；禁止在 `gimbal.c` 内调用 `imu_frame_init()`。重建一律使用局部 `float R[3][3]`。
- 仅支持 `IMU_USE_EKF==1`（`docs/imu.md:82` 记录当前为 1，实施前复核 `rp_driver_config.h`）。MAHONY 路径不发布四元数时，惯性分支因有效性门限自动回退。
- 新参数集中在 `task_up/Application/ConfigLayer/gimbal_rate_config.h`；Keil Watch 在线可调的字段**追加在 `gimbal_tune_t` 末尾**，不得插入中间，以免打乱已有 Watch 布局。
- 单位：几何重建内部使用 rad / rad·s；写入 `base_info` 供观察与控制器使用的仰角、仰角速率使用 deg / deg·s；速率环输出为 N·m。
- 不写测试用例、不写 Mock、不做 TDD 红绿循环、不自动烧录、不进入自测排错循环。每个任务的收尾是**回读改动文件做静态核对**与**人工实机验收项**。
- 注释规范：结构体成员与 config 层逐行注释（物理含义 + 量纲/范围），函数业务注释 ≤15 字，只用 `TODO:` / `FIXME:` / `NOTE:` / `WARNING:` 四种标签。
- 交付粒度按完整单元（完整函数、完整结构体片段），不得使用占位符或缩减版代码。

---

## Review Focus

1. **`a_B`（关节转轴）方向写反** —— 人的预期是“启用后手推枪管，重建仰角朝物理正确方向变化”；写反则 `J` 符号错，速率环变成正反馈，启用瞬间往机械端点冲。由 Task 4 的退化自检核对与人工验收第 2 步拦截。
2. **四元数转矩阵写成了 world→body** —— 人的预期是“车体纯偏航时仰角不变、车体纯俯仰时仰角跟着变”；转置错误表现为行驶中仰角随偏航乱跳、roll 大时完全失真。由 Task 4 的矩阵逐元素核对拦截。
3. **速率环反馈误用机械角速度** —— 人的预期是“抬起车头，枪管保持水平参照”；若把 `pitch_mec_speed`（或其 `/J`）当反馈，底盘俯仰扰动完全不被抵消，静态却看不出异常。由 Task 6 的分支核对与人工验收第 3 步拦截。
4. **升降锁零期间误入惯性域** —— 人的预期是“升降下行时枪管回到机械零位”。惯性域的 `0` 是“枪管水平”而不是“机械中点”，混用会让升降联锁失效。由 Task 5 的锁零分支核对与人工验收第 4 步拦截。
5. **EKF 冻结或 IMU 失效未及时回退** —— 人的预期是“IMU 拔掉后 10 ms 内退出惯性环且不再累加目标”；无回退则目标持续累加、输出打满。由 Task 4 的门限核对与人工验收第 5 步拦截。

---

## File Structure

| 文件 | 动作 | 职责 |
| --- | --- | --- |
| `task_up/Application/DeviceLayer/Sensor/imu_sensor.h` | 修改 | 在 `imu_fused_t` 中发布机体→世界四元数、去偏机体系角速度、EKF 更新计数 |
| `task_up/Application/DeviceLayer/Sensor/imu_sensor.c` | 修改 | 在 `ekf_update()` 之后把 `g_ekf.q` / `g_ekf.Gyro` / `g_ekf.UpdateCount` 复制进 `base_info` |
| `task_up/Application/ConfigLayer/gimbal_rate_config.h` | 修改 | 集中放置开关、安装几何、门限、新环初值、滤波与限速 |
| `task_up/Application/ModuleLayer/gimbal.h` | 修改 | 新增目标域枚举、观察量、惯性目标、两个惯性 PID、Watch 可调字段 |
| `task_up/Application/ModuleLayer/gimbal.c` | 修改 | 几何重建、三分支目标生成、`/J` 速率环输出、状态维护与回退 |
| `task_up/docs/gimbal.md` | 修改 | 补速控 Pitch 惯性自稳章节、目标域表、配置表与观察量 |
| `task_up/docs/imu.md` | 修改 | 补新发布字段与四元数约定说明 |

分工边界：设备层只负责“把 EKF 结果按单位发布出来”，不做任何几何换算；`gimbal.c` 负责全部几何、门限、回退与控制；配置层只放数值。

---

## 数学基准（实施前先读，已按本工程约定逐条复核）

约定：列向量；`B` = EKF 使用的机体系（已过 `imu_frame_rotate`）；`W` = EKF 世界系（Z 向上，依据 `bmi_EKF.c:325-327` 的观测预测 `h(x)=[2(q1q3−q0q2), 2(q0q1+q2q3), q0²−q1²−q2²+q3²]` 恰为标准 `R(body→world)` 第三行）；`θ` = pitch 关节机械角（rad，零点为 `pitch_mec_angle == 0`）；`a_B` = 关节角增大方向的单位转轴；`u_B0` = `θ=0` 时枪管朝向单位向量。

**1. 四元数转机体→世界矩阵（w,x,y,z）**

```text
R = [ 1-2(q2²+q3²),   2(q1q2-q0q3),   2(q1q3+q0q2) ]
    [ 2(q1q2+q0q3),   1-2(q1²+q3²),   2(q2q3-q0q1) ]
    [ 2(q1q3-q0q2),   2(q2q3+q0q1),   1-2(q1²+q2²) ]
```

**2. 关节旋转（Rodrigues）与世界系枪管方向**

```text
u_B = u_B0·cosθ + (a_B×u_B0)·sinθ + a_B·(a_B·u_B0)·(1-cosθ)
u_W = R·u_B
h   = sqrt(u_Wx² + u_Wy²)          // 等于 cos(仰角)
```

**3. 仰角、仰角速率、传动比**

```text
α_raw  = atan2(-u_Wz, h)                                   // 与 g_ekf.Pitch 同号约定
ω_W    = R·(ω_B + a_B·θ̇)                                   // ω_B = g_ekf.Gyro，rad/s
u̇_W    = ω_W × u_W
α̇_raw  = -u̇_Wz / h
J_raw  = -[R·(a_B×u_B)]_z / h                              // = dα_raw/dθ
```

**WARNING:** `J` 里的叉乘必须用**旋转后的** `u_B`（即 `a_B×u_B`），不是 `a_B×u_B0`。两者一般不平行，取错会让传动比在关节行程两端明显偏差。

**4. 符号约定与对外量**：本 EKF 的 ZYX 提取满足 `R31 = −sinθ`，故 `α_raw` 在“机体 X 轴下俯”时为正。对外发布的量定义为**仰角**（枪管上抬为正），统一乘 `pitch_inertial_sign`（默认 `-1.0f`）：

```text
elev_deg       = sign · α_raw · RAD_TO_DEG
elev_rate_dps  = sign · α̇_raw · RAD_TO_DEG
J_signed       = sign · J_raw
```

`sign` 只影响“操作方向”，不影响稳定性与几何正确性；几何正确性由 `a_B` / `u_B0` 决定，两者职责不可互换。

**5. 基座项分离**（端点拦截与关节速率限幅要用）

```text
elev_rate_base_dps = elev_rate_dps - J_signed · θ̇_deg      // 关节不动时，仅基座运动带来的仰角变化率
θ̇_target           = (elev_rate_target - elev_rate_base_dps) / J_signed
```

**6. 静态退化自检（实施时先做，是本方案唯一的廉价正确性证明）**：取 `u_B0=(1,0,0)`、`a_B=(0,1,0)`、`roll=yaw=0`、四元数对应机体俯仰 `θ_b = g_ekf.Pitch` 时，可手算得

```text
u_B = (cosθ, 0, -sinθ)，u_W = (cos(θ_b+θ), 0, -sin(θ_b+θ))
elev_deg = -(g_ekf.Pitch + pitch_mec_angle)
J_signed = -1
```

即恒等式 **`elev_deg + g_ekf.Pitch + pitch_mec_angle == 0`**（误差在浮点噪声级），且 `J_signed == -1`。

`NOTE:` 默认 `a_B=(0,1,0)` 在本 EKF 约定下表示**关节角增大时枪管下压**。实车若相反，把 `a_B` 三轴取反——这条恒等式会立刻暴露符号错误。

---

## Task 1: 设备层发布四元数与去偏机体系角速度

**Files:**
- Modify: `task_up/Application/DeviceLayer/Sensor/imu_sensor.h:66-86`（`imu_fused_t`）
- Modify: `task_up/Application/DeviceLayer/Sensor/imu_sensor.c:236-256`（`IMU_USE_EKF` 块内）

**Interfaces:**
- Consumes: `g_ekf.q[4]`、`g_ekf.Gyro[3]`、`g_ekf.UpdateCount`（`bmi_EKF.h:34-38`、`:32`）
- Produces:
  - `imu_fused_t.q[4]` — 机体→世界四元数，顺序 w,x,y,z，无量纲
  - `imu_fused_t.gyro_body_rad_s[3]` — 去偏机体系角速度，rad/s（z 轴零偏被强制为 0，见 `bmi_EKF.c:189`）
  - `imu_fused_t.ekf_update_count` — EKF 更新计数快照，无量纲，用于判活

- [ ] **Step 1: 在 `imu_fused_t` 中追加三个字段**

在 `task_up/Application/DeviceLayer/Sensor/imu_sensor.h` 的 `float temperature;` 之前追加：

```c
	float q[4];                  /* 机体到世界四元数，顺序w,x,y,z，无量纲 */
	float gyro_body_rad_s[3];    /* 去偏后机体系角速度，rad/s */
	uint32_t ekf_update_count;   /* EKF姿态更新计数快照，无量纲，判活用 */
```

- [ ] **Step 2: 在 `imu_update()` 的 EKF 块末尾发布**

在 `task_up/Application/DeviceLayer/Sensor/imu_sensor.c` 第 255 行 `imu_data->base_info.yaw_total_angle = g_ekf.YawTotalAngle;` 之后、`#endif` 之前插入：

```c
	{
		uint8_t i;
		for (i = 0U; i < 4U; ++i)
		{ imu_data->base_info.q[i] = g_ekf.q[i]; }
		for (i = 0U; i < 3U; ++i)
		{ imu_data->base_info.gyro_body_rad_s[i] = g_ekf.Gyro[i]; }
		imu_data->base_info.ekf_update_count = (uint32_t)g_ekf.UpdateCount;
	}
```

- [ ] **Step 3: 静态核对**

  - 确认新增赋值只在 `#if IMU_USE_EKF == 1` 块内；MAHONY 路径不引用 `g_ekf`（否则编译报未声明）。
  - 确认 `imu_debug_update()`（`control_task.c:17-49`）逐字段拷贝，不受结构体扩长影响。
  - 确认全工程没有 `memcpy`/`sizeof` 作用于 `imu_fused_t` 或 `imu_data_t`（已核：无）。
  - 确认 `g_ekf.q` 与 `g_ekf.Gyro` 在 `ekf_update()` 内无条件赋值（`bmi_EKF.c:113-115`、`:183-186`），不存在“本次未解算”的旧值陷阱。

- [ ] **Step 4: 提交**

```bash
git add task_up/Application/DeviceLayer/Sensor/imu_sensor.h task_up/Application/DeviceLayer/Sensor/imu_sensor.c
git commit -m "上板IMU发布四元数与机体系角速度"
```

---

## Task 2: 新增惯性自稳配置

**Files:**
- Modify: `task_up/Application/ConfigLayer/gimbal_rate_config.h:1-8`（追加）

**Interfaces:**
- Produces: 全部 `GIMBAL_PITCH_INERTIAL_*` 宏，供 Task 3 的 `Gimbal_Init()` 装载

- [ ] **Step 1: 追加配置块**

在 `task_up/Application/ConfigLayer/gimbal_rate_config.h` 的 `#define GIMBAL_YAW_RELEASE_TIMEOUT_MS 300u` 之后、`#endif` 之前追加：

```c
/* ===== 速控 Pitch 惯性自稳（仅 G_RATE 使用） ===== */

/* 总开关：0=只算不控并更新观察量（默认），1=启用惯性闭环。Watch 可改 gimbal_tune.pitch_inertial_enable */
#define GIMBAL_PITCH_INERTIAL_ENABLE             0u

/* 仰角正方向符号，仅 ±1：-1 时枪管上抬为正（默认），+1 时与 g_ekf.Pitch 同向 */
#define GIMBAL_PITCH_INERTIAL_SIGN               (-1.0f)

/* 安装几何，机体系，θ=0 定义为机械中点（pitch_mec_angle==0）。必须人工核对 */
#define GIMBAL_PITCH_INERTIAL_BARREL_X           1.0f  /* 零位枪管朝向u_B0的x分量，无量纲 */
#define GIMBAL_PITCH_INERTIAL_BARREL_Y           0.0f  /* 零位枪管朝向u_B0的y分量，无量纲 */
#define GIMBAL_PITCH_INERTIAL_BARREL_Z           0.0f  /* 零位枪管朝向u_B0的z分量，无量纲 */
#define GIMBAL_PITCH_INERTIAL_AXIS_X             0.0f  /* 关节转轴a_B的x分量，无量纲 */
#define GIMBAL_PITCH_INERTIAL_AXIS_Y             1.0f  /* 关节转轴a_B的y分量，无量纲 */
#define GIMBAL_PITCH_INERTIAL_AXIS_Z             0.0f  /* 关节转轴a_B的z分量，无量纲 */

/* 反馈有效性门限 */
#define GIMBAL_PITCH_INERTIAL_TIMEOUT_MS         10u    /* EKF更新计数停滞判死门限，ms */
#define GIMBAL_PITCH_INERTIAL_RECOVER_MS         20u    /* 恢复需连续有效时间，ms */
#define GIMBAL_PITCH_INERTIAL_Q_NORM_MIN         0.9f   /* 四元数模长下限，无量纲 */
#define GIMBAL_PITCH_INERTIAL_Q_NORM_MAX         1.1f   /* 四元数模长上限，无量纲 */
#define GIMBAL_PITCH_INERTIAL_MIN_H              0.1736f /* cos80°，水平投影下限，无量纲 */
#define GIMBAL_PITCH_INERTIAL_MIN_J              0.2f   /* |dα/dθ|下限，无量纲 */

/* 仰角保持环初值：输入deg误差，输出deg/s仰角速率目标 */
#define GIMBAL_PITCH_INERTIAL_HOLD_KP            50.0f  /* 保持环Kp，deg/s每deg */
#define GIMBAL_PITCH_INERTIAL_HOLD_KI            0.0f   /* 保持环Ki，无量纲 */
#define GIMBAL_PITCH_INERTIAL_HOLD_INTEGRAL_MAX  5000.0f/* 保持环积分限幅，deg·拍 */
#define GIMBAL_PITCH_INERTIAL_HOLD_OUT_MAX       1000.0f/* 保持环输出限幅，deg/s */

/* 关节速率环初值：输入deg/s关节速率误差，输出N·m */
#define GIMBAL_PITCH_INERTIAL_RATE_KP            0.03f  /* 速率环Kp，N·m每deg/s */
#define GIMBAL_PITCH_INERTIAL_RATE_KI            0.0f   /* 速率环Ki，无量纲 */
#define GIMBAL_PITCH_INERTIAL_RATE_KD            0.0f   /* 速率环Kd，无量纲 */
#define GIMBAL_PITCH_INERTIAL_RATE_INTEGRAL_MAX  0.0f   /* 速率环积分限幅，deg/s·拍 */
#define GIMBAL_PITCH_INERTIAL_RATE_OUT_MAX_NM    10.0f  /* 速率环输出限幅，N·m */

/* 滤波与限速 */
#define GIMBAL_PITCH_INERTIAL_RATE_LPF_K         0.5f   /* 仰角速率低通新样本权重，0~1，1=不过滤 */
#define GIMBAL_PITCH_INERTIAL_JOINT_RATE_MAX_DPS 200.0f /* 关节速率需求上限，deg/s */
#define GIMBAL_PITCH_INERTIAL_ELEV_DEADBAND_DEG  0.2f   /* 保持环误差死区，deg */
```

- [ ] **Step 2: 静态核对**

  - `GIMBAL_PITCH_INERTIAL_HOLD_OUT_MAX`（1000）与现有 `GIMBAL_PITCH_HOLD_OUT_MAX`（`gimbal.h:163`）一致。
  - `GIMBAL_PITCH_INERTIAL_RATE_KP`（0.03）与现有 `pitch_gyro_inner.kp`（`gimbal.c:211`）一致。
  - `GIMBAL_PITCH_INERTIAL_HOLD_KP`（50）与现有 `GIMBAL_PITCH_HOLD_KP`（`gimbal.h:160`）一致。
  - 最终人工速率上限复用现有 `pitch_manual_rate_max_deg_s`（150 deg/s），本文件不重复定义。
  - 力矩上限复用现有 `gimbal_tune.pitch_torque_limit_nm`（6 N·m），本文件不重复定义。

- [ ] **Step 3: 提交**

```bash
git add task_up/Application/ConfigLayer/gimbal_rate_config.h
git commit -m "新增速控Pitch惯性自稳配置"
```

---

## Task 3: 云台头文件与调参结构体扩展

**Files:**
- Modify: `task_up/Application/ModuleLayer/gimbal.h:169-178`、`:219-246`、`:307-333`、`:336-355`、`:249-304`

**Interfaces:**
- Consumes: Task 2 的全部宏
- Produces:
  - `gimbal_pitch_domain_e { GIMBAL_PITCH_DOMAIN_MECH, GIMBAL_PITCH_DOMAIN_INERTIAL }`
  - `gimbal_base_info_t.pitch_barrel_elev_deg` / `.pitch_barrel_elev_rate_dps` / `.pitch_barrel_j` / `.pitch_inertial_valid` / `.pitch_inertial_block_reason`
  - `gimbal_feedforward_t.pitch_inertial_hold_deg` / `.pitch_inertial_domain`
  - `gimbal_pid_info_t.pitch_inertial_hold` / `.pitch_inertial_rate`
  - `gimbal_tune_t` 末尾的 21 个 `pitch_inertial_*` 字段

- [ ] **Step 1: 新增目标域枚举**

在 `gimbal.h` 的 `gimbal_mode_e` 定义（`:169-178`）之后追加：

```c
/* 运行期 Pitch 控制目标域。 */
typedef enum
{
    GIMBAL_PITCH_DOMAIN_MECH = 0,  /* 编码器机械角域：G_MEC、升降锁零、回退共用 */
    GIMBAL_PITCH_DOMAIN_INERTIAL   /* 枪管惯性仰角域：仅 G_RATE 且反馈有效 */
} gimbal_pitch_domain_e;
```

- [ ] **Step 2: 扩展 `gimbal_feedforward_t`**

在 `gimbal.h:244` 的 `uint8_t pitch_zero_hold_last;` 之后追加：

```c
    float pitch_inertial_hold_deg;  // Pitch惯性仰角保持目标，deg，仅惯性域更新
    uint8_t pitch_inertial_domain;  // 本周期Pitch目标域，0=机械/1=惯性
```

- [ ] **Step 3: 扩展 `gimbal_pid_info_t`**

在 `gimbal.h:332` 的 `pid_ctrl_t pitch_hold;` 之后追加：

```c
    pid_ctrl_t pitch_inertial_hold;/* Pitch惯性仰角保持环，deg误差->deg/s */
    pid_ctrl_t pitch_inertial_rate;/* Pitch关节速率环，deg/s误差->N·m */
```

- [ ] **Step 4: 扩展 `gimbal_base_info_t`**

在 `gimbal.h:354` 的 `float mec_yaw_err_deg;` 之后追加：

```c
    /* 速控 Pitch 惯性自稳观察量；开关关闭时也持续更新 */
    float pitch_barrel_elev_deg;      /* 枪管相对水平面仰角，deg，上抬为正 */
    float pitch_barrel_elev_rate_dps; /* 低通后仰角变化率，deg/s */
    float pitch_barrel_j;             /* 关节角到仰角传动比dα/dθ，无量纲 */
    uint8_t pitch_inertial_valid;     /* 重建反馈本周期有效，1/0 */
    uint8_t pitch_inertial_block_reason; /* 回退原因码，0=无 */
```

- [ ] **Step 5: 在 `gimbal_tune_t` 末尾追加 Watch 字段**

在 `gimbal.h:303` 的 `mec_yaw_gyro_direction` 之后追加（**必须追加在末尾**）：

```c
    /* 速控 Pitch 惯性自稳：Keil Watch 在线可改 */
    volatile float pitch_inertial_enable;            /* 总开关，0关闭/1启用 */
    volatile float pitch_inertial_sign;              /* 仰角正方向符号，仅±1 */
    volatile float pitch_inertial_barrel_x;          /* 零位枪管朝向u_B0的x分量 */
    volatile float pitch_inertial_barrel_y;          /* 零位枪管朝向u_B0的y分量 */
    volatile float pitch_inertial_barrel_z;          /* 零位枪管朝向u_B0的z分量 */
    volatile float pitch_inertial_axis_x;            /* 关节转轴a_B的x分量 */
    volatile float pitch_inertial_axis_y;            /* 关节转轴a_B的y分量 */
    volatile float pitch_inertial_axis_z;            /* 关节转轴a_B的z分量 */
    volatile float pitch_inertial_hold_kp;           /* 仰角保持环Kp，deg/s每deg */
    volatile float pitch_inertial_hold_ki;           /* 仰角保持环Ki */
    volatile float pitch_inertial_hold_integral_max; /* 仰角保持环积分限幅 */
    volatile float pitch_inertial_hold_out_max;      /* 仰角保持环输出限幅，deg/s */
    volatile float pitch_inertial_rate_kp;           /* 关节速率环Kp，N·m每deg/s */
    volatile float pitch_inertial_rate_ki;           /* 关节速率环Ki */
    volatile float pitch_inertial_rate_kd;           /* 关节速率环Kd */
    volatile float pitch_inertial_rate_integral_max; /* 关节速率环积分限幅 */
    volatile float pitch_inertial_rate_out_max_nm;   /* 关节速率环输出限幅，N·m */
    volatile float pitch_inertial_rate_lpf_k;        /* 仰角速率低通权重，0~1 */
    volatile float pitch_inertial_joint_rate_max_dps;/* 关节速率需求上限，deg/s */
    volatile float pitch_inertial_min_h;             /* 水平投影下限，无量纲 */
    volatile float pitch_inertial_min_j;             /* 传动比绝对值下限，无量纲 */
```

- [ ] **Step 6: 静态核对**

  - 确认 `gimbal_tune_t` 新字段全部在末尾（旧字段序号不变，Keil Watch 布局不被扰动）。
  - 确认 `gimbal.h` 已 `#include "gimbal_rate_config.h"`（`:14` 已有），无新增 include。
  - 确认 `gimbal_base_info_t` / `gimbal_feedforward_t` / `gimbal_pid_info_t` / `gimbal_tune_t` 在全工程没有被 `memcpy` 或 `sizeof` 整体使用（已核：`communicate.c:128-130` 只按字段取值）。

- [ ] **Step 7: 提交**

```bash
git add task_up/Application/ModuleLayer/gimbal.h
git commit -m "云台头文件新增Pitch惯性自稳接口"
```

---

## Task 4: 几何重建函数与有效性门限

**Files:**
- Modify: `task_up/Application/ModuleLayer/gimbal.c:13`（追加文件级状态）
- Modify: `task_up/Application/ModuleLayer/gimbal.c:627-629`（在 `gimbal_update_rate_targets` 之前插入 6 个静态函数；本任务不修改任何现有函数体）

**Interfaces:**
- Consumes: `imu_dev.info->base_info.q` / `.gyro_body_rad_s` / `.ekf_update_count`（Task 1）；`gimbal_tune.pitch_inertial_*`（Task 3）；`gimbal->base_info.pitch_mec_angle`、`gimbal->pitch_motor`
- Produces:
  - `static uint8_t gimbal_normalize3(float *v)`
  - `static void gimbal_quat_to_matrix(float q0, float q1, float q2, float q3, float r[3][3])`
  - `static void gimbal_motor_feedback_snapshot(dm_motor_t *motor, float *angle_deg, float *speed_rad_s)`
  - `static void gimbal_barrel_geometry(const float r[3][3], const float *u_b0, const float *axis, float theta_rad, float sign, float *u_w, float *elev_rad, float *jacobian)`
  - `static void gimbal_barrel_inertial_update(gimbal_t *gimbal)`
  - `static uint8_t gimbal_barrel_feedback_ready(const gimbal_t *gimbal)`
  - 文件级状态 `gimbal_pitch_inertial`

- [ ] **Step 1: 定义文件级内部状态**

在 `gimbal.c:13` 的 `gimbal_tune_t gimbal_tune;` 之后追加：

```c
/* 速控 Pitch 惯性自稳内部状态；对外观察量另存于 base_info。 */
typedef struct
{
    uint8_t  valid;              /* 本周期重建反馈有效，1/0 */
    uint8_t  ready;              /* 本周期允许进入惯性域，1/0 */
    uint16_t valid_cnt;          /* 连续有效周期计数，拍 */
    uint32_t valid_start_ms;     /* 本次连续有效的起始时刻，ms */
    uint32_t ekf_last_count;     /* 上次EKF更新计数，无量纲 */
    uint32_t ekf_change_ms;      /* EKF计数上次变化的时刻，ms */
    uint8_t  rate_lpf_seeded;    /* 仰角速率低通是否已播种，1/0 */
    float    elev_base_dps;      /* 关节不动时基座带来的仰角变化率，deg/s */
    float    elev_lo_deg;        /* 关节下限处的可达仰角，deg */
    float    elev_hi_deg;        /* 关节上限处的可达仰角，deg */
} gimbal_pitch_inertial_state_t;

static gimbal_pitch_inertial_state_t gimbal_pitch_inertial;
```

- [ ] **Step 2: 插入 6 个静态函数**

在 `gimbal.c:627`（`gimbal_yaw_release_update()` 结束的 `}`）之后、`:629` 的 `// NOTE: 增量角输入保留原锁角` 之前插入。**放在此处**是为了让 `gimbal_wrap_deg`、`gimbal_abs`、`gimbal_clamp`、`gimbal_pid_clear` 都已定义在前。

```c
/* 三维向量归一化，模长过小返回 0。 */
static uint8_t gimbal_normalize3(float *v)
{
    float n = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);

    if (n < 1.0e-4f) { return 0u; }
    v[0] /= n;
    v[1] /= n;
    v[2] /= n;
    return 1u;
}

/* 四元数转机体到世界旋转矩阵，顺序w,x,y,z。 */
static void gimbal_quat_to_matrix(float q0, float q1, float q2, float q3, float r[3][3])
{
    r[0][0] = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    r[0][1] = 2.0f * (q1 * q2 - q0 * q3);
    r[0][2] = 2.0f * (q1 * q3 + q0 * q2);
    r[1][0] = 2.0f * (q1 * q2 + q0 * q3);
    r[1][1] = 1.0f - 2.0f * (q1 * q1 + q3 * q3);
    r[1][2] = 2.0f * (q2 * q3 - q0 * q1);
    r[2][0] = 2.0f * (q1 * q3 - q0 * q2);
    r[2][1] = 2.0f * (q2 * q3 + q0 * q1);
    r[2][2] = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
}

/* 原子复制关节机械角与角速度，避免跨帧不一致。 */
static void gimbal_motor_feedback_snapshot(dm_motor_t *motor, float *angle_deg, float *speed_rad_s)
{
    uint32_t irq_state;

    if ((motor == NULL) || (motor->rx_info == NULL)) { return; }
    irq_state = __get_PRIMASK();
    __disable_irq();
    *angle_deg = gimbal_wrap_deg(motor->rx_info->motor_angle * GIMBAL_RAD_TO_DEG -
                                 GIMBAL_PITCH_MIDDLE_DEG);
    *speed_rad_s = motor->rx_info->speed;
    __set_PRIMASK(irq_state);
}

/* 单次几何求解：由关节角得世界系枪管方向、仰角与传动比。 */
static void gimbal_barrel_geometry(const float r[3][3], const float *u_b0,
                                   const float *axis, float theta_rad, float sign,
                                   float *u_w, float *elev_rad, float *jacobian)
{
    float c = cosf(theta_rad);
    float s = sinf(theta_rad);
    float dot, cross0[3], cross1[3], u_b[3], w, h;

    /* 关节旋转 u_B = u_B0·c + (a×u_B0)·s + a·(a·u_B0)·(1-c) */
    dot = axis[0] * u_b0[0] + axis[1] * u_b0[1] + axis[2] * u_b0[2];
    cross0[0] = axis[1] * u_b0[2] - axis[2] * u_b0[1];
    cross0[1] = axis[2] * u_b0[0] - axis[0] * u_b0[2];
    cross0[2] = axis[0] * u_b0[1] - axis[1] * u_b0[0];
    u_b[0] = u_b0[0] * c + cross0[0] * s + axis[0] * dot * (1.0f - c);
    u_b[1] = u_b0[1] * c + cross0[1] * s + axis[1] * dot * (1.0f - c);
    u_b[2] = u_b0[2] * c + cross0[2] * s + axis[2] * dot * (1.0f - c);

    /* 转世界系 u_W = R·u_B */
    u_w[0] = r[0][0] * u_b[0] + r[0][1] * u_b[1] + r[0][2] * u_b[2];
    u_w[1] = r[1][0] * u_b[0] + r[1][1] * u_b[1] + r[1][2] * u_b[2];
    u_w[2] = r[2][0] * u_b[0] + r[2][1] * u_b[1] + r[2][2] * u_b[2];

    h = sqrtf(u_w[0] * u_w[0] + u_w[1] * u_w[1]);
    /* 上抬为正：sign=-1 取 u_Wz，sign=+1 取 -u_Wz */
    *elev_rad = (sign < 0.0f) ? atan2f(u_w[2], h) : atan2f(-u_w[2], h);

    /* 传动比 J_raw = -[R·(a×u_B)]_z / h，叉乘必须用旋转后的 u_B */
    cross1[0] = axis[1] * u_b[2] - axis[2] * u_b[1];
    cross1[1] = axis[2] * u_b[0] - axis[0] * u_b[2];
    cross1[2] = axis[0] * u_b[1] - axis[1] * u_b[0];
    w = r[2][0] * cross1[0] + r[2][1] * cross1[1] + r[2][2] * cross1[2];
    if (h > 1.0e-6f)
    { *jacobian = (sign < 0.0f) ? (w / h) : (-w / h); }
    else
    { *jacobian = 0.0f; }
}

/* 惯性域是否可用：重建反馈有效且总开关打开。 */
static uint8_t gimbal_barrel_feedback_ready(const gimbal_t *gimbal)
{
    return ((gimbal->base_info.pitch_inertial_valid != 0u) &&
            (gimbal_tune.pitch_inertial_enable != 0.0f)) ? 1u : 0u;
}

/* 门限与姿态重建：每周期调用一次，开关关闭时也更新观察量。 */
static void gimbal_barrel_inertial_update(gimbal_t *gimbal)
{
    const imu_data_t *imu = imu_dev.info;
    imu_fused_t *fb = &imu->base_info;
    float ub0[3], axis[3], sign, lpf_k, theta_rad, joint_speed_rad_s, joint_speed_dps;
    float r[3][3], u_w[3], u_w_min[3], u_w_max[3], ud[3], rw[3], w[3];
    float elev_rad, elev_min_rad, elev_max_rad, j, j_min, j_max, h;
    float elev_raw_dps, q0, q1, q2, q3, norm, joint_angle_deg;
    uint32_t now, reason = 0u;
    uint8_t i;

    now = HAL_GetTick();
    if ((imu == NULL) || (fb == NULL)) { return; }

    /* 1. 设备与算法门限。cali_end 是常置锁存，不能当校准判据。 */
    if (imu_dev.work_state.dev_state != DEV_ONLINE) { reason = 1u; }
    else if (imu_dev.work_state.err_code != IMU_E_NONE) { reason = 2u; }

    if (fb->ekf_update_count != gimbal_pitch_inertial.ekf_last_count)
    {
        gimbal_pitch_inertial.ekf_last_count = fb->ekf_update_count;
        gimbal_pitch_inertial.ekf_change_ms = now;
    }
    else if ((uint32_t)(now - gimbal_pitch_inertial.ekf_change_ms) >
             GIMBAL_PITCH_INERTIAL_TIMEOUT_MS)
    { if (reason == 0u) { reason = 3u; } }

    q0 = fb->q[0];
    q1 = fb->q[1];
    q2 = fb->q[2];
    q3 = fb->q[3];
    norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if ((norm < GIMBAL_PITCH_INERTIAL_Q_NORM_MIN) ||
        (norm > GIMBAL_PITCH_INERTIAL_Q_NORM_MAX))
    { if (reason == 0u) { reason = 5u; } }
    if ((!isfinite(q0)) || (!isfinite(q1)) || (!isfinite(q2)) || (!isfinite(q3)) ||
        (!isfinite(fb->gyro_body_rad_s[0])) || (!isfinite(fb->gyro_body_rad_s[1])) ||
        (!isfinite(fb->gyro_body_rad_s[2])))
    { if (reason == 0u) { reason = 4u; } }

    /* 2. 归一化安装几何；sign 只取符号。 */
    ub0[0] = gimbal_tune.pitch_inertial_barrel_x;
    ub0[1] = gimbal_tune.pitch_inertial_barrel_y;
    ub0[2] = gimbal_tune.pitch_inertial_barrel_z;
    axis[0] = gimbal_tune.pitch_inertial_axis_x;
    axis[1] = gimbal_tune.pitch_inertial_axis_y;
    axis[2] = gimbal_tune.pitch_inertial_axis_z;
    sign = (gimbal_tune.pitch_inertial_sign < 0.0f) ? -1.0f : 1.0f;

    if ((gimbal_normalize3(ub0) == 0u) || (gimbal_normalize3(axis) == 0u))
    {
        gimbal->base_info.pitch_inertial_valid = 0u;
        gimbal->base_info.pitch_inertial_block_reason = 6u;
        gimbal_pitch_inertial.valid = 0u;
        gimbal_pitch_inertial.ready = 0u;
        gimbal_pitch_inertial.valid_cnt = 0u;
        return;
    }

    /* 3. 关节反馈与四元数矩阵。 */
    gimbal_motor_feedback_snapshot(gimbal->pitch_motor, &joint_angle_deg, &joint_speed_rad_s);
    theta_rad = joint_angle_deg * GIMBAL_DEG_TO_RAD;
    joint_speed_dps = joint_speed_rad_s * GIMBAL_RAD_TO_DEG;
    gimbal_quat_to_matrix(q0, q1, q2, q3, r);

    /* 4. 几何求解：实测角 + 两个机械端点，共用同一公式。 */
    gimbal_barrel_geometry(r, ub0, axis, theta_rad, sign, u_w, &elev_rad, &j);
    gimbal_barrel_geometry(r, ub0, axis, GIMBAL_PITCH_MIN_DEG * GIMBAL_DEG_TO_RAD,
                           sign, u_w_min, &elev_min_rad, &j_min);
    gimbal_barrel_geometry(r, ub0, axis, GIMBAL_PITCH_MAX_DEG * GIMBAL_DEG_TO_RAD,
                           sign, u_w_max, &elev_max_rad, &j_max);

    /* 5. 仰角速率投影 ω_W = R·(ω_B + a_B·θ̇)，u̇_W = ω_W × u_W，α̇ = -u̇_Wz/h */
    for (i = 0u; i < 3u; ++i)
    { w[i] = fb->gyro_body_rad_s[i] + axis[i] * joint_speed_rad_s; }
    for (i = 0u; i < 3u; ++i)
    { rw[i] = r[i][0] * w[0] + r[i][1] * w[1] + r[i][2] * w[2]; }
    ud[0] = rw[1] * u_w[2] - rw[2] * u_w[1];
    ud[1] = rw[2] * u_w[0] - rw[0] * u_w[2];
    ud[2] = rw[0] * u_w[1] - rw[1] * u_w[0];
    h = sqrtf(u_w[0] * u_w[0] + u_w[1] * u_w[1]);
    if (h > 1.0e-6f)
    { elev_raw_dps = -ud[2] / h * GIMBAL_RAD_TO_DEG * sign; }
    else
    {
        elev_raw_dps = 0.0f;
        if (reason == 0u) { reason = 7u; }
    }

    /* 6. 水平投影与传动比门限。h 等于 cos(仰角)。 */
    if (h < gimbal_tune.pitch_inertial_min_h) { if (reason == 0u) { reason = 8u; } }
    if (gimbal_abs(j) < gimbal_tune.pitch_inertial_min_j) { if (reason == 0u) { reason = 9u; } }

    /* 7. 一阶低通；首次进入直接播种，避免从0拉起的假暂态。 */
    lpf_k = gimbal_tune.pitch_inertial_rate_lpf_k;
    if (!(lpf_k > 0.0f && lpf_k <= 1.0f)) { lpf_k = 1.0f; }
    if (gimbal_pitch_inertial.rate_lpf_seeded == 0u)
    {
        gimbal_pitch_inertial.rate_lpf_seeded = 1u;
        gimbal->base_info.pitch_barrel_elev_rate_dps = elev_raw_dps;
    }
    else
    {
        gimbal->base_info.pitch_barrel_elev_rate_dps = Lowpass(
            gimbal->base_info.pitch_barrel_elev_rate_dps, elev_raw_dps, lpf_k);
    }

    /* 8. 对外观察量与派生量。 */
    gimbal->base_info.pitch_barrel_elev_deg = elev_rad * GIMBAL_RAD_TO_DEG;
    gimbal->base_info.pitch_barrel_j = j;
    gimbal_pitch_inertial.elev_base_dps =
        gimbal->base_info.pitch_barrel_elev_rate_dps - j * joint_speed_dps;
    gimbal_pitch_inertial.elev_lo_deg = fminf(elev_min_rad, elev_max_rad) * GIMBAL_RAD_TO_DEG;
    gimbal_pitch_inertial.elev_hi_deg = fmaxf(elev_min_rad, elev_max_rad) * GIMBAL_RAD_TO_DEG;

    /* 9. 有效性与恢复去抖。 */
    gimbal_pitch_inertial.valid = (reason == 0u) ? 1u : 0u;
    gimbal->base_info.pitch_inertial_valid = gimbal_pitch_inertial.valid;
    gimbal->base_info.pitch_inertial_block_reason = (uint8_t)reason;
    if (gimbal_pitch_inertial.valid != 0u)
    {
        if (gimbal_pitch_inertial.valid_cnt == 0u)
        { gimbal_pitch_inertial.valid_start_ms = now; }
        if (gimbal_pitch_inertial.valid_cnt < 0xFFFFu)
        { gimbal_pitch_inertial.valid_cnt++; }
    }
    else
    { gimbal_pitch_inertial.valid_cnt = 0u; }

    gimbal_pitch_inertial.ready =
        ((gimbal_pitch_inertial.valid != 0u) &&
         (gimbal_tune.pitch_inertial_enable != 0.0f) &&
         ((uint32_t)(now - gimbal_pitch_inertial.valid_start_ms) >=
          GIMBAL_PITCH_INERTIAL_RECOVER_MS)) ? 1u : 0u;
}
```

`NOTE:` 回退原因码含义（Task 8 文档同步要用）：1=IMU 离线，2=`err_code` 非 `IMU_E_NONE`，3=EKF 更新计数停滞，4=数值非有限，5=四元数模长越界，6=安装几何退化，7=水平投影过小，8=`h < min_h`，9=`|J| < min_j`。

- [ ] **Step 3: 静态核对（对应 Review Focus 1、2、5）**

  - **矩阵逐元素对照**：把 `gimbal_quat_to_matrix()` 的 9 个元素与本文档“数学基准 1”逐项比对，并确认 `h(x)`（`bmi_EKF.c:325-327`）恰等于 `r[2][0..2]`（第三行）。
  - **退化自检**：代入 `q=(1,0,0,0)`、`u_B0=(1,0,0)`、`a_B=(0,1,0)`、`sign=-1`、`θ` 任意，手算应得 `elev_deg = -θ`、`J = -1`，与“数学基准 6”的恒等式 `elev + g_ekf.Pitch + pitch_mec_angle == 0` 一致。
  - **叉乘来源**：确认 `J` 用的 `cross1` 由 `u_b`（旋转后）算出，不是 `cross0`（`a×u_B0`）。
  - **回退路径**：确认 `reason != 0` 时 `pitch_inertial_valid == 0` 且 `ready == 0`，本函数不写任何目标量。
  - **观测独立性**：确认 `pitch_barrel_elev_deg` / `pitch_barrel_elev_rate_dps` / `pitch_barrel_j` 在 `enable == 0` 时仍被赋值（只算不控）。
  - **无全局矩阵污染**：确认新增代码未出现 `ekf_trans` / `ekf_src` / `ekf_dst` / `imu_frame_init`。
  - **临界区配对**：确认 `__disable_irq()` 与 `__set_PRIMASK(irq_state)` 成对，且 `gimbal_motor_feedback_snapshot` 的两个出参在 `motor == NULL` 时不被写入（调用方后续会用到，需在 `gimbal_barrel_inertial_update` 调用前把两个局部变量初始化为 0）。

- [ ] **Step 4: 修正调用点局部变量初始化**

为使上一条成立，`gimbal_barrel_inertial_update()` 中声明处改为：

```c
    float joint_angle_deg = 0.0f, joint_speed_rad_s = 0.0f;
```

- [ ] **Step 5: 人工验收项（记录在案，本步不做实机动作）**

  1. 固定偏航部分，手动单独改变 pitch 关节，确认 `imu_dbg.pitch` 基本不动、`pitch_mec_angle` 正常变化 → 坐实安装前提。
  2. 在 `pitch_inertial_enable = 0` 下缓慢手动改 pitch，确认 `Gimbal.base_info.pitch_barrel_elev_deg` 的变化方向与物理枪管运动一致；若反向，翻转 `pitch_inertial_axis_*` 三个分量符号。

- [ ] **Step 6: 提交**

```bash
git add task_up/Application/ModuleLayer/gimbal.c
git commit -m "新增枪管惯性仰角重建与有效性门限"
```

---

## Task 5: 惯性保持目标与三分支接入

**Files:**
- Modify: `task_up/Application/ModuleLayer/gimbal.c:630-780`（`gimbal_update_rate_targets`）

**Interfaces:**
- Consumes: Task 4 的 `gimbal_barrel_feedback_ready()`、`gimbal_pitch_inertial`、`base_info.pitch_barrel_*`；Task 3 的 `pitch_inertial_domain`、`pitch_inertial_hold_deg`、`pitch_inertial_hold` PID
- Produces: `gimbal_feedforward.pitch_rate_target_deg_s` 在惯性域表示**仰角速率目标**，在机械域表示**关节角速率目标**；两者的区分只由 `pitch_inertial_domain` 决定

**WARNING:** 域决策必须放在函数**前半段**（第 641 行之后），因为后面的鼠标累加（696-704）与遥控重捕获（738-743）都要读它。放在函数末尾会让这两处用上一周期的域。

- [ ] **Step 1: 在函数前半段插入域决策与切域复位**

在 `gimbal.c:641`（`uint8_t pitch_zero_transition = (pitch_zero_hold != gimbal->feedforward.pitch_zero_hold_last);` 结束）之后、`:642` 的 `if (pitch_zero_transition != 0u)` 之前插入：

```c
    /* 目标域决策：锁零/机械模式/反馈失效一律机械域，仅G_RATE且反馈有效走惯性域。 */
    {
        uint8_t next_domain = GIMBAL_PITCH_DOMAIN_MECH;

        if ((gimbal->gimbal_mode == G_RATE) && (pitch_zero_hold == 0u) &&
            (gimbal_barrel_feedback_ready(gimbal) != 0u))
        { next_domain = GIMBAL_PITCH_DOMAIN_INERTIAL; }

        if (next_domain != gimbal->feedforward.pitch_inertial_domain)
        {
            /* 切域清闭环并从当前反馈重捕获，绝不复用另一域的旧目标。 */
            if (next_domain == GIMBAL_PITCH_DOMAIN_INERTIAL)
            {
                gimbal_pid_clear(&gimbal->pid_info.pitch_inertial_hold);
                gimbal_pid_clear(&gimbal->pid_info.pitch_inertial_rate);
                gimbal->feedforward.pitch_inertial_hold_deg =
                    gimbal->base_info.pitch_barrel_elev_deg;
            }
            else
            {
                gimbal_pid_clear(&gimbal->pid_info.pitch_hold);
                gimbal_pid_clear(&gimbal->pid_info.pitch_gyro_inner);
                if (pitch_zero_hold == 0u)
                { gimbal->feedforward.pitch_hold_angle_deg = gimbal->base_info.pitch_mec_angle; }
            }
            gimbal->feedforward.pitch_inertial_domain = next_domain;
        }
    }
```

- [ ] **Step 2: 同步新环增益**

在 `gimbal.c:656` 的 `gimbal->pid_info.pitch_hold.out_max = gimbal_tune.pitch_hold_out_max;` 之后追加：

```c
    gimbal->pid_info.pitch_inertial_hold.kp = gimbal_tune.pitch_inertial_hold_kp;
    gimbal->pid_info.pitch_inertial_hold.ki = gimbal_tune.pitch_inertial_hold_ki;
    gimbal->pid_info.pitch_inertial_hold.integral_max =
        gimbal_tune.pitch_inertial_hold_integral_max;
    gimbal->pid_info.pitch_inertial_hold.out_max = gimbal_tune.pitch_inertial_hold_out_max;
    gimbal->pid_info.pitch_inertial_rate.kp = gimbal_tune.pitch_inertial_rate_kp;
    gimbal->pid_info.pitch_inertial_rate.ki = gimbal_tune.pitch_inertial_rate_ki;
    gimbal->pid_info.pitch_inertial_rate.kd = gimbal_tune.pitch_inertial_rate_kd;
    gimbal->pid_info.pitch_inertial_rate.integral_max =
        gimbal_tune.pitch_inertial_rate_integral_max;
    gimbal->pid_info.pitch_inertial_rate.out_max = gimbal_tune.pitch_inertial_rate_out_max_nm;
```

- [ ] **Step 3: 输入源切换时同步惯性目标**

把 `gimbal.c:662-665` 的 pitch 段替换为：

```c
        if ((pitch_zero_hold == 0u) && (pitch_zero_transition == 0u))
        {
            if (gimbal->feedforward.pitch_inertial_domain == GIMBAL_PITCH_DOMAIN_INERTIAL)
            {
                gimbal->feedforward.pitch_inertial_hold_deg =
                    gimbal->base_info.pitch_barrel_elev_deg;
            }
            else
            {
                gimbal->feedforward.pitch_hold_angle_deg = gimbal->base_info.pitch_mec_angle;
            }
        }
```

- [ ] **Step 4: 鼠标目标累加分域**

把 `gimbal.c:696-704` 替换为：

```c
        if (pitch_delta != 0.0f)
        {
            if (gimbal->feedforward.pitch_inertial_domain == GIMBAL_PITCH_DOMAIN_INERTIAL)
            {
                /* 惯性域目标含义为枪管仰角，限幅用当时可达的仰角范围。 */
                gimbal->feedforward.pitch_inertial_hold_deg = gimbal_clamp(
                    gimbal->feedforward.pitch_inertial_hold_deg +
                    pitch_delta * gimbal_tune.mouse_pitch_deg_per_count,
                    gimbal_pitch_inertial.elev_lo_deg,
                    gimbal_pitch_inertial.elev_hi_deg);
                integral_to_zero(&gimbal->pid_info.pitch_inertial_hold);
            }
            else
            {
                gimbal->feedforward.pitch_hold_angle_deg = gimbal_clamp(
                    gimbal->feedforward.pitch_hold_angle_deg +
                    pitch_delta * gimbal_tune.mouse_pitch_deg_per_count,
                    GIMBAL_PITCH_MIN_DEG,
                    GIMBAL_PITCH_MAX_DEG);
                integral_to_zero(&gimbal->pid_info.pitch_hold);
            }
        }
```

- [ ] **Step 5: 遥控输入的松杆重捕获分域**

把 `gimbal.c:738-743` 替换为：

```c
        if ((pitch_zero_hold == 0u) && (pitch_mag > gimbal_tune.rate_hold_enter_deg_s))
        {
            if (gimbal->feedforward.pitch_inertial_domain == GIMBAL_PITCH_DOMAIN_INERTIAL)
            {
                gimbal->feedforward.pitch_inertial_hold_deg =
                    gimbal->base_info.pitch_barrel_elev_deg;
                integral_to_zero(&gimbal->pid_info.pitch_inertial_hold);
            }
            else
            {
                gimbal->feedforward.pitch_hold_angle_deg = gimbal->base_info.pitch_mec_angle;
                integral_to_zero(&gimbal->pid_info.pitch_hold);
            }
        }
```

- [ ] **Step 6: 保持环误差分域**

把 `gimbal.c:763-770`（含原有 NOTE 注释块）替换为：

```c
    /*
     * 机械域：Pitch 保持环用编码器机械角，不用 IMU 欧拉角。
     * IMU 装在偏航部分，云台俯仰时它测不到俯仰变化，用 IMU 角度会得到
     * 恒为零的误差，保持环就不会出力。编码器给的是精确相对角。
     * 惯性域：反馈换成重建的枪管仰角，IMU 无效时切回机械域。
     */
    if (gimbal->feedforward.pitch_inertial_domain == GIMBAL_PITCH_DOMAIN_INERTIAL)
    {
        gimbal->pid_info.pitch_inertial_hold.err =
            gimbal->feedforward.pitch_inertial_hold_deg -
            gimbal->base_info.pitch_barrel_elev_deg;
        single_pid_ctrl(&gimbal->pid_info.pitch_inertial_hold);
    }
    else
    {
        gimbal->pid_info.pitch_hold.err =
            gimbal->feedforward.pitch_hold_angle_deg - gimbal->base_info.pitch_mec_angle;
        single_pid_ctrl(&gimbal->pid_info.pitch_hold);
    }
```

- [ ] **Step 7: 最终目标合成、关节速率限幅与端点拦截**

把 `gimbal.c:776-779` 替换为：

```c
    if (gimbal->feedforward.pitch_inertial_domain == GIMBAL_PITCH_DOMAIN_INERTIAL)
    {
        float j = gimbal->base_info.pitch_barrel_j;
        float base_dps = gimbal_pitch_inertial.elev_base_dps;
        float span, rate, joint_cmd;

        rate = gimbal_clamp(
            pitch_cmd + pitch_blend * gimbal->pid_info.pitch_inertial_hold.out,
            -gimbal_tune.pitch_inertial_hold_out_max,
            gimbal_tune.pitch_inertial_hold_out_max);
        /* 关节速率需求限幅：|(rate - base_dps)/J| <= joint_rate_max */
        span = gimbal_abs(j) * gimbal_tune.pitch_inertial_joint_rate_max_dps;
        rate = gimbal_clamp(rate, base_dps - span, base_dps + span);
        /* 机械端点拦截：按所需关节指令方向判断，不能只看仰角速率符号 */
        joint_cmd = (rate - base_dps) / j;
        if (((gimbal->base_info.pitch_mec_angle <= GIMBAL_PITCH_MIN_DEG) && (joint_cmd < 0.0f)) ||
            ((gimbal->base_info.pitch_mec_angle >= GIMBAL_PITCH_MAX_DEG) && (joint_cmd > 0.0f)))
        {
            rate = base_dps;
            integral_to_zero(&gimbal->pid_info.pitch_inertial_hold);
        }
        gimbal->feedforward.pitch_rate_target_deg_s = rate;
    }
    else
    {
        gimbal->feedforward.pitch_rate_target_deg_s = gimbal_clamp(
            pitch_cmd + pitch_blend * gimbal->pid_info.pitch_hold.out,
            -gimbal_tune.pitch_manual_rate_max_deg_s,
            gimbal_tune.pitch_manual_rate_max_deg_s);
    }
```

`NOTE:` `joint_cmd = (rate - base_dps)/J` 等价于 `θ̇_target`：因为 `base_dps = elev_rate - J·θ̇`，代入得 `(rate - elev_rate)/J + θ̇`，正是“速率环误差 + 当前关节速率”。端点置 `rate = base_dps` 后 `θ̇_target == 0`。

- [ ] **Step 8: 静态核对（对应 Review Focus 3、4）**

  - 逐行比对：`G_MEC` 进入本函数时 `gimbal_mode != G_RATE`，域恒为 `MECH`，走到的每条分支与改动前一致（机械鼠标累加、机械重捕获、机械保持环、机械最终限幅）。
  - 确认升降锁零（`pitch_zero_hold != 0`）时域强制为 `MECH`，`gimbal.c:748-755` 的 `pitch_hold_angle_deg = 0` / `pitch_target = 0` 逻辑未被改动。
  - 确认惯性域内没有任何地方读写 `pitch_hold_angle_deg` 或 `pitch_hold.out`，反之亦然。
  - 确认域决策位于 `:641` 之后、`:642` 之前，早于所有使用点。
  - 确认 `j` 非零：Task 4 的 `min_j` 门限保证 `|J| >= 0.2`，除法安全。

- [ ] **Step 9: 提交**

```bash
git add task_up/Application/ModuleLayer/gimbal.c
git commit -m "速控Pitch接入惯性仰角目标与三分支"
```

---

## Task 6: G_RATE Pitch 输出分支与关节域速率环

**Files:**
- Modify: `task_up/Application/ModuleLayer/gimbal.c:1332-1354`（`gimbal_calc_output` 的 `case G_RATE`）

**Interfaces:**
- Consumes: Task 5 产出的 `pitch_inertial_domain` 与 `pitch_rate_target_deg_s`；Task 3 的 `pitch_inertial_rate` PID
- Produces: `base_info.output_gimbal_p`（N·m）

- [ ] **Step 1: 拆分 G_RATE 的 Pitch 输出**

把 `gimbal.c:1335-1343` 替换为：

```c
        if (gimbal->feedforward.pitch_inertial_domain == GIMBAL_PITCH_DOMAIN_INERTIAL)
        {
            /* 关节域速率环：目标与反馈同除J，回路增益不随横滚变化。
             * WARNING: 反馈必须是重建仰角速率，用pitch_mec_speed会丢掉底盘扰动。 */
            float j = gimbal->base_info.pitch_barrel_j;

            gimbal->base_info.output_gimbal_p =
                all_pid_calc(NULL,
                             &gimbal->pid_info.pitch_inertial_rate,
                             gimbal->feedforward.pitch_rate_target_deg_s / j,
                             0.0f,
                             gimbal->base_info.pitch_barrel_elev_rate_dps / j,
                             0.0f,
                             1.0f,
                             0) + gravity;
        }
        else
        {
            gimbal->base_info.output_gimbal_p =
                all_pid_calc(NULL,
                             &gimbal->pid_info.pitch_gyro_inner,
                             gimbal->feedforward.pitch_rate_target_deg_s,
                             0.0f,
                             gimbal->base_info.pitch_mec_speed,
                             0.0f,
                             GIMBAL_RAD_TO_DEG,
                             0) + gravity;
        }
```

- [ ] **Step 2: 静态核对**

  - 对照 `PID.c:94-101`：`out == NULL` 时 `inn->target = target + rate_feedforward`、`inn->measure = mea_in * inner_scale`。本处 `target = rate/J`、`mea_in = elev_rate/J`、`inner_scale = 1.0f`，误差为 `(rate - elev_rate)/J`，即关节速率误差，回路增益与 `J` 无关。
  - 确认 `j` 绝不为零（Task 4 的 `min_j` 门限）。
  - 确认 `G_MEC` 的 Pitch 段（`gimbal.c:1259-1267`）一字未改。
  - 确认重力补偿 `+ gravity`、力矩前馈与 6 N·m 限幅（`gimbal.c:1364-1385`）逻辑未变。

- [ ] **Step 3: 提交**

```bash
git add task_up/Application/ModuleLayer/gimbal.c
git commit -m "G_RATE Pitch新增惯性关节域速率环输出"
```

---

## Task 7: 初始化、清理与模式切换维护

**Files:**
- Modify: `task_up/Application/ModuleLayer/gimbal.c:172-188`（`gimbal_clear_all_pid`）
- Modify: `task_up/Application/ModuleLayer/gimbal.c:290-297`（`gimbal_pid_init`）
- Modify: `task_up/Application/ModuleLayer/gimbal.c:1394-1508`（`Gimbal_Init`）
- Modify: `task_up/Application/ModuleLayer/gimbal.c:1511-1574`（`Gimbal_Work`）

**Interfaces:**
- Consumes: Task 3 的全部新字段
- Produces: 每周期在 `gimbal_info_update()` 之后调用一次 `gimbal_barrel_inertial_update()`

- [ ] **Step 1: `gimbal_clear_all_pid` 补两个环**

在 `gimbal.c:187` 的 `gimbal_pid_clear(&gimbal->pid_info.pitch_hold);` 之后追加：

```c
    gimbal_pid_clear(&gimbal->pid_info.pitch_inertial_hold);
    gimbal_pid_clear(&gimbal->pid_info.pitch_inertial_rate);
```

- [ ] **Step 2: `gimbal_pid_init` 装载新环初值**

在 `gimbal.c:295`（`pitch_hold` 段末的 `pid->d_filter_alpha = 0.0f;`）之后追加：

```c
    /* 速控 Pitch 惯性仰角保持环：仰角误差 -> 仰角速率 */
    pid = &gimbal->pid_info.pitch_inertial_hold;
    pid->kp = GIMBAL_PITCH_INERTIAL_HOLD_KP;
    pid->ki = GIMBAL_PITCH_INERTIAL_HOLD_KI;
    pid->kd = 0.0f;
    pid->integral_max = GIMBAL_PITCH_INERTIAL_HOLD_INTEGRAL_MAX;
    pid->out_max = GIMBAL_PITCH_INERTIAL_HOLD_OUT_MAX;
    pid->deadband = GIMBAL_PITCH_INERTIAL_ELEV_DEADBAND_DEG;
    pid->d_filter_alpha = 0.0f;

    /* 速控 Pitch 关节速率环：关节速率误差 -> 力矩 */
    pid = &gimbal->pid_info.pitch_inertial_rate;
    pid->kp = GIMBAL_PITCH_INERTIAL_RATE_KP;
    pid->ki = GIMBAL_PITCH_INERTIAL_RATE_KI;
    pid->kd = GIMBAL_PITCH_INERTIAL_RATE_KD;
    pid->integral_max = GIMBAL_PITCH_INERTIAL_RATE_INTEGRAL_MAX;
    pid->out_max = GIMBAL_PITCH_INERTIAL_RATE_OUT_MAX_NM;
    pid->deadband = 0.0f;
    pid->d_filter_alpha = 0.0f;
```

- [ ] **Step 3: `Gimbal_Init` 在 `gimbal_pid_init()` 之前装载 Watch 默认值**

在 `gimbal.c:1444` 的 `gimbal_tune.mec_yaw_gyro_direction = 1.0f;` 之后追加：

```c
    /* 速控 Pitch 惯性自稳装载默认值 */
    gimbal_tune.pitch_inertial_enable = (float)GIMBAL_PITCH_INERTIAL_ENABLE;
    gimbal_tune.pitch_inertial_sign = GIMBAL_PITCH_INERTIAL_SIGN;
    gimbal_tune.pitch_inertial_barrel_x = GIMBAL_PITCH_INERTIAL_BARREL_X;
    gimbal_tune.pitch_inertial_barrel_y = GIMBAL_PITCH_INERTIAL_BARREL_Y;
    gimbal_tune.pitch_inertial_barrel_z = GIMBAL_PITCH_INERTIAL_BARREL_Z;
    gimbal_tune.pitch_inertial_axis_x = GIMBAL_PITCH_INERTIAL_AXIS_X;
    gimbal_tune.pitch_inertial_axis_y = GIMBAL_PITCH_INERTIAL_AXIS_Y;
    gimbal_tune.pitch_inertial_axis_z = GIMBAL_PITCH_INERTIAL_AXIS_Z;
    gimbal_tune.pitch_inertial_hold_kp = GIMBAL_PITCH_INERTIAL_HOLD_KP;
    gimbal_tune.pitch_inertial_hold_ki = GIMBAL_PITCH_INERTIAL_HOLD_KI;
    gimbal_tune.pitch_inertial_hold_integral_max = GIMBAL_PITCH_INERTIAL_HOLD_INTEGRAL_MAX;
    gimbal_tune.pitch_inertial_hold_out_max = GIMBAL_PITCH_INERTIAL_HOLD_OUT_MAX;
    gimbal_tune.pitch_inertial_rate_kp = GIMBAL_PITCH_INERTIAL_RATE_KP;
    gimbal_tune.pitch_inertial_rate_ki = GIMBAL_PITCH_INERTIAL_RATE_KI;
    gimbal_tune.pitch_inertial_rate_kd = GIMBAL_PITCH_INERTIAL_RATE_KD;
    gimbal_tune.pitch_inertial_rate_integral_max = GIMBAL_PITCH_INERTIAL_RATE_INTEGRAL_MAX;
    gimbal_tune.pitch_inertial_rate_out_max_nm = GIMBAL_PITCH_INERTIAL_RATE_OUT_MAX_NM;
    gimbal_tune.pitch_inertial_rate_lpf_k = GIMBAL_PITCH_INERTIAL_RATE_LPF_K;
    gimbal_tune.pitch_inertial_joint_rate_max_dps = GIMBAL_PITCH_INERTIAL_JOINT_RATE_MAX_DPS;
    gimbal_tune.pitch_inertial_min_h = GIMBAL_PITCH_INERTIAL_MIN_H;
    gimbal_tune.pitch_inertial_min_j = GIMBAL_PITCH_INERTIAL_MIN_J;
```

- [ ] **Step 4: `Gimbal_Init` 清零新状态**

在 `gimbal.c:1492` 的 `gimbal->feedforward.pitch_zero_hold_last = 0u;` 之后追加：

```c
    gimbal->feedforward.pitch_inertial_hold_deg = 0.0f;
    gimbal->feedforward.pitch_inertial_domain = GIMBAL_PITCH_DOMAIN_MECH;
    gimbal->base_info.pitch_barrel_elev_deg = 0.0f;
    gimbal->base_info.pitch_barrel_elev_rate_dps = 0.0f;
    gimbal->base_info.pitch_barrel_j = 0.0f;
    gimbal->base_info.pitch_inertial_valid = 0u;
    gimbal->base_info.pitch_inertial_block_reason = 0u;
    gimbal_pitch_inertial.valid = 0u;
    gimbal_pitch_inertial.ready = 0u;
    gimbal_pitch_inertial.valid_cnt = 0u;
    gimbal_pitch_inertial.valid_start_ms = 0u;
    gimbal_pitch_inertial.ekf_last_count = 0u;
    gimbal_pitch_inertial.ekf_change_ms = 0u;
    gimbal_pitch_inertial.rate_lpf_seeded = 0u;
    gimbal_pitch_inertial.elev_base_dps = 0.0f;
    gimbal_pitch_inertial.elev_lo_deg = GIMBAL_PITCH_MIN_DEG;
    gimbal_pitch_inertial.elev_hi_deg = GIMBAL_PITCH_MAX_DEG;
```

- [ ] **Step 5: `Gimbal_Work` 每周期调用重建**

在 `gimbal.c:1518` 的 `gimbal_info_update(gimbal);` 之后追加：

```c
    /* 姿态重建必须在模式仲裁与闭环之前完成；开关关闭时也更新观察量 */
    gimbal_barrel_inertial_update(gimbal);
```

- [ ] **Step 6: 模式切换帧的捕获分域**

把 `gimbal.c:1546-1550` 替换为：

```c
        gimbal->feedforward.yaw_hold_angle_deg = gimbal->base_info.yaw_imu_angle;
        /* 切换帧先回机械域；下一周期由域决策按需重捕获惯性目标 */
        gimbal->feedforward.pitch_inertial_domain = GIMBAL_PITCH_DOMAIN_MECH;
        gimbal->feedforward.pitch_inertial_hold_deg =
            gimbal->base_info.pitch_barrel_elev_deg;
        gimbal->feedforward.pitch_hold_angle_deg =
            ((Lift_IsPitchZeroHoldActive() != 0u) ||
             (gimbal->feedforward.pitch_zero_hold_last != 0u)) ?
            0.0f : gimbal->base_info.pitch_mec_angle;
```

- [ ] **Step 7: 静态核对**

  - 确认两个新 PID 在 `gimbal_pid_init` 与 `gimbal_clear_all_pid` 中成对出现。
  - 确认 `gimbal_pitch_inertial` 的全部字段在 `Gimbal_Init` 中初始化，无未初始化读取。
  - 确认 Step 3 的装载位于 `gimbal_pid_init(gimbal)` 调用（`gimbal.c:1499`）**之前**，Step 4 的清零也位于其之前——若清零在之后，会把 PID 初值覆盖掉。
  - 确认 `gimbal_barrel_inertial_update()` 在 `Gimbal_Work` 的两条路径（模式切换帧 / 正常闭环帧）之前都已执行——它在 if/else 之外，成立。
  - 确认新增字段不引入新的跨文件符号（全部为 `static` 或 `gimbal_t` 成员）。

- [ ] **Step 8: 提交**

```bash
git add task_up/Application/ModuleLayer/gimbal.c
git commit -m "补齐Pitch惯性自稳初始化与模式切换维护"
```

---

## Task 8: 文档同步

**Files:**
- Modify: `task_up/docs/gimbal.md:23-33`、`:37-42`、`:134-143`
- Modify: `task_up/docs/imu.md:48-61`、`:84-91`

**Interfaces:**
- Consumes: Task 1–7 的最终字段名与配置名
- Produces: 与代码一致的两份文档

- [ ] **Step 1: `gimbal.md` 拆分目标域表**

在“速控模式和机械模式不要混讲”表（`gimbal.md:134-143`）中，把“运行 Pitch”一行拆成两行：

  - 机械域：`pitch_hold` + `pitch_gyro_inner`，反馈 `pitch_mec_angle` / `pitch_mec_speed`，参数入口 `gimbal_tune` 的 `pitch_hold_*`。
  - 惯性域：`pitch_inertial_hold` + `pitch_inertial_rate`，反馈 `pitch_barrel_elev_deg` / `pitch_barrel_elev_rate_dps`，参数入口 `gimbal_tune` 的 `pitch_inertial_*`；仅 `G_RATE` 且 `gimbal_tune.pitch_inertial_enable != 0` 且重建反馈有效时进入。

- [ ] **Step 2: `gimbal.md` 新增“速控 Pitch 惯性自稳”小节**

内容必须包含：

  - 三分支判据：升降锁零 → 机械零位；`G_RATE` 且反馈有效且开关打开 → 惯性；否则 → 机械回退。
  - 公式与单位：`α`、`α̇`、`J = dα/dθ`，并说明“目标与反馈同除 J，使回路增益不随横滚变化”。
  - 配置表：`gimbal_rate_config.h` 的全部 `GIMBAL_PITCH_INERTIAL_*`（当前值 + 量纲/范围）。
  - 回退原因码表：1=IMU 离线，2=`err_code` 非 `IMU_E_NONE`，3=EKF 更新停滞，4=数值非有限，5=四元数模长越界，6=安装几何退化，7=水平投影过小，8=`h < min_h`，9=`|J| < min_j`。
  - Watch 项：`Gimbal.base_info.pitch_barrel_elev_deg` / `pitch_barrel_elev_rate_dps` / `pitch_barrel_j` / `pitch_inertial_valid` / `pitch_inertial_block_reason`、`Gimbal.feedforward.pitch_inertial_domain` / `pitch_inertial_hold_deg`、`gimbal_pitch_inertial.*`。
  - 一条 `WARNING:`：`pitch_mec_angle` 零点由 `GIMBAL_PITCH_MIDDLE_DEG` 定义，仰角零点由 EKF 世界系定义，两者的 `0` 不是同一个物理位置；升降锁零使用机械域的 `0`。

- [ ] **Step 3: `imu.md` 补发布字段与约定**

在“观察字段和定位步骤”表（`imu.md:48-61`）中补 `imu_data.base_info.q[4]` / `gyro_body_rad_s[3]` / `ekf_update_count`，并在“当前处理路径的细节”（`imu.md:84-91`）补：

  - `q` 为机体→世界四元数（w,x,y,z），世界系 Z 向上；依据 `ekf_update_xhat()` 的观测预测恰为 `R(body→world)` 第三行。
  - `gyro_body_rad_s` 为去偏机体系角速度（`g_ekf.Gyro`），z 轴零偏被强制为 0。
  - `ekf_update_count` 为 `g_ekf.UpdateCount` 快照，用于判活。
  - `cali_end` 是置 1 后不再清零的锁存，不能作为“校准完成”的持续判据；持续判据用 `err_code == IMU_E_NONE`。
  - 复述既有事实：`Lowpass(..., 1)` 等价于不过滤，故 `rate_pitch/rate_roll/rate_yaw` 是未滤波值。

- [ ] **Step 4: 静态核对**

  - 文档中的每个字段名、宏名与最终代码逐一 grep 对齐。
  - 文档中的当前值表格与 `gimbal_rate_config.h` 数值逐一对齐。
  - 遵守文档约定：不写“本方案旨在…”，用表格与代码块。

- [ ] **Step 5: 提交**

```bash
git add task_up/docs/gimbal.md task_up/docs/imu.md
git commit -m "同步云台与IMU文档的Pitch惯性自稳说明"
```

---

## 人工验收顺序（编译通过后执行，不由实现方自动执行）

编译命令（人工执行，工程已配置好，仅校验语法与头文件包含）：

```powershell
UV4 -b task_up\MDK-ARM\My_C.uvprojx -t My_C -o build_up.log
```

1. **安装关系**：固定偏航部分，单独改变 pitch 关节，确认 `imu_dbg.pitch` 基本不动、`pitch_mec_angle` 正常变化。若 IMU 实际随 pitch 转动，**停止本方案**，本重建会重复计入关节运动，应改用参考工程那条直接路线。
2. **观测模式**（`pitch_inertial_enable = 0`）：核对 `pitch_barrel_elev_deg` 的零点、方向和连续性。车体水平静止时用手（或临时用机械档）改变 pitch，先验证恒等式 `pitch_barrel_elev_deg + imu_dbg.pitch + pitch_mec_angle ≈ 0` 与 `pitch_barrel_j ≈ -1`；再确认方向与物理抬枪一致，反向就翻转 `pitch_inertial_axis_*` 符号。
3. **倾斜与横滚**：抬起车头、侧倾车体，确认 `pitch_barrel_elev_deg` 与 `pitch_barrel_elev_rate_dps` 连续、方向正确；再置 `pitch_inertial_enable = 1`，检查枪管是否保持水平参照。
4. **边界**：机械两端（含 `J` 与端点拦截）、鼠标/遥控源切换、`G_MEC` ↔ `G_RATE` 切换、升降锁零与解除（确认锁零时枪管回到机械零点、与改动前一致）。
5. **反馈异常**：运行中遮挡/拔掉 IMU，确认 10 ms 内 `pitch_inertial_valid = 0`、`pitch_inertial_block_reason != 0`、域切回机械且无旧目标跳变；恢复后确认从当前仰角重新捕获。算法更新计数只能证明解算在跑，不能证明底层传感器未冻结，需同时观察 `imu_dbg` 的原始值。

---

## Self-Review

**Spec coverage**

| 原方案条目 | 落点 |
| --- | --- |
| 只改 `G_RATE` | Task 5 Step 1 域决策；Task 5 Step 8、Task 6 Step 2 逐行比对 |
| 用 IMU 基座姿态 + 编码器重建仰角 | Task 1 + Task 4 |
| `G_MEC`/归中/升降锁零不变 | Task 5 Step 8；Task 7 Step 6 |
| IMU 无效退回机械，恢复重捕获 | Task 4 Step 2（门限）+ Task 5 Step 1（切域复位） |
| 新参数独立、初值沿用 | Task 2 + Task 3 Step 5 + Task 7 Step 2/3 |
| 重力补偿沿用机械角算法 | `gimbal_gravity_compensation()`（`gimbal.c:784-800`）未改动 |
| 首版只算不控、Watch 开关启用 | Task 2 `GIMBAL_PITCH_INERTIAL_ENABLE 0u` + `gimbal_tune.pitch_inertial_enable` |
| 使用现有 EKF 四元数与去偏角速度 | Task 1（原方案缺口：`g_ekf` 原本对 `gimbal.c` 不可见） |
| 矩阵与姿态重建（α、α̇、J、ω_W） | 数学基准 + Task 4 Step 2 |
| 不用简单角度相加 | Task 4 Step 2 的向量形式；数学基准 6 说明其在 roll=yaw=0 时等价 |
| 不重复应用安装变换、不混用转置 | Global Constraints + Task 4 Step 3 核对项 |
| 鼠标 0.04 deg/count、遥控速率与松杆保持 | Task 5 Step 4/5（原量纲与逻辑保留） |
| 保持环 Kp=50 Ki=0、速率环 Kp=0.03 | Task 2 宏值 + Task 7 Step 2 |
| 速率上限 150 deg/s、力矩 6 N·m | Task 2 Step 2 复用现有参数 |
| 机械行程 [-7.5°, 30°]、端点重建限幅 | Task 4 Step 2（端点仰角）+ Task 5 Step 4/7 |
| 端点阻断外向关节指令（含基座运动） | Task 5 Step 7 的 `joint_cmd` 判据 |
| 超限清保持积分 | Task 5 Step 7 的 `integral_to_zero` |
| 升降锁零优先、屏蔽 pitch 输入 | Task 5 Step 1（域强制机械） |
| 切模式/换源/恢复重置 PID 并捕获 | Task 5 Step 1/3/5；Task 7 Step 6 |
| 不重置 Yaw 状态 | Task 5 全部改动都限定在 pitch 段；Yaw 分支未触碰 |
| 开关默认 0、关闭时仍更新观察量 | Task 4 Step 2（重建无条件执行）+ Task 7 Step 5 |
| 几何初值 (1,0,0)/(0,1,0) 须人工核对 | Task 2 宏值 + 人工验收第 2 步 |
| IMU 在线/校准/无错误/四元数与速度有限/更新推进 | Task 4 Step 2 的门限 1–5 |
| 超时 10 ms、恢复 20 ms | Task 2 `TIMEOUT_MS` / `RECOVER_MS` + Task 4 Step 2 段 9 |
| h 门限、J 门限 | Task 2 `MIN_H` / `MIN_J` + Task 4 Step 2 段 6 |
| 回退捕获机械角、恢复捕获惯性角、编码器无效置零 | Task 5 Step 1；编码器无效时 Task 4 的门限使域恒为 MECH，机械环由既有 `gimbal_safety_allows_control` 与电机在线位保护 |
| 不改协议/不改安装矩阵 | Global Constraints |
| 静态交付、跳过测试与烧录 | Global Constraints + 各任务 Step「静态核对」 |
| 安装关系/观测/倾斜/边界/异常五步验收 | 人工验收顺序 |

**原方案未覆盖、本计划补齐的项**

| 缺口 | 落点 |
| --- | --- |
| `g_ekf` 对云台层不可见 | Task 1（新增设备层发布） |
| `cali_end` 是常置锁存，不能当校准判据 | Task 4 Step 2 门限 2（改用 `err_code == IMU_E_NONE`） |
| α̇ 没有任何低通可用（现有 `Lowpass(...,1)` 等于不过滤） | Task 2 `RATE_LPF_K` + Task 4 Step 2 段 7 |
| 除 J 后关节指令可被放大 | Task 2 `JOINT_RATE_MAX_DPS` + Task 5 Step 7 的 `span` 限幅 |
| 符号体系未定义（EKF 的 Pitch 为下俯为正） | 数学基准 4 + `pitch_inertial_sign` 默认 `-1.0f` |
| “0”在机械域与惯性域含义冲突 | Task 5 Step 1 锁零强制机械域 + Task 8 Step 2 的 `WARNING:` |
| 惯性域与机械域共用 PID 会污染回退行为 | Task 3 Step 3（独立 PID）+ Task 5 Step 1 |
| 电机反馈跨帧不一致 | Task 4 Step 2 的临界区快照 |
| 复用 EKF 全局矩阵句柄的风险 | Global Constraints + Task 4 Step 3 核对项 |
| 域决策位置晚于使用点 | Task 5 的 `WARNING:` + Step 1 指定插入位置 |

**Type consistency**：`pitch_inertial_domain`（`uint8_t`，取值 `gimbal_pitch_domain_e`）在 Task 3 定义、Task 5/6/7 使用；`pitch_barrel_elev_deg` / `pitch_barrel_elev_rate_dps` / `pitch_barrel_j` 均为 `float`，Task 4 写入、Task 5/6 读取、Task 8 记录；`gimbal_pitch_inertial.elev_base_dps` / `elev_lo_deg` / `elev_hi_deg` / `valid_cnt` / `valid_start_ms` 三处命名在 Task 4、5、7 一致；`gimbal_barrel_feedback_ready()` 签名在 Task 4 定义、Task 5 Step 1 调用一致；`gimbal_barrel_geometry()` 的 8 个形参在 Task 4 的 3 个调用点一致。

**Proportion**：文档约 900 行，对应改动约 600 行 C 代码（含注释），其中必须给出的整段实现是几何求解（`gimbal_barrel_geometry`）、重建主函数与三个分支段——这些是本方案唯一不能由签名推定的算法，其余均为字段追加与分支改写。
