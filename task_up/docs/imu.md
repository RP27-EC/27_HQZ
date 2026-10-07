# 上板 BMI088 与姿态估算

[返回上板模块索引](README.md) · [返回上板工程说明](../Readme.md) · [云台闭环](gimbal.md)

## 采集与处理链路

```mermaid
flowchart LR
  SPI[SPI1: BMI088 accel/gyro] --> RAW[原始加速度/角速度/温度]
  RAW --> FRAME[imu_frame_rotate 坐标变换]
  FRAME --> CALI[静置陀螺零偏校准]
  CALI --> EKF[EKF 姿态解算]
  EKF --> DEVICE[imu_dev / imu_data]
  DEVICE --> GIMBAL[ControlTask → Gimbal]
```

核心文件：

| 路径 | 作用 |
| --- | --- |
| `task_up/Core/Src/spi.c` | SPI1 主机配置：8 bit、MSB first、CPOL High/2nd Edge、软件 NSS、分频 16；PB3 SCK、PB4 MISO、PA7 MOSI |
| `task_up/Application/DeviceLayer/Imu/BMI088driver.c/.h` | 芯片寄存器初始化、加速度/陀螺仪/温度采集；本工程量程设为 ±3 g、±2000 deg/s |
| `task_up/Application/DeviceLayer/Imu/BMI088Middleware.c/.h` | 将原始器件数据适配到姿态算法接口 |
| `task_up/Application/DeviceLayer/Imu/bmi_EKF.c/.h` | EKF 状态更新 |
| `task_up/Application/DeviceLayer/Sensor/imu_sensor.c/.h` | 设备状态、坐标变换、零偏、角度/角速度输出 |
| `task_up/Application/TaskLayer/control_task.c` | 1 ms 控制任务中调用更新；错误/校准状态决定是否进入闭环数据路径 |

传感器由 SPI 轮询/驱动调用采样；ControlTask 每周期调 `imu_dev.update()` 的 1 ms 调度频率不等于已证明硬件每 1 ms 有新样本。不要把任务频率误写成 BMI088 采样率。

## 上电初始化与陀螺校准

`imu_init()` 先调用 `BMI088_init()`；初始化错误会重试，错误计数达到 200 时触发系统复位。成功后标记设备在线、初始化 EKF 并将错误状态设为 `IMU_E_CALI`。校准阶段累计 2000 次更新估计三轴陀螺偏置，完成后错误状态转为 `IMU_E_NONE` 并置 `cali_end`。校准时机器人需静止；转动中的样本会污染零偏估计。

数据更新流程为 `BMI088_read()` → 坐标旋转 → 陀螺零偏修正 → EKF → `imu_data.base_info`。出现连续数据错误时 `err_cnt` 到 100 会将设备标记为离线并设置 `IMU_E_DATA`。`imu_heart_beat()` 有独立离线计数；单看角度变量仍可能读到上次的旧值。

## 设备生命周期

| 阶段 | 代码动作 | 应观察内容 |
| --- | --- | --- |
| 驱动探测 | 初始化 BMI088 加速度计、陀螺仪寄存器与量程 | 驱动返回码、SPI 数据是否稳定；失败最多重试 200 次后系统复位 |
| 姿态算法建立 | 初始化 EKF 和 IMU 数据结构 | `init_flag`、EKF 初始状态是否已写入 |
| 零偏采样 | 连续收集 2000 次陀螺更新估计 bias | 设备必须静止；校准期间姿态角可更新但仍处于校准错误状态 |
| 正常更新 | 坐标变换、gyro bias 修正、EKF 更新 | `err_code=IMU_E_NONE`、在线状态及角度/角速度连续性 |
| 数据故障 | 错误计数达到 100 | `IMU_E_DATA` 与离线状态；数值变量可能仍留有旧样本 |

该状态顺序说明初始化成功、完成校准和持续在线是三个判据。复位成功也只说明软件重新启动，不能证明 SPI 连接或姿态估计已恢复。

## 观察字段和定位步骤

| 观察量 | 判断用途 |
| --- | --- |
| `imu_dev.work_state.dev_state` | 设备在线/离线 |
| `imu_dev.work_state.err_code` | 初始化、校准、数据错误或正常状态 |
| `imu_dev.work_state.cali_end` / `imu_data.init_flag` | 校准/数据初始化进度；结合错误状态解释 |
| `imu_data.raw_info` | 坐标旋转前的传感器原始量 |
| `imu_data.base_info` | 坐标处理与姿态解算后的输出 |
| `imu_dbg` | 工程中提供的调试快照；检查更新时刻、解算值和状态 |

排查先确认 SPI 线序/片选和 BMI 初始化返回，再看静置校准期间三轴是否稳定，然后检查坐标系旋转方向、角度符号和 EKF 输出。若仅某轴符号错误，先核对安装方向与 frame 映射，不能直接改云台 PID。

温度控制接口当前没有有效的闭环输出：`imu_set_temperature()` 的 PID/PWM 内容处于注释状态；不可据此声称 IMU 加热或温控已启用。传感器数据可用于控制，但该文档不替代静态、转台或实机姿态精度标定。

### 坐标与单位检查

BMI088 芯片量程为 ±3 g 和 ±2000 deg/s；中间层输出经过轴向映射/单位换算后才进入 EKF。检查符号时按“芯片原始轴 → `imu_frame_rotate` 后轴 → `imu_data.base_info` → Gimbal 使用字段”逐层对照。同一物理转动下 raw 轴符号与云台逻辑轴符号可能因安装方向不同，不宜只凭传感器数据表推断软件正方向。
