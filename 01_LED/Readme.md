# 01_LED 独立参考工程

[返回双板项目总览](../README.md)

该目录包含一个单独的 STM32F407IGHx Keil 工程，用于外设与 RGB LED 任务参考。它不参与 `task_up` 上板或 `task_down` 下板的构建和控制链路。

## Architecture

```mermaid
flowchart LR
  MAIN[main: HAL/时钟/外设初始化] --> DEV[DEVICE_Init]
  DEV --> DRV[DRIVER_Init/CAN Filter]
  DRV --> RTOS[FreeRTOS]
  RTOS --> LEDTASK[LedTask]
  LEDTASK --> LEDWORK[led_work 状态/闪烁调度]
  LEDWORK --> GPIO[GPIOH 10/11/12 RGB LED]
```

LED 的应用层状态为 `LED_OFF/LED_ON/LED_BLINK`，颜色为 red/green/blue；每次调用只驱动所选颜色。`StartLedTask()` 启动时绿灯常亮 500 ms，然后持续闪烁。

## Prerequisites

| 项目 | 工程配置 |
| --- | --- |
| MCU / Target | STM32F407IGHx / `My_C` |
| 编译器 | ARM Compiler 5.06 update 7（build 960） |
| Device Pack | `Keil.STM32F4xx_DFP.2.17.1` |
| HAL / RTOS | 仓库内 STM32F4 HAL v1.8.3、FreeRTOS V10.3.1 |
| 工具 | Windows + Keil MDK/µVision；无 GCC/Clang/CMake 工程 |
| LED 引脚 | PH10 蓝、PH11 绿、PH12 红；GPIO 推挽输出、高电平点亮 |

## Getting Started

仓库根目录构建：

```powershell
UV4 -b 01_LED\MDK-ARM\My_C.uvprojx -t My_C -o "$env:TEMP\Train_code_plus_led.log"
Get-Content "$env:TEMP\Train_code_plus_led.log"
```

输出目录为 `01_LED/MDK-ARM/My_C/`。工程 XML 未配置可复用的命令行 Flash Driver；人工下载前在 µVision 中选择实际探针并配置该 MCU 的 Flash Algorithm。

上电后的绿色闪烁只能证明 LED 任务运行，不是传感器、电机、CAN 或整机自检结果。工程没有统一串口启动日志或自动验收标志。

## Core Control Pipeline

FreeRTOS tick 为 1 kHz；ControlTask 和 MonitorTask 优先级为 Realtime，LED 任务为 AboveNormal。此目录的 LED 任务在初始化后每 1 tick 调用一次 `led_work()`；闪烁切换基于 `HAL_GetTick()`。

| 文件 | 控制作用 |
| --- | --- |
| `Core/Src/main.c` | 系统时钟、GPIO、DMA、SPI、TIM、CAN、ADC、UART、USB 初始化，随后调用设备/驱动初始化并启动 RTOS |
| `Core/Src/freertos.c` | 创建 Control、Monitor、Led、Community 任务；应用弱任务壳循环延时 |
| `Application/TaskLayer/led_task.c` | 绿灯开机 500 ms 后设置 5 Hz 闪烁并持续轮询 |
| `Application/DeviceLayer/led.c`、`led.h` | LED 颜色/状态到 GPIO 电平的映射 |
| `Core/Src/gpio.c`、`Core/Inc/main.h` | GPIOH 三色 LED 引脚初始化与宏定义 |

`led_work()` 中 5 Hz 通过 100 ms 翻转间隔实现（一个完整亮灭周期约 200 ms）。此目录包含其他已初始化外设及驱动文件；LED 示例本身不意味着那些外设任务已经形成机器人控制闭环。

## Protocol & Tuning

本工程没有使用板间 D1–D5/C1–C2 协议，也没有 LED 总线协议。调节颜色、模式或频率的位置：

| 参数 | 文件 | 当前值/单位 |
| --- | --- | --- |
| 点亮颜色/初始状态 | `Application/DeviceLayer/led.c` | 初始 red/off；任务启动后切 green/on |
| 运行闪烁频率 | `Application/TaskLayer/led_task.c` | `blink_fre=5` 次/s |
| GPIO 电平 | `Application/DeviceLayer/led.h` | 输出高电平点亮，低电平熄灭 |
| Pin map | `Core/Inc/main.h`、`Core/Src/gpio.c` | GPIOH PH10/PH11/PH12 |

## Troubleshooting & Safety

- 绿色灯不亮时检查 3.3 V/地、PH11 引脚复用和 GPIO 输出初始化，再检查 LedTask 是否创建并运行。
- 灯常亮不表示其余初始化全部成功；`Error_Handler()` 会关闭中断并停在循环，当前没有专用错误灯码。
- PH10/PH11/PH12 为板级 GPIO 资源，外接 LED 需核对极性和串联限流，避免把 MCU 管脚直接接到电源。
- 烧录前确认选择 `01_LED/MDK-ARM/My_C.uvprojx`，不要将此工程固件误刷到整车上下板。
