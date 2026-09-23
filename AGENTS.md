# Repository Guidelines

## Project Structure

本仓库是 RoboMaster STM32 上下板工程：

- `task_up/`：上板控制，包含云台、发射机构、IMU、PID、电机驱动和板间协议。
- `task_down/`：下板控制，包含底盘、任务调度、遥控输入和板间通信。
- `01_LED/`：独立外设参考工程，不属于当前上下板主程序。
- `Application/`：业务代码按 Config、Device、Driver、Hardware、Module、Protocol、Task 分层。
- `MDK-ARM/`：Keil 工程与编译产物，不手工修改生成文件。

新增模块优先放入对应 Layer，配置参数集中到 `ConfigLayer` 的头文件。

## Build And Validation

使用 Keil MDK 打开对应工程：

```text
task_up/MDK-ARM/My_C.uvprojx
task_down/MDK-ARM/DM-MC02.uvprojx
```

Target 分别为 `My_C` 和 `DM-MC02`。命令行示例：

```powershell
UV4 -b task_up\MDK-ARM\My_C.uvprojx -t My_C -o build_up.log
UV4 -b task_down\MDK-ARM\DM-MC02.uvprojx -t DM-MC02 -o build_down.log
```

当前没有单元测试框架。提交前必须保证目标工程 `0 errors`，并完成相关 CAN、遥控器或电机台架验证。协议修改需同时验证上下板。

## Coding Style

保持 C99 风格和现有代码布局。命名使用模块前缀，例如 `Gimbal_`、`Chassis_`、`Launcher_`；宏使用全大写；配置项集中在 `*_config.h`。

注释只解释原因和约束。使用 `TODO:`、`FIXME:`、`NOTE:`、`WARNING:` 标签，删除失效代码，不保留注释掉的旧实现。

## Commit And Pull Request

提交信息使用简短中文，说明实际变化，例如：

```text
完成发射机构单发控制
修正底盘跟随死区
```

提交前确认修改目录与目标板一致。PR 需包含：修改目的、涉及板卡、编译结果、硬件验证步骤；协议或参数变化需说明兼容性。没有硬件验证时必须明确标注。

## Configuration And Safety

CAN ID、字节序和板间协议布局修改时，必须同步检查 `task_up` 与 `task_down`。未使用功能保持宏关闭，避免影响调试链。禁止提交临时脚本、备份文件、日志和本地调试产物。
