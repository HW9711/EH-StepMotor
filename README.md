# EH-StepMotor 步进电机驱动工程

本仓库保存 EH 系统泵用步进电机驱动板固件，目标 MCU 为 STM32L431CC。

## 工程入口

- `GE2415_BuJin.ioc`：STM32CubeMX 外设与时钟配置。
- `MDK-ARM/GE2415_BuJin.uvprojx`：Keil MDK 工程，使用 ARM Compiler 5。
- `MDK-ARM/GE2415_BuJin.code-workspace`：VS Code / EIDE 工作区。
- `MDK-ARM/.eide/eide.yml`：EIDE 构建配置，目标名为 `GE2415_BuJin`。

## 目录说明

- `USER/`：电机控制、步进换相、通信解析和保护逻辑，是业务代码的主要维护位置。
- `Core/`：STM32 初始化、中断、DMA、定时器、串口等底层代码。
- `Drivers/`：STM32 HAL 与 CMSIS 依赖。
- `MDK-ARM/`：Keil、EIDE、启动文件和调试配置。

## 当前安全控制约束

- 主控速度命令采用固定 6 字节帧，并通过 CRC16-MODBUS 校验后才生效。
- 连续 100 ms 未收到合法控制帧时，驱动进入通信故障并执行硬停机。
- 上电或故障恢复后，必须先收到合法零速命令解除运行闭锁，再接受非零速度命令。
- TIM1 BREAK 硬件故障会立即关闭 PWM 主输出，并由软件锁存故障状态。

## 构建说明

使用 EIDE 打开 `MDK-ARM/GE2415_BuJin.code-workspace` 后，选择 `GE2415_BuJin` 目标并执行 Rebuild。也可以在 Keil MDK 中打开 `MDK-ARM/GE2415_BuJin.uvprojx` 构建。

编译输出、本机缓存、Keil 用户会话和 J-Link 日志均由 `.gitignore` 排除，不作为源码提交。
