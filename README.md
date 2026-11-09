# STM32 多模态轮足机器人

面向 STM32F407 的轮足机器人控制程序。控制任务以 200 Hz 为目标，负责 IMU 姿态融合、SBUS 遥控、模式切换、步态和运动学求解。

## 项目推进

| 阶段 | 执行器与软件平台 |
| --- | --- |
| 早期原型 | 串行总线舵机关节、RM3508 轮电机、STM32 标准外设库和 uC/OS-II。见早期提交。 |
| 标准库升级 | 12 个 eduLite05 关节和 4 个 DM-H6215 轮电机接入双 CAN。电机驱动、遥控分发和应用控制分层，主任务按实际经过的 tick 推进步态。 |

## 标准库工程

- Keil 入口为 [USER/F407Practice.uvprojx](USER/F407Practice.uvprojx)，目标 `Target 1`，使用 Arm Compiler 6.24。
- 主任务在 [USER/UpctrlTask.c](USER/UpctrlTask.c)，步态、正逆运动学、IMU、遥控和电机驱动在 [Multiple_modal_robot](Multiple_modal_robot)。
- 轮电机走 CAN1（PA11 RX、PA12 TX），关节电机走 CAN2（PB12 RX、PB13 TX），两路均为 1 Mbps。
- ICM-42688-P 使用 I2C1（PB6 SCL、PB7 SDA），蓝牙 SBUS 使用 USART6（PC6 TX、PC7 RX，100000 baud）。
- 支持 Trot、Walk、Pronk 及站立、轮式、折叠和跳跃。当前使用几何逆运动学，雅可比迭代和重力补偿开关默认关闭。

电机使能开关默认为关闭。实机运行前，在 [robot_config.h](Multiple_modal_robot/robot_config.h) 和 [Foot_mode_control.c](Multiple_modal_robot/Foot_mode_control.c) 核对电机 ID、方向、零位和限位，并验证通信与控制周期。
