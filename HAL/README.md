# STM32 HAL 迁移版

从仓库根目录的标准库版本迁移。步态、运动学、姿态控制、遥控事件和电机控制均接入 STM32F407 的 HAL 工程。

- Keil 工程为 [MDK-ARM/RCS_Template_F407.uvprojx](MDK-ARM/RCS_Template_F407.uvprojx)，目标 `RCS_Template_F407`，使用 Arm Compiler 6.24。
- CubeMX 配置为 [RCS_Template_F407.ioc](RCS_Template_F407.ioc)。主任务为 [robot_task.c](RCS/user/robot/robot_task.c)，使用 FreeRTOS CMSIS-RTOS2 的 5 ms 调度。
- DM-H6215 使用 CAN1（PB8 RX、PB9 TX），eduLite05 使用 CAN2（PB12 RX、PB13 TX），两路均为 1 Mbps。
- ICM-42688-P 使用 I2C1（PB6 SCL、PB7 SDA，400 kHz）。蓝牙 SBUS 使用 USART6（PC6 TX、PC7 RX，100000 baud）。
- 步态、运动学、遥控、IMU 和电机驱动都在 [RCS/user/robot](RCS/user/robot)。电机开关在 [robot_config.h](RCS/user/robot/robot_config.h)，默认关闭，实机标定后再使能。
