/**
 * @file wit_gyro_test.h
 * @brief 维特智能陀螺仪评估代码，用于代码编写参考，或是检查陀螺仪好坏
 * @author CYK-Dot
 * @date 2025-05-23
 * ------------------------------------------------------------------------------------------
 * @note 评估方式：
 * 1. 完成硬件连接。
 * 2. 在CubeMX中配置串口波特率为115200，以及启用串口中断。
 * 3. 在keil的包管理器中安装EventRecorder插件，并开启。
 * 4. 在keil中将printf重定向到DAP接口。
 * 5. 进入keil调试，开启Debug(printf)Viewer。
 * 6. 运行代码，通过Debug(printf)Viewer查看输出。
 * ------------------------------------------------------------------------------------------
 * @note 借鉴方式：
 * 1. 不要在正式代码中使用DAP-printf，这里只是为了调试方便引入的。如果需要使用printf，请使用rcs_uart.h中的函数
 * 2. 为了方便评估，这里的代码创建了一个任务用于循环输出数据，但在使用的时候，任务不是必须的。
 */

#ifdef __cplusplus
    extern "C" {
#endif

#pragma once

#include "rcs_private_config.h"
#include "rcs_err.h"
#include "rcs_uart.h"
#include "rcs_wit_gyro.h"

void wit_gyro_test_init(HAL_UART_HandleTypeDef *huart);

#ifdef __cplusplus
    }
#endif

