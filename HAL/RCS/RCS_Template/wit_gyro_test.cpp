/**
 * @file wit_gyro_test.cpp
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
 * 3. 维特智能陀螺仪以C++类的方式提供，不可能在C语言中使用，要注意。请注意文件要是.cpp不能是.c
 */


#include <EventRecorder.h>//如果报错找不到，请检查是否正确启用keil::EventRecorder包
#include <stdio.h>
#include <FreeRTOS.h>
#include <task.h>

#include "wit_gyro_test.h"

static rcs_wit_gyro wit_gyro;
static TaskHandle_t wit_gyro_task_handle;
static void wit_gyro_task(void *pvParameters);

/**
 * @brief 维特智能陀螺仪初始化
 * @param huart 串口句柄，可以传入CubeMX中生成的串口句柄，比如&huart1
 */
void wit_gyro_test_init(HAL_UART_HandleTypeDef *huart)
{
    printf("wit gyro setup start\n");
    rcs_err_t err;
    err = wit_gyro.init(huart, WIT_PROTOCOL_NORMAL, 0x60);
    if (err != RCS_OK)
    {
        printf("wit gyro setup failed\n");
    }
    printf("create task\n");
    BaseType_t ret = xTaskCreate(wit_gyro_task, "wit_gyro_task", 2048, NULL, 5, &wit_gyro_task_handle);
    if (ret != pdPASS)
    {
        printf("create task failed\n");
    }
    printf("create task success\n");
}

/**
 * @brief 维特智能陀螺仪评估任务
 */
static void wit_gyro_task(void *pvParameters)
{
    while (1)
    {
        printf("pitch: %f, roll: %f, yaw: %f\n", wit_gyro.get_pitch(), wit_gyro.get_roll(), wit_gyro.get_yaw());
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

