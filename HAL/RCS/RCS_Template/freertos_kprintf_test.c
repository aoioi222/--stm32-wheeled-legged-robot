/**
 * @file freertos_kprintf_test.c
 * @brief freertos和rcs_os代码编写参考
 * @author CYK-Dot
 * @date 2025-05-27
 * ------------------------------------------------------------------------------------------
 * @note 评估方式：
 * 
 * step1:硬件连接和外设配置
 * 1. 完成硬件连接：只需要利用DAPLink接上单片机即可。
 * 2. 在keil的包管理器中启用EventRecorder插件，然后将Printf重定向到DAP。
 * 
 * step2:调用评估代码
 * 3. 在你需要的位置(例如app_main)调用freertos_kprintf_test_init()
 * 
 * step3:观察现象
 * 4. 进入keil调试，开启Debug(printf)Viewer。
 * 5. 运行代码，通过Debug(printf)Viewer查看日志输出。
 * 
 * ------------------------------------------------------------------------------------------
 * @note 借鉴方式：
 * 1. rcs_kprintf用于简单的前期调试，比如检查代码是否正确运行。
 *    因为keil::EventRecorder的bug，在进入调试界面后往往只有第一次运行可以采到数据，后面几次都采不到，需要重新退出进入debug
 *    正因如此，后期调试时建议转到使用rcs_printf，多连一条串口线，利用串口上位机接收日志
 */
#include "freertos_kprintf_test.h"

#include <FreeRTOS.h>
#include <task.h>
#include "rcs_os.h"

static void kprintf_task1(void *args)
{
    while(1) {
        rcs_kprintf("task1:Hello!\n",2); // 最多允许等待2ms，如果等不到就不打印了
        vTaskDelay(pdMS_TO_TICKS(100)); // 延时100ms，因为前面有可能等待，因此理想情况下的循环周期为100~102 ms
    }
}

static void kprintf_task2(void *args)
{
    static uint32_t count = 0;
    while(1) {
        rcs_kprintf("task2:count = %u\n",portMAX_DELAY,count); // 会一直等待直到没有人更优先的任务使用kprintf
        vTaskDelay(pdMS_TO_TICKS(600));// 延时600ms，因为前面有可能等待，因此理想情况下的循环周期为600~∞ ms
    }
}

void freertos_kprintf_test_init(void)
{
    rcs_kprintf_init();
    rcs_kprintf("freertos_test:setup tasks.\n",portMAX_DELAY);
    if (xTaskCreate(kprintf_task1, "kprintf_task1", configMINIMAL_STACK_SIZE, NULL, 5, NULL) != pdPASS) {
        rcs_kprintf("freertos_test:task1 create failed.\n",portMAX_DELAY);
    }
    if (xTaskCreate(kprintf_task2, "kprintf_task2", configMINIMAL_STACK_SIZE, NULL, 3, NULL) != pdPASS) {
        rcs_kprintf("freertos_test:task2 create failed.\n",portMAX_DELAY);
    }
    rcs_kprintf("freertos_test:tasks created.\n",portMAX_DELAY);
}