/**
 * @file rcs_os.c
 * @brief RCS在RTOS之上实现的私有内核功能
 * @author CYK-Dot
 * -------------------------------------------------------------------------------------------
 * @date 2025-05-27
 * @version 1.0.0
 * @changelog 
 * 1. 实现了线程安全的kprintf，通过keil::EventRecorder发送到调试窗口，无需额外占有硬件资源
 * 2. 对老代码中的非阻塞延时进行了面向对象封装
 * -------------------------------------------------------------------------------------------
 */
#include "rcs_os.h"

#include <EventRecorder.h>
#include <stdio.h>
#include <stdarg.h>

#include <FreeRTOS.h>
#include <semphr.h>

static SemaphoreHandle_t rcs_kprintf_sem = NULL;
static size_t rcs_counter[RCS_COUNTER_MAX] = {0};

/**
 * @brief RCS私有打印初始化函数
 * 
 * @return 是否成功
 *
 * @note keil默认配置下，kprintf总是时有时无。可以参考下面这篇文章，按照他后半部分提到的方法做，就能解决问题
 *       https://blog.csdn.net/wuhenyouyuyouyu/article/details/129680677
 */
BaseType_t rcs_kprintf_init(void)
{
    if (rcs_kprintf_sem != NULL) {
        return pdPASS;
    }
    // 开启EventRecorder，记录全部事件
	EventRecorderInitialize(0xFFFFFFFF,1);
	EventRecorderStart();
    // 创建printf锁，避免printf被打断后导致数据不连续
    rcs_kprintf_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(rcs_kprintf_sem);
	
    return rcs_kprintf_sem != NULL ? pdPASS : pdFAIL;
}

/**
 * @brief RCS私有打印函数(kprintf)
 * 
 * @param format 格式化字符串
 * @param timeout 超时时间
 * @param ... 可变参数，用法和printf一致
 * @return 是否成功(pdPASS:成功,pdFAIL:失败)
 */
BaseType_t rcs_kprintf(const char *format,uint32_t timeout, ...)
{
    if (rcs_kprintf_sem == NULL) {
        return pdFAIL;
    }
    // 获取信号量
    if (xSemaphoreTake(rcs_kprintf_sem, timeout) != pdPASS) {
        return pdFAIL;
    }
    // 打印
    va_list args;
    va_start(args, timeout);
    vprintf(format, args);
    va_end(args);
    // 释放信号量
    return xSemaphoreGive(rcs_kprintf_sem);
}

/**
 * @brief RCS私有打印函数(kprintf)，中断安全版本
 * 
 * @param format 格式化字符串
 * @param pxHigherPriorityTaskWoken 是否需要唤醒更高优先级的任务
 * @param ... 可变参数，用法和printf一致
 * @return 是否成功(pdPASS:成功,pdFAIL:失败)
 */
BaseType_t rcs_kprintf_from_isr(const char *format,BaseType_t *pxHigherPriorityTaskWoken, ...)
{
    if (rcs_kprintf_sem == NULL) {
        return pdFAIL;
    }
    // 获取信号量
    if (xSemaphoreTakeFromISR(rcs_kprintf_sem, pxHigherPriorityTaskWoken) != pdPASS) {
        return pdFAIL;
    }
    // 打印
    va_list args;
    va_start(args, pxHigherPriorityTaskWoken);
    vprintf(format, args);
    va_end(args);
    // 释放信号量
    return xSemaphoreGiveFromISR(rcs_kprintf_sem, pxHigherPriorityTaskWoken);
}

/**
 * @brief 同步非阻塞延时(本质是计数器)
 * @param counter_id 使用哪一路计数器
 * @param count_period 计数记到多少返回pdTRUE，如果没有抵达，返回pdFALSE
 * 
 * @note FreeRTOS提供的软件定时器需要异步回调函数，代码不容易编写。
 *       而状态机一般是定时轮询的，因此可以利用轮询周期，把计数器实现成一个同步的非阻塞延时
 * 
 * @warning 使用方式是 
 *          if (rcs_count_delay(1,100)) {
 *              延时抵达后干的事();
 *          }
 *          而不是
 *          rcs_count_delay(1,100)
 *          延时抵达后干的事();
 */
BaseType_t rcs_count_delay(uint16_t counter_id,size_t count_period)
{
    // 入参校验
    if (counter_id >= RCS_COUNTER_MAX){
        return pdFAIL;
    }
    // 计数抵达，返回true
    else if (rcs_counter[counter_id] >= count_period) {
        rcs_counter[counter_id] = 0;
        return pdTRUE;
    }
    // 还没抵达，返回false
    else {
        rcs_counter[counter_id]++;
        return pdFALSE;
    }
}

/**
 * @brief 复位计数器的计数
 * @param counter_id 哪一路计数器
 * @warning 如果当计数器尚未抵达终点时需要弃用，请先reset，否则下次使用的时候就不准了
 */
BaseType_t rcs_count_reset(uint16_t counter_id)
{
    rcs_counter[counter_id] = 0;
    return pdTRUE;
}

