/**
 * @file rcs_actor_bus.cpp
 * @author 陈煜楷
 * @brief 将所有执行器抽象为总线，提供统一的控制接口和波形观察接口。
 * @version 0.1
 * @date 2025-06-01
 * 
 * @copyright Copyright (c) 2025
 * ------------------------------------------------------------------
 * @note 理解actor-bus的设计逻辑
 * 
 * 1. 执行器总线采用二级链表的设计，一级链表描述执行器的种类，二级链表描述执行单元。
 * - 假如一个机器人上有4个气缸、4个RM3508电机、2个GM6020电机、1个VESC电调
 * - 那么一级链表共有四个成员，分别为cylinder、RM3508、GM6020、VESC
 * - 每个成员下又是一个二级链表，分别存放了各个执行单元，例如cylinder下有4个气缸。
 * 
 * 1. 一级链表存放的是函数指针，用于描述如何控制和监测某种执行器。
 * - 一级链表类为rcs_actor_bus，是单例类，不应被实例化，由本文件负责实现。
 * - 通过actor_add和monitor_add函数，可以向一级链表中加入新的执行器种类。
 * - rcs_actor_bus类一般不再修改，因为它们是执行器的抽象。如果需要新增执行器，请编写一个新的二级链表类。
 * 
 * 2. 二级链表存放的是CAN总线ID、GPIO等，用于描述同种执行器下不同执行单元的差异。
 * - 二级链表类是rcs_actor的子类，由rcs_actor文件夹下的文件负责实现。
 * - 二级链表不是单例类，需要被实例化。实例化的过程，就是向二级链表中加入新成员的过程。
 * - 
 */

/* 头文件引入 ----------------------------------------------------------------------------------------*/

#include "rcs_actor_bus.h"
#include "rcs_os.h"
#include <vector>

/* 类私有成员 -----------------------------------------------------------------------------------------*/

std::vector<rcs_actor::excute_fptr> rcs_actor_bus::excute_list{};  
std::vector<rcs_actor::reset_fptr> rcs_actor_bus::reset_list{};    
rcs_uart_t rcs_actor_bus::uart = RCS_UART_NUM_MAX;                                 
std::vector<uint8_t> rcs_actor_bus::monitor_stream{};             
uint8_t rcs_actor_bus::monitor_channel_count = 0;                    
std::vector<int16_t> rcs_actor_bus::monitor_index{};               
std::vector<rcs_actor::monitor_fptr> rcs_actor_bus::monitor_list{};
TaskHandle_t rcs_actor_bus::excute_task_handle = nullptr;                  
TaskHandle_t rcs_actor_bus::watch_task_handle = nullptr;                  
EventGroupHandle_t rcs_actor_bus::event_group = nullptr;                       

/* 全局静态方法 ---------------------------------------------------------------------------------------*/

/**
 * @brief 执行器总线执行任务
 * 
 * @param param 未使用
 */
void rcs_actor_bus::actor_bus_excute_task(void* param)
{
    // 等待用户启用任务
    xEventGroupWaitBits(
        event_group,
        EVENT_BIT_ENABLE_CTRL,
        pdTRUE,
        pdFALSE,
        portMAX_DELAY
    );
    // 循环执行
    while(1)
    {
        // 遍历excute_list并执行函数指针
        for(int i=0;i<excute_list.size();i++){
            excute_list[i]();
        }
        // 遍历reseter_list并执行函数指针
        for(int i=0;i<reset_list.size();i++){
            reset_list[i]();
        }
        // 延时
        vTaskDelay(RCS_ACTOR_BUS_EXCUTE_TASK_DELAY);
    }
}

/**
 * @brief 执行器总线观测任务
 * 
 * @param param 未使用
 */
void rcs_actor_bus::actor_bus_monitor_task(void* param)
{
    // 等待用户启用任务
    xEventGroupWaitBits(
        event_group,
        EVENT_BIT_ENABLE_WATCH,
        pdTRUE,
        pdFALSE,
        portMAX_DELAY
    );
    // 循环执行
    while(1)
    {
        // 获取各个通道上的数据，并拆解成字节流
        for(uint8_t i=0;i<monitor_channel_count;i++)
        {
            float value = monitor_list[monitor_index[i]](i);
			//rcs_kprintf("ch=%hhd,f=%.1f\n",portMAX_DELAY,i,value.f);
			//rcs_kprintf("ch=%hhd,actor=%hhd,moni=%#X\n",portMAX_DELAY,i,monitor_index[i],monitor_list[monitor_index[i]]);
            monitor_stream[4*i+0] = rcs_byte_read((const void*)&value,0);
            monitor_stream[4*i+1] = rcs_byte_read((const void*)&value,1);
            monitor_stream[4*i+2] = rcs_byte_read((const void*)&value,2);
            monitor_stream[4*i+3] = rcs_byte_read((const void*)&value,3); 
        }
        // JustFloat帧定界符(4字节)
        monitor_stream[4*monitor_channel_count+0] = 0x00;
        monitor_stream[4*monitor_channel_count+1] = 0x00;
        monitor_stream[4*monitor_channel_count+2] = 0x80;
        monitor_stream[4*monitor_channel_count+3] = 0x7F;
        //rcs_kprintf("%.1f %.1f %.1f %.1f\n",portMAX_DELAY,all_value[0].f,all_value[1].f,all_value[2].f,all_value[3].f);

        // 发送数据并轮询
        rcs_err_t err = rcs_uart_send(uart, monitor_stream.data(),monitor_stream.capacity(),pdMS_TO_TICKS(1));
		if (err != RCS_OK) rcs_kprintf("actor-bus: monitor send uart failed,code=%d\n",portMAX_DELAY,err);
        vTaskDelay(RCS_ACTOR_BUS_MONITOR_TASK_DELAY);
    }
}

/* 类公有方法 -----------------------------------------------------------------------------------------*/

/**
 * @brief 添加执行器到链表中
 * 
 * @param motor 电机节点指针
 * @return rcs_err_t 是否成功
 */
rcs_err_t rcs_actor_bus::actor_add(rcs_actor::excute_fptr excuter,rcs_actor::reset_fptr reseter)
{
    rcs_kprintf("actor-bus:add actor at %X/%X\n",portMAX_DELAY,(uint32_t)excuter,(uint32_t)reseter);
	BaseType_t err;
    // 参数校验
    if (excuter == nullptr || reseter == nullptr) {
        rcs_kprintf("actor-bus: add actor failed, param is invalid\n",portMAX_DELAY);
        return RCS_ERR_INVALID_PARAM;
    }
    // 如果是第一次添加执行器，则初始化相关资源(一个事件组两个任务)
    if (rcs_actor_bus::event_group == NULL) {
        rcs_actor_bus::event_group = xEventGroupCreate();
        if (rcs_actor_bus::event_group == NULL) {
            rcs_kprintf("actor-bus: add actor failed,no mem!\n",portMAX_DELAY);
            return RCS_ERR_NO_MEM;
        }
    }
    if (rcs_actor_bus::excute_task_handle == NULL) {
        err = xTaskCreate(actor_bus_excute_task,
            "actor_excute_task",
            RCS_ACTOR_BUS_EXCUTE_TASK_STACK_SIZE,
            NULL,
            RCS_ACTOR_BUS_EXCUTE_TASK_STACK_PRIO,
            &rcs_actor_bus::excute_task_handle
        );
        if (err != pdPASS) {
            rcs_kprintf("actor-bus: add actor failed,no mem!\n",portMAX_DELAY);
            return RCS_ERR_NO_MEM;
        }
    }
    if (rcs_actor_bus::watch_task_handle == NULL) {
        err = xTaskCreate(actor_bus_monitor_task,
            "actor_watch_task",
            RCS_ACTOR_BUS_MONITOR_TASK_STACK_SIZE,
            NULL,
            RCS_ACTOR_BUS_MONITOR_TASK_STACK_PRIO,
            &rcs_actor_bus::watch_task_handle
        );
        if (err != pdPASS) {
            rcs_kprintf("actor-bus: add actor failed,no mem!\n",portMAX_DELAY);
            return RCS_ERR_NO_MEM;
        }
    }
    // 检查此前是否有添加过此类执行器
    for(int i=0;i<excute_list.size();i++)
    {
        if(excute_list[i] == excuter)
        {
            rcs_kprintf("actor-bus: stoped adding actor,for it was added before\n",portMAX_DELAY);
            return RCS_OK;
        }
    }
    // 没有添加过此类执行器，则添加
    excute_list.push_back(excuter);
    reset_list.push_back(reseter);
    rcs_kprintf("actor-bus: add actor success.%d actor in list.\n",portMAX_DELAY,excute_list.size());
    return RCS_OK;
}

/**
 * @brief 让执行器总线开始工作
 * 
 * @return rcs_err_t 
 */
rcs_err_t rcs_actor_bus::actor_start(void)
{
    rcs_kprintf("actor-bus: start actor bus\n",portMAX_DELAY);
    // 未经初始化就启动，不允许
    if (event_group == NULL || excute_task_handle == NULL) {
        rcs_kprintf("actor-bus: start actor bus failed,not init\n",portMAX_DELAY);
        return RCS_ERR_NOT_ALLOWED;
    }
    // 启动excute任务
    if (xEventGroupGetBits(event_group) & EVENT_BIT_ENABLE_CTRL) {
        rcs_kprintf("actor-bus: start actor bus stoped,for already started\n",portMAX_DELAY);
        return RCS_OK;
    }
    xEventGroupSetBits(event_group, EVENT_BIT_ENABLE_CTRL);
    vTaskResume(excute_task_handle);
    rcs_kprintf("actor-bus: start actor bus success\n",portMAX_DELAY);
    return RCS_OK;
}

/**
 * @brief 关闭执行器总线，也即全局软急停
 * 
 * @return rcs_err_t 
 */
rcs_err_t rcs_actor_bus::actor_stop(void)
{
    rcs_kprintf("actor-bus: stop actor bus\n",portMAX_DELAY);
    // 未经初始化就关闭，不允许
    if (event_group == NULL || excute_task_handle == NULL) {
        rcs_kprintf("actor-bus: stop actor bus failed,not init\n",portMAX_DELAY);
        return RCS_ERR_NOT_ALLOWED;
    }
    // 停下excute任务
    xEventGroupClearBits(event_group, EVENT_BIT_ENABLE_CTRL);
    vTaskSuspend(excute_task_handle);
    // 遍历reseter_list并执行函数指针
    for(int i=0;i<reset_list.size();i++){
        reset_list[i]();
    }
    // 遍历reseter_list并执行函数指针
    for(int i=0;i<excute_list.size();i++){
        excute_list[i]();
    }
    rcs_kprintf("actor-bus: stop actor bus success\n",portMAX_DELAY);
    return RCS_OK;
}

/**
 * @brief 初始化执行器总线的观测功能
 * 
 * @param monitor_uart 通过哪一路串口输出观测波形
 * @param max_channel_count 总共有几路波形
 * @return rcs_err_t 是否成功
 * 
 * @warning 本函数不负责对UART进行初始化，请在调用本函数之前使用rcs_uart.h中的接口完成UART的初始化。
 *          由于执行器总线不会从UART读取数据，在配置时可以把Rx相关内容都给成NULL或0
 */
rcs_err_t rcs_actor_bus::monitor_setup(rcs_uart_t monitor_uart,uint8_t max_channel_count)
{
    if (monitor_uart >= RCS_UART_MAX_INSTANCE) {
        rcs_kprintf("actor_bus: monitor setup failed,uart%d is not exist\n",portMAX_DELAY,monitor_uart+1);
        return RCS_ERR_NOT_ALLOWED;
    }
    rcs_kprintf("actor_bus: monitor setup on UART%d....\n",portMAX_DELAY,monitor_uart+1);
    uart = monitor_uart;
    monitor_channel_count = max_channel_count;
    //justfloat格式，每个通道4个字节，加上4个字节的帧头
    monitor_stream.reserve(max_channel_count * 4 + 4);
    // 初始化索引
    monitor_index.reserve(max_channel_count);
    for(int i=0;i<max_channel_count;i++){
        monitor_index[i] = -1;
    }
    rcs_kprintf("actor_bus: monitor on UART%d setup finish.\n",portMAX_DELAY,monitor_uart+1);
    return RCS_OK;
}

/**
 * @brief 把连续的JustFloat波形通道绑定到某个执行器上，以观测该执行器
 * 
 * @param actor_node 哪一个执行器
 * @param start_channel 从哪个通道开始，最小值为0
 * @param end_channel   到哪个通道结束，最大值为rcs_actor_bus::monitor_channel_count-1
 * @return rcs_err_t 是否成功
 * 
 * @warning 通道数量如何得知？请自行阅读对应执行器的注释。比如RM3508执行器最少占去3通道(只观测角度或只观测速度)，最多占去6通道(速度角度都观测)。
 */
rcs_err_t rcs_actor_bus::monitor_add(rcs_actor::monitor_fptr monitor,uint8_t start_channel,uint8_t end_channel)
{
    rcs_kprintf("actor_bus: register monitor on channel %hhd-%hhd\n",portMAX_DELAY,start_channel,end_channel);
    // 参数检查
    if (monitor == nullptr) {
        rcs_kprintf("actor_bus: register monitor failed,uart is null\n",portMAX_DELAY);
        return RCS_ERR_INVALID_PARAM;
    }
    if (start_channel >= monitor_channel_count || end_channel >= monitor_channel_count || end_channel < start_channel) {
        rcs_kprintf("actor_bus: register monitor failed,channel invalid\n",portMAX_DELAY);
        return RCS_ERR_INVALID_PARAM;
    }
    // 检查此前是否有添加过此类观测器
    for(int i=0;i<monitor_list.size();i++)
    {
        if(monitor_list[i] == monitor)
        {
            rcs_kprintf("actor_bus: register monitor OK,done before\n",portMAX_DELAY);
            return RCS_OK;
        }
    }
    // 没有添加过此类观测器，则添加到观测下标映射表和观测链表
    for(uint8_t i=start_channel;i<=end_channel;i++){
        monitor_index[i] = monitor_list.size();
    }
    monitor_list.push_back(monitor);
	rcs_kprintf("actor_bus: register monitor OK.%d actor in list\n",portMAX_DELAY,monitor_list.size());
    return RCS_OK;
}

/**
 * @brief 启动执行器总线的观测功能
 * 
 * @return rcs_err_t 
 */
rcs_err_t rcs_actor_bus::monitor_start(void)
{
    rcs_kprintf("actor-bus: start monitor\n",portMAX_DELAY);
    // 未经初始化就启动，不允许
    if (event_group == NULL || watch_task_handle == NULL) {
        rcs_kprintf("actor-bus: start monitor failed,not init\n",portMAX_DELAY);
        return RCS_ERR_NOT_ALLOWED;
    }
    xEventGroupSetBits(event_group, EVENT_BIT_ENABLE_WATCH);
    vTaskResume(watch_task_handle);
    rcs_kprintf("actor-bus: start monitor success\n",portMAX_DELAY);
    return RCS_OK;
}

/**
 * @brief 关闭执行器总线的观测波形输出
 * 
 * @return rcs_err_t 
 */
rcs_err_t rcs_actor_bus::monitor_stop(void)
{
    rcs_kprintf("actor-bus: stop monitor\n",portMAX_DELAY);
    // 未经初始化就启动，不允许
    if (event_group == NULL || watch_task_handle == NULL) {
        rcs_kprintf("actor-bus: stop monitor failed,not init\n",portMAX_DELAY);
        return RCS_ERR_NOT_ALLOWED;
    }
    xEventGroupClearBits(event_group, EVENT_BIT_ENABLE_WATCH);
    if (watch_task_handle != NULL) {
        vTaskSuspend(watch_task_handle);
    }
    rcs_kprintf("actor-bus: stop monitor success\n",portMAX_DELAY);
    return RCS_OK;
}