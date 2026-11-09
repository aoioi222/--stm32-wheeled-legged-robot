#pragma once

#include "rcs_private_config.h"
#include "rcs_types.h"
#include "rcs_uart.h"

#include "rcs_cxx.h"

#define RCS_ACTOR_BUS_EXCUTE_TASK_DELAY pdMS_TO_TICKS(5)
#define RCS_ACTOR_BUS_EXCUTE_TASK_STACK_SIZE 512
#define RCS_ACTOR_BUS_EXCUTE_TASK_STACK_PRIO 2

#define RCS_ACTOR_BUS_MONITOR_TASK_DELAY pdMS_TO_TICKS(50)
#define RCS_ACTOR_BUS_MONITOR_TASK_STACK_SIZE 128
#define RCS_ACTOR_BUS_MONITOR_TASK_STACK_PRIO 1


/**
 * @brief 执行器基类
 * @note 无论你是电机，还是气缸，还是其他什么执行器，都必须继承自此基类，才能通过执行器总线进行统一管理
 * @warning 请勿使用该基类生成对象，因为该类是抽象类
 * 
 * @note 如果哪一年新加入了一个执行器，请参考其他执行器的代码编写方式，继承该类，以确保机器人上所有的执行器都能被执行器总线维护
 *       为何要这么做？这是为了从制度上避免控制不连续
 * 
 * @note 使用方式：
 *       1. 编写一个新的类，让他继承自 rcs_actor_base
 *       2. 在该类中，根据执行器的情况，自定义update函数的实现
 *       3. 在该类中，实现 get_excute_fptr 和 get_reset_fptr 两个虚函数
 *       4. 在该类中，实现 get_monitor_data 虚函数，用于实时观测
 *       5. 单独对该类进行测试，确保其功能正常。
 */
class rcs_actor
{
    public:
        // 虚构造函数，防止被实例化
        virtual ~rcs_actor() = default;
        // 子类必须实现三个函数才能被actor_bus管理，三者的函数签名如下所示
        using excute_fptr = rcs_err_t (*)(void);
        using reset_fptr = rcs_err_t (*)(void);
        using monitor_fptr = float (*)(uint8_t channel);
};

/**
 * @brief 执行器总线类，提供统一的控制和观测接口
 * @warning 为了避免并发问题，执行器总线类只提供单例模式，全部功能都是类直属的，请勿实例化对象
 * 
 * @note 使用方式(初始化)：
 *       1. 先根据需求编写执行器类，并先确保其能正常工作，否则不要使用执行器总线。如果有现成的类可用，可以跳过这一步。
 *       2. 初始化每一个执行器，初始化方式取决于执行器类是怎么编写的。
 *       3. 调用rcs_actor_bus::setup函数，初始化执行器总线。
 *       4. 逐个调用rcs_actor_bus::add_actor函数，确保每一个执行器都被添加到执行器总线中。
 *       5. 调用rcs_actor_bus::start函数，启动执行器总线。
 * 
 * @note 使用方式(控制):
 *       1. 先确保执行器总线已经完整初始化并启动，不要试图在初始化之前控制。
 *       2. 整个控制过程都不需要和执行器总线打交道，只需要调用执行器类提供的接口即可，很自由。
 * 
 * @note 使用方式(监测):
 *       1. 使用rcs_uart的接口创建一个串口对象，用于实时观测。
 *       2. 调用rcs_actor_bus::monitor_setup函数，初始化监测功能。建议放在rcs_actor_bus::start之前。
 *       3. 逐个调用执行器提供的观测接口，指定要观测哪些数据。
 *       4. 逐个调用rcs_actor_bus::monitor_add函数，只需要加入自己关心的执行器即可，太多了波形很乱。
 *       5. 调用rcs_actor_bus::monitor_start函数，启动监测功能。建议放在rcs_actor_bus::start之后。
 *       6. 完成了观测的初始化之后，无需任何操作，串口会自动输出监测数据，在VOFA接收即可。
 */
class rcs_actor_bus
{
    private:
        // 类私有成员1：宏
        static constexpr uint32_t EVENT_BIT_ENABLE_CTRL  = (1 << 0);
        static constexpr uint32_t EVENT_BIT_ENABLE_WATCH = (1 << 1);
        // 类私有成员2：已注册的执行器
        static std::vector<rcs_actor::excute_fptr> excute_list;  ///< 执行器链表(执行)
        static std::vector<rcs_actor::reset_fptr> reset_list;    ///< 执行器链表(复位)
        // 类私有成员3：justfloat打印波形相关
        static rcs_uart_t uart;                                  ///< 波形输出串口
        static std::vector<uint8_t> monitor_stream;              ///< 波形输出数据流
        static uint8_t monitor_channel_count;                    ///< 波形输出通道数
        static std::vector<int16_t> monitor_index;               ///< 通道-执行器下标映射表(monitor_index[通道]=monitor_list下标)
        static std::vector<rcs_actor::monitor_fptr> monitor_list;///< 执行器链表(监测)
        // 类私有成员4：总线运行时信息
        static TaskHandle_t excute_task_handle;                  ///< 执行器总线任务句柄
        static TaskHandle_t watch_task_handle;                   ///< 执行器总线监测任务句柄
        static EventGroupHandle_t event_group;                   ///< 执行器总线事件组
		// 类私有方法：总线任务
		static void actor_bus_excute_task(void* param);          ///< 执行器总线任务函数
		static void actor_bus_monitor_task(void* param);         ///< 执行器总线监测任务函数
       
    public:
        // 类公开方法1：设备控制
        static rcs_err_t actor_add(rcs_actor::excute_fptr excuter,rcs_actor::reset_fptr reseter);
        static rcs_err_t actor_start(void);
        static rcs_err_t actor_stop(void);
        // 类公开方法2：设备监测
        static rcs_err_t monitor_setup(rcs_uart_t monitor_uart,uint8_t max_channel_count);
        static rcs_err_t monitor_add(rcs_actor::monitor_fptr monitor,uint8_t start_channel,uint8_t end_channel);
        static rcs_err_t monitor_start(void);
        static rcs_err_t monitor_stop(void);
};
