/**
 * @file rmmotor_test.c
 * @brief 大疆电机评估代码
 * @author CYK-Dot@2025-05-22
 *--------------------------------------------------------------------------------------------------
 * @note 评估方式：
 * 
 * step1:硬件连接和外设配置
 * 1. 完成硬件连接：将CAN_L和CAN_H连接到大疆电调上的CAN口，并使用DAPLink v2(例如DAPLink、WCHLinkW、正点原子无线烧录器)连接单片机
 * 2. 在CubeMX中配置CAN，以及启用CAN中断，注意CAN1和CAN2都要配置。
 * 3. 在keil的包管理器中启用EventRecorder插件，然后将Printf重定向到DAP。
 * 
 * step2:调用评估代码
 * 4. 在你需要的位置(例如app_main)调用rmmotor_test_init()
 * 
 * step3:观察现象
 * 5. 进入keil调试，开启Debug(printf)Viewer。
 * 6. 运行代码，通过Debug(printf)Viewer查看日志输出，如果没有输出OK，代表初始化失败
 * 7. ID为1~4的RM3508/RM2006电机将会以400的转速旋转，两条CAN线上的电机都会转。
 * 8. 如果电机不转，代表CAN总线不通。如果电机疯转，代表主控的CAN只能发不能收，或者任务被打断。
 * 
 *--------------------------------------------------------------------------------------------------
 * @note 借鉴方式：
 * 1. 不建议在正式代码中使用keil提供的DAP-printf，这里只是为了调试方便引入的。
 *    (因为你在评估CAN，自然不能通过CAN输出调试信息，要是CAN不能用，那么你什么信息都看不到)
 *    如果需要在其他场合使用printf，建议使用rcs_uart.h中的函数，以安全地printf到UART
 * 
 * 2. 为了方便评估，这里的代码创建了两个任务用于循环输出数据和打印数据，但在使用的时候，任务个数看你情况给，不一定是2个。
 *    只要确保CAN句柄是全局变量，并且确保电机控制函数在任务上下文中被调用即可。
 * 
 * 3. 不建议使用本代码写的方式控制电机，而应使用MotorList，以面向对象的方式控制。
 *    本代码仅用于展示如何用最原始的方式控制电机(也即MotorList的底层实现原理)，或是检查CAN总线好坏。
**/

/* 包含头文件 --------------------------------------------------------------------------------------*/

#include <stdio.h>
#include <EventRecorder.h>

#include <FreeRTOS.h>
#include <task.h>

#include <array>

#include "rcs_os.h"

#include "sync_pid.h"
#include "rm_motor_raw.h"

#include "rmmotor_test.h"

/* 全局变量 ----------------------------------------------------------------------------------------*/

// 将HAL_CAN对象和RCS_CAN对象绑定
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

// 3508的PID参数
static std::array<sync_pid,4> rm3508_pid_can1 = {sync_pid(RCS_ASYN_PID_DEFAULT_RM3508_SPPED_PID), sync_pid(RCS_ASYN_PID_DEFAULT_RM3508_SPPED_PID), sync_pid(RCS_ASYN_PID_DEFAULT_RM3508_SPPED_PID), sync_pid(RCS_ASYN_PID_DEFAULT_RM3508_SPPED_PID)};
static std::array<sync_pid,4> rm3508_pid_can2 = {sync_pid(RCS_ASYN_PID_DEFAULT_RM3508_SPPED_PID), sync_pid(RCS_ASYN_PID_DEFAULT_RM3508_SPPED_PID), sync_pid(RCS_ASYN_PID_DEFAULT_RM3508_SPPED_PID), sync_pid(RCS_ASYN_PID_DEFAULT_RM3508_SPPED_PID)};

/* 测试任务代码 ------------------------------------------------------------------------------------*/

static void rmmotor_test_task(void *pvParameters)
{
    static int16_t current_can1[4] = {0};
    static int16_t current_can2[4] = {0};

    // 使用原始的接口编写电机控制代码，必须遵循严格的代码编写逻辑：
    //   1. 统一计算输出值，无论是通过PID还是通过其他什么方式，而且计算的过程不允许阻塞。
    //   2. 统一发送CAN报文，不能隔一段时间发送，或者一个电机用一条报文控制。（因为一条报文控制四个电机，所以四个电机必须一起控，否则会导致控制频率出错，进而引发PID计算异常，最终导致电机发疯)
    //   3. 进行延时，延时时间取决于PID的环路控制频率。然后回到第一步。
    //
    // 为了方便沟通，以上三步队里的习惯称呼是：
    //   1. 更新(update)
    //   2. 执行(excute)
    //   3. 延时(delay_ms)
    // 
    // 早期时候，上述做法没有用函数的方式做硬性约束，仅靠学长口口相传。
    // 随着这两年培训时间的缩短，有不少新队员不重视这个做法，写出了各种各样奇怪的代码，导致疯车伤人，或是代码整合困难。
    // 所以在2024年的时候，在原始代码之上新加了“电机链表+电机节点”的概念，通过代码接口，从底层制止不规范的写法。
    // 但电机链表屏蔽了CAN总线底层细节，新队员几乎不懂电机是怎么转起来的，只懂代码怎么用，出了问题更是无从下手。
    // 正因如此，今年新整理的代码，强制要求所有队员必须先跑通原始做法，然后再去学习更易使用的电机节点+链表。
    // 除此之外，2024年的电机链表使用C面向对象，利用了很多高级技巧，不便于代码阅读，因此今年更换为了C++，代码简洁了不少。
    while(1)
    {
        /* 更新-------------------------------------------------------------------------------------*/
        // 这一部分的代码只负责进行计算，不负责控制任何硬件
        // 在标准库老代码(F407_Practice)写法中，状态机会写在这个部分，但这里比较简单不需要状态机，所以省略了
        current_can1[0] = rm3508_pid_can1[0].calculate(400,rm_motor_get_speed_rm3508_rpm(RCS_CAN_NUM_1,1));
        current_can1[1] = rm3508_pid_can1[1].calculate(400,rm_motor_get_speed_rm3508_rpm(RCS_CAN_NUM_1,2));
        current_can1[2] = rm3508_pid_can1[2].calculate(400,rm_motor_get_speed_rm3508_rpm(RCS_CAN_NUM_1,3));
        current_can1[3] = rm3508_pid_can1[3].calculate(400,rm_motor_get_speed_rm3508_rpm(RCS_CAN_NUM_1,4));
        current_can2[0] = rm3508_pid_can2[0].calculate(400,rm_motor_get_speed_rm3508_rpm(RCS_CAN_NUM_2,1));
        current_can2[1] = rm3508_pid_can2[1].calculate(400,rm_motor_get_speed_rm3508_rpm(RCS_CAN_NUM_2,2));
        current_can2[2] = rm3508_pid_can2[2].calculate(400,rm_motor_get_speed_rm3508_rpm(RCS_CAN_NUM_2,3));
        current_can2[3] = rm3508_pid_can2[3].calculate(400,rm_motor_get_speed_rm3508_rpm(RCS_CAN_NUM_2,4));

        /* 执行-------------------------------------------------------------------------------------*/
        // 这一部分的代码只负责控制硬件，不负责任何运算
        rm_motor_send_current_rm3508_front(RCS_CAN_NUM_1,current_can1[0],current_can1[1],current_can1[2],current_can1[3],0);
        rm_motor_send_current_rm3508_front(RCS_CAN_NUM_2,current_can2[0],current_can2[1],current_can2[2],current_can2[3],0);

        /* 延时-------------------------------------------------------------------------------------*/
        // RTOS延时，此时CPU会去执行其他任务，延时抵达了才会回来
        // 标准库老代码使用uC/OS-II提供的delay_ms进行延时，新代码换到了更容易使用的FreeRTOS，使用vTaskDelay延时
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

static void rmmotor_test_watch_task(void *pvParameters)
{
    while(1)
    {
        rcs_kprintf("can1 angle=%.1f %.1f %.1f %.1f\n",
			portMAX_DELAY,
            rm_motor_get_angle_rm3508_deg(RCS_CAN_NUM_1,1),
            rm_motor_get_angle_rm3508_deg(RCS_CAN_NUM_1,2),
            rm_motor_get_angle_rm3508_deg(RCS_CAN_NUM_1,3),
            rm_motor_get_angle_rm3508_deg(RCS_CAN_NUM_1,4)
        );
        rcs_kprintf("can2 angle=%.1f %.1f %.1f %.1f\n",
			portMAX_DELAY,
            rm_motor_get_angle_rm3508_deg(RCS_CAN_NUM_2,1),
            rm_motor_get_angle_rm3508_deg(RCS_CAN_NUM_2,2),
            rm_motor_get_angle_rm3508_deg(RCS_CAN_NUM_2,3),
            rm_motor_get_angle_rm3508_deg(RCS_CAN_NUM_2,4)
        );
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* 导出函数定义 ------------------------------------------------------------------------------------*/

/**
 * @brief 用最为原始的方式控制大疆电机(也即一次控制4个)
 * @param can_index CAN索引，0为CAN1，1为CAN2
 * @return CAN对象
 */
void rmmotor_test_init(void)
{
    // 初始化kprintf
    rcs_kprintf_init();
    rcs_kprintf("rmmotor_test:init start\n",portMAX_DELAY);

    // 将HAL_CAN句柄绑定到RCS_CAN上，并让CAN支持大疆电机
	// 不同于老代码，新代码的CAN初始化和电机初始化整合到了一起，避免使用上的反直觉
    rm_motor_init(&hcan1,RCS_CAN_NUM_1);
    rm_motor_init(&hcan2,RCS_CAN_NUM_2);
    
    // 初始化电机控制任务(原upctrl_task)
    rcs_kprintf("rmmotor_test:init ctrl task\n",portMAX_DELAY);
    if (xTaskCreate(rmmotor_test_task,"rmmotor_test",1024,NULL,5,NULL) != pdPASS) {
        rcs_kprintf("rmmotor_test:init task failed\n",portMAX_DELAY);
    }

   // 初始化角度观测任务(原ble_task)
   printf("rmmotor_test:init watch task\n");
   if (xTaskCreate(rmmotor_test_watch_task,"rmmotor_test_watch",1024,NULL,6,NULL) != pdPASS) {
       printf("rmmotor_test:init watch task failed\n");
   }

    // 成功
    rcs_kprintf("rmmotor_test:init OK.\n",portMAX_DELAY);
}