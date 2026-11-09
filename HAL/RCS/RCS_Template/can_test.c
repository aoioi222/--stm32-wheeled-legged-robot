/**
 * @file can_test.c
 * @brief CAN总线评估代码，用于学习CAN总线设备的驱动编写和调试方式
 * @author CYK-Dot@2025-05-28
 *--------------------------------------------------------------------------------------------------
 * @note 评估方式
 * 
 * 评估总览
 * 1. 回环测试：只需要一块主控板，无需接入CAN设备，用于学习CAN外设相关的知识
 * 2. 在线测试：用于评估CAN总线拥挤程度
 * 
 * 评估1：回环测试
 * 1. 在CubeMX中将CAN1配置为回环模式(loopback)，然后重新生成代码
 * 2. 在需要的地方(如app_main)，调用一次can_test_init
 * 3. 烧录程序并进入keil调试，然后开启Debug(printf)Viewer
 * 4. 让程序开始运行，将会源源不断地出现check OK
 * 5. 尝试将CAN_TEST_MSG_COUNT改到大于3，编译烧录调试，观察结果。
 * 6. 尝试将CAN_TEST_MSG_COUNT改到天文数字(比如100)，编译烧录调试，观察结果。
 * 7. 使用HAL库提供的CAN发送函数，尝试一次发送超过3条报文，编译烧录调试，观察结果。
 * 8. 翻阅STM32F4xx参考手册，了解为什么在没有缓冲区时，一次发送超过3条报文会出问题。
 * 9. 上网学习生产者-消费者模式，了解缓冲区的本质。
 * 
 * 评估2：在线测试
 * 1. 在CubeMX中将CAN1配置为常规模式(normal)，然后重新生成代码
 * 2. 将主控板的CAN1接入待测CAN总线。
 * 3. 在需要的地方(如app_main)，调用一次can_test_init
 * 4. 根据你的情况，修改CAN_TEST_INTERVAL_MS，也即多少ms集中发送一批报文，一般是5ms。
 * 4. 烧录程序并进入keil调试，然后开启Debug(printf)Viewer
 * 5. 让程序开始运行，将会源源不断地出现check OK
 * 6. 不断调高CAN_TEST_MSG_COUNT，直到不再OK
 * 7. 得到的CAN_TEST_MSG_COUNT就是待测CAN总线的承载力上限。
 **/
#include "can_test.h"
#include "rcs_can.h"
#include "rcs_os.h"
#include <string.h>
#include <stdio.h>

/* 私有定义 ------------------------------------------------------------------------------------------*/

// 一次发送多少条报文
#define CAN_TEST_MSG_COUNT 6

// 每批报文发送的间隔
#define CAN_TEST_INTERVAL_MS 500
// 发送多少批检查一次是否发送正确
#define CAN_TEST_VALIDATE_INTERVAL 1

// 测试任务的栈大小和优先级
#define CAN_TEST_TASK_STACK_SIZE 256
#define CAN_TEST_TASK_PRIORITY 3

/* 私有变量 ------------------------------------------------------------------------------------------*/

extern CAN_HandleTypeDef hcan1;

static rcs_can_t g_canHandle = NULL;
static TaskHandle_t g_testTaskHandle = NULL;
uint8_t g_rxCount = 0;
static bool g_rxSuccess = false;

/* 私有函数声明 --------------------------------------------------------------------------------------*/

static void can_test_task(void *pvParameters);
static void can_test_rx_callback(rcs_can_t handle, const CAN_RxHeaderTypeDef *header, uint8_t *data);

/* 函数实现 ------------------------------------------------------------------------------------------*/

/**
 * @brief CAN接收回调函数--检查数据是否被成功发送
 * @param handle CAN句柄
 * @param header 接收到的报文头
 * @param data 接收到的数据
 */
static void can_test_rx_callback(rcs_can_t handle, const CAN_RxHeaderTypeDef *header, uint8_t *data)
{
    // 通过ID区分不同设备，如果一个设备会发送多种ID的报文，可以使用if(xxx||xxx)
    if (header->StdId == 0x123) {
        g_rxCount++;
        if (g_rxCount >= CAN_TEST_MSG_COUNT) {
            g_rxSuccess = true; // 本函数位于中断上下文中，因此不应做过多操作，例如printf
            g_rxCount = 0;
        }
    }
}

/**
 * @brief CAN测试任务
 */
static void can_test_task(void *pvParameters)
{
    CAN_TxHeaderTypeDef txHeader;
    uint8_t txData[8];
    
    // 配置发送报文头
    txHeader.StdId = 0x123;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = 8;
    
    while (1) {

        // 发送6条测试报文
        for (uint8_t i = 0; i < CAN_TEST_MSG_COUNT; i++) {
            memset(txData, 0, sizeof(txData));
            txData[0] = i;
            if (!rcs_can_send(g_canHandle, &txHeader, txData, pdMS_TO_TICKS(10))) {
                rcs_kprintf("can_test:txfailed for %d\n",0,i);
            }
        }

        // 等待下一次测试
        vTaskDelay(pdMS_TO_TICKS(CAN_TEST_INTERVAL_MS));

        // 检查接收结果
        if (g_rxSuccess) {
            rcs_kprintf("can_test:tx check OK\n",0);
			g_rxCount = 0;
            g_rxSuccess = false;
        } 
        else {
			g_rxCount = 0;
            rcs_kprintf("can_test:tx check Failed\n",0);
        }
    }
}

/**
 * @brief 初始化CAN测试
 * @return 是否初始化成功
 */
bool can_test_init(void)
{
	// 初始化printf
	rcs_kprintf_init();
    // 创建CAN对象
    rcs_can_conf_t canConf = RCS_CAN_DEFAULT_CONF_CAN1;
    g_canHandle = rcs_can_create(&canConf);
    if (!g_canHandle) {
        printf("can_test:can init failed\n");
        return false;
    }
    
    // 订阅CAN接收回调
    if (!rcs_can_subscribe(g_canHandle, can_test_rx_callback)) {
        printf("can_test:can subscribe failed\n");
        return false;
    }
    
    // 创建测试任务
    BaseType_t ret = xTaskCreate(can_test_task,
                                "canTest",
                                CAN_TEST_TASK_STACK_SIZE,
                                NULL,
                                CAN_TEST_TASK_PRIORITY,
                                &g_testTaskHandle);
    if (ret != pdPASS) {
        printf("can_test:can task create failed\n");
        return false;
    }
    
    printf("can_test: init OK...\n");
    return true;
}



