/**
 * @file uart_test.c
 * @brief rcs_uart代码编写参考/外设好坏检查代码
 * @author CYK-Dot
 * @date 2025-05-23
 * ------------------------------------------------------------------------------------------
 * @note 评估方式：
 * 
 * step1:硬件连接和外设配置
 * 1. 完成硬件连接：将指定串口通过USB转串口模块连接到PC(tips:DAPLink v2既可以烧录也带有一路串口，可以使用DAPLink烧录+调试一条龙)
 * 2. 在CubeMX中配置串口，以及启用串口中断。
 * 3. 在keil的包管理器中启用EventRecorder插件，然后将Printf重定向到DAP。
 * 
 * step2:调用评估代码
 * 4. 在你需要的位置(例如app_main)调用uartTestInit()
 * 
 * step3:观察现象
 * 5. 进入keil调试，开启Debug(printf)Viewer。
 * 6. 运行代码，通过Debug(printf)Viewer查看日志输出，如果没有输出OK，代表初始化失败
 * 7. 打开串口上位机(如VOFA+)，观察UART输出。
 * 8. 通过串口上位机发送4个字节，STM32将会通过UART反馈你发送的字节
 * 
 * ------------------------------------------------------------------------------------------
 * @note 借鉴方式：
 * 1. 不建议在正式代码中使用keil提供的DAP-printf，这里只是为了调试方便引入的。
 *    (因为你在评估UART，自然不能通过UART输出调试信息，要是UART不能用，那么你什么信息都看不到)
 *    如果需要在其他场合使用printf，建议使用rcs_uart.h中的函数，以安全地printf到UART
 * 
 * 2. 为了方便评估，这里的代码创建了一个任务用于循环输出数据，但在使用的时候，任务不是必须的，
 *    只要确保uart句柄是全局变量即可
 */
#include <EventRecorder.h>
#include <stdio.h>

#include "uart_test.h"

#define UART_TEST_RX_SIZE 4

/* 全局变量 --------------------------------------------------*/

static rcs_uart_t g_uart;
uint8_t g_rxData[UART_TEST_RX_SIZE];

/* 接收回调函数 ----------------------------------------------*/

// 此函数将会在中断中被调用，不允许在函数中调用任何非中断安全的函数，或是进行阻塞
size_t uartTestaCallback(rcs_uart_t RCS_UART_NUM_x,const uint8_t *data,size_t len)
{
	if (g_uart == RCS_UART_NUM_x) {
		// 将收到的数据拷贝到一个全局变量中保存
		memcpy(g_rxData,data,UART_TEST_RX_SIZE);
		printf("uartTest:Rx Complete.\n");
	}
	return len;
}

/* 任务入口函数 -----------------------------------------------*/

static void uartTestTask(void *pvParameters)
{
	while(1)
	{
		vTaskDelay(pdMS_TO_TICKS(500));
		printf("uartTest:Send.\n");
		rcs_uart_send(g_uart,(uint8_t*)"\nHello",sizeof("\nHello"),0);
		rcs_uart_send(g_uart,(uint8_t*)" World\n",sizeof(" World\n"),0);
		rcs_uart_send(g_uart,g_rxData,4,0);
	}
}

/* 初始化函数 -------------------------------------------------*/

/**
 * @brief 初始化UART评估代码，以供学习RCS_UART的用法，或是检查外设是否损坏
 * @param huartx 串口句柄，请填入CubeMX自动生成的串口句柄，例如&huart1
 * 
 * @return 成功返回RCS_OK，失败返回RCS_FAIL
 */
rcs_err_t uart_test_init(UART_HandleTypeDef *huartx,rcs_uart_t RCS_UART_NUM_x)
{
	// 开启EventRecorder，记录全部事件
	EventRecorderInitialize(0xFFFFFFFF,1);
	EventRecorderStart();

	// 初始化UART句柄
	printf("uartTest:Init uart\n");
	const rcs_uart_conf_t conf = {
		.huartx = huartx,               // 串口1
		.rxBufferSize = UART_TEST_RX_SIZE,// 收到4个字节，进一次回调函数
		.rxCallback = uartTestaCallback,// 回调函数
		.rxMode = MODE_IT,              // 通过RXNE中断接收数据
		.txBufferSize = 100,            // 发送缓冲区大小100字节
		.txMode = MODE_IT,              // 通过TXE中断发送数据
		.txTimeout = pdMS_TO_TICKS(1),  // 每1ms清空一次缓冲区(被全部发送出去了,所以称为清空)
	};
	if (rcs_uart_driver_install(&conf,RCS_UART_NUM_x)!=RCS_OK)
	{
		printf("uartTest:Init uart failed!\n");
		return RCS_FAIL;
	}
	g_uart = RCS_UART_NUM_x;

	// 创建任务
	printf("uartTest:Init task\n");
	if (xTaskCreate(uartTestTask, "uartTestTask", configMINIMAL_STACK_SIZE, NULL, 5, NULL)!= pdPASS)
	{
		printf("uartTest:Init task failed!\n");
		return RCS_FAIL;
	}

	// 初始化成功
	printf("uartTest:Init OK!wait task to run\n");
	if (g_uart == NULL) return RCS_FAIL;
	return RCS_OK;
}
