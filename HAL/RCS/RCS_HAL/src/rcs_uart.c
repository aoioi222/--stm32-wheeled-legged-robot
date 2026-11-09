/**
 *@filename rcs_uart.c
 *@brief RCS对USART外设操作进行的二次封装，用于通用的UART设备，例如陀螺仪、串口示波器、printf、RS-485等
 *--------------------------------------------------------------------------------------------------
 *@version v1.0
 *@author  CYK-Dot@2025-05-20
 *@changelog 标准库时期的UART接口不太合理，因此在HAL之上进行了重新设计，提供非阻塞并且有缓冲机制的UART发送
             应用场景：数据发送存在周期性，一段时间内会冒出很多待发送数据，而一段时间内数据很少，甚至没有数据
			 不适合的场景：时时刻刻都有数据需要发送，没有周期性，从不停歇。碰到这种情况，请使用HAL库原生接口的DMA发送
 *---------------------------------------------------------------------------------------------------
**/
#include <string.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_uart.h>
#include <cmsis_compiler.h>

#include "rcs_private_config.h"
#include "rcs_uart.h"
#include "rcs_os.h"

/* 私有类型定义 -----------------------------------------------------------------------------------*/

/**
 *@brief 串口对象的内部结构，对使用者来说用不到
**/
typedef struct {
	rcs_uart_conf_t config;

	size_t txBufferSize;       ///< 发送缓冲区大小
	size_t txWriteIndex;       ///< 发送缓冲区写入位置
	uint8_t *txBuffer;         ///< 发送缓冲区
	TimerHandle_t txTimeoutTimer;
	SemaphoreHandle_t txMutex;

	uint8_t *rxBuffer;
	size_t rxBufferSize;
} rcs_uart_internal_t;

/* 静态全局变量定义 --------------------------------------------------------------------------------*/

static rcs_uart_internal_t *g_uartList[RCS_UART_MAX_INSTANCE]; ///< 串口对象列表，按照顺序排列

/* 静态函数声明 ------------------------------------------------------------------------------------*/

static rcs_uart_internal_t *rcs_uart_find_by_handle(UART_HandleTypeDef *huart);
static void rcs_uart_tx_timeout_callback(xTimerHandle xTimer);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

/* 导出函数定义 -------------------------------------------------------------------------------------*/

/**
 *@brief 安装RCS_UART驱动
 *@param conf 配置参数较多，详见 @ref rcs_uart_conf_t
 *@param RCS_UART_NUM_x 串口编号，RCS_UART_NUM_1代表UART1
 *
 *@note 只有安装驱动之后，才可以使用rcs_uart_t提供的RCS_UART串口
**/
rcs_err_t rcs_uart_driver_install(const rcs_uart_conf_t *conf,rcs_uart_t RCS_UART_NUM_x)
{
	rcs_kprintf("rcs_uart:installing driver on UART%d....\n",portMAX_DELAY,RCS_UART_NUM_x+1);
	// 参数合法性检查
	if (!conf || RCS_UART_NUM_x >= RCS_UART_MAX_INSTANCE) {
		rcs_kprintf("rcs_uart:driver install failed, invalid param\n",portMAX_DELAY);
		return RCS_ERR_INVALID_PARAM;
	}
	// 检查此前是否有安装过配置
	if (g_uartList[RCS_UART_NUM_x] != NULL) {
		rcs_kprintf("rcs_uart:driver install failed, already installed before\n",portMAX_DELAY);
		return RCS_ERR_NOT_ALLOWED;
	}
	// 申请RCS串口对象
	g_uartList[RCS_UART_NUM_x] = (rcs_uart_internal_t *)RCS_UART_MALLOC(sizeof(rcs_uart_internal_t));
	if (!g_uartList[RCS_UART_NUM_x]) {
		rcs_kprintf("rcs_uart:driver install failed, malloc failed\n",portMAX_DELAY);
		return RCS_ERR_NO_MEM;
	}
	// 如果缓冲区大小大于0，则初始化缓冲区及相关功能
	if (conf->txBufferSize > 0) {
		// 初始化发送缓冲区
		g_uartList[RCS_UART_NUM_x]->txBuffer = (uint8_t *)RCS_UART_MALLOC(conf->txBufferSize);
		if (!g_uartList[RCS_UART_NUM_x]->txBuffer) {
			RCS_UART_FREE(g_uartList[RCS_UART_NUM_x]);
			rcs_kprintf("rcs_uart:driver install failed, malloc failed\n",portMAX_DELAY);
			return RCS_ERR_NO_MEM;
		}	
		memset(g_uartList[RCS_UART_NUM_x]->txBuffer,0,conf->txBufferSize);
		g_uartList[RCS_UART_NUM_x]->txWriteIndex = 0;
		g_uartList[RCS_UART_NUM_x]->txBufferSize = conf->txBufferSize;
		
		// 初始化发送锁
		g_uartList[RCS_UART_NUM_x]->txMutex = xSemaphoreCreateBinary();
		if (!g_uartList[RCS_UART_NUM_x]->txMutex) {
			RCS_UART_FREE(g_uartList[RCS_UART_NUM_x]->txBuffer);
			RCS_UART_FREE(g_uartList[RCS_UART_NUM_x]);
			rcs_kprintf("rcs_uart:driver install failed, malloc failed\n",portMAX_DELAY);
			return RCS_ERR_NO_MEM;
		}
		xSemaphoreGive(g_uartList[RCS_UART_NUM_x]->txMutex);
		
		// 初始化发送定时器
		g_uartList[RCS_UART_NUM_x]->txTimeoutTimer = xTimerCreate(NULL, //没有名字
			conf->txTimeout,                      //超时时间	
			pdFALSE,                              //是否自动重置
			(void*)g_uartList[RCS_UART_NUM_x],    //定时器ID，用于区分不同定时器，这里使用uart对象的地址作为ID，因为地址是唯一的
			rcs_uart_tx_timeout_callback          //回调函数，当定时器超时，会调用此函数
		);
		if (!g_uartList[RCS_UART_NUM_x]->txTimeoutTimer) {
			vSemaphoreDelete(g_uartList[RCS_UART_NUM_x]->txMutex);
			RCS_UART_FREE(g_uartList[RCS_UART_NUM_x]->txBuffer);
			RCS_UART_FREE(g_uartList[RCS_UART_NUM_x]);
			rcs_kprintf("rcs_uart:driver install failed, timer create failed\n",portMAX_DELAY);
			return RCS_ERR_NO_MEM;
			return NULL;
		}	
	}

	// 初始化接收缓冲区
	if (conf->rxMode != MODE_NONE && conf->rxMode != MODE_BLOCK) { 
		g_uartList[RCS_UART_NUM_x]->rxBuffer = (uint8_t *)RCS_UART_MALLOC(conf->rxBufferSize);
		if (!g_uartList[RCS_UART_NUM_x]->rxBuffer) {
			vSemaphoreDelete(g_uartList[RCS_UART_NUM_x]->txMutex);
			xTimerDelete(g_uartList[RCS_UART_NUM_x]->txTimeoutTimer, portMAX_DELAY);
			RCS_UART_FREE(g_uartList[RCS_UART_NUM_x]->txBuffer);
			RCS_UART_FREE(g_uartList[RCS_UART_NUM_x]);
			rcs_kprintf("rcs_uart:driver install failed, malloc failed\n",portMAX_DELAY);
			return RCS_ERR_NO_MEM;
			return NULL;
		}		
		g_uartList[RCS_UART_NUM_x]->rxBufferSize = conf->rxBufferSize;
	}

	// 初始化配置
	memcpy(&g_uartList[RCS_UART_NUM_x]->config, conf, sizeof(rcs_uart_conf_t));	
	rcs_kprintf("rcs_uart:UART%d driver installed.\n",portMAX_DELAY,RCS_UART_NUM_x+1);
	return RCS_OK;
}

/**
 *@brief 利用缓冲区，从串口发送数据
 *@param handle RCS串口对象
 *@param data 待发送的数据
 *@param size 长度
 *@retval 是否成功
 *
 *@note 本函数采用超时缓冲区的发送思路：
 * 1. 如果缓冲区为空，不直接通过HAL库接口发送数据，因为会有逻辑问题，而是视为下面第二种情况
 * 2. 如果缓冲区不满且可用，则将数据写入缓冲区，并重置或启动1ms定时器
 * 3. 如果缓冲区已满且可用，视为缓冲区开小了，不推荐发生，直接返回错误
 * 4. 如果缓冲区不可用，则等待缓冲区锁解除，如果超时则返回错误
 * 5. 如果1ms定时器超时抵达(也即1ms内没有人发送数据到缓冲区)，则对缓冲区上锁，然后等待RxCpltCallback回调函数解锁
 * 
 *@warning 如果缓冲区满了，并且在缓冲区锁解除之前有多个任务在等锁，那么当锁可用后，发出去的数据有可能会乱。
 *         正因如此，本函数会通过返回值告知调用者是否出现此种情况
 *
 *@warning 本函数不允许在中断中调用(包括HAL库提供的中断回调函数)，会导致死锁
 *         如有需要，请使用rcs_uart_send_from_isr函数
 *
 *@note 后续如果有时间精力，可以考虑将FreeRTOS自带的缓冲区替换为自己写的无锁缓冲区，进一步提高串口缓冲区效率
**/
rcs_err_t rcs_uart_send(rcs_uart_t RCS_UART_NUM_x,const uint8_t *data,size_t size,TickType_t xTicksToWait)
{
	// 不允许在中断中调用
	if (__get_IPSR() != 0) return RCS_ERR_NOT_ALLOWED;
	// 不允许在未经初始化的前提下使用
	if (!g_uartList[RCS_UART_NUM_x]) {
		rcs_kprintf("rcs_uart: UART%d send failed,driver not installed\n",portMAX_DELAY,RCS_UART_NUM_x+1);
		return RCS_ERR_NOT_ALLOWED;
	}
	// 参数合法性检查
	if (RCS_UART_NUM_x >=RCS_UART_MAX_INSTANCE || !data || size == 0) {
		rcs_kprintf("rcs_uart: UART%d send failed,invalid param\n",portMAX_DELAY,RCS_UART_NUM_x+1);
		return RCS_ERR_INVALID_PARAM;
	}
	
	// 如果缓冲区不存在，直接通过HAL库接口发送数据
	if (!g_uartList[RCS_UART_NUM_x]->txBuffer) {
		return HAL_UART_Transmit(g_uartList[RCS_UART_NUM_x]->config.huartx, data, size, xTicksToWait) == HAL_OK ? RCS_OK : RCS_ERR_HW;
	}	
	// 先等待发送缓冲区锁可用
	else if (xSemaphoreTake(g_uartList[RCS_UART_NUM_x]->txMutex, xTicksToWait) == pdPASS) {
		// 缓冲区不满：向缓冲区写入数据，然后重置或启动1ms定时器
		if (g_uartList[RCS_UART_NUM_x]->txWriteIndex + size <= g_uartList[RCS_UART_NUM_x]->txBufferSize) {
			memcpy(g_uartList[RCS_UART_NUM_x]->txBuffer + g_uartList[RCS_UART_NUM_x]->txWriteIndex, data, size);
			g_uartList[RCS_UART_NUM_x]->txWriteIndex += size;
			xSemaphoreGive(g_uartList[RCS_UART_NUM_x]->txMutex);//注意竞态，此函数必须早于xTimerReset
			xTimerReset(g_uartList[RCS_UART_NUM_x]->txTimeoutTimer, xTicksToWait);
			xTimerStart(g_uartList[RCS_UART_NUM_x]->txTimeoutTimer, xTicksToWait);
			return RCS_OK;
		}
		// 缓冲区满：视为缓冲区开小了，不推荐发生，直接返回错误
		else {
			xSemaphoreGive(g_uartList[RCS_UART_NUM_x]->txMutex);
			rcs_kprintf("rcs_uart: send failed,buffer overflow\n",portMAX_DELAY);
			return RCS_ERR_NO_MEM;
		}	
	}
	// 再反之，意味着锁没有在xTicksToWait时间内被释放，返回错误	
	else {
		rcs_kprintf("rcs_uart: send failed,uart busy\n",portMAX_DELAY);
		return RCS_ERR_BUSY;
	}	
}

/**
 *@brief 从中断中向UART发送数据	
 *@param handle RCS串口对象
 *@param data 待发送的数据
 *@param size 长度
 *@param pxHigherPriorityTaskWoken 是否需要唤醒更高优先级的任务
 *@retval 是否成功
 *
 *@note 本函数为rcs_uart_send的ISR版本，用于在中断中发送数据
 *
 *@warning 请在本函数结束后，到中断结束前调用portYIELD_FROM_ISR(pxHigherPriorityTaskWoken)，
 *         让RTOS在中断退出后，提前启用高优先级任务
 */
rcs_err_t rcs_uart_send_from_isr(rcs_uart_t RCS_UART_NUM_x,const uint8_t *data,size_t size,BaseType_t *pxHigherPriorityTaskWoken)
{
	// 不允许在非中断中调用
	if (__get_IPSR() == 0) return RCS_ERR_NOT_ALLOWED;
	// 不允许在未经初始化的前提下使用
	if (!g_uartList[RCS_UART_NUM_x]) {
		return RCS_ERR_NOT_ALLOWED;
	}
	// 参数合法性检查
	if (RCS_UART_NUM_x >=RCS_UART_MAX_INSTANCE || !data || !pxHigherPriorityTaskWoken || size == 0) {
		return RCS_ERR_INVALID_PARAM;
	}

	// 如果缓冲区不存在，则不允许在中断内发数据
	if (!g_uartList[RCS_UART_NUM_x]->txBuffer) {
		return RCS_ERR_NOT_ALLOWED;
	}	
	// 先等待发送缓冲区锁可用
	else if (xSemaphoreTakeFromISR(g_uartList[RCS_UART_NUM_x]->txMutex, pxHigherPriorityTaskWoken) == pdPASS) {
		// 缓冲区不满：向缓冲区写入数据，然后重置或启动1ms定时器
		if (g_uartList[RCS_UART_NUM_x]->txWriteIndex + size <= g_uartList[RCS_UART_NUM_x]->txBufferSize) {
			memcpy(g_uartList[RCS_UART_NUM_x]->txBuffer + g_uartList[RCS_UART_NUM_x]->txWriteIndex, data, size);
			g_uartList[RCS_UART_NUM_x]->txWriteIndex += size;
			xSemaphoreGiveFromISR(g_uartList[RCS_UART_NUM_x]->txMutex, pxHigherPriorityTaskWoken);
			xTimerResetFromISR(g_uartList[RCS_UART_NUM_x]->txTimeoutTimer, pxHigherPriorityTaskWoken);
			xTimerStartFromISR(g_uartList[RCS_UART_NUM_x]->txTimeoutTimer, pxHigherPriorityTaskWoken);
			return RCS_OK;
		}
		// 缓冲区满：视为缓冲区开小了，不推荐发生，直接返回错误
		else {
			xSemaphoreGiveFromISR(g_uartList[RCS_UART_NUM_x]->txMutex, pxHigherPriorityTaskWoken);
			return RCS_ERR_NO_MEM;
		}	
	}
	// 再反之，意味着锁没有在xTicksToWait时间内被释放，返回错误	
	else {
		return RCS_ERR_BUSY;
	}	
}

/**
 *@brief rcs的安全函数printf，通过串口发送
 *@param handle       RCS串口对象
 *@param maxLen       最大长度
 *@param xTicksToWait 最大等待时间，可以使用pdMS_TO_TICKS(n)获取n毫秒对应的滴答数
 *@param format       字符串格式，用法和printf一致
 *@param ...          可变参数，用法和printf一致
 *@retval 是否成功
 *
 *@warning 本函数不允许在中断中调用(包括HAL库提供的中断回调函数)，也不提供中断版本
 */
int rcs_printf(rcs_uart_t RCS_UART_NUM_x,size_t maxLen,TickType_t xTicksToWait,const char *format,...)
{
	// 不允许在中断中调用
	if (__get_IPSR() != 0) return RCS_ERR_NOT_ALLOWED;
	// 不允许在未经初始化的前提下使用
	if (!g_uartList[RCS_UART_NUM_x]) {
		rcs_kprintf("rcs_uart: UART%d printf failed,driver not installed\n",portMAX_DELAY,RCS_UART_NUM_x+1);
		return RCS_ERR_NOT_ALLOWED;
	}
	// 如果缓冲区不存在，则应先申请缓冲区，然后在结束的时候释放(不推荐)
	if (!g_uartList[RCS_UART_NUM_x]->txBuffer) {
		g_uartList[RCS_UART_NUM_x]->txBuffer = (uint8_t *)RCS_UART_MALLOC(maxLen);
		if (!g_uartList[RCS_UART_NUM_x]->txBuffer) return RCS_ERR_NO_MEM;
		va_list args;
		va_start(args, format);
		vsnprintf((char*)g_uartList[RCS_UART_NUM_x]->txBuffer, maxLen, format, args);
		va_end(args);	
		HAL_UART_Transmit(g_uartList[RCS_UART_NUM_x]->config.huartx, g_uartList[RCS_UART_NUM_x]->txBuffer, maxLen, portMAX_DELAY);
		RCS_UART_FREE(g_uartList[RCS_UART_NUM_x]->txBuffer);	
		return RCS_OK;
	}
	// 如果缓冲区存在，则直接从缓冲区中申请maxLen大小的空间，用于格式化字符串，格式化后再发送
	else {
		if (xSemaphoreTake(g_uartList[RCS_UART_NUM_x]->txMutex, portMAX_DELAY) == pdPASS) {
			// 缓冲区不满：向缓冲区写入数据，然后重置或启动1ms定时器
			if (g_uartList[RCS_UART_NUM_x]->txWriteIndex + maxLen <= g_uartList[RCS_UART_NUM_x]->txBufferSize) {
				for(int i=g_uartList[RCS_UART_NUM_x]->txWriteIndex;i<g_uartList[RCS_UART_NUM_x]->txWriteIndex+maxLen;i++)
					g_uartList[RCS_UART_NUM_x]->txBuffer[i] = '\0';
				va_list args;
				va_start(args, format);
				int realSize = vsnprintf((char*)(g_uartList[RCS_UART_NUM_x]->txBuffer + g_uartList[RCS_UART_NUM_x]->txWriteIndex), maxLen, format, args);
				va_end(args);
				g_uartList[RCS_UART_NUM_x]->txWriteIndex += realSize;
				xSemaphoreGive(g_uartList[RCS_UART_NUM_x]->txMutex);
				xTimerReset(g_uartList[RCS_UART_NUM_x]->txTimeoutTimer, xTicksToWait);
				xTimerStart(g_uartList[RCS_UART_NUM_x]->txTimeoutTimer, xTicksToWait);
				return RCS_OK;
			}
			// 缓冲区满：视为缓冲区开小了，不推荐发生，直接返回错误
			else {
				xSemaphoreGive(g_uartList[RCS_UART_NUM_x]->txMutex);
				return RCS_ERR_NO_MEM;
			}	
		}
		// 再反之，意味着锁没有在xTicksToWait时间内被释放，返回错误	
		else {
			return RCS_ERR_BUSY;
		}	
	}
	return 0;
}

/* 静态函数定义 -------------------------------------------------------------------------------------*/

/**
 *@brief 从HAL库串口对象反查RCS串口对象
 *@param huart HAL库串口对象
 *@retval RCS串口对象
 *
 *@note 本函数用于从HAL库串口对象反查RCS串口对象
 *@note 本函数为静态函数，仅供内部使用
 */
static rcs_uart_internal_t *rcs_uart_find_by_handle(UART_HandleTypeDef *huart)
{
	for (int i = 0; i < RCS_UART_MAX_INSTANCE; i++) {
		if (g_uartList[i] && g_uartList[i]->config.huartx == huart)
			return g_uartList[i];
	}
	return NULL;
}
static rcs_uart_t rcs_uartidx_find_by_handle(UART_HandleTypeDef *huart)
{
	for (rcs_uart_t i = 0; i < RCS_UART_MAX_INSTANCE; i++) {
		if (g_uartList[i] && g_uartList[i]->config.huartx == huart)
			return i;
	}
	return RCS_UART_NUM_MAX;
}

/**
 *@brief 1ms定时器超时回调函数，负责发送数据
 *@param xTimer 定时器句柄
 *
 *@note 本函数用于1ms定时器超时回调函数，负责发送数据
 *@note 本函数为静态函数，仅供内部使用
 */
static void rcs_uart_tx_timeout_callback(xTimerHandle xTimer)
{
	// 获取定时器ID，并转换为RCS串口对象
	rcs_uart_internal_t *uart = (rcs_uart_internal_t *)pvTimerGetTimerID(xTimer);
	if (!uart) {
		rcs_kprintf("rcs_uart: tx timeout callback failed,uart is NULL\n",portMAX_DELAY);
		return;
	}
	// 对缓冲区上锁(在RxCpltCallback回调函数解锁)，并发送数据
	if (xSemaphoreTake(uart->txMutex, portMAX_DELAY) == pdPASS) {//portMAX_DELAY，等待其他人先把数据写完
		if (uart->txWriteIndex > 0 && uart->txWriteIndex <= uart->txBufferSize) {
			switch(uart->config.txMode) {
				case MODE_IT:
					HAL_UART_Transmit_IT(uart->config.huartx,uart->txBuffer, uart->txWriteIndex);
					break;
				case MODE_DMA:
					HAL_UART_Transmit_DMA(uart->config.huartx, uart->txBuffer, uart->txWriteIndex);
					break;	
				default:
					break;
			}
		}
	}
}

/* 回调函数定义 -------------------------------------------------------------------------------------*/

/**
 *@brief 空函数，用于HAL库串口发送完成回调函数代理
**/
__WEAK void RCS_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	__nop();
}

/**
 *@brief 空函数，用于HAL库串口接收完成回调函数代理
**/
__WEAK void RCS_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	__nop();
}

/**
 *@brief 空函数，用于HAL库串口错误回调函数
**/
__WEAK void RCS_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	__nop();
}

/**
 *@brief HAL库串口发送完成回调函数实现
**/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	RCS_UART_TxCpltCallback(huart);
	rcs_uart_internal_t *uart =rcs_uart_find_by_handle(huart);
	BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
	// 校验 UART 句柄匹配
	if (!uart || huart != uart->config.huartx) return;
	// 缓冲区复位
	uart->txWriteIndex = 0;
	// 解锁发送队列
	xSemaphoreGiveFromISR(uart->txMutex, &pxHigherPriorityTaskWoken);	
	// 唤醒等待任务
	portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);	
}

/**
 *@brief HAL库串口接收完成回调函数实现
**/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	RCS_UART_RxCpltCallback(huart);
	rcs_uart_internal_t *uart =rcs_uart_find_by_handle(huart);
	
	// 校验 UART 句柄匹配
	if (!uart || huart != uart->config.huartx) return;

	// 只有配置了回调和缓冲区，才执行
	if (uart->config.rxCallback && uart->rxBuffer && uart->config.rxBufferSize > 0) {
		uart->config.rxCallback(rcs_uartidx_find_by_handle(huart),uart->rxBuffer, uart->config.rxBufferSize);

		// 重新启动接收
		if (uart->config.rxMode == MODE_IT) {
			HAL_UART_Receive_IT(huart, uart->rxBuffer, uart->config.rxBufferSize);
		}
		else if (uart->config.rxMode == MODE_DMA) {
			HAL_UART_Receive_DMA(huart, uart->rxBuffer, uart->config.rxBufferSize);
		}	
	}
}

/**
 *@brief HAL库串口错误回调函数实现
**/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	RCS_UART_ErrorCallback(huart);
}	
