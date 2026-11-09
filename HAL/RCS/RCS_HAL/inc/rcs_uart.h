/**
 *@filename rcs_uart.h
 *@brief RCS对USART外设操作进行的二次封装，用于通用的UART设备，例如陀螺仪、串口示波器、printf、RS-485等
 *--------------------------------------------------------------------------------------------------
 *@version v1.0
 *@author  CYK-Dot@2025-05-20
 *@changelog 标准库时期的UART接口不太合理，因此在HAL之上进行了重新设计，提供非阻塞并且有缓冲机制的UART发送
 *---------------------------------------------------------------------------------------------------
**/
#pragma once

/* 头文件依赖 -------------------------------------------------------------------------------------*/

#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <math.h>

#include <FreeRTOS.h>
#include <task.h>
#include <timers.h>
#include <semphr.h>
#include <queue.h>

#include "rcs_private_config.h"
#include "rcs_types.h"


/* 配置宏定义 -------------------------------------------------------------------------------------*/

#define RCS_UART_MALLOC pvPortMalloc  ///< 如果没有使用FreeRTOS，改为malloc
#define RCS_UART_FREE   vPortFree     ///< 如果没有使用FreeRTOS，改为free

#define RCS_UART_MAX_INSTANCE 6       ///< 最多允许同时存在多少个RCS串口对象

/* 导出类型定义 -----------------------------------------------------------------------------------*/

typedef enum
{
	RCS_UART_NUM_1 = 0,
	RCS_UART_NUM_2,
	RCS_UART_NUM_3,
	RCS_UART_NUM_4,
	RCS_UART_NUM_5,
	RCS_UART_NUM_6,
	RCS_UART_NUM_7,
	RCS_UART_NUM_8,
	RCS_UART_NUM_9,
	RCS_UART_NUM_10,
	RCS_UART_NUM_11,
	RCS_UART_NUM_12,
	RCS_UART_NUM_13,
	RCS_UART_NUM_14,
	RCS_UART_NUM_15,
	RCS_UART_NUM_MAX,
}rcs_uart_t;

/**
 *@brief 串口接受回调函数的声明结构
 *@param handle 串口对象的句柄，用于区分是哪个串口
 *@param data 收到的数据，以数组的方式提供
 *@param len 收到的数据长度
 *@retval size_t 下一次调用回调函数时要接受几个字节，如果没有特别需求，可以返回len 
**/
typedef size_t (*rcs_uart_callback_ptr) (rcs_uart_t RCS_UART_NUM_x,const uint8_t*,size_t);

/**
 *@brief 串口发送/接收模式
**/
typedef enum 
{
	MODE_NONE  = 0, ///< 不使用
	MODE_BLOCK = 1, ///< 阻塞模式
	MODE_IT    = 2, ///< 中断模式
	MODE_DMA   = 3, ///< DMA模式
}rcs_uart_mode_t;

/**
 *@brief RCS串口对象的配置参数
 *
 *@param huartx        HAL库串口对象的指针，CubeMX会帮我们自动生成，比如UART1就是huart1
 *
 *@param txMode        使用什么方式发送， @ref rcs_uart_mode_t
 *@param txBufferSize  发送缓冲区大小，单位字节。如果不需要发送，或发送模式为MODE_BLOCK，给0即可
 *@param txTimeout     发送超时时间，需要确保txBufferSize个字节能在txTimeout时间内发送完毕
 *
 *@param rxMode        使用什么方式接收。请勿使用MODE_BLOCK，其他模式详见 @ref rcs_uart_mode_t
 *@param rxBufferSize  接收缓冲区大小，单位字节。如果不需要接收，给0即可。如果追求响应速度，请使用MODE_IT接收，并让缓冲区大小为1
 *@param rxCallback    当接收字节数量抵达缓冲区大小时，此回调函数将会被调用，可以在里面收数据
**/
typedef struct 
{
	UART_HandleTypeDef *huartx; ///< HAL库串口对象的指针
	rcs_uart_mode_t txMode;     ///< 发送模式
	size_t txBufferSize;       ///< 发送缓冲区大小
	TickType_t txTimeout;      ///< 发送超时时间，需要确保txBufferSize个字节能在txTimeout时间内发送完毕，可以用波特率计算

	rcs_uart_mode_t rxMode;           ///< 接收模式
	size_t rxBufferSize;              ///< 接收缓冲区大小
	rcs_uart_callback_ptr rxCallback; ///< 接收回调函数
}rcs_uart_conf_t;



/* 导出函数声明 -----------------------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

// 构造与析构函数
rcs_err_t rcs_uart_driver_install(const rcs_uart_conf_t *conf,rcs_uart_t RCS_UART_NUM_x);

// 成员函数
rcs_err_t rcs_uart_send(rcs_uart_t RCS_UART_NUM_x,const uint8_t *data,size_t size,TickType_t xTicksToWait);
rcs_err_t rcs_uart_send_from_isr(rcs_uart_t RCS_UART_NUM_x,const uint8_t *data,size_t size,BaseType_t *pxHigherPriorityTaskWoken);
int rcs_printf(rcs_uart_t RCS_UART_NUM_x,size_t maxLen,TickType_t xTicksToWait,const char *format,...);

// 导出回调函数(HAL原生回调函数代理)
__WEAK void RCS_UART_TxCpltCallback(UART_HandleTypeDef *huart);
__WEAK void RCS_UART_RxCpltCallback(UART_HandleTypeDef *huart);
__WEAK void RCS_UART_ErrorCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

