/**
 * @filename:RCS_Pin_Mapping.h
 * @brief:MCU各外设管脚->主控板管脚的映射
*/
#ifndef RCS_PIN_MAPPING_H_
#define RCS_PIN_MAPPING_H_

/* --------头文件 ------------------------------*/
//C语言公共库
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

//BSP与中间件
#include "stm32f4xx.h"        //STM32寄存器定义
#include "stm32f4xx_conf.h"   //STM32标准库头文件
#include "bsp.h"              //uCOS底层组件
#include "includes.h"         //uCOS必备组件
#include "delay.h"            //延时函数

//RCSLIB硬件抽象层
#include "RCS_HAL.h"

/* --------私有结构体---------------------------*/
typedef struct io_map
{
    GPIO_TypeDef* GPIOx;
    uint32_t      GPIO_Pin_x;
}RCS_MAP_IO;

/* ---------导出结构体 ------------------------*/
typedef struct can_map
{
    RCS_GPIO_T* GPIOx;
    uint32_t      GPIO_Pin_Tx;
    uint32_t      GPIO_Pin_Rx;
    RCS_CAN_T*    CANx;
}RCS_PIN_CAN;//主控上的4pinCAN接口

typedef struct usart_map
{
    RCS_GPIO_T*    GPIOx;
    uint32_t       GPIO_Pin_Tx;
    uint32_t       GPIO_Pin_Rx;
    RCS_UART_T*    USARTx;
}RCS_PIN_USART;//主控上的4pin串口

typedef struct gh6pin_io_map
{
    RCS_MAP_IO IO[4];
}RCS_PIN_IO;//主控上的6pinIO接口

typedef struct gh6pin_tim_map
{
    TIM_TypeDef* TIMx[4];
    uint8_t      Channel[4];
    RCS_MAP_IO   IO[4];
}RCS_PIN_TIM;//主控上的6pin定时器接口

typedef RCS_PIN_IO RCS_PIN_CYLINDER;//主控上的6pinQG接口
typedef RCC_ClocksTypeDef RCS_MCU_CLK;//单片机的时钟





#endif