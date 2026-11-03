#ifndef CORE407_RCS12_PINMAPPING_H_
#define CORE407_RCS12_PINMAPPING_H_

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
//RCSLIB硬件抽象层
#include "RCS_Pin_Mapping.h"

/* ------导出变量名------------------------------*/
extern RCS_PIN_USART    USART1_MAP,USART2_MAP,USART3_MAP,USART4_MAP,USART5_MAP,USART6_MAP;
extern RCS_PIN_CYLINDER CY1_MAP,CY2_MAP,CY3_MAP;
extern RCS_PIN_IO       IO1_MAP,IO2_MAP,IO3_MAP,IO4_MAP;
extern RCS_PIN_CAN      CAN1_MAP,CAN2_MAP;
extern RCS_PIN_TIM      TIMER1_MAP,TIMER2_MAP,TIMER3_MAP,TIMER4_MAP;
extern RCS_PIN_IO       IO_DEBUG_MAP;

/* -------导出函数-------------------------------*/
void RCS_Core407_PinMap_Init(void);
RCS_MCU_CLK RCS_Get_SYSCLK(void);

#endif