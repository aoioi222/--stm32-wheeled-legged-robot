//@filename: Timer.c
//@date: 2012-08-16
//@author: 鏉庝竾闆?
//@brief: 鏃堕挓涓柇

#ifndef _TIMER_H_
#define _TIMER_H_

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
#include "RCS_Types.h"       //获取MCU的外设-时钟连接关系
#include "RCS_HAL.h"         //获取MCU的寄存器定义
#include "RCS_Pin_Mapping.h" //主控板的引脚映射和外设映射


//@name: InitTimerInt
//@brief: 鍚姩鏃堕挓涓柇
//@param:TIM_TypeDef * _TIM 瀹氭椂鍣ㄥ彿
//@param:uint32_t _TIM_period 璁℃暟鍛ㄦ湡
//@param:uint32_t  _div 灏嗕竴绉掑垎鍓蹭负_div涓崟浣?,蹇呴』澶т簬3000锛屽惁鍒欏垎棰戞孩鍑?
//@param:FNCT_VOID _TIM_ISR: ISR鎸囬拡
//@param:uint8_t _priority 锛氬墠4浣嶄负鎶㈠崰浼樺厛绾э紝鍚?4浣嶄负鍝嶅簲浼樺厛绾?
//@notes锛欼SR鍑芥暟鏈�鍚庨渶瑕? 娓呬腑鏂?
//TIM_ClearITPendingBit(TIMx,TIM_IT_Update);
//鍛ㄦ湡璁＄畻涓? _TIM_period/_div 绉?
void InitTimerInt(TIM_TypeDef *_TIM, uint32_t _TIM_period, uint32_t  _div,
				  FNCT_VOID _TIM_ISR, uint8_t _priority);

//@name: StopTimer
//@brief: 鍋滄鏃堕挓涓柇
//@param:TIM_TypeDef * _TIM 瀹氭椂鍣ㄥ彿
void StopTimer(TIM_TypeDef *_TIM );
void StartTimer(TIM_TypeDef *_TIM);
#endif
