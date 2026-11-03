#ifndef RCS_DMA_H_
#define RCS_DMA_H_
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
#endif