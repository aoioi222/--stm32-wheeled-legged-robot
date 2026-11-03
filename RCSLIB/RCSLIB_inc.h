/**
 * @filename:RCSLIB_inc.h
 * @brief:RCS硬件抽象层的大头文件
*/

/* ------.h head------------------------*/
#ifndef RCSLIB_INC_H_
#define RCSLIB_INC_H_

#include "RCS_Types.h"       //获取MCU的外设-时钟连接关系
#include "RCS_HAL.h"         //获取MCU的寄存器定义

#include "RCS_GPIO.h"
#include "RCS_Exti.h"
#include "RCS_Usart.h"
#include "RCS_ADC.h"
#include "RCS_Timer.h"
#include "RCS_CAN.h"
#include "RCS_DMA.h"
#include "RCS_spi.h"

#include "RCS_Pin_Mapping.h" //主控板的引脚映射和外设映射
//#include "RCS_dsp.h"         //定义常见的数学运算和常数
/* ------.h end--------------------------*/
#endif