/* ----C++支持----------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif
/* -----.h Head----------------------------------------------*/
#ifndef _RCS_CAN_H_
#define _RCS_CAN_H_

/* -----包含头文件---------------------------------------------*/
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


/* ------配置宏---------------------------------------------------*/
#define CAN1_BAUD     1000000
#define CAN2_BAUD     1000000

/* ------导出函数接口----------------------------------------------*/

//配置函数
void RCS_CAN1_Config(RCS_PIN_CAN CANx_MAP,FNCT_VOID _isr, uint32_t _baudRate);
void RCS_CAN2_Config(RCS_PIN_CAN CANx_MAP,FNCT_VOID _isr, uint32_t _baudRate);

//接收函数
void RCS_CANx_Recieve(RCS_CAN_T * CANx,RCS_CAN2B_DATA_FM_RX* RxMessage);
//发送函数
void RCS_CANx_Transmit(RCS_CAN_T * CANx,RCS_CAN2B_DATA_FM_TX* TxMessage);

/* ------私有宏与结构体--------------------------------------------*/
#define CAN_DATAFM_MAX_DLC 8 //STM32F4仅支持CAN2.0B,即最大8个字节

typedef struct 
{
    //todo
}RCS_CAN2B_FLR;//CAN滤波器配置

/* ----C++支持----------------------------------------------*/
#endif
#ifdef __cplusplus
}
#endif
