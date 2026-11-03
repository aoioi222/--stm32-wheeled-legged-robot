#ifndef G431_GPS_H_
#define G431_GPS_H_

//C语言公共库
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

//BSP与中间件
#include "matrix_functions.h" //ARM提供的矩阵运算库(硬)
#include "stm32f4xx.h"        //STM32寄存器定义
#include "stm32f4xx_conf.h"   //STM32标准库头文件
#include "bsp.h"              //uCOS底层组件
#include "includes.h"         //uCOS必备组件
#include "delay.h"            //延时函数
#include "matrix.h"           //Matrix_Hub矩阵运算库(软)

//RCSLIB硬件抽象层
#include "RCS_HAL.h"           //寄存器抽象
#include "RCSLIB_inc.h"        //外设硬件抽象

//RCSLIB模块驱动层(算法层)
#include "RCS_PIDctrl.h"       //PID算法
#include "RCS_Filter.h"        //滤波算法
#include "DoubleRing.h"        //双环PID算法
#include "FSM_Lite.h"          //有限状态机
#include "RCS_DataStructure.h" //常用数据结构
#include "RCS_Ptl.h"           //常用通信协议




//=========================配置宏=======================================

#define G431GPS_START_BYTE 0xEF
#define G431GPS_PKG_LEN    12
#define G431_GPS_BAUD      115200

typedef enum{
	CMD_ID_LOGIN = 0x01,
	CMD_ID_LOGOUT= 0x02,
	CMD_ID_SET_TX= 0x03,
	CMD_ID_SET_TY= 0x04,
	CMD_ID_SET_DX= 0x05,
	CMD_ID_SET_DY= 0x06,
	CMD_ID_SET_AX= 0x07,
	CMD_ID_SET_AY= 0x08,
	CMD_ID_SET_CF= 0x09,
	CMD_ID_SET_CR= 0x0A,
}Uart_Cmd;

//========================导出函数======================================
void G431GPS_Init(RCS_PIN_USART USARTx_MAP,uint8_t _pri);
float G431GPS_Get_X(void);
float G431GPS_Get_Y(void);
float G431GPS_Get_Z(void);
void G431GPS_Set_Parameter(Uart_Cmd CMD_ID_xx,float param);

#endif
