#ifndef RCS_HAL_H_
#define RCS_HAL_H_

#include "stm32f4xx.h"        //STM32寄存器定义

/* -------导出寄存器名---------------------------*/

//CAN2.0B数据帧描述符的HAL定义
typedef CanTxMsg  RCS_CAN2B_DATA_FM_TX; 
typedef CanRxMsg  RCS_CAN2B_DATA_FM_RX;

//CAN外设中断的HAL定义
#define RCS_CAN_RXIT_FLAG(CANx) CAN_GetITStatus(CANx, CAN_IT_FMP0) 

/*--------导出外设名------------------------------*/
typedef CAN_TypeDef   RCS_CAN_T;
typedef GPIO_TypeDef  RCS_GPIO_T;
typedef USART_TypeDef RCS_UART_T;




#endif