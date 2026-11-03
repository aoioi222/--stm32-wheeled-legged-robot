//@filename:RCS_usart.h
//@author:  Morris Zhang
//@date:    18-August-2012
//@brief:   USART串口通信  @brief函数封装类库

#ifndef _RCS_USART_H_
#define _RCS_USART_H_

//C���Թ�����
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

//BSP���м��
#include "stm32f4xx.h"        //STM32�Ĵ�������
#include "stm32f4xx_conf.h"   //STM32��׼��ͷ�ļ�
#include "bsp.h"              //uCOS�ײ����
#include "includes.h"         //uCOS�ر����
#include "delay.h"            //��ʱ����

//RCSLIBӲ�������
#include "RCS_Types.h"       //��ȡMCU������-ʱ�����ӹ�ϵ
#include "RCS_HAL.h"         //��ȡMCU�ļĴ�������
#include "RCS_Pin_Mapping.h" //���ذ������ӳ�������ӳ��


//@name: USART_Config
//@brief: 配置USART的串口通信@brief
//@param: _USARTx          需要绑定的USART串口
//@param: _GPIOx           需要绑定的GPIO�?
//@param: _GPIO_PinX_T     需要绑定的串口发送管�?
//@param:_GPIO_PinX_R      需要绑定的串口接收管脚
//@param: _intFuc          接收中断函数的指�?
//@param:_baudRate         传送的波特�?
//@param: _pri             优先�?
//@note:清除中断标志�?
//USART_ClearITPendingBit(USARTx, USART_IT_RXNE);
void RCS_USART_Config(USART_TypeDef *_USARTx, GPIO_TypeDef *_GPIOx, uint32_t _GPIO_PinX_T,
					  uint32_t _GPIO_PinX_R, FNCT_VOID _intFuc, uint32_t _baudRate, uint8_t _pri);
void RCS_USART_With_NoIsr_Config(USART_TypeDef *_USARTx, GPIO_TypeDef *_GPIOx, uint32_t _GPIO_PinX_T,
                      uint32_t _GPIO_PinX_R, uint32_t _baudRate);

//@name: RCS_USART_Send_Char
//@brief: 通过已定义的串口发送一个字�?
//@param:_USARTx     已经绑定的USART串口
//@param:_character  要发送的字符

__inline void RCS_USART_Send_Char( USART_TypeDef *_USARTx, uint8_t _character)
{
	USART_SendData(_USARTx, (uint16_t)_character & 0x00ff);
	while (USART_GetFlagStatus(_USARTx, USART_FLAG_TC) == RESET);
}

//@name: RCS_USART_Send_Str
//@brief: 通过已定义的串口发送一个字符串
//@param:_USARTx   已经绑定的USART串口
//@param:_TxBuffer 要发送的字符串数�?
__inline void RCS_USART_Send_Str(USART_TypeDef *_USARTx, uint8_t _TxBuffer[])
{
	uint8_t i = 0;
	while (USART_GetFlagStatus(_USARTx, USART_FLAG_TC) == RESET);
	for (; _TxBuffer[i] != '\0'; i++)
	{
		USART_SendData(_USARTx, (uint16_t)_TxBuffer[i]);
		while (USART_GetFlagStatus(_USARTx, USART_FLAG_TC) == RESET);
	}
}

//@name: RCS_USART_Send_Data
//@brief: 通过已定义的串口发送一串数�?
//@param:_USARTx 已经绑定的USART串口
//@param:address 数据起始地址
//@param:length  数据长度
__INLINE void RCS_USART_Send_Data(USART_TypeDef *_USARTx, uint8_t *address, uint8_t length)
{
	uint8_t i = 0;
	while (USART_GetFlagStatus(_USARTx, USART_FLAG_TC) == RESET);
	for (; i < length ; i++)
	{
		USART_SendData(_USARTx, (uint16_t) * (address + i));
		while (USART_GetFlagStatus(_USARTx, USART_FLAG_TC) == RESET);
	}
}
//@name: RCS_USART_Accept_Char
//@brief: 通过已定义的串口接收一个字�?
//@param:_USARTx 已经绑定的USART串口
//@retval: uint16位的字符
__INLINE uint16_t RCS_USART_Accept_Char(USART_TypeDef *_USARTx)
{
	return USART_ReceiveData(_USARTx);
}

__INLINE void RCS_USART_Clear_RcvCplt_ITPendingBit(USART_TypeDef *_USARTx)
{
	USART_ClearITPendingBit(_USARTx,USART_IT_RXNE);
}

__INLINE int RCS_USART_Judge_RcvCplt_ITPendingBit(USART_TypeDef *_USARTx)
{
	if(USART_GetITStatus(_USARTx, USART_IT_RXNE) == SET) return 1;
	else                                                 return 0;
}

#endif //_RCS_USART_H_

