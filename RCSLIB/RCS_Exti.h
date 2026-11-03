//文件名：  RCS_exti.h
//日期�? 2012-08-17
//作者： 郑文�?&&魏闻
//文件说明：外部中断模块封�?
//修改历史�?
//2012-08-17    23:30   柯国�?   修改函数�? 参数�? 文件�?
//2012-08-18    09:30   柯国�?   同步注释和代码，修改时间格式，建议统一�? yyyy-mm-dd
//2012-08-25	22:00	柯国�?	大量修改,改用函数封装
//2012-10-14    14:00   柯国�?   添加中断优先级的设定
//2012-12-08    17:00 	柯国�?   增加入口参数检查，规范注释
#ifndef _RCS_EXTI_H_
#define _RCS_EXTI_H_

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
#include "RCS_GPIO.h"				//duquGPIOzhuangtai

//输入输出管脚初始化及读写电平
//void SetGpioInput(GPIO_TypeDef *_port, uint32_t _pin);
//void SetGpioOutput(GPIO_TypeDef *_port, uint32_t _pin);
//uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
//void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

//@name:RCS_InitEXTI
//@brief:   初始化外部中断功�?
//@param:GPIO_TypeDef *  _port: 端口
//@param:uint32_t _pin: 端口序号
//@param:EXTITrigger_TypeDef _trigger
//      @args:EXTI_Trigger_Rising
//      @args:EXTI_Trigger_Falling
//      @args:EXTI_Trigger_Rising_Falling
//@param:FNCT_VOID _isr: 中断服务程序的函数指�?
//@param:uint8_t _priority ：前4位为抢占优先级，�?4位为响应优先�?
//@note: 清零中断函数 EXTI_ClearITPendingBit(GetRCS_EXTI_Line(_pin));

#define EXTI_MAX_NUM 	20
#define PIN_NUM				16
/// EXTI������
typedef enum rcs_exti_interface_err
{
	RCS_EXTI_OK=0,
	RCS_EXTI_REDIFINITION=-1,//��Ӧpin�Ѿ�����ʼ����
	RCS_EXTI_FULL=-2,//ע���жϺ�������������������EXTI_MAX_NUM
	RCS_EXTI_Register_callback_Error=-3,//��Ӧ��pinû�н����жϳ�ʼ��
}RCS_EXTI_Err_t;

/// EXTIģʽ
typedef enum rcs_exti_mode
{
	RCS_EXTI_RISE=1,//������
	RCS_EXTI_FALL=0,//�½���
}RCS_EXTI_CURRENT_TRIGGER;

typedef void (*RCS_EXTI_CB) (GPIO_TypeDef*,uint16_t,uint8_t);

RCS_EXTI_Err_t RCS_InitEXTI(GPIO_TypeDef *_port, uint32_t _pin,uint8_t _priority);
RCS_EXTI_Err_t RCS_EXTI_Register_callback(GPIO_TypeDef* GPIOx,uint16_t pin,RCS_EXTI_CB cb);
void SetGpioOutput(GPIO_TypeDef *_port, uint32_t _pin);
void SetGpioInput(GPIO_TypeDef *_port, uint32_t _pin);

static FNCT_VOID GetRCS_EXTI_Function_Name(uint32_t _pin);
int GetRCS_EXTI_Pin_Num(uint32_t _pin);

inline void SetGpioLow(GPIO_TypeDef *_port, uint32_t _pin)
{
	/* Check the parameters */
	assert_param(IS_GPIO_ALL_PERIPH(_port));
	assert_param(IS_GPIO_PIN(_pin));
	_port->BSRRH = _pin;
}
inline void SetGpioHigh(GPIO_TypeDef *_port, uint32_t _pin)
{
	/* Check the parameters */
	assert_param(IS_GPIO_ALL_PERIPH(_port));
	assert_param(IS_GPIO_PIN(_pin));
	_port->BSRRL = _pin;
}
inline void ToggleGpio(GPIO_TypeDef *_port, uint32_t _pin)
{
	assert_param(IS_GPIO_ALL_PERIPH(_port));
	_port->ODR ^= _pin;
}

inline uint8_t ReadGpio(GPIO_TypeDef *_port, uint32_t _pin)
{
	return GPIO_ReadInputDataBit(_port, _pin);
}
#endif  //  _RCS_EXTI_H_
