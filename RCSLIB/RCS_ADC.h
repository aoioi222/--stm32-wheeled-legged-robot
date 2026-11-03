//@filename: ADC.h
//@date: 2019-07-22
//@author: é—«é”
//@brief: æ¨¡æ‹Ÿé‡è½¬æ¢æ•°å­—é‡
#ifndef _RCS_ADC_H_
#define _RCS_ADC_H_

//CÓïÑÔ¹«¹²¿â
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

//BSPÓëÖÐ¼ä¼þ
#include "stm32f4xx.h"        //STM32¼Ä´æÆ÷¶¨Òå
#include "stm32f4xx_conf.h"   //STM32±ê×¼¿âÍ·ÎÄ¼þ
#include "bsp.h"              //uCOSµ×²ã×é¼þ
#include "includes.h"         //uCOS±Ø±¸×é¼þ
#include "delay.h"            //ÑÓÊ±º¯Êý

//RCSLIBÓ²¼þ³éÏó²ã
#include "RCS_Types.h"       //»ñÈ¡MCUµÄÍâÉè-Ê±ÖÓÁ¬½Ó¹ØÏµ
#include "RCS_HAL.h"         //»ñÈ¡MCUµÄ¼Ä´æÆ÷¶¨Òå
#include "RCS_Pin_Mapping.h" //Ö÷¿Ø°åµÄÒý½ÅÓ³ÉäºÍÍâÉèÓ³Éä

#define LASER_ADC_ONE                   ADC1
#define LASER_GPIO_ONE                  GPIOA
#define LASER_PIN_ONE                   GPIO_Pin_4
#define LASER_CHANNEL_ONE               ADC_Channel_4

#define LASER_ADC_TWO                   ADC1
#define LASER_GPIO_TWO                  GPIOA
#define LASER_PIN_TWO                   GPIO_Pin_5
#define LASER_CHANNEL_TWO               ADC_Channel_5

#define LASER_ADC_THREE                 ADC1
#define LASER_GPIO_THREE                GPIOA
#define LASER_PIN_THREE                 GPIO_Pin_6
#define LASER_CHANNEL_THREE             ADC_Channel_6

#define LASER_ADC_FOUR                  ADC1
#define LASER_GPIO_FOUR                 GPIOA
#define LASER_PIN_FOUR                  GPIO_Pin_7
#define LASER_CHANNEL_FOUR              ADC_Channel_7

#define VOLTAGE_REFERANCE	3.3f//å‚è€ƒç”µåŽ?
#define	MAX_RANGE			4096//æœ€å¤§é‡ç¨?

//@name: RCS_ADC_Init
//@brief: åˆå§‹åŒ–ADC
//@param:ADC_TypeDef *_ADCx ADCå?
//@param:GPIO_TypeDef *_GPIOx GPIOç»?
//@param:uint32_t _pin ç®¡è„š
//@note:æµ‹é‡ç”µåŽ‹ä¸å¾—è¶…è¿‡3.3V
void RCS_ADC_Init(ADC_TypeDef *_ADCx, GPIO_TypeDef *_GPIOx, uint32_t _pin);

//@name: RCS_Get_ADC
//@brief: èŽ·å–ADCçš„å€?
//@param:ADC_TypeDef *_ADCx ADCå?
//@param:ADC_Channel_x ç®¡è„šé€šé“å?
uint16_t RCS_Get_ADC(ADC_TypeDef *_ADCx, uint8_t ADC_Channel_x);

//@name: RCS_Get_Voltage        
//@brief: èŽ·å–ADCçš„ç”µåŽ‹å€?
//@param:ADC_TypeDef *_ADCx ADCå?
//@param:_channel ç®¡è„šé€šé“å?
float RCS_Get_Voltage(ADC_TypeDef *_ADCx, uint8_t ADC_Channel_x);

#endif
