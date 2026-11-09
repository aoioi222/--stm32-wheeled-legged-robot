/**
 *@filename ADC.h
 *@brief RCS对ADC外设操作进行的二次封装，用于读取DT35激光或是其他需要模拟量的地方
 *@date  2025-05-19
 *@author CYK-Dot
 *
 *@author 原始接口设计者：闫锐@2019-07-22
 *--------------------------------------------------------------------------------------------------
 *@version v1.0
 *@note   受限于时间，迁移到HAL库后只保留了原始接口，并没有考虑拓展，例如闫师兄读取ADC的方式其实不太合适
 *        因此，如果哪一年有更多空闲时间，请尽快考虑学习HAL库并重构代码，添加更加高效的ADC读取方式。
**/

#pragma once


#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "rcs_private_config.h"
#include "rcs_types.h"


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

#ifdef __cplusplus
	extern "C" {
#endif

int rcs_get_adc(ADC_HandleTypeDef *phadc, uint8_t ADC_Channel_x,uint16_t timeout);

#ifdef __cplusplus
	}
#endif