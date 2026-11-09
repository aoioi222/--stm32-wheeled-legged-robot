/**
 *@filename rcs_exti.h
 *@brief RCS对EXTI外设操作进行的二次封装，用于读取光电门等需要一瞬间反应的GPIO外围设备
 *@author 原始接口设计者：柯国霖@2012-12-08
 *--------------------------------------------------------------------------------------------------
 *@version v2.0
 *@author CYK-Dot@2025-05-20
 *@changelog 在保存原有代码行为不变的前提下，将代码移植到HAL库，并添加错误检查，让排查问题变得简单
 *@note   受限于时间，迁移到HAL库后只保留了原始接口，并没有考虑拓展
 *        因此，如果哪一年有更多空闲时间，请尽快考虑学习HAL库并重构代码
 *--------------------------------------------------------------------------------------------------
**/
#pragma once

/* 头文件依赖 -------------------------------------------------------------------------------------*/

#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "rcs_private_config.h"
#include "rcs_types.h"

/* 配置宏定义 ---------------------------------------------------------------------------------------*/

#define RCS_EXTI_MAX_CALLBACK_COUNT 16

/* 导出类型定义 -------------------------------------------------------------------------------------*/

typedef void (*rcs_exti_callback_ptr) (GPIO_TypeDef *GPIOx, uint32_t pin);

/* 导出函数声明 -------------------------------------------------------------------------------------*/

#ifdef __cplusplus
	extern "C" {
#endif

rcs_err_t rcs_exti_register_callback(GPIO_TypeDef *GPIOx,uint32_t GPIO_PIN_x,rcs_exti_callback_ptr cb);

__WEAK void RCS_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#ifdef __cplusplus
	}
#endif