/**
 *@filename rcs_exti.c
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
#include "rcs_exti.h"

/* 静态全局变量定义 ---------------------------------------------------------------------------------*/

static rcs_exti_callback_ptr g_extiCallback[RCS_EXTI_MAX_CALLBACK_COUNT];
static GPIO_TypeDef *g_extiGpioPort[RCS_EXTI_MAX_CALLBACK_COUNT];
static uint32_t g_extiGpioPin[RCS_EXTI_MAX_CALLBACK_COUNT];
static uint16_t g_extiCount = 0;

/* 导出函数定义 -------------------------------------------------------------------------------------*/

/**
 *@brief 向某个管脚注册中断回调函数
 *@param  GPIOx      哪一个GPIO
 *@param  GPIO_PIN_x 哪一个Pin，不允许或运算
 *@param  cb         回调函数的函数指针
 *@retval rcs_err_t 详见rcs_types.h
 *
 *@note    本函数基于HAL库的回调函数，提前帮大家把“同一中断不同Pin”的情况区分开了，因此请不要再使用HAL库的回调函数
 *@warning 本函数不负责初始化EXTI外设，请自行在CubeMX中配置。
 *@warning EXTI不区分GPIO的Port。比如PA1和PB1不能同时开启外部中断，只能有一者存在。
**/
rcs_err_t rcs_exti_register_callback(GPIO_TypeDef *GPIOx,uint32_t GPIO_PIN_x,rcs_exti_callback_ptr cb)
{
	if (g_extiCount == RCS_EXTI_MAX_CALLBACK_COUNT) {
		return RCS_ERR_OVERFLOW;
	}
	if (cb == NULL) {
		return RCS_ERR_INVALID_PARAM; // 空回调函数
	}
	for(uint16_t i=0;i<g_extiCount;i++) {
		if (g_extiCallback[i] == cb)
			return RCS_ERR_MULTIPLE_USED;// 重复注册同一个回调函数
		if (g_extiGpioPin[i] == GPIO_PIN_x)
			return RCS_ERR_MULTIPLE_USED;// 该Pin已经被其他人使用了 
	}
	
	g_extiCallback[g_extiCount] = cb;
	g_extiGpioPort[g_extiCount] = GPIOx;
	g_extiGpioPin[g_extiCount] = GPIO_PIN_x;
	g_extiCount++;
	return RCS_OK;
}

/* 回调函数定义 -------------------------------------------------------------------------------------*/

/**
 *@brief HAL库EXTI弱函数HAL_GPIO_EXTI_Callback的代理函数
 *
 *@note 如果需要移植网上的代码，而代码中使用了HAL_GPIO_EXTI_Callback，可以把名字改成我们的
**/
__WEAK void RCS_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	__NOP();
}

/**
 *@brief STM32F4/F1系列HAL库的EXTI回调函数
 *@warning 请勿调用、修改、重新定义本函数
 *         想要使用EXTI，请调用rcs_exti_register_callback注册自己的回调函数，用起来更方便
**/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	RCS_GPIO_EXTI_Callback(GPIO_Pin);
	for (uint16_t i = 0; i < g_extiCount; i++)
		if (GPIO_Pin == g_extiGpioPin[i])
			g_extiCallback[i](g_extiGpioPort[i],GPIO_Pin);
}
