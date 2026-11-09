/**
 *@filename rcs_gpio.c
 *@brief RCS对GPIO外设操作进行的二次封装，用于快速将GPIO应用到各种场景
 *@author 原始接口设计者：陈志伟@2020-07-31
 *--------------------------------------------------------------------------------------------------
 *@version v2.0
 *@author CYK-Dot@2025-05-19
 *@changelog 在保存原有代码行为不变的前提下，将代码移植到HAL库，并添加错误检查，让排查问题变得简单
 *@note   受限于时间，迁移到HAL库后只保留了原始接口，并没有考虑拓展
 *        因此，如果哪一年有更多空闲时间，请尽快考虑学习HAL库并重构代码
 *--------------------------------------------------------------------------------------------------
**/
#include "rcs_gpio.h"


/**
 *@brief 将GPIO输出到高电平
**/
void rcs_gpio_set(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
}

/**
 *@brief 将GPIO输出到低电平
**/
void rcs_gpio_reset(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}

/**
 *@brief 设置GPIO的输出电平
**/
void rcs_gpio_write(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin,bool state)
{
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin,(GPIO_PinState)state);
}

/**
 *@brief 翻转GPIO的电平
**/
void rcs_gpio_toggle(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    HAL_GPIO_TogglePin(GPIOx, GPIO_Pin);
}

/**
 *@brief 读取GPIO的电平
**/
uint8_t rcs_gpio_read(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    return HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
}

/**
 *@brief 将GPIO配置为输出模式
 *@warning 迁移到CubeMX后，不建议使用代码初始化外设。
 *         本函数仅作为保留接口，用于学习HAL库代码，非必要不要使用。
**/
void rcs_gpio_output_init_legacy(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
 
	/* GPIO Ports Clock Enable */
	if (GPIOx == GPIOA) 
		__HAL_RCC_GPIOA_CLK_ENABLE();
	else if (GPIOx == GPIOB) 
		__HAL_RCC_GPIOB_CLK_ENABLE();
	else if (GPIOx == GPIOC) 
		__HAL_RCC_GPIOC_CLK_ENABLE();
	else if (GPIOx == GPIOD) 
		__HAL_RCC_GPIOD_CLK_ENABLE();
	else if (GPIOx == GPIOE) 
		__HAL_RCC_GPIOE_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : PtPin */
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}
