/**
 *@filename rcs_adc.c
 *@brief RCS对ADC外设操作进行的二次封装，用于读取DT35激光或是其他需要模拟量的地方
 *@author 原始接口设计者：闫锐@2019-07-22
 *--------------------------------------------------------------------------------------------------
 *@version v2.0
 *@author CYK-Dot@2025-05-19
 *@changelog 在保存原有代码行为不变的前提下，将代码移植到HAL库，并添加错误检查，让排查问题变得简单
 *@note   受限于时间，迁移到HAL库后只保留了原始接口，并没有考虑拓展，例如闫师兄读取ADC的方式其实不太合适
 *        因此，如果哪一年有更多空闲时间，请尽快考虑学习HAL库并重构代码，添加更加高效的ADC读取方式。
 *--------------------------------------------------------------------------------------------------
**/
#include "rcs_adc.h"

/**
 *@brief 阻塞程序，获取ADC外设在某个通道上的数据
 *
 *@param [IN] phadc          ADC外设的句柄。CubeMX帮我们创建好了，比如ADC1，就是hadc1，填入&hadc1即可
 *@param [IN] ADC_Channel_x  ADC外设的哪一个通道，可以填入ADC_CHANNEL_0代表第0个通道
 *@param [IN] timeout        超过多久采集不到数据就退出，单位us。如果想一直等，填入HAL_MAX_DELAY
 *
 *@retval 如果获取成功，返回ADC采集到的数据，比如STM32F4是12位的ADC，那么就是0~2的12次方
 *        如果获取失败，返回负数错误码，错误码的含义详见rcs_types.h
 *
 *@note 怎么知道哪一个GPIO对应哪一个ADC通道？配置CubeMX的时候会指定的！
 *@warning 阻塞程序不是好事
**/
int rcs_get_adc(ADC_HandleTypeDef *phadc, uint8_t ADC_Channel_x,uint16_t timeout)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_Channel_x;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	
	HAL_StatusTypeDef err;
    err = HAL_ADC_ConfigChannel(phadc, &sConfig);
	if (err != HAL_OK) {
		return RCS_FAIL;
	}
    err = HAL_ADC_Start(phadc);
	if (err != HAL_OK) {
		return RCS_FAIL;
	}
    err = HAL_ADC_PollForConversion(phadc, HAL_MAX_DELAY);
	if (err != HAL_OK) {
		return RCS_FAIL;
	}
    return HAL_ADC_GetValue(phadc);
}

/**
 *@brief   初始化ADC外设
 *@warning 迁移到CubeMX后，不建议使用代码初始化外设。
 *         本函数仅作为保留特性，用于学习HAL库代码，不推荐使用。
**/
rcs_err_t rcs_adc_init_legacy(ADC_TypeDef *_ADCx, GPIO_TypeDef *_GPIOx, uint32_t _pin)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    ADC_ChannelConfTypeDef sConfig = {0};
    ADC_HandleTypeDef *phadc = NULL;

    if (_ADCx == ADC1) {
        __HAL_RCC_ADC1_CLK_ENABLE();
        static ADC_HandleTypeDef hadc1;
        hadc1.Instance = ADC1;
        phadc = &hadc1;
    } else if (_ADCx == ADC2) {
        __HAL_RCC_ADC2_CLK_ENABLE();
        static ADC_HandleTypeDef hadc2;
        hadc2.Instance = ADC2;
        phadc = &hadc2;
    } else if (_ADCx == ADC3) {
        __HAL_RCC_ADC3_CLK_ENABLE();
        static ADC_HandleTypeDef hadc3;
        hadc3.Instance = ADC3;
        phadc = &hadc3;
    }

    if (!phadc) return RCS_FAIL;

    /* GPIO Init */
    if (_GPIOx == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
    else if (_GPIOx == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
    else if (_GPIOx == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
    // 你可以复用你的 GetRCS_RCC_AHB1Periph_GPIO 替代上面写死部分

    GPIO_InitStruct.Pin = _pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(_GPIOx, &GPIO_InitStruct);

    /* ADC Init */
    phadc->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    phadc->Init.Resolution = ADC_RESOLUTION_12B;
    phadc->Init.ScanConvMode = DISABLE;
    phadc->Init.ContinuousConvMode = DISABLE;
    phadc->Init.DiscontinuousConvMode = DISABLE;
    phadc->Init.ExternalTrigConv = ADC_SOFTWARE_START;
    phadc->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    phadc->Init.DataAlign = ADC_DATAALIGN_RIGHT;
    phadc->Init.NbrOfConversion = 1;
    phadc->Init.DMAContinuousRequests = DISABLE;
    phadc->Init.EOCSelection = ADC_EOC_SINGLE_CONV;

    HAL_ADC_Init(phadc);
	return RCS_OK;
}

