//@filename: ADC.c
//@date: 2019-07-22
//@author: 闫锐
//@brief: 模拟量转换数字量
#include "RCS_ADC.h"

//@name: RCS_ADC_Init
//@brief: 初始化ADC
//@param:ADC_TypeDef *_ADCx ADC号
//@param:GPIO_TypeDef *_GPIOx GPIO组
//@param:uint32_t _pin 管脚
//@note:测量电压不得超过3.3V
void RCS_ADC_Init(ADC_TypeDef *_ADCx, GPIO_TypeDef *_GPIOx, uint32_t _pin)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef ADC_InitStructure;

	RCC_AHB1PeriphClockCmd(GetRCS_RCC_AHB1Periph_GPIO(_GPIOx), ENABLE); //使能GPIO时钟
	RCC_APB2PeriphClockCmd(GetRCS_RCC_APB2Periph_ADC(_ADCx), ENABLE); //使能ADC时钟

	/*端口设置为模拟输入*/
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN; /*模拟输入*/
	GPIO_InitStructure.GPIO_Pin = _pin;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; /*­不带上下拉*/
	GPIO_Init(_GPIOx, &GPIO_InitStructure);/*初始化*/

	/*设置通用控制寄存器 */

	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;/*DMA失能*/
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent; /*独立模式*/
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;/*设置ADC的频率为APB2/4 即84/4=21M*/
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;/*采样间隔时间*/
	ADC_CommonInit(&ADC_CommonInitStructure);/*初始化*/

	/*ADC的初始化 */

	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;/*12位模式*/
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;/*非扫描模式*/
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;/*关闭连续转换*/
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;/*禁止触发检测 使用软件触发*/
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;/*右对齐*/
	ADC_InitStructure.ADC_NbrOfConversion = 1;/*只使用1个通道采样*/
	ADC_Init(_ADCx, &ADC_InitStructure); /*初始化*/
	ADC_Cmd(_ADCx, ENABLE); /*开启ADC*/
}

//@name: RCS_Get_ADC
//@brief: 获取ADC的值
//@param:ADC_TypeDef *_ADCx ADC号
//@param:_channel 管脚通道号
uint16_t RCS_Get_ADC(ADC_TypeDef *_ADCx, uint8_t ADC_Channel_x)
{
	ADC_RegularChannelConfig(_ADCx, ADC_Channel_x, 1, ADC_SampleTime_480Cycles); /*设置规则通道3 一个序列 采样时间 */
	ADC_SoftwareStartConv(_ADCx);/*启动软件转换*/
	while (!ADC_GetFlagStatus(_ADCx, ADC_FLAG_EOC)); /*等待转换结束*/
	return ADC_GetConversionValue(_ADCx);/*读取转换结果*/
}

//@name: RCS_Get_Voltage
//@brief: 获取ADC的电压值
//@param:ADC_TypeDef *_ADCx ADC号
//@param:_channel 管脚通道号
float RCS_Get_Voltage(ADC_TypeDef *_ADCx, uint8_t ADC_Channel_x)
{
	return (float)RCS_Get_ADC(_ADCx, ADC_Channel_x) * VOLTAGE_REFERANCE / MAX_RANGE;
}
