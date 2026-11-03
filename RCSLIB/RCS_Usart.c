//@filename:RCS_usart.c
//@author:  Morris Zhang
//@date:    18-August-2012
//@brief:   USART串口通信  功能函数封装类库


#include "RCS_Usart.h"


static void USART_GPIO_Pin_Config(USART_TypeDef *_USARTx, GPIO_TypeDef *_GPIOx,
                                  uint32_t _GPIO_PinX_T, uint32_t _GPIO_PinX_R);

static void USART_Config(USART_TypeDef *_USARTx, uint32_t _baudRate);

static void USART_Int_Config(USART_TypeDef *_USARTx, FNCT_VOID _intFuc, uint8_t _pri);

//@name: USART_Config
//@brief: 配置USART的串口通信功能
//@param: _USARTx          需要绑定的USART串口
//@param: _GPIOx           需要绑定的GPIO组
//@param: _GPIO_PinX_T     需要绑定的串口发送管脚
//@param:_GPIO_PinX_R      需要绑定的串口接收管脚
//@param: _intFuc          接收中断函数的指针
//@param:_baudRate         传送的波特率
//@param: _pri             优先级
//@note:清除中断标志位
//USART_ClearITPendingBit(USARTx, USART_IT_RXNE);
void RCS_USART_Config(USART_TypeDef *_USARTx, GPIO_TypeDef *_GPIOx, uint32_t _GPIO_PinX_T,
                      uint32_t _GPIO_PinX_R, FNCT_VOID _intFuc, uint32_t _baudRate, uint8_t _pri)
{

	USART_GPIO_Pin_Config(_USARTx, _GPIOx, _GPIO_PinX_T, _GPIO_PinX_R);

	USART_Config(_USARTx, _baudRate);

	USART_Int_Config(_USARTx, _intFuc, _pri);
}
//@name: RCS_USART_With_NoIsr_Config
//@brief: 配置USART的串口通信功能,仅发送
//@param: _USARTx          需要绑定的USART串口
//@param: _GPIOx           需要绑定的GPIO组
//@param: _GPIO_PinX_T     需要绑定的串口发送管脚
//@param:_GPIO_PinX_R      需要绑定的串口接收管脚
//@param:_baudRate         传送的波特率
void RCS_USART_With_NoIsr_Config(USART_TypeDef *_USARTx, GPIO_TypeDef *_GPIOx, uint32_t _GPIO_PinX_T,uint32_t _GPIO_PinX_R, uint32_t _baudRate)
{
	USART_GPIO_Pin_Config(_USARTx, _GPIOx, _GPIO_PinX_T, _GPIO_PinX_R);
	USART_Config(_USARTx, _baudRate);
}

//@name: USART_GPIO_Pin_Config
//@brief: USART串口初始化的GPIO及其引脚配置
//@param: _USARTx          需要绑定的USART串口
//@param: _GPIOx           需要绑定的GPIO组
//@param: _GPIO_PinX_T     需要绑定的串口发送管脚
//@param:_GPIO_PinX_R      需要绑定的串口接收管脚

static void USART_GPIO_Pin_Config(USART_TypeDef *_USARTx, GPIO_TypeDef *_GPIOx,
                                  uint32_t _GPIO_PinX_T, uint32_t _GPIO_PinX_R)
{
	// 定义配置初始化变量
	GPIO_InitTypeDef GPIO_Struct;

	// 开启时钟
	if (_USARTx == USART1 || _USARTx == USART6)
	{

		RCC_APB2PeriphClockCmd(GetRCS_RCC_APB2Periph_USART(_USARTx), ENABLE);
	}
	else
	{
		RCC_APB1PeriphClockCmd(GetRCS_RCC_APB1Periph_USART(_USARTx), ENABLE);
	}
	RCC_AHB1PeriphClockCmd(GetRCS_RCC_AHB1Periph_GPIO(_GPIOx), ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	// 将管脚和USART模块绑定
	GPIO_PinAFConfig(_GPIOx, GetRCS_GPIO_PinSource(_GPIO_PinX_T), GetRCS_GPIO_AF(_USARTx));
	GPIO_PinAFConfig(_GPIOx, GetRCS_GPIO_PinSource(_GPIO_PinX_R), GetRCS_GPIO_AF(_USARTx));

	// 配置发送引脚
	GPIO_StructInit(&GPIO_Struct);
	GPIO_Struct.GPIO_Pin = _GPIO_PinX_T;
	GPIO_Struct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Struct.GPIO_Speed = GPIO_Speed_50MHz;
#ifdef  OD_OUTPUT
	GPIO_Struct.GPIO_OType = GPIO_OType_OD;
	GPIO_Struct.GPIO_PuPd = GPIO_PuPd_NOPULL;
#else
	GPIO_Struct.GPIO_OType = GPIO_OType_PP;
	GPIO_Struct.GPIO_PuPd = GPIO_PuPd_UP;
#endif
	GPIO_Init(_GPIOx, &GPIO_Struct);

	// 配置接收引脚
	GPIO_Struct.GPIO_Pin = _GPIO_PinX_R;
	GPIO_Struct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Struct.GPIO_Speed = GPIO_Speed_50MHz;
#ifdef  OD_OUTPUT
	GPIO_Struct.GPIO_OType = GPIO_OType_OD;
	GPIO_Struct.GPIO_PuPd = GPIO_PuPd_NOPULL;
#else
	GPIO_Struct.GPIO_OType = GPIO_OType_PP;
	GPIO_Struct.GPIO_PuPd = GPIO_PuPd_UP;
#endif

	GPIO_Init(_GPIOx, &GPIO_Struct);
}

//@name: USART_Config
//@brief: 串口初始化配置
//@param:_USARTx       需要绑定的USART串口
//@param:_baudRate     传送的波特率
static void USART_Config(USART_TypeDef *_USARTx, uint32_t _baudRate)
{
	// 定义配置初始化变量
	USART_InitTypeDef USART_InitStructure;

	// 配置USART模块
	USART_StructInit(&USART_InitStructure);
	USART_InitStructure.USART_BaudRate = _baudRate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(_USARTx, &USART_InitStructure);
	USART_ClearFlag(USART1, USART_FLAG_TC);
	USART_Cmd(_USARTx, ENABLE);

	//允许接收终端
	USART_ITConfig(_USARTx, USART_IT_RXNE, ENABLE);
}

//@name: USART_Int_Config
//@brief:USART接收中断初始化配置
//@param: _USARTx       需要绑定的USART串口
//@param: _intFuc       接收中断函数的指针
//@param: _pri          优先级
static void USART_Int_Config(USART_TypeDef *_USARTx, FNCT_VOID _intFuc, uint8_t _pri)
{
	NVIC_InitTypeDef nvic_initStruct;
	if (_intFuc != NULL)
	{
		// 绑定中断函数
		BSP_IntVectSet(GetBSP_INT_ID_USART(_USARTx), _intFuc);

		// 配置NVIC模块
		nvic_initStruct.NVIC_IRQChannel = GetRCS_USART_IRQn(_USARTx);
		nvic_initStruct.NVIC_IRQChannelPreemptionPriority = (_pri >> 4 ) & 0x0f;
		nvic_initStruct.NVIC_IRQChannelSubPriority = _pri & 0x0f;
		nvic_initStruct.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvic_initStruct);
	}
}