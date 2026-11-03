/* ==============头文件=======================================*/
#include "R1_UpCtrl_Motion.h"

/* ==============静态函数声明==================================*/
static void R1_Usart_Callback(void);

/* ==============私有全局变量==================================*/
RCS_PIN_USART R1_USART;
RCS_Easy_Ptl  R1_USART_Protocol;

volatile uint8_t rcv_state;
volatile uint8_t rcv_action[7];
volatile uint8_t tmp[7];
volatile uint8_t buffer[100];
volatile uint8_t  buf_tail;

/***********************************************************
 *              @addtogroup:串口交互
************************************************************/
void R1_Usart_Init(RCS_PIN_USART USARTx_MAP)
{
	//硬件层初始化
	R1_USART=USARTx_MAP;
	RCS_USART_Config(USARTx_MAP.USARTx,USARTx_MAP.GPIOx,USARTx_MAP.GPIO_Pin_Tx,USARTx_MAP.GPIO_Pin_Rx,R1_Usart_Callback,115200,R1_CALLBACK_PRI);
	//协议层初始化
	R1_USART_Protocol.Start_Byte=0xFF;
	R1_USART_Protocol.End_Byte=0xFE;
	R1_USART_Protocol.Len=4;
}

static void R1_Usart_Callback(void)
{
	uint16_t rcv_char;
	if(USART_GetITStatus(R1_USART.USARTx, USART_IT_RXNE) == SET)
	{
		rcv_char=(u8)USART_ReceiveData(R1_USART.USARTx);
		Protocol_Rcv_Easy(rcv_char,tmp,&rcv_state,rcv_action,&R1_USART_Protocol);
		USART_ClearITPendingBit(R1_USART.USARTx,USART_IT_RXNE);
	}
}

/***********************************************************
 *              @addtogroup:测试连续动作
************************************************************/
void R1_BLE2CAN_Server(void)
{
	//R1_Upctrl(rcv_action[0],rcv_action[1],rcv_action[2],rcv_action[3]);
}