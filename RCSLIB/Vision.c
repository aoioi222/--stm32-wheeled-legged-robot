/**
 @name:Vision.c
 @brief:视觉串口的应用层驱动
**/

/*==============头文件====================================*/
#include "Vision.h"

/*==============静态函数声明==============================*/
void Vision_Callback(uint8_t *output_arr);
void Vision_IRQHandler(void);

/*==============全局变量声明==============================*/
RCS_PIN_USART receiver_USART_MAP;

RCS_Easy_Ptl2 easy_protocol = 
{
    .Start_Byte1 = 0x2c,
	  .Start_Byte2 = 0x12,
    .Len = 2,  // 数据长度（不包括起始和结束字节）
    .End_Byte = 0x5B,
    .Finish_Callback=Vision_Callback,  
};

uint8_t temp_arr[10];
uint8_t output_arr[5];
uint8_t Serial_RxData;
uint8_t output_flag=0;
uint8_t pointcheck = 0;
float target_arry[2];
uint8_t is_available;


/*==============导出函数定义==============================*/

void Vision_Init(RCS_PIN_USART USARTx_MAP)
{
	receiver_USART_MAP=USARTx_MAP;
  RCS_USART_Config(USARTx_MAP.USARTx, USARTx_MAP.GPIOx,USARTx_MAP.GPIO_Pin_Tx,USARTx_MAP.GPIO_Pin_Rx,Vision_IRQHandler ,9600, 1);
	delay_ms(100);
}

float Vision_Get_X(void)
{
	return target_arry[0];
}


float Vision_Get_Y(void)
{
	return target_arry[1];
}

uint8_t Vision_Is_Available(void)
{
	return is_available;
}

/*==============静态函数定义==============================*/
void Vision_IRQHandler(void)
{
	if (USART_GetITStatus(receiver_USART_MAP.USARTx, USART_IT_RXNE) == SET)	
	{
		pointcheck = 1;
		Serial_RxData=RCS_USART_Accept_Char(receiver_USART_MAP.USARTx);
		Receive_visiondata( Serial_RxData,temp_arr,
                        &output_flag,output_arr,
                        &easy_protocol);
		USART_ClearITPendingBit(receiver_USART_MAP.USARTx, USART_IT_RXNE);														
	}
}


void Vision_Callback(uint8_t *output_arr) 
{   
	  uint8_t temp1=output_arr[0],temp2=output_arr[1];
    float value1,value2;  
    value1=(float)temp1;
   	value2=(float)temp2;
	  target_arry[0]=value1;
	  target_arry[1]=value2; 
	if(target_arry[0]==0&&target_arry[1]==0)
		is_available = 0;//视觉需要返回视野中是否有目标物！否则机械臂会试图去追(0,0)
	else
		is_available = 1;
}  
int x_flag=220;
int y_flag=160;
int Vision_identify()
{
	if(Vision_Get_X()<=x_flag)	return 1;
	else if(Vision_Get_Y()<=y_flag)	return 2;
	else return 3;
}