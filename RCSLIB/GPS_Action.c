/*@filename: GPS_Action.c
 *@author     胡兴国       
 *@brief:     东大全场定位,帧率200Hz
 *@date: 2023-7-28
*/

#include "GPS_Action.h"
/*---------------全局变量-------------*/
static float pos_x = 0;
static float pos_y = 0;
static float zangle = 0;
static float xangle = 0;
static float yangle = 0;
static float w_z = 0;
RCS_PIN_USART ACTION_GPS_USART_MAP;
/*---------------静态函数-------------*/
static void GPS_Action_Interrupt(void);
/*------------------------------------*/


/**
	@name: GPS_Action_Init
	@brief: 东大全场定位初始化
**/
void GPS_Action_Init(RCS_PIN_USART USARTx_MAP)
{
	ACTION_GPS_USART_MAP=USARTx_MAP;
  RCS_USART_Config(USARTx_MAP.USARTx,USARTx_MAP.GPIOx,USARTx_MAP.GPIO_Pin_Tx,USARTx_MAP.GPIO_Pin_Rx,GPS_Action_Interrupt,115200,0x12);
	delay_ms(100);
}

/**
	@name: GPS_Action_Interrupt
	@brief: 中断接受函数(使用时映射成对应USART的中断函数)
**/
static void GPS_Action_Interrupt(void)
{
	static uint8_t ch;
	static union
	{
		uint8_t date[24];
		float ActVal[6];
	}posture;
	static uint8_t count = 0;
	static uint8_t i = 0;

	if(USART_GetITStatus(ACTION_GPS_USART_MAP.USARTx, USART_IT_RXNE) == SET)
	{
		ch = USART_ReceiveData(ACTION_GPS_USART_MAP.USARTx);	
		switch(count)
	{
		case 0: if(ch==0x0d)
		        {
							count++;
						}
						else
						{
							count=0;
						}
						break;
						
		case 1: if(ch==0x0a)
						{
							i=0;
							count++;
						}
						else if(ch==0x0d);
						else
						{
							count=0;
						}
						break;				
						
	  case 2: posture.date[i]=ch;
						i++;
						if(i>=24)
						{
							i=0;
							count++;
						}
						break;  

		case 3: if(ch==0x0a)
		        {
							count++;
						}
					  else
						{
							count=0;
						}
					  break; 				
						
		case 4: if(ch==0x0d)
						{
							zangle=posture.ActVal[0];
							xangle=posture.ActVal[1];
							yangle=posture.ActVal[2];
							pos_x=posture.ActVal[3];
							pos_y=posture.ActVal[4];
							w_z=posture.ActVal[5];
						}
						count=0;
						break;  
						
		default:count=0;
		        break;				
	}
//		USART_ClearITPendingBit(ACTION_GPS_USART_MAP.USARTx, USART_FLAG_RXNE);
	}
}
/**
	@name:  Get_GPS_Action_X
	@brief: 获取X坐标
	@return: X坐标
**/
float Get_GPS_Action_X(void)
{
	return pos_x;
}

/**
	@name:  Get_GPS_Action_Y
	@brief: 获取Y坐标
	@return: Y坐标
**/
float Get_GPS_Action_Y(void)
{
  return pos_y;
}

/**
	@name:  Get_GPS_Action_Z
	@brief: 获取偏航角
	@return: 偏航角
**/
float Get_GPS_Action_Z(void)
{
	return zangle;
}