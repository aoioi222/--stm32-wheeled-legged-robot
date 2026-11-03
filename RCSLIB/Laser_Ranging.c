/*@filename: Laser_Ranging.c
 *@author     胡兴国       
 *@brief:     黑色外壳版摩天激光测距模块，Usart通讯
 *@brief:     测距范围0.05m~20m,帧率20HZ，精度正负1mm
 *@date: 2023-8-30
*/

#include "Laser_Ranging.h"
/*---------------全局变量-------------*/
static float data = 0;
static float laser_data = 0;
static uint8_t check = 0;   //校验
RCS_PIN_USART Laser_USART_MAP;
/*---------------静态函数-------------*/
static void LASER_Interrupt(void);
/*------------------------------------*/


/**
	@name: Laser_Ranging_Init
	@brief: 激光测距初始化
**/
void Laser_Ranging_Init(RCS_PIN_USART USARTx_MAP)
{
		Laser_USART_MAP=USARTx_MAP;
    RCS_USART_Config(USARTx_MAP.USARTx,USARTx_MAP.GPIOx,USARTx_MAP.GPIO_Pin_Tx,USARTx_MAP.GPIO_Pin_Rx,LASER_Interrupt,LASER_BAUD,LASER_PRI);
	  delay_ms(100);
}

/**
	@name: LASER_Interrupt
	@brief: 中断接受函数(使用时映射成对应USART的中断函数)
**/
static void LASER_Interrupt(void)
{
	uint8_t receive_char;
	static uint8_t step = 0;
	
	if(USART_GetITStatus(Laser_USART_MAP.USARTx, USART_IT_RXNE) == SET)
	{
		receive_char = RCS_USART_Accept_Char(Laser_USART_MAP.USARTx);
		
		switch(step)
		{
			case 0: if(receive_char==0xB4)
							{
								data = 0;
								check = 0;
								check ^= receive_char;
								step ++;
							}
							else
							{
								step = 0;
							}
							break;
							
			case 1: if(receive_char==0x69)
							{
								check ^= receive_char;
								step ++;
							}
							else
							{
								step = 0;
							}
							break;				
							
			case 2: if(receive_char==0x04)
							{
								check ^= receive_char;
								step ++;
							}
							else
							{
								step = 0;
							}
							break;			

			case 3: data += (receive_char<<24)&0xFF000000;
							check ^= receive_char;
							step ++;
							break; 				
							
			case 4: data += (receive_char<<16)&0xFF0000;
							check ^= receive_char;
							step ++;
							break; 		
							
			case 5: data += (receive_char<<8)&0xFF00;
							check ^= receive_char;
							step ++;
							break; 		

      case 6: data += receive_char;
							check ^= receive_char;
							step ++;
							break; 		

      case 7: if(receive_char == check)  //校验
			        {
								laser_data = data;
			        }
				      step = 0;
							break; 					
							
			default:step = 0;
							break;				
		}
	}
	USART_ClearITPendingBit(Laser_USART_MAP.USARTx, USART_IT_RXNE);
}

/**
	@name: Get_Laser_Data
	@brief:获取激光测距值
**/
float Get_Laser_Data(void)
{
	return laser_data/10.0f;
}

/**
	@name: Get_Ranging_Y
	@brief:激光测距Y坐标
**/
float Get_Ranging_Y(void)
{
	return (laser_data/10.0f - 2354.0f) * cos(Get_GPS_Z() * DEG2RAD);
}	