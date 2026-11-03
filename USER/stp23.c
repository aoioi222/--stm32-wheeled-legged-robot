#include "stp23.h"
#define HEADER 0x54
#define VERLEN  0x2C
uint16_t receive_cnt3;//帧率计数
uint16_t distance3;   //实际返回距离
uint16_t receive_cnt4;
uint16_t distance4;
void STP23_IRQHandler3(void);//中断设置
void STP23_IRQHandler4(void);
LiDARFrameTypeDef Pack3_Data;
LiDARFrameTypeDef Pack4_Data;
RCS_PIN_USART STP23_USART_MAP3;
RCS_PIN_USART STP23_USART_MAP4;

void STP23_Init(RCS_PIN_USART USARTx_MAP , RCS_PIN_USART USARTy_MAP)//扩展接口复制即可
{
	STP23_USART_MAP3=USARTx_MAP;
	STP23_USART_MAP4=USARTy_MAP;
  RCS_USART_Config(USARTx_MAP.USARTx, USARTx_MAP.GPIOx,USARTx_MAP.GPIO_Pin_Tx,USARTx_MAP.GPIO_Pin_Rx,STP23_IRQHandler3 ,STP23_BAUD, STP23_PRI);
	RCS_USART_Config(USARTy_MAP.USARTx, USARTy_MAP.GPIOx,USARTy_MAP.GPIO_Pin_Tx,USARTy_MAP.GPIO_Pin_Rx,STP23_IRQHandler4 ,STP23_BAUD, STP23_PRI);
	delay_ms(100);
}




void data3_process(void)/*数据处理函数，完成一帧之后可进行数据处理*/
{
	static u8 cnt = 0;
	u8 i;
	static u16 count = 0;
	static u32 sum = 0;
	for(i=0;i<12;i++)//12个点取平均
	{
		if(Pack3_Data.point[i].distance != 0)//去除0的点
		{
			count++;
			sum += Pack3_Data.point[i].distance;
		}
	}
	if(++cnt == 20)//20个数据帧计算一次距离
	{
		distance3 = sum/count;
		sum = 0;
		count = 0;
		cnt = 0;
	}
}

void data4_process(void)/*数据处理函数，完成一帧之后可进行数据处理*/
{
	static u8 cnt = 0;
	u8 i;
	static u16 count = 0;
	static u32 sum = 0;
	for(i=0;i<12;i++)//12个点取平均
	{
		if(Pack4_Data.point[i].distance != 0)//去除0的点
		{
			count++;
			sum += Pack4_Data.point[i].distance;
		}
	}
	if(++cnt == 20)//20个数据帧计算一次距离
	{
		distance4 = sum/count;
		sum = 0;
		count = 0;
		cnt = 0;
	}
}

void STP23_IRQHandler3(void)
{	
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		static uint8_t state = 0;//状态位	
		static uint8_t crc = 0;//校验和
		static uint8_t cnt = 0;//用于一帧12个点的计数
		uint8_t temp_data;
		
		temp_data = USART_ReceiveData(USART3);
		if (state > 5)
		{
			if(state < 42)
			{
				if(state%3 == 0)//一帧数据中的序号为6,9.....39的数据，距离值低8位
				{
					
					Pack3_Data.point[cnt].distance = (uint16_t)temp_data;
					state++;
					crc = CrcTable[(crc^temp_data) & 0xff];
				}
				else if(state%3 == 1)//一帧数据中的序号为7,10.....40的数据，距离值高8位
				{
					Pack3_Data.point[cnt].distance = ((uint16_t)temp_data<<8)+Pack3_Data.point[cnt].distance;
					state++;
					crc = CrcTable[(crc^temp_data) & 0xff];
				}
				else//一帧数据中的序号为8,11.....41的数据，置信度
				{
					Pack3_Data.point[cnt].intensity = temp_data;
					cnt++;	
					state++;
					crc = CrcTable[(crc^temp_data) & 0xff];
				}
			}
			else 
			{
				switch(state)
				{
					case 42:
						Pack3_Data.end_angle = (uint16_t)temp_data;//结束角度低8位
						state++;
						crc = CrcTable[(crc^temp_data) & 0xff];
						break;
					case 43:
						Pack3_Data.end_angle = ((uint16_t)temp_data<<8)+Pack3_Data.end_angle;//结束角度高8位
						state++;
						crc = CrcTable[(crc^temp_data) & 0xff];
						break;
					case 44:
						Pack3_Data.timestamp = (uint16_t)temp_data;//时间戳低8位
						state++;
						crc = CrcTable[(crc^temp_data) & 0xff];
						break;
					case 45:
						Pack3_Data.timestamp = ((uint16_t)temp_data<<8)+Pack3_Data.timestamp;//时间戳高8位
						state++;
						crc = CrcTable[(crc^temp_data) & 0xff];
						break;
					case 46:
						Pack3_Data.crc8 = temp_data;//雷达传来的校验和
						if(Pack3_Data.crc8 == crc)//校验正确
						{
							data3_process();//接收到一帧且校验正确可以进行数据处理
							receive_cnt3++;//输出接收到正确数据的次数
						}
						else
						{
						//校验不正确
						}
						crc = 0;
						state = 0;
						cnt = 0;//复位
					default: break;
				}
			}
		}
		else 
		{
			switch(state)
			{
				case 0:
					if(temp_data == HEADER)//头固定
					{
						Pack3_Data.header = temp_data;
						state++;
						crc = CrcTable[(crc^temp_data) & 0xff];//开始进行校验
					} else state = 0,crc = 0;
					break;
				case 1:
					if(temp_data == VERLEN)//测量的点数，目前固定
					{
						Pack3_Data.ver_len = temp_data;
						state++;
						crc = CrcTable[(crc^temp_data) & 0xff];
					} else state = 0,crc = 0;
					break;
				case 2:
					Pack3_Data.temperature = (uint16_t)temp_data;//温度低8位，一共16位ADC，0--4096，无量纲
					state++;
					crc = CrcTable[(crc^temp_data) & 0xff];
					break;
				case 3:
					Pack3_Data.temperature = ((uint16_t)temp_data<<8)+Pack3_Data.temperature;//温度高8位
					state++;
					crc = CrcTable[(crc^temp_data) & 0xff];
					break;
				case 4:
					Pack3_Data.start_angle = (uint16_t)temp_data;//开始角度低8位，放大了100倍
					state++;
					crc = CrcTable[(crc^temp_data) & 0xff];
					break;
				case 5:
					Pack3_Data.start_angle = ((uint16_t)temp_data<<8)+Pack3_Data.start_angle;
					state++;
					crc = CrcTable[(crc^temp_data) & 0xff];
					break;
				default: break;
			}
		}
	}
}





void STP23_IRQHandler4(void)
{	
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
	{
		static uint8_t state = 0;//状态位	
		static uint8_t crc = 0;//校验和
		static uint8_t cnt = 0;//用于一帧12个点的计数
		uint8_t temp_data;
		
		temp_data = USART_ReceiveData(UART4);
		if (state > 5)
		{
			if(state < 42)
			{
				if(state%3 == 0)//一帧数据中的序号为6,9.....39的数据，距离值低8位
				{
					
					Pack4_Data.point[cnt].distance = (uint16_t)temp_data;
					state++;
					crc = CrcTable[(crc^temp_data) & 0xff];
				}
				else if(state%3 == 1)//一帧数据中的序号为7,10.....40的数据，距离值高8位
				{
					Pack4_Data.point[cnt].distance = ((uint16_t)temp_data<<8)+Pack4_Data.point[cnt].distance;
					state++;
					crc = CrcTable[(crc^temp_data) & 0xff];
				}
				else//一帧数据中的序号为8,11.....41的数据，置信度
				{
					Pack4_Data.point[cnt].intensity = temp_data;
					cnt++;	
					state++;
					crc = CrcTable[(crc^temp_data) & 0xff];
				}
			}
			else 
			{
				switch(state)
				{
					case 42:
						Pack4_Data.end_angle = (uint16_t)temp_data;//结束角度低8位
						state++;
						crc = CrcTable[(crc^temp_data) & 0xff];
						break;
					case 43:
						Pack4_Data.end_angle = ((uint16_t)temp_data<<8)+Pack4_Data.end_angle;//结束角度高8位
						state++;
						crc = CrcTable[(crc^temp_data) & 0xff];
						break;
					case 44:
						Pack4_Data.timestamp = (uint16_t)temp_data;//时间戳低8位
						state++;
						crc = CrcTable[(crc^temp_data) & 0xff];
						break;
					case 45:
						Pack4_Data.timestamp = ((uint16_t)temp_data<<8)+Pack4_Data.timestamp;//时间戳高8位
						state++;
						crc = CrcTable[(crc^temp_data) & 0xff];
						break;
					case 46:
						Pack4_Data.crc8 = temp_data;//雷达传来的校验和
						if(Pack4_Data.crc8 == crc)//校验正确
						{
							data4_process();//接收到一帧且校验正确可以进行数据处理
							receive_cnt4++;//输出接收到正确数据的次数
						}
						else
						{
						//校验不正确
						}
						crc = 0;
						state = 0;
						cnt = 0;//复位
					default: break;
				}
			}
		}
		else 
		{
			switch(state)
			{
				case 0:
					if(temp_data == HEADER)//头固定
					{
						Pack4_Data.header = temp_data;
						state++;
						crc = CrcTable[(crc^temp_data) & 0xff];//开始进行校验
					} else state = 0,crc = 0;
					break;
				case 1:
					if(temp_data == VERLEN)//测量的点数，目前固定
					{
						Pack4_Data.ver_len = temp_data;
						state++;
						crc = CrcTable[(crc^temp_data) & 0xff];
					} else state = 0,crc = 0;
					break;
				case 2:
					Pack4_Data.temperature = (uint16_t)temp_data;//温度低8位，一共16位ADC，0--4096，无量纲
					state++;
					crc = CrcTable[(crc^temp_data) & 0xff];
					break;
				case 3:
					Pack4_Data.temperature = ((uint16_t)temp_data<<8)+Pack4_Data.temperature;//温度高8位
					state++;
					crc = CrcTable[(crc^temp_data) & 0xff];
					break;
				case 4:
					Pack4_Data.start_angle = (uint16_t)temp_data;//开始角度低8位，放大了100倍
					state++;
					crc = CrcTable[(crc^temp_data) & 0xff];
					break;
				case 5:
					Pack4_Data.start_angle = ((uint16_t)temp_data<<8)+Pack4_Data.start_angle;
					state++;
					crc = CrcTable[(crc^temp_data) & 0xff];
					break;
				default: break;
			}
		}
	}
}
