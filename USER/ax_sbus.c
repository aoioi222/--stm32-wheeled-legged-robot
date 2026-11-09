#include "ax_sbus.h"

#define SBUS_RECV_MAX    25
#define SBUS_START       0x0F
#define SBUS_END         0x00

#define SBUS_BAUD 100000
#define SBUS_PRI  0x01
#define SBUS_SIGNAL_TIMEOUT_TICKS (OS_TICKS_PER_SEC / 10u)
#define SBUS_INVALID_FLAGS 0x0Cu

static volatile uint32_t sbus_frame_tick;
static volatile bool sbus_frame_valid;

uint8_t  uart_sbus_rx_ok = 0;      //接收成功标志
uint16_t  uart_sbus_rx_con=0;       //接收计数器
uint16_t  uart_sbus_rx_buf[40];     //接收缓冲，数据内容小于等于32Byte

uint16_t uart_sbus_data[16];				//接收处理后的数据（通道值）

void SBUS_IRQHandler(void);
uint8_t AX_SBUS_Handle_RxData(void);

RCS_PIN_USART SBUS__USART_MAP;


/**
  * @简  述  SBUS串口初始化
  * @参  数  RCS_PIN_USART USARTx_MAP
  * @返回值	 无
  */
void SBUS_Init(RCS_PIN_USART USARTx_MAP )//
{
	SBUS__USART_MAP=USARTx_MAP;
  	RCS_USART_Config(USARTx_MAP.USARTx, USARTx_MAP.GPIOx,USARTx_MAP.GPIO_Pin_Tx,USARTx_MAP.GPIO_Pin_Rx,SBUS_IRQHandler ,SBUS_BAUD, SBUS_PRI);
	delay_ms(100);
}



/**
  * @简  述  UART (SBUS) 串口中断服务函数
  * @参  数  无 
  * @返回值  无
  */
void SBUS_IRQHandler(void)
{
    uint8_t Res;
   //接收中断
	if(USART_GetITStatus(SBUS__USART_MAP.USARTx, USART_IT_RXNE) != RESET)  
	{		
		USART_ClearITPendingBit(SBUS__USART_MAP.USARTx, USART_IT_RXNE);	
		Res = USART_ReceiveData(SBUS__USART_MAP.USARTx);			
		if(uart_sbus_rx_con == 0)
		{
			//检测数据开始字节
			if(Res == SBUS_START)
			{
				uart_sbus_rx_buf[uart_sbus_rx_con] = Res;				
				uart_sbus_rx_con++; 
			}			
		}
		else
		{
			//检测数据结束字节
			if((uart_sbus_rx_con == (SBUS_RECV_MAX-1)) && (Res ==  SBUS_END))
			{
				//一帧数据接收完毕
				uart_sbus_rx_ok = 1;									
				//复位接收计数器
				uart_sbus_rx_con = 0;
				//解析数据
                if (uart_sbus_rx_buf[23] & SBUS_INVALID_FLAGS)
                {
                    sbus_frame_valid = false;
                    uart_sbus_rx_ok = 0;
                }
                else if (AX_SBUS_Handle_RxData())
                {
                    sbus_frame_tick = OSTimeGet();
                    sbus_frame_valid = true;
                }
			}
			else
			{
				uart_sbus_rx_buf[uart_sbus_rx_con] = Res;				
				uart_sbus_rx_con++; 			
				if(uart_sbus_rx_con>=SBUS_RECV_MAX)
				{
						uart_sbus_rx_con = 0;//复位接收计数器
				}				
			}
		}
	}
}

/**
  * @简  述  处理接收的数据转换为通道值
  * @参  数  无
  * @返回值	 0-无数据接收，1-处理完成
  * @数据说明：所有通道理论上最大值1792，最小值192，中位992，实际遥杆可能存在偏差（如：右遥杆上下98，左右69；左摇杆上下左右95）
  */
uint8_t AX_SBUS_Handle_RxData(void)
{

	if(uart_sbus_rx_ok != 0)
	{
		//计算遥控器通道数据
		uart_sbus_data[0] = ((int16_t)uart_sbus_rx_buf[ 1] >> 0 | ((int16_t)uart_sbus_rx_buf[ 2] << 8 )) & 0x07FF;//右摇杆左右（遥感统一上增下减，右增左减）
		uart_sbus_data[1] = ((int16_t)uart_sbus_rx_buf[ 2] >> 3 | ((int16_t)uart_sbus_rx_buf[ 3] << 5 )) & 0x07FF;//左摇杆上下
		uart_sbus_data[2] = ((int16_t)uart_sbus_rx_buf[ 3] >> 6 | ((int16_t)uart_sbus_rx_buf[ 4] << 2 )  | (int16_t)uart_sbus_rx_buf[ 5] << 10 ) & 0x07FF;//右摇杆上下
		uart_sbus_data[3] = ((int16_t)uart_sbus_rx_buf[ 5] >> 1 | ((int16_t)uart_sbus_rx_buf[ 6] << 7 )) & 0x07FF;//左摇杆左右
		uart_sbus_data[4] = ((int16_t)uart_sbus_rx_buf[ 6] >> 4 | ((int16_t)uart_sbus_rx_buf[ 7] << 4 )) & 0x07FF;//SWA（统一下增）
		uart_sbus_data[5] = ((int16_t)uart_sbus_rx_buf[ 7] >> 7 | ((int16_t)uart_sbus_rx_buf[ 8] << 1 )  | (int16_t)uart_sbus_rx_buf[9] <<  9 ) & 0x07FF;//SWB
		uart_sbus_data[6] = ((int16_t)uart_sbus_rx_buf[ 9] >> 2 | ((int16_t)uart_sbus_rx_buf[10] << 6 )) & 0x07FF;//SWC
		uart_sbus_data[7] = ((int16_t)uart_sbus_rx_buf[10] >> 5 | ((int16_t)uart_sbus_rx_buf[11] << 3 )) & 0x07FF;//SWD		
		uart_sbus_data[8] = ((int16_t)uart_sbus_rx_buf[12] << 0 | ((int16_t)uart_sbus_rx_buf[13] << 8 )) & 0x07FF;//左旋钮（顺时针增）
		uart_sbus_data[9] = ((int16_t)uart_sbus_rx_buf[13] >> 3 | ((int16_t)uart_sbus_rx_buf[14] << 5 )) & 0x07FF;//右旋钮
		//暂时无用
		uart_sbus_data[10] = ((int16_t)uart_sbus_rx_buf[14] >> 6 | ((int16_t)uart_sbus_rx_buf[15] << 2 )  | (int16_t)uart_sbus_rx_buf[16] << 10 ) & 0x07FF;
		uart_sbus_data[11] = ((int16_t)uart_sbus_rx_buf[16] >> 1 | ((int16_t)uart_sbus_rx_buf[17] << 7 )) & 0x07FF;
		uart_sbus_data[12] = ((int16_t)uart_sbus_rx_buf[17] >> 4 | ((int16_t)uart_sbus_rx_buf[18] << 4 )) & 0x07FF;
		uart_sbus_data[13] = ((int16_t)uart_sbus_rx_buf[18] >> 7 | ((int16_t)uart_sbus_rx_buf[19] << 1 )  | (int16_t)uart_sbus_rx_buf[20] <<  9 ) & 0x07FF;
		uart_sbus_data[14] = ((int16_t)uart_sbus_rx_buf[20] >> 2 | ((int16_t)uart_sbus_rx_buf[21] << 6 )) & 0x07FF;
		uart_sbus_data[15] = ((int16_t)uart_sbus_rx_buf[21] >> 5 | ((int16_t)uart_sbus_rx_buf[22] << 3 )) & 0x07FF;
			
		//复位标志位
		uart_sbus_rx_ok = 0;
		return 1;
	}
	else
	{
		return 0;
	}	
}




/**
  * @简  述  获取接收的数据
  * @参  数  *pbuf：通道数据
  * @返回值	 0-无数据接收，1-有数据
  */
uint8_t AX_SBUS_GetRxData(uint16_t *pbuf)
{

	if(uart_sbus_rx_ok != 0)
	{
		//计算遥控器通道数据
		*(pbuf+0) = ((int16_t)uart_sbus_rx_buf[ 1] >> 0 | ((int16_t)uart_sbus_rx_buf[ 2] << 8 )) & 0x07FF;
		*(pbuf+1) = ((int16_t)uart_sbus_rx_buf[ 2] >> 3 | ((int16_t)uart_sbus_rx_buf[ 3] << 5 )) & 0x07FF;
		*(pbuf+2) = ((int16_t)uart_sbus_rx_buf[ 3] >> 6 | ((int16_t)uart_sbus_rx_buf[ 4] << 2 )  | (int16_t)uart_sbus_rx_buf[ 5] << 10 ) & 0x07FF;
		*(pbuf+3) = ((int16_t)uart_sbus_rx_buf[ 5] >> 1 | ((int16_t)uart_sbus_rx_buf[ 6] << 7 )) & 0x07FF;
		*(pbuf+4) = ((int16_t)uart_sbus_rx_buf[ 6] >> 4 | ((int16_t)uart_sbus_rx_buf[ 7] << 4 )) & 0x07FF;
		*(pbuf+5) = ((int16_t)uart_sbus_rx_buf[ 7] >> 7 | ((int16_t)uart_sbus_rx_buf[ 8] << 1 )  | (int16_t)uart_sbus_rx_buf[9] <<  9 ) & 0x07FF;
		*(pbuf+6) = ((int16_t)uart_sbus_rx_buf[9] >> 2 | ((int16_t)uart_sbus_rx_buf[10] << 6 )) & 0x07FF;
		*(pbuf+7) = ((int16_t)uart_sbus_rx_buf[10] >> 5 | ((int16_t)uart_sbus_rx_buf[11] << 3 )) & 0x07FF;
		
		*(pbuf+8) = ((int16_t)uart_sbus_rx_buf[12] << 0 | ((int16_t)uart_sbus_rx_buf[13] << 8 )) & 0x07FF;
		*(pbuf+9) = ((int16_t)uart_sbus_rx_buf[13] >> 3 | ((int16_t)uart_sbus_rx_buf[14] << 5 )) & 0x07FF;
		*(pbuf+10) = ((int16_t)uart_sbus_rx_buf[14] >> 6 | ((int16_t)uart_sbus_rx_buf[15] << 2 )  | (int16_t)uart_sbus_rx_buf[16] << 10 ) & 0x07FF;
		*(pbuf+11) = ((int16_t)uart_sbus_rx_buf[16] >> 1 | ((int16_t)uart_sbus_rx_buf[17] << 7 )) & 0x07FF;
		*(pbuf+12) = ((int16_t)uart_sbus_rx_buf[17] >> 4 | ((int16_t)uart_sbus_rx_buf[18] << 4 )) & 0x07FF;
		*(pbuf+13) = ((int16_t)uart_sbus_rx_buf[18] >> 7 | ((int16_t)uart_sbus_rx_buf[19] << 1 )  | (int16_t)uart_sbus_rx_buf[20] <<  9 ) & 0x07FF;
		*(pbuf+14) = ((int16_t)uart_sbus_rx_buf[20] >> 2 | ((int16_t)uart_sbus_rx_buf[21] << 6 )) & 0x07FF;
		*(pbuf+15) = ((int16_t)uart_sbus_rx_buf[21] >> 5 | ((int16_t)uart_sbus_rx_buf[22] << 3 )) & 0x07FF;
				
		//复位标志位
		uart_sbus_rx_ok = 0;
		return 1;
	}
	else
	{
		return 0;
	}	
}

bool AX_SBUS_GetLiveChannels(uint16_t channels[16])
{
    OS_CPU_SR cpu_sr;
    OS_ENTER_CRITICAL();
    bool valid = sbus_frame_valid && (OSTimeGet() - sbus_frame_tick < SBUS_SIGNAL_TIMEOUT_TICKS);
    if (valid)
        memcpy(channels, uart_sbus_data, sizeof(uart_sbus_data));
    else
        sbus_frame_valid = false;
    OS_EXIT_CRITICAL();
    return valid;
}
