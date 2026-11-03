#include "rcs.h"

uint8_t tof_data[20];
int32_t   tof_distance;

int32_t Get_Tof_Data(void)
{
	 static float outputs[10];
	 static uint8_t index=0;
	 index++;
	 if (index==10) index=0;
	 outputs[index]=tof_distance;
	 return (int32_t)Get_Median(outputs,10);
	//return (tof_distance/1000.0f);
}

static void tof_isr(void)
{
	static uint8_t tof_rcv_flag=0;
	static uint32_t tof_check;
	uint8_t tof_rcv_char;
	
	if(USART_GetITStatus(UART4, USART_IT_RXNE) == SET)
	{
		tof_rcv_char = RCS_USART_Accept_Char(UART4);
		switch(tof_rcv_flag)
		{
			//----------包头------------------------
			case 0:
				if (tof_rcv_char==0x57)
					tof_rcv_flag++;
			break;
			case 1:
				if (tof_rcv_char==0x00)
				{
					tof_rcv_flag++;
					tof_check=0x57;
				}
				else
				 	tof_rcv_flag=0;
			break;
			//----------数据----------------------
			default:
				tof_data[tof_rcv_flag]=tof_rcv_char;
				if (tof_rcv_flag!=15) 
				{
					tof_check+=tof_rcv_char;
					tof_check&=0x00ff;
				}
				tof_rcv_flag++;

				if ((tof_rcv_flag==16)&&(tof_rcv_char==tof_check))
				{
					tof_distance=(int32_t)((tof_data[8]<<8)|(tof_data[9]<<16)|(tof_data[10]<<24));
					tof_rcv_flag=0;
				}	
			break;
		}
	}
}
