/**
 * @filename:G431_GPS.c
 * @brief:与自制G431全场定位板进行交互(常规模式)
 * @contribute:CYK-Dot @2024-6-1
 --------------------------------------------------
 * @todo:将主控挂在调试模式下,如果在Uart接收中断中打断点,G431就会在主控恢复运行之后卡死
**/

/*=================头文件===========================================*/
#include "G431_GPS.h"


/*=================全局变量=========================================*/

RCS_PIN_USART        G431_GPS_USART;                //管脚
RCS_LenVerify_Ptl    G431_GPS_Ptl;                  //协议

uint8_t              G431_GPS_Pkg_Rcv_Char;         //单个接收到的字节
uint8_t              G431_GPS_Pkg_Rcv_State;        //协议状态机返回的日志
uint8_t              G431_GPS_Pkg_RcvBufr_Arr[20];  //报文缓冲区
uint8_t              G431_GPS_Pkg_RcvCplt_Arr[20];  //完成接收的报文字节流
uint8_t              G431_GPS_Pkg_Txn[32];


float                G431_GPS_Pkg_X;                //获取到的XYZ坐标
float                G431_GPS_Pkg_Y;
float                G431_GPS_Pkg_Z;

/*===============静态与弱函数声明====================================*/

static void G431GPS_Usart_Isr_Callback(void);
__WEAK void G431GPS_RcvCplt_Callback(void);
__WEAK void G431GPS_RcvFail_Callback(void);

/*=================导出函数定义=====================================*/

/**
 * @name: G431GPS_Init
 * @brief:初始化相关串口，并指定初始化的管脚和中断优先级
**/
void G431GPS_Init(RCS_PIN_USART USARTx_MAP,uint8_t _pri)
{
	G431_GPS_USART=USARTx_MAP;
	RCS_USART_Config(USARTx_MAP.USARTx,
	                 USARTx_MAP.GPIOx,USARTx_MAP.GPIO_Pin_Tx,USARTx_MAP.GPIO_Pin_Rx,
	                 G431GPS_Usart_Isr_Callback,
	                 G431_GPS_BAUD,_pri);

	Protocol_Init_LenVerify(&G431_GPS_Ptl,G431GPS_START_BYTE,G431GPS_PKG_LEN,G431_GPS_Pkg_RcvBufr_Arr);
}

/**
 * @name:G431GPS_Get_X
 * @brief:获取G431全场定位返回的X坐标
**/
float G431GPS_Get_X(void)
{
	return G431_GPS_Pkg_X;
}

/**
 * @name:G431GPS_Get_Y
 * @brief:获取G431全场定位返回的Y坐标
**/
float G431GPS_Get_Y(void)
{
	return G431_GPS_Pkg_Y;
}

/**
 * @name:G431GPS_Get_Z
 * @brief:获取G431全场定位返回的Z坐标(角度制)
**/
float G431GPS_Get_Z(void)
{
	return G431_GPS_Pkg_Z;
}

/**
 * @name:G431GPS_Set_Parameter
 * @brief:设置全场定位的参数
 * @param:CMD_ID_xx 命令名,敲入CMD_ID_后keil会自动联想   @ref:Uart_Cmd@G431_GPS.h
 * @param:param     参数
**/
void G431GPS_Set_Parameter(Uart_Cmd CMD_ID_xx,float param)
{
	uint8_t i;
	G431_GPS_Pkg_Txn[0]=CMD_ID_xx;
	sprintf(&G431_GPS_Pkg_Txn[1],"%.6f\n\r",param);
	while(G431_GPS_Pkg_Txn[i]!='\r')
	{
		RCS_USART_Send_Char(G431_GPS_USART.USARTx,G431_GPS_Pkg_Txn[i]);
	}
	RCS_USART_Send_Char(G431_GPS_USART.USARTx,'\r');
}


/*=================静态函数定义=====================================*/

/**
 * @name:G431GPS_Usart_Isr_Callback
 * @brief:Uart接收中断
**/
static void G431GPS_Usart_Isr_Callback(void)
{
	//--------------局部变量---------------------------------------------------
	uint32_t temp_u32_x;
	uint32_t temp_u32_y;
	uint32_t temp_u32_z;

	//---------判断是否为RXNE中断标志位---------------------------------------
	if (RCS_USART_Judge_RcvCplt_ITPendingBit(G431_GPS_USART.USARTx)==1)
	{
		//将接收到的字节送入协议状态机
		G431_GPS_Pkg_Rcv_Char =(uint8_t)RCS_USART_Accept_Char(G431_GPS_USART.USARTx);
		G431_GPS_Pkg_Rcv_State=Protocol_Rcv_LenVerify_IT((void*)&G431_GPS_Ptl,G431_GPS_Pkg_Rcv_Char,G431_GPS_Pkg_RcvCplt_Arr);

		//状态机返回接收成功
		if (G431_GPS_Pkg_Rcv_State==RCV_CPLT)
		{
			//不能直接赋值为float，会被编译器视为强制类型转换，必须使用联合体或者memcpy
			temp_u32_x =((uint32_t)G431_GPS_Pkg_RcvCplt_Arr[0] << 24) |
                  ((uint32_t)G431_GPS_Pkg_RcvCplt_Arr[1] << 16) |
                  ((uint32_t)G431_GPS_Pkg_RcvCplt_Arr[2] << 8)  |
                  ((uint32_t)G431_GPS_Pkg_RcvCplt_Arr[3]);
			temp_u32_y =((uint32_t)G431_GPS_Pkg_RcvCplt_Arr[4] << 24) |
                  ((uint32_t)G431_GPS_Pkg_RcvCplt_Arr[5] << 16) |
                  ((uint32_t)G431_GPS_Pkg_RcvCplt_Arr[6] << 8)  |
                  ((uint32_t)G431_GPS_Pkg_RcvCplt_Arr[7]);
			temp_u32_z =((uint32_t)G431_GPS_Pkg_RcvCplt_Arr[8] << 24) |
                  ((uint32_t)G431_GPS_Pkg_RcvCplt_Arr[9] << 16) |
                  ((uint32_t)G431_GPS_Pkg_RcvCplt_Arr[10] << 8) |
                  ((uint32_t)G431_GPS_Pkg_RcvCplt_Arr[11]);

			memcpy(&G431_GPS_Pkg_X,&temp_u32_x,4);
			memcpy(&G431_GPS_Pkg_Y,&temp_u32_y,4);
			memcpy(&G431_GPS_Pkg_Z,&temp_u32_z,4);

			//回调函数
			G431GPS_RcvCplt_Callback();
		}
		else if (G431_GPS_Pkg_Rcv_State==RCV_FAIL)
		{
			//回调函数
			G431GPS_RcvFail_Callback();
		}
	}
	//-------------------清除中断标志位---------------------------------------
	RCS_USART_Clear_RcvCplt_ITPendingBit(G431_GPS_USART.USARTx);
}

/*=================弱函数定义=======================================*/

/**
 * @name:G431GPS_RcvCplt_Callback
 * @brief:当完成一次报文接收时会被调用的回调函数
**/
__WEAK void G431GPS_RcvCplt_Callback(void)
{

}
/**
 * @name:G431GPS_RcvCplt_Callback
 * @brief:当报文校验失败时会被调用的回调函数
**/
__WEAK void G431GPS_RcvFail_Callback(void)
{

}
