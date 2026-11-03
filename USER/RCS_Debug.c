/**
 * @filename:RCS_Debug.c
 * @brief:与终端上位机(MobaXterm)、串口波形上位机(VOFA)交互
 * @tips:只能在BleTask中使用Debug相关函数,否则会影响其他任务的实时性
 * @contribute:陈煜楷 @2024-2-2
 * @changlog:  陈煜楷 @2024-2-20 添加了纯log版本的调试控制台 
*/
#include "RCS_Debug.h"
#include "RCS_ANSI.h"

/* ==== 常量与全局变量 ==================================*/
const char RCS_SHELL_ASCII_DRAW[8][50]={
" _____   _____  _____    _____ _          _ _", 
"|  __ \\ / ____|/ ____|  / ____| |        | | |",
"| |__) | |    | (___   | (___ | |__   ___| | |",
"|  _  /| |     \\___ \\   \\___ \\| '_ \\ / _ \\ | |",
"| | \\ \\| |____ ____) |  ____) | | | |  __/ | |",
"|_|  \\_\\\\_____|_____/  |_____/|_| |_|\\___|_|_|"
};//https://patorjk.com/software/taag/#p=display&v=3&f=Big&t=RCS%20Shell
RCS_Shell_Outs RCSLIB_Debug_View;
USART_TypeDef* shell_usart;
USART_TypeDef* vofa_usart;

/* ==== 静态函数声明 ====================================*/
static void RCS_Shell_Init_USART(RCS_PIN_USART USARTx_MAP);
static void RCS_JustFloat_Init_USART(RCS_PIN_USART USARTx_MAP);
static void RCS_Shell_Process_Cmd(void);
static void RCS_Shell_Print_Logo(void);

/* ==== 函数定义  =======================================*/

/**
 * @name:RCS_Shell_Init
 * @brief:指定一个串口为调试终端输出口,初始化完成后展示Logo
 * @param:USARTx_MAP 主控板上的第几个USART组
 * @param:Shell_Type 终端风格,SHELL_TYPE_LOG_WATCH不滚屏,SHELL_TYPE_LOG滚屏
 * @addtogroup:串口终端
*/
void RCS_Shell_Init(RCS_Shell_Outs* debug_view,RCS_PIN_USART USARTx_MAP,uint8_t Shell_Type)
{
	//初始化USART外设
	RCS_Shell_Init_USART(USARTx_MAP);
	//定义printf重定向到哪个串口
	shell_usart=USARTx_MAP.USARTx;
	//Shell初始化
	debug_view->logs_len=0;
	debug_view->points_len=0;
	debug_view->shell_style=Shell_Type;
	ANSI_CLEAR_ALL();
	ANSI_MOVE_CURSOR(0,0);
	RCS_Shell_Print_Logo();
}

/**
 * @name:RCS_JustFloat_Init
 * @brief:指定一个串口为串口波形输出口
 * @param:USARTx_MAP 主控板上的第几个USART组
 * @addtogroup:串口波形
*/
void RCS_JustFloat_Init(RCS_PIN_USART USARTx_MAP)
{
	vofa_usart=USARTx_MAP.USARTx;
	RCS_JustFloat_Init_USART(USARTx_MAP);
	ANSI_Set_OutPort(USARTx_MAP.USARTx);
}

/**
 * @name:RCS_Shell_Add_Watch_Points
 * @brief:添加终端观测变量值
 * @param:Watch_Name 给这个观测值取一个名字
 * @param:value 观测值所在的地址,和keil调试类似,要注意使用有确定地址的变量，此处仅支持int类型
 * @addtogroup:串口终端
*/
void RCS_Shell_Add_Watch_Points(RCS_Shell_Outs* debug_view,char* Watch_Name,int* value)
{
	debug_view->points_len++;
	if (debug_view->points_len>=RCS_SHELL_WATCH_MAX_LEN) debug_view->points_len=0;
	
	debug_view->points_name[debug_view->points_len]=Watch_Name;
	debug_view->watch_points[debug_view->points_len]=value;
}

/**
 * @name:RCS_Shell_Logs
 * @brief:向终端输出一条日志,用于观测程序运行状态
 * @param:Log_String,日志的内容
 * @addtogroup:串口终端
*/
void RCS_Shell_Logs(RCS_Shell_Outs* debug_view,char* Log_String)
{
	//无论打印何种日志,总是把日志加入队列
	debug_view->logs_len++;
	if (debug_view->logs_len>=RCS_SHELL_LOGS_MAX_LEN) debug_view->logs_len=0;
	debug_view->watch_logs[debug_view->logs_len]=Log_String;
	//滚屏的Shell,直接打印
	if (debug_view->shell_style == SHELL_TYPE_LOG)
	{
		printf(Log_String);
		printf("\n\r");
	}
}

/**
 * @name:RCS_Shell_Main
 * @brief:终端主程序,请放在BleTask中
 * @addtogroup:串口终端
*/
void RCS_Shell_Main(RCS_Shell_Outs* debug_view)
{
	//---------------------WATCH-----------------------------------------
	if ((debug_view->shell_style == SHELL_TYPE_LOG_WATCH)||(debug_view->shell_style == SHELL_TYPE_WATCH))
	{
		//排版
		ANSI_MOVE_CURSOR(8,1);
		ANSI_CLEAR_CURSOR();   
		ANSI_CHANGE_COLOR(ANSI_SHADE_NONE,ANSI_COLOR_GREEN);
		printf("Watch Points\n\r");
		ANSI_CHANGE_COLOR(ANSI_SHADE_NONE,ANSI_COLOR_WHITE);
		//观测点打印
		for(int i=1;i<=debug_view->points_len;i++)
		{
			printf("%s",debug_view->points_name[i]);
			ANSI_MOVE_CURSOR(8+i,25);
			printf("%d\n\r",*(debug_view->watch_points[i]));
		}
	}
	//---------------------LOG-----------------------------------------
	if (debug_view->shell_style == SHELL_TYPE_LOG_WATCH)
	{
		//排版
		ANSI_MOVE_CURSOR(8+RCS_SHELL_WATCH_MAX_LEN+4,1);
		ANSI_CHANGE_COLOR(ANSI_SHADE_NONE,ANSI_COLOR_RED);
		printf("Logs\n\r");
		ANSI_CHANGE_COLOR(ANSI_SHADE_NONE,ANSI_COLOR_WHITE);	
		//日志打印	
		for(int i=1;i<=debug_view->logs_len;i++)
		{
			printf("%s",debug_view->watch_logs[i]);
			printf("\n\r");
		}
	}
	else if (debug_view->shell_style == SHELL_TYPE_LOG)
	{
		return;//滚屏，不控制光标位置
	}
	else if (debug_view->shell_style == SHELL_TYPE_WATCH)
	{
		return;
	}
}

/**
 * @name:JustFloat_Printf
 * @brief:按JustFloat格式向VOFA上位机输送波形数据
 * @param:fdata 一个浮点数组,存放诺干个通道的波形
 * @param:channel_count 通道总数
 * @param:USARTx_MAP 主控板上的第几个USART组
 * @addtogroup:串口波形
*/
void JustFloat_Printf(float* fdata,uint16_t channel_count,RCS_PIN_USART USARTx_MAP)
{
	ANSI_Set_OutPort(USARTx_MAP.USARTx);
	DataType_32Bit output;
	uint8_t c_data[4];
	for(int i=0;i<channel_count;i++)
	{
		output.fdata=fdata[i];
		Bit32_Split_BigEndian(output,c_data);
		RCS_USART_Send_Char(USARTx_MAP.USARTx,c_data[3]);
		RCS_USART_Send_Char(USARTx_MAP.USARTx,c_data[2]);
		RCS_USART_Send_Char(USARTx_MAP.USARTx,c_data[1]);
		RCS_USART_Send_Char(USARTx_MAP.USARTx,c_data[0]);
	}
	RCS_USART_Send_Char(USARTx_MAP.USARTx,0x00);
	RCS_USART_Send_Char(USARTx_MAP.USARTx,0x00);
	RCS_USART_Send_Char(USARTx_MAP.USARTx,0x80);
	RCS_USART_Send_Char(USARTx_MAP.USARTx,0x7f);
}

/* === 静态函数实现 ==================================*/

//初始化串口外设
static void RCS_Shell_Init_USART(RCS_PIN_USART USARTx_MAP)
{
	RCS_USART_With_NoIsr_Config(USARTx_MAP.USARTx,USARTx_MAP.GPIOx,USARTx_MAP.GPIO_Pin_Tx,USARTx_MAP.GPIO_Pin_Rx,DEBUG_BAUD);
}
static void RCS_JustFloat_Init_USART(RCS_PIN_USART USARTx_MAP)
{
	RCS_USART_With_NoIsr_Config(USARTx_MAP.USARTx,USARTx_MAP.GPIOx,USARTx_MAP.GPIO_Pin_Tx,USARTx_MAP.GPIO_Pin_Rx,DEBUG_BAUD);
}

static void RCS_Shell_Print_Logo(void)
{
	printf("\033[34m");//blue
	printf("\033[1m\n\r");//width
	printf("%s\n\r",RCS_SHELL_ASCII_DRAW[0]);
	printf("%s\n\r",RCS_SHELL_ASCII_DRAW[1]);
	printf("%s\n\r",RCS_SHELL_ASCII_DRAW[2]);
	printf("%s\n\r",RCS_SHELL_ASCII_DRAW[3]);
	printf("%s\n\r",RCS_SHELL_ASCII_DRAW[4]);
	printf("%s\n\r",RCS_SHELL_ASCII_DRAW[5]);
	ANSI_CHANGE_COLOR(ANSI_SHADE_DARK,ANSI_COLOR_WHITE);
}


//todo:处理终端输入
static void RCS_Shell_Process_Cmd(void)
{
	
}



//开启终端响应后，对scanf做重定向
#ifdef RCS_SHELL_PROCESS_CMD
	int fgetc(FILE *stream)
	{
		while(!(shell_usart->SR & (1 << 5))){};//等待数据接收完成
		return shell_usart->DR;
	}
#endif 

