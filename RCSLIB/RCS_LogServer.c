/**
 * @name:RCS_LogServer.c
 * @brief:串口日志功能的实现
 * @usage:串口日志解决了屏幕日志无法捕获暂态事件的痛点。但是屏幕日志解决了串口日志数据淹没的痛点。二者各有优缺点，需要各位扬长避短，提高调试效率。
 * @contribute:CYK-Dot @2024-6-10
 * -------------------------------------------------------------
 * @todo:strcpy存在数组越界访问的问题,一旦开的数组空间小于字符串长度,strcpy就会写到不该写的地方
 **/

/*=================头文件===================================================*/
#include "RCS_LogServer.h"

/*=================静态与弱函数声明==========================================*/

static void      LogServer_Change_PortRoute(Log_Port_Handler_t* Port_t);
static float     LogServer_Get_TimeStamp(void);
__WEAK void      LogServer_Overflow_Callback(LogServer_t* logserver);

/*=================导出函数定义==============================================*/
/**
 * @name:LogServer_Init
 * @brief:初始化一个日志系统
 * @param:Port_t    一般使用串口作为日志的硬件接口,因此此处填入USARTx/UARTx即可
 * @param:logserver 需要被初始化的日志系统的指针
 * @param:Name      为该日志系统取一个名字,请传入字符串常量
 **/
void LogServer_Init(LogServer_t* logserver,Log_Port_Handler_t* Port_t,char* Name)
{
	memset(logserver->Record,sizeof(logserver->Record),0);
	logserver->Log_ptr=0;
	logserver->Log_Port=Port_t;
	strcpy(logserver->Name,Name);
}

/**
 * @name:R2_LogServer_Reset
 * @brief:清空日志缓冲区
 * @param:logserver 需要被清空的日志系统的指针
 **/
void LogServer_Reset(LogServer_t* logserver)
{
	memset(logserver->Record,sizeof(logserver->Record),0);
	logserver->Log_ptr=0;
}


/**
 * @name:R2_LogServer_Collect
 * @brief:往日志缓冲区中写入一条日志
 * @param:logserver 需要被写入的日志系统的指针
 * @param:log_type 本条日志的类型                                        
 * @param:log_main 本条日志的主要内容
 * @param:log_addl  本条日志的附加内容
 **/
void LogServer_Collect(LogServer_t* logserver,char* log_type,char* log_main,char* log_addl)
{
	//日志缓冲区将满
	if (logserver->Log_ptr == LOG_BUFF_MAX_SIZE-2)
	{
		strcpy((logserver->Record[logserver->Log_ptr]).Log_Type,&"E");
		strcpy((logserver->Record[logserver->Log_ptr]).Log_Main,&"LogServer Overflow!");
		strcpy((logserver->Record[logserver->Log_ptr]).Log_Main,&"define LOG_BUFF_MAX_SIZE should be larger!");
		logserver->Log_ptr++;
	}
	//日志缓冲区未满
	else if (logserver->Log_ptr < LOG_BUFF_MAX_SIZE-2)
	{
		if (log_type != NULL) 
			strcpy((logserver->Record[logserver->Log_ptr]).Log_Type,log_type); 
		else 
			strcpy((logserver->Record[logserver->Log_ptr]).Log_Type,&"Null");
		
		if (log_main != NULL) 
			strcpy((logserver->Record[logserver->Log_ptr]).Log_Main,log_main); 
		else 
			strcpy((logserver->Record[logserver->Log_ptr]).Log_Main,&"Null");
		
		if (log_addl != NULL) 
			strcpy((logserver->Record[logserver->Log_ptr]).Log_Addl,log_addl); 
		else 
			strcpy((logserver->Record[logserver->Log_ptr]).Log_Addl,&"Null");
		
		logserver->Log_ptr++;
	}
	//日志缓冲区满
	else
	{
		LogServer_Overflow_Callback(logserver);
	}
}

/**
 * @name:R2_LogServer_Output
 * @brief:输出日志,输出后自动清空缓冲区
 * @param:logserver 需要被输出的日志系统的指针
 **/
void LogServer_Output(LogServer_t* logserver)
{
	//AC5兼容写法,局部变量只放在函数开头
	uint16_t ptr;
	//有日志来才会打印
	if (logserver->Log_ptr!=0)
	{
		//更改printf线路
		LogServer_Change_PortRoute(logserver->Log_Port);
		//逐条打印
		printf("[%.1f]========FROM",LogServer_Get_TimeStamp());
		printf(" %s  ",logserver->Name);
		printf("===========================================\n\r");
		
		for (ptr=0;ptr<logserver->Log_ptr;ptr++)
		{
			ANSI_CHANGE_COLOR(ANSI_SHADE_NONE,ANSI_COLOR_GREEN);
			printf("|%s",(logserver->Record[ptr]).Log_Type);
			
			ANSI_MOVE_CURSOR_COLOMN(LOG_TYPE_MAX_LEN);
			ANSI_CHANGE_COLOR(ANSI_SHADE_NONE,ANSI_COLOR_YELLOW);
			printf("|   %s   ",(logserver->Record[ptr]).Log_Main);
			
			ANSI_CHANGE_COLOR(ANSI_SHADE_NONE,ANSI_COLOR_BLUE);
			printf("#%s\n\r",(logserver->Record[ptr]).Log_Addl);
			
			ANSI_CHANGE_COLOR(ANSI_SHADE_NONE,ANSI_COLOR_WHITE);
		}
		printf("\n\r");
		//清空缓冲区
		LogServer_Reset(logserver);
	}
}

/**
 * @name:LogServer_Output_TimeStamp
 * @brief:手动指定时间戳的日志输出;输出后自动清空缓冲区
 * @param:logserver 需要被输出的日志系统的指针
 * @param:TimeStamp 时间戳
 * @usage:通常用于需要手动指定时间戳的场景，比如单次输出多个日志系统的日志,但希望赋予相同时间戳
 **/
void LogServer_Output_TimeStamp(LogServer_t* logserver,uint64_t TimeStamp)
{
	//AC5兼容写法,局部变量只放在函数开头
	uint16_t ptr;
	//有日志来才会打印
	if (logserver->Log_ptr!=0)
	{
		//更改printf线路
		LogServer_Change_PortRoute(logserver->Log_Port);
		//逐条打印
		printf("[%d]========FROM %s  ",TimeStamp/1000,logserver->Name);
		printf("===========================================\n\r");
		
		for (ptr=0;ptr<logserver->Log_ptr;ptr++)
		{
			ANSI_CHANGE_COLOR(ANSI_SHADE_NONE,ANSI_COLOR_GREEN);
			printf("|%s",(logserver->Record[ptr]).Log_Type);
			
			ANSI_MOVE_CURSOR_COLOMN(LOG_TYPE_MAX_LEN);
			ANSI_CHANGE_COLOR(ANSI_SHADE_NONE,ANSI_COLOR_YELLOW);
			printf("|   %s   ",(logserver->Record[ptr]).Log_Main);
			
			ANSI_CHANGE_COLOR(ANSI_SHADE_NONE,ANSI_COLOR_BLUE);
			printf("#%s\n\r",(logserver->Record[ptr]).Log_Addl);
			
			ANSI_CHANGE_COLOR(ANSI_SHADE_NONE,ANSI_COLOR_WHITE);
		}
		printf("\n\r");
		//清空缓冲区
		LogServer_Reset(logserver);
	}
}

/*=================静态与弱函数定义==========================================*/

/**
 * @name:LogServer_Change_PortRoute
 * @brief:日志系统的硬件抽象层,移植时需要实现LogServer_Change_PortRoute,输入线路名称,可以改变printf的输出位置
 **/
static void LogServer_Change_PortRoute(Log_Port_Handler_t* Port_t)
{
	ANSI_Set_OutPort((USART_TypeDef*)Port_t);
}
/**
 * @name:LogServer_Change_PortRoute
 * @brief:日志系统的硬件抽象层,移植时可以实现LogServer_Get_TimeStamp,为所有日志添上时间戳。不用时间戳时return 0即可
 **/
static float  LogServer_Get_TimeStamp(void)
{
	return 0.0f;
	//return (float)RTOS_Get_RealTime_Us()/1000.0f;
}
/**
 * @name:LogServer_Overflow_Callback
 * @brief:日志系统的回调函数,当日志数量达到记录上限时被触发,被weak修饰,可以由应用层重写
 **/
__WEAK void LogServer_Overflow_Callback(LogServer_t* logserver)
{

}