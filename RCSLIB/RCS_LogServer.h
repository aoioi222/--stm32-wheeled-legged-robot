/**
 * @name:RCS_LogServer.c
 * @brief:串口日志功能的实现
 * @usage:串口日志解决了屏幕日志无法捕获暂态事件的痛点。但是屏幕日志解决了串口日志数据淹没的痛点。二者各有优缺点，需要各位扬长避短，提高调试效率。
 * @contribute:CYK-Dot @2024-6-10
 * -------------------------------------------------------------
 * @todo:strcpy存在数组越界访问的问题,一旦开的数组空间小于字符串长度,strcpy就会写到不该写的地方
 **/

#ifndef RCS_LOGSERVER_H_
#define RCS_LOGSERVER_H_

/*----------正常工作所必备的头文件---------------------------*/
#include "string.h"
#include "stdint.h"
#include "RCS_ANSI.h"

/*----------配置相关----------------------------------------*/

//配置硬件抽象层所需头文件
#include "RCS_RTOS.h"
//配置日志输出口类型句柄
#define Log_Port_Handler_t    USART_TypeDef
//配置日志最大长度
#define LOG_TYPE_MAX_LEN  20  //日志类型字符串的最大长度
#define LOG_MAIN_MAX_LEN  70  //主要信息字符串的最大长度
#define LOG_ADDL_MAX_LEN  100 //附加信息字符串的最大长度
#define LOG_NAME_MAX_LEN  10  //日志系统名称的最大长度
#define LOG_BUFF_MAX_SIZE 80  //日志缓冲区长度

/*----------导出结构体---------------------------------------*/

//单条日志结构体
typedef struct{
	char Log_Type[LOG_TYPE_MAX_LEN];
	char Log_Main[LOG_MAIN_MAX_LEN];
	char Log_Addl[LOG_ADDL_MAX_LEN];
}LogRecord_t;

//日志句柄结构体
typedef struct{
	char     Name[LOG_NAME_MAX_LEN];   
	uint16_t Log_ptr;
	LogRecord_t Record[LOG_BUFF_MAX_SIZE];
	Log_Port_Handler_t* Log_Port;
}LogServer_t;

//常用日志类型
#define LogType_Status_Str      "S "
#define LogType_EventStatus_Str "SE"
/*----------导出函数-------------------------------------------*/

void LogServer_Init(LogServer_t* logserver,Log_Port_Handler_t* Port_t,char* Name);
void LogServer_Reset(LogServer_t* logserver);
void LogServer_Collect(LogServer_t* logserver,char* log_type,char* log_main,char* log_addl);
void LogServer_Output(LogServer_t* logserver);
void LogServer_Output_TimeStamp(LogServer_t* logserver,uint64_t TimeStamp);

#endif