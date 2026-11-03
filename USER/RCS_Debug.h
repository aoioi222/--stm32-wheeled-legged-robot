/**
 * @filename:RCS_Debug.c
 * @brief:与串口终端上位机、串口波形上位机交互
 * @tips:只能在BleTask中使用Debug相关函数,否则会影响其他任务的实时性
 * @date:2024-2-2
*/
#ifndef RCS_DEBUG_H_
#define RCS_DEBUG_H_

/* ---- 头文件 -----------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include "stm32f4xx_usart.h"
#include "RCS_Usart.h"
#include "RCS_Types.h"
#include "RCS_Pin_Mapping.h"
#include "RCS_DataStructure.h"
#include "RCS_ANSI.h"

/* ----设置 -------------------------------------*/
//硬件相关配置
#define RCS_SHELL_PRINTF            //启用串口重映射
//#define RCS_SHELL_PROCESS_CMD      //开启终端交互

#define DEBUG_BAUD 115200           //调试串口波特率

//终端GUI相关配置
#define RCS_SHELL_LOGS_MAX_LEN  100   //最大记录日志数量
#define RCS_SHELL_WATCH_MAX_LEN 25   //最大观测点数量
#define RCS_SHELL_CMD_COUNT     2    //最大命令数量

typedef int DEBUG_DATA_T;

/* ----导出结构体 --------------------------------*/
typedef union 
{
	float    fdata;
	uint32_t ldata;
}DataType_32Bit;//justfloat

typedef struct shell_outs
{
	DEBUG_DATA_T* watch_points[RCS_SHELL_WATCH_MAX_LEN];
	char*         points_name[RCS_SHELL_WATCH_MAX_LEN];
	char*         watch_logs[RCS_SHELL_LOGS_MAX_LEN];
	uint8_t points_len;
	uint8_t logs_len;
	uint8_t shell_style;
}RCS_Shell_Outs;//Shell本体

typedef struct shell_cmd
{
	FNCT_VOID    void_cmd[RCS_SHELL_CMD_COUNT];
	FNCT_U16_U8  u8_cmd[RCS_SHELL_CMD_COUNT];
}RCS_Shell_Cmd;//Shell命令

typedef enum{
	SHELL_TYPE_LOG_WATCH=0,//观测点+日志输出
	SHELL_TYPE_LOG,        //滚屏日志
	SHELL_TYPE_WATCH,      //观测点
}SHELL_TYPE;//终端界面类型




/* ----导出函数 ---------------------------------*/
void RCS_Shell_Init(RCS_Shell_Outs* debug_view,RCS_PIN_USART USARTx_MAP,uint8_t Shell_Type);
void RCS_Shell_Add_Watch_Points(RCS_Shell_Outs* debug_view,char* Watch_Name,int* value);
void RCS_Shell_Logs(RCS_Shell_Outs* debug_view,char* Log_String);
void RCS_Shell_Main(RCS_Shell_Outs* debug_view);

void RCS_JustFloat_Init(RCS_PIN_USART USARTx_MAP);
void JustFloat_Printf(float* fdata,uint16_t channel_count,RCS_PIN_USART USARTx_MAP);

extern RCS_Shell_Outs RCSLIB_Debug_View;
/* ----常用的内联函数 ----------------------------*/
__inline void Bit16_Split_BigEndian(int16_t Current_Data, uint8_t *High_Data, uint8_t *Low_Data)
{
	*High_Data = (Current_Data & 0xff00) >> 8;
	*Low_Data = (Current_Data & 0x00ff);
}
__inline void Bit32_Split_BigEndian(DataType_32Bit Input,uint8_t* Output)
{
	Output[3]= Input.ldata & 0xff;
	Output[2]=(Input.ldata & 0xff00) >> 8;
	Output[1]=(Input.ldata & 0xff0000) >> 16;
	Output[0]=(Input.ldata & 0xff000000) >> 24;
}
__inline void Int16_to_Int8(int16_t Current_Data, uint8_t *High_Data, uint8_t *Low_Data)
{
	*High_Data = (Current_Data & 0xff00) >> 8;
	*Low_Data = (Current_Data & 0x00ff);
}
__inline void Int32_to_Int8(int32_t Input,uint8_t* Output)
{
	Output[3]= Input & 0xff;
	Output[2]=(Input & 0xff00) >> 8;
	Output[1]=(Input & 0xff0000) >> 16;
	Output[0]=(Input & 0xff000000) >> 24;
}

__INLINE void Float32_to_Int8(float Input,uint8_t* Output,uint16_t mult_scale)
{
	int32_t int32_input=(int32_t)(Input*mult_scale);
	Int32_to_Int8(int32_input,Output);
}

#endif