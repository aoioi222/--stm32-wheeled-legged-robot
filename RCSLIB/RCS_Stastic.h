#ifndef RCS_STASTIC_H_
#define RCS_STASTIC_H_

#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "cmsis_armclang.h"


#define WINDOW_MAX_BYTE 400     //窗口最大字节数量    float/float_t四个字节  double 八个字节

#define WINDOW_CALLBACK_FNCT    //使用回调函数的方式处理窗口事件
#define WINDOW_CALLBACK_BIT     //使用标志位标记窗口事件

typedef void (*FNCT_VOID_WDATA) (struct common_window*);

typedef struct common_window{
	volatile uint8_t member_data[WINDOW_MAX_BYTE];
	uint8_t member_buff[WINDOW_MAX_BYTE];
	uint8_t member_size;
	uint8_t member_ptr;
	
	uint8_t window_size;
	uint8_t window_type;
	
	#ifdef WINDOW_CALLBACK_FNCT
	FNCT_VOID_WDATA Member_Update_Callback;
	FNCT_VOID_WDATA Window_Cplt_Callback;
	FNCT_VOID_WDATA Window_Redy_Callback;
	#endif

	#ifdef WINDOW_CALLBACK_BIT
	uint8_t Member_Update_Flag;
	uint8_t Window_Cplt_Flag;
	uint8_t Window_Redy_Flag;
	#endif
}Comm_WindowData_t;


float ArrayFloat_Get_Mean(float* arr,uint16_t len);
float ArrayFloat_Get_Var(float* arr,uint16_t len);

void WindowFloat_Init_Fixed(Comm_WindowData_t* hdl,uint8_t window_size);
void WindowFloat_Init_Slide(Comm_WindowData_t* hdl,uint8_t window_size);

void WindowFloat_Update_Member(Comm_WindowData_t* hdl,float data);
void WindowFloat_Update_Size(Comm_WindowData_t* hdl,uint16_t new_size);
void* WindowFloat_Get_Member_Ptr(Comm_WindowData_t* hdl,uint16_t arr_id);
float WindowFloat_Get_Mean(Comm_WindowData_t* hdl);
float WindowFloat_Get_Var(Comm_WindowData_t* hdl);
uint8_t WindowFloat_Get_Update_Flag(Comm_WindowData_t* hdl);
uint8_t WindowFloat_Get_Cplt_Flag(Comm_WindowData_t* hdl);
uint8_t WindowFloat_Get_Redy_Flag(Comm_WindowData_t* hdl);

void WindowFloat_Test(Comm_WindowData_t* hdl);
#endif