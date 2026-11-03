#include "RCS_Stastic.h"

__WEAK void WindowFloat_MemberUpdate_Callback(Comm_WindowData_t* hdl);
__WEAK void WindowFloat_WindowCplt_Callback(Comm_WindowData_t* hdl);
__WEAK void WindowFloat_WindowRedy_Callback(Comm_WindowData_t* hdl);

float ArrayFloat_Get_Mean(float* arr,uint16_t len)
{
	if (len==0) return 0;
	float reval=0;
	for(uint16_t i=0;i<len;i++) reval+=arr[i];
	reval=reval/len;
	return reval;
}

float ArrayFloat_Get_Var(float* arr,uint16_t len)
{
	if (len==0) return 0;
	float mean=ArrayFloat_Get_Mean(arr,len);
	float reval=0;

	for(uint16_t i=0;i<len;i++)
	{
		reval+=sqrtf((arr[i]-mean)*(arr[i]-mean));
	} 
	reval=reval/(1.0f*len);
	return reval;
}

/**
 * @name: WindowFloat_Init_Fixed
 * @brief:初始化一个浮点固定窗口,只有收集完全数据后才会将其更新到窗口中
 * @param:window_size 窗口中有几个浮点数
**/
// void WindowFloat_Init_Fixed(Comm_WindowData_t* hdl,uint8_t window_size)
// {
// 	memset(hdl->member_buff,0,sizeof(hdl->member_buff));
// 	memset(hdl->member_data,0,sizeof(hdl->member_data));
// 	hdl->member_ptr=0;
// 	hdl->member_size=sizeof(float);
// 	hdl->window_size=window_size;
// 	hdl->window_type=1;
// }

/**
 * @name: WindowFloat_Init_Slide
 * @brief:初始化一个浮点滑动窗口,新数据进来会把旧数据代替掉
 * @param:window_size 窗口中有几个浮点数
**/
void WindowFloat_Init_Slide(Comm_WindowData_t* hdl,uint8_t window_size)
{
	memset(hdl->member_buff,0,sizeof(hdl->member_buff));
	memset(hdl->member_data,0,sizeof(hdl->member_data));
	hdl->member_ptr=0;
	hdl->member_size=sizeof(float);
	hdl->window_size=window_size;
	hdl->window_type=1;
}

/**
 * @name: WindowFloat_Update_Member
 * @brief:将数据送入窗口中
**/
void WindowFloat_Update_Member(Comm_WindowData_t* hdl,float data)
{
	float temp_data=data;
	
	switch(hdl->window_type)
	{
		//滑动窗口
		case 1:
			//----------------老数据往右移动一个数据的位置------------------------------------------
			memcpy(&(hdl->member_buff[hdl->member_size]),&(hdl->member_data[0]),(hdl->window_size-1)*hdl->member_size);

			//----------------新数据总是填入第一个位置------------------------------------------
			memcpy(&(hdl->member_data[hdl->member_size]),&(hdl->member_buff[hdl->member_size]),(hdl->window_size-1)*hdl->member_size);
			memcpy(&(hdl->member_data[0]),&temp_data,sizeof(float));			//bug

			//----------------完成数据更新。更新标志位或者是调用回调函数------------------------------------------
			#ifdef WINDOW_CALLBACK_FNCT
			//完成一次数据更新，调用回调函数
			if (hdl->Member_Update_Callback!=NULL) hdl->Member_Update_Callback(hdl);
			WindowFloat_MemberUpdate_Callback(hdl);
			#endif
			#ifdef WINDOW_CALLBACK_BIT
			//完成一次数据更新，更新标志位
			hdl->Member_Update_Flag=1;
			#endif

			//----------------窗口队列的队头抵达终点。更新标志位或者是调用回调函数----------------------------------
			hdl->member_ptr++;                    //窗口队头计数
			if(hdl->member_ptr==hdl->window_size) //队头抵达终点，即窗口被填满
			{
				hdl->member_ptr=0;

				#ifdef WINDOW_CALLBACK_FNCT
				//完成一次接收，调用回调函数
				if (hdl->Window_Cplt_Callback!=NULL) hdl->Window_Cplt_Callback(hdl);
				WindowFloat_WindowCplt_Callback(hdl);

				//窗口被第一次填满，调用回调函数
				if (hdl->Window_Redy_Flag==0)  
				{
					if (hdl->Window_Redy_Callback!=NULL) hdl->Window_Redy_Callback(hdl);
					WindowFloat_WindowRedy_Callback(hdl);
				}				
				#endif

				#ifdef WINDOW_CALLBACK_BIT
				//完成一次接收，更新标志位
				hdl->Window_Cplt_Flag=1;
				//窗口被第一次填满，更新标志位
				if (hdl->Window_Redy_Flag==0) hdl->Window_Redy_Flag=1;
				#endif
			}
		break;
			
		//固定窗口
		case 0:
			// memcpy(&(hdl->member_buff[hdl->member_ptr]),&temp_data,sizeof(float));
			// hdl->Member_Update_Callback(hdl);
			// WindowFloat_MemberUpdate_Callback(hdl);
		
			// hdl->member_ptr+=hdl->member_size;
			// if (hdl->member_ptr >= sizeof(float)*hdl->window_size)
			// {
			// 	memcpy(hdl->member_data,hdl->member_buff,sizeof(hdl->member_data));
			// 	hdl->member_ptr=0;
			// 	hdl->Window_Cplt_Callback(hdl);
			// 	WindowFloat_WindowCplt_Callback(hdl);
			// }
		break;
	}
}

/**
 * @name: WindowFloat_Update_Size
 * @brief:更改窗口的大小
**/
void WindowFloat_Update_Size(Comm_WindowData_t* hdl,uint16_t new_size)
{

	#ifdef WINDOW_CALLBACK_BIT
	if (hdl->window_size!=new_size) hdl->Window_Redy_Flag=0;
	#endif

	hdl->window_size=new_size;
}

void* WindowFloat_Get_Member_Ptr(Comm_WindowData_t* hdl,uint16_t arr_id)
{
	return &(hdl->member_data[arr_id*hdl->member_size]);
}

/**
 * @name: WindowFloat_Get_Mean
 * @brief:获取窗口的平均值
**/
float WindowFloat_Get_Mean(Comm_WindowData_t* hdl)
{
	volatile float reval=0;
	volatile float member; 
	for(int i=0;i<hdl->window_size;i++)
	{
		memcpy(&member,&(hdl->member_data[i*hdl->member_size]),hdl->member_size);
		reval+=member;
	}
	return reval/(1.0f*hdl->window_size);
}

/**
 * @name: WindowFloat_Get_Var
 * @brief:获取窗口的方差
**/
float WindowFloat_Get_Var(Comm_WindowData_t* hdl)
{
	volatile float reval=0;
	volatile float member;
	volatile float mean=WindowFloat_Get_Mean(hdl);
	for(int i=0;i<hdl->window_size;i++)
	{
		memcpy(&member,&(hdl->member_data[i*hdl->member_size]),hdl->member_size);
		reval+=sqrtf((member-mean)*(member-mean));
	}
	return reval;
}

/**
 * @name: WindowFloat_Get_Update_Flag/WindowFloat_Get_Cplt_Flag
 * @brief:获取软中断标志位,获取过后自动清除标志位
**/
#ifdef WINDOW_CALLBACK_BIT
uint8_t WindowFloat_Get_Update_Flag(Comm_WindowData_t* hdl)
{
	uint8_t reval=hdl->Member_Update_Flag;
	hdl->Member_Update_Flag=0;
	return reval;
}
uint8_t WindowFloat_Get_Cplt_Flag(Comm_WindowData_t* hdl)
{
	uint8_t reval=hdl->Window_Cplt_Flag;
	hdl->Window_Cplt_Flag=0;
	return reval;
}
uint8_t WindowFloat_Get_Redy_Flag(Comm_WindowData_t* hdl)
{
	return hdl->Window_Redy_Flag;
}
#endif


__WEAK void WindowFloat_MemberUpdate_Callback(Comm_WindowData_t* hdl)
{
	__NOP();
}
__WEAK void WindowFloat_WindowCplt_Callback(Comm_WindowData_t* hdl)
{
	__NOP();
}
__WEAK void WindowFloat_WindowRedy_Callback(Comm_WindowData_t* hdl)
{
	__NOP();
}


void WindowFloat_Test(Comm_WindowData_t* hdl)
{
	uint16_t i;
	for(i=0;i<20;i++)
	{
		WindowFloat_Update_Member(hdl,1.0f*i+0.5f);
		__NOP();
	}
}