/* .h head --------------------------------------------------------------*/
#ifndef RCS_PROTOCOL_H_
#define RCS_PROTOCOL_H_

/* 头文件 ---------------------------------------------------------------*/
#include "stdint.h"
#include "string.h"
#include "RCS_Types.h"

/* 私有类型 --------------------------------------------------------------*/

//简单头尾协议参数
typedef struct protocol{
    uint8_t Start_Byte;
    uint8_t Len;
    uint8_t End_Byte;
		FNCT_VOID_PU8 Finish_Callback;
}RCS_Easy_Ptl;
typedef struct protocol2
{
    uint8_t Start_Byte1; 	// 第一个起始字节
	  uint8_t Start_Byte2; 	// 第一个起始字节
    int Len;
    uint8_t End_Byte;
		FNCT_VOID_PU8 Finish_Callback;
}RCS_Easy_Ptl2;

//简单长度校验协议
typedef struct{

    uint8_t Start_Byte;  //头字节
    uint8_t Len;         //报文有效长度

    uint8_t  RcvStat_Fsm; //接收状态机当前的状态
    uint8_t  RcvBufr_Pnt; //缓冲区队列当前下标
    uint8_t* RcvBufr_Arr; //缓冲区队列

}RCS_LenVerify_Ptl;

//变长头尾协议日志回馈
typedef enum comm_ptl_log{
    RCV_PROC =0,          //帧尚未完成接收
    RCV_CPLT =1,          //帧完成接收而且校验成功
    RCV_FAIL =2,          //帧完成接收但是校验失败
    RCV_ERR  =3,          //接收状态机的状态被意外改变,进入了不应该进入的case
    RCV_IDLE =4,          //空闲,并未处于工作状态
}RCS_Common_Ptl_Log;

//变长头尾协议帧类型
typedef enum comm_ptl_cfg{
    Pkg_Type_Verify     =0,   //带求和校验的数据包
    Pkg_Type_Direct     =1,   //带包尾校验的数据包
    Pkg_Type_FullVerify =2,   //带求和+包尾校验的数据包(todo
		Pkg_Type_Fix        =3,
}RCS_Common_Ptl_PkgType;

//变长头尾协议参数
typedef struct comm_protocol{
    volatile uint8_t                Start_Byte;    //使用的包头字节
    volatile RCS_Common_Ptl_PkgType Pkg_Type;      //使用的帧类型
    volatile uint8_t                End_Byte;      //使用的包尾字节
		volatile uint8_t                Max_Len;
}RCS_Common_Ptl;

//变长头尾协议缓冲区
typedef struct comm_ptl_buffer{
    volatile int8_t   status_flag;  //接收状态机的状态
    volatile uint8_t* rcv_buffer;   //接收缓冲区,在初始化的时候指定,大小不应小于数据长度+2
    volatile uint8_t  rcv_pointer;  //数据写入接收缓冲区的下标

    volatile uint8_t  rcv_pkg_len;  //接收到的报文长度
    volatile uint8_t  rcv_pkg_type; //接收到的帧的类型
    volatile uint8_t  crc_h;        //实际计算出的总和的第16~8位
    volatile uint8_t  crc_l;        //实际计算出的总和的第7~0位
}RCS_Common_Ptl_Buffer;

/* 导出定义 --------------------------------------------------------------*/
//链路层协议函数指针
typedef uint8_t (*FNCT_PTL_PACK)      (void*,uint8_t*,uint8_t*,uint8_t);//将数据进行打包的函数指针
typedef uint8_t (*FNCT_PTL_UNPACK_ARR)(void*,uint8_t*,uint8_t*,uint8_t);//解析一串数据的函数指针
typedef uint8_t (*FNCT_PTL_UNPACK_IT) (void*,uint8_t ,uint8_t*);        //解析单个字节的函数指针


/* 导出类型 --------------------------------------------------------------*/
typedef struct comm_ptl_handler{
    //帧格式
    RCS_Common_Ptl        PTL_Param;
    //帧接收状态
    RCS_Common_Ptl_Buffer PTL_Rcv_Buffer;
    RCS_Common_Ptl_Log    PTL_Log;
    //链路层接口
    FNCT_PTL_PACK         PTL_Get;
    FNCT_PTL_UNPACK_ARR   PTL_RcvArr;
    FNCT_PTL_UNPACK_IT    PTL_RcvChr;
}RCS_Comm_Ptl_Handler;



/* 导出函数 ---------------------------------------------------------------*/

//Easy Protocol
void Protocol_Rcv_Easy(uint8_t input_byte,  uint8_t* temp_arr,
                       uint8_t* output_flag,uint8_t* output_arr,
                       RCS_Easy_Ptl* protocol);

void Protocol_Get_Easy(uint8_t* input_arr,
                       uint8_t* output_arr,
                       RCS_Easy_Ptl* protocol);

//LenVerify Protocol
void Protocol_Init_LenVerify(RCS_LenVerify_Ptl* ptl,uint8_t start_byte,uint8_t len,uint8_t* buffer_arr);
uint8_t Protocol_Rcv_LenVerify_IT(void* ptl_handler_void,uint8_t input_byte,uint8_t* output_arr);
uint8_t Protocol_Get_LenVerify(void* ptl_handler_void,uint8_t* output_arr,uint8_t* input_arr,uint8_t len);
void Receive_visiondata (uint8_t input_byte,  uint8_t* temp_arr,
                        uint8_t* output_flag,uint8_t* output_arr,
                        RCS_Easy_Ptl2* protocol);
//BleNet Protocol
uint8_t Protocol_Rcv_RCSBleNet_IT(void* ptl_handler_void,uint8_t input_byte,uint8_t* output_arr);
uint8_t Protocol_Rcv_RCSBleNet_ARR(void* ptl_handler_void,uint8_t* input_byte,uint8_t* output_arr,uint8_t input_len);//todo
uint8_t Protocol_Get_RCSBleNet(void* ptl_handler_void,uint8_t* output_arr,uint8_t* input_arr,uint8_t len);
uint8_t Protocol_Init_RCSBleNet(RCS_Comm_Ptl_Handler* ptl_Handler_p,uint8_t start_byte,uint8_t end_byte,RCS_Common_Ptl_PkgType pkg_type,uint8_t* buffer_arr);

/* 导出内联函数 ----------------------------------------------------------*/
inline void PTL_CHANGE_BIT(uint32_t* target_byte,uint8_t position,uint8_t new_state)
{
    if (new_state==1)
    {
        *target_byte |= 1 << position;
    }
    else
    {
        *target_byte &= ~(1 << position);
    }
}
inline void PTL_SET_BIT(uint32_t* target_byte,uint8_t position)
{
    *target_byte |= 1 << position;
}
inline void PTL_RESET_BIT(uint32_t* target_byte,uint8_t position)
{
    *target_byte &= ~(1 << position);
}
/* .h head --------------------------------------------------------------*/
#endif
