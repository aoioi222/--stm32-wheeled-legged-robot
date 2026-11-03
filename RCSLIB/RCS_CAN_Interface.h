/* --------.h head------------------------------------*/
#ifndef RCS_CAN_INTERFACE_H_
#define RCS_CAN_INTERFACE_H_

/* --------头文件--------------------------------------*/
//C语言公共库
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

//BSP与中间件
#include "matrix_functions.h" //ARM提供的矩阵运算库(硬)
#include "stm32f4xx.h"        //STM32寄存器定义
#include "stm32f4xx_conf.h"   //STM32标准库头文件
#include "bsp.h"              //uCOS底层组件
#include "includes.h"         //uCOS必备组件
#include "delay.h"            //延时函数
#include "matrix.h"           //Matrix_Hub矩阵运算库(软)

//RCSLIB硬件抽象层
#include "RCS_HAL.h"           //寄存器抽象
#include "RCSLIB_inc.h"        //外设硬件抽象

//RCSLIB模块驱动层(算法层)
#include "RCS_DataStructure.h" //常用数据结构

/* --------导出宏--------------------------------------*/
typedef void (*CAN_HANDLER_VOID)(RCS_CAN2B_DATA_FM_RX);//CAN服务函数指针类型

/* --------配置宏--------------------------------------*/
#define CAN_BUFFER_ALL_FUNCTION   //把所有CAN发送函数配置成软件buffer发送
#define CAN_DEBUG_PRINT           //开启调试输出    
#define CAN_HANDLER_MAX_LEN 5     //最大有多少个CAN服务可以被注册    
#define CAN_PERI_COUNT      2     //有几路CAN外设

#define CAN_SOFT_BUFFER_LEN  60        //环队列长度
#define CAN_BUFFER_TIMER_FRQ 1000000   //驱动环队列的定时器的频率(1us)
#define CAN_BUFFER_SEND_FRQ  150       //定时器经过几个count之后将队头送入邮箱 (50us)(<150us时，控制6020会有卡顿)


/* ---------导出函数-----------------------------------*/

//配置CAN外设以及CAN输出引脚
void RCS_CANx_Config_With_Buffer(RCS_PIN_CAN CANx_MAP, uint32_t _baudRate);
//为CAN外设添加接收服务句柄
void RCS_CANx_Add_Handler(RCS_CAN_T *CANx,CAN_HANDLER_VOID Handler);

//发送标准帧
void RCS_CANx_Send_STDID(RCS_CAN_T *CANx,uint32_t Id, uint8_t Length, uint8_t* sendData);
//发送扩展帧
void RCS_CANx_Send_EXTID(RCS_CAN_T *CANx,uint32_t Id, uint8_t Length, uint8_t* sendData);
//发送标准帧/扩展帧
void RCS_CANx_Send(RCS_CAN_T *CANx,uint32_t Id, uint8_t Length, uint8_t* sendData);

//获取当前的服务数量
uint8_t RCS_Get_CAN_Service_Len(RCS_CAN_T *CANx);
//获取CANx上第ID个被加入的服务，ID从0开始
CAN_HANDLER_VOID RCS_Get_CAN_Service_Handler(RCS_CAN_T *CANx,uint8_t ID);
//判断服务是否已经在队列中
uint8_t RCS_Judge_CAN_Service(RCS_CAN_T *CANx,CAN_HANDLER_VOID Handler);

void CAN1_Rx_Service_Handler(void);
void CAN2_Rx_Service_Handler(void);


/* --------.h head------------------------------------*/
#endif