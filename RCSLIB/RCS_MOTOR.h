/*@filename: RCS_MOTOR.h
 *@author     胡兴国、陈煜楷       
 *@brief:     CAN网络电机控制
 *@date: 2023-7-28
 *@data: 2023-11-15
*/
/* ----------.h head-----------------------------------*/
#ifndef _RCS_MOTOR_H_
#define _RCS_MOTOR_H_

/* -----------头文件------------------------------------*/
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
#include "RCS_PIDctrl.h"       //PID算法
#include "RCS_DataStructure.h" //常用数据结构

//RCSLIB模块驱动层(CAN网络功能)
#include "RCS_CAN_Interface.h" //CAN总线交互接口		

/* ----------配置宏-------------------------------------*/
#define RCS_MOTOR_DEBUG        //启用调试输出
#define MAX_RMESC_ON_CAN  13   //RM电调回传报文不会超过12个
#define BATTERY_VOLTAGE   24

#define M2006_SLD_RATE    36.0f
#define M3508_SLD_RATE    19.0f

/* ------------私有宏--------------------------------------*/
#define ENCODER2ANGLE	  0.0439453125 //360/8192

typedef enum {
	RM_FRONT=0x200,          //ID1~4 电流
	RM_BEHIND=0x1FF,         //ID5~8 电流
	RM_GM6020_V_FRONT=0x1FF, //ID1~4 (6020电压)
	RM_GM6020_V_BEHIND=0x2FF,//ID5~7 (6020电压)
	RM_GM6020_FRONT=0x1FE,   //ID1~4 (6020电流)
	RM_GM6020_BEHIND=0x2FE,  //ID5~8 (6020电流)
} RM_PACKET_ID;              //RM电机CAN数据包ID

/* -----------导出函数(新接口)-------------------------------------*/
//初始化
void RMMotor_CAN_Init(CAN_TypeDef* CANx);

//更新输出值
void RM3508_Update_Current(CAN_TypeDef* CANx,uint8_t id,int16_t current);
void GM6020_Update_Current(CAN_TypeDef* CANx,uint8_t id,int16_t current);
void GM6020_Update_Voltage(CAN_TypeDef* CANx,uint8_t id,int16_t volt);

//执行输出值
void RM3508_Excute_Front_Current(CAN_TypeDef* CANx);
void RM3508_Excute_Behind_Current(CAN_TypeDef* CANx);
void GM6020_Excute_Front_Current(CAN_TypeDef* CANx);
void GM6020_Excute_Behind_Current(CAN_TypeDef* CANx);
void GM6020_Excute_Front_Voltage(CAN_TypeDef* CANx);
void GM6020_Excute_Behind_Voltage(CAN_TypeDef* CANx);

//获取工作状态(末端转子)
float RM3508_Get_Angle_Deg(CAN_TypeDef* CANx,uint8_t id);
float GM6020_Get_Angle_Deg(CAN_TypeDef* CANx,uint8_t id);
int16_t RM3508_Get_Speed_Rpm(CAN_TypeDef* CANx,uint8_t id);
int16_t GM6020_Get_Speed_Rpm(CAN_TypeDef* CANx,uint8_t id);

//获取工作状态(输出轴)
float RM3508_Get_Angle_Deg_Sld(CAN_TypeDef* CANx,uint8_t id);//角度输出轴
float RM2006_Get_Angle_Deg_Sld(CAN_TypeDef* CANx,uint8_t id);
float RM3508_Get_Angle_Rad_Sld(CAN_TypeDef* CANx,uint8_t id);//弧度输出轴
float RM2006_Get_Angle_Rad_Sld(CAN_TypeDef* CANx,uint8_t id);
float RM3508_Get_Speed_Rpm_Sld(CAN_TypeDef* CANx,uint8_t id);//转速输出轴
float RM2006_Get_Speed_Rpm_Sld(CAN_TypeDef* CANx,uint8_t id);

/* -----------导出函数(老接口)-------------------------------------*/

//电机参数初始化
void Motor_Init(void);		
void Motor_Init2(void);		

//RM电机控制命令
void Motor_Send2(int16_t ID1_Current, int16_t ID2_Current, int16_t ID3_Current, int16_t ID4_Current);	
void Motor_Send(int16_t ID1_Current, int16_t ID2_Current, int16_t ID3_Current, int16_t ID4_Current);
void Motor_Send_ADD(int16_t ID5_Current, int16_t ID6_Current, int16_t ID7_Current, int16_t ID8_Current);	
void Motor_Send2_ADD(int16_t ID5_Current, int16_t ID6_Current, int16_t ID7_Current, int16_t ID8_Current);
void GM6020_Send(int16_t ID1_Current,int16_t ID2_Current,int16_t ID3_Current,int16_t ID4_Current);
void GM6020_Send_ADD(int16_t ID5_Current,int16_t ID6_Current,int16_t ID7_Current);
void GM6020_Send2(int16_t ID1_Current,int16_t ID2_Current,int16_t ID3_Current,int16_t ID4_Current);
void GM6020_Send2_ADD(int16_t ID5_Current,int16_t ID6_Current,int16_t ID7_Current);
void Motor_Send_6020(int16_t ID1_Voltage, int16_t ID2_Voltage, int16_t ID3_Voltage, int16_t ID4_Voltage);
void Motor_Send_6020_ADD(int16_t ID5_Voltage, int16_t ID6_Voltage, int16_t ID7_Voltage);
void Motor_Send2_6020(int16_t ID1_Voltage, int16_t ID2_Voltage, int16_t ID3_Voltage, int16_t ID4_Voltage);
void Motor_Send2_6020_ADD(int16_t ID5_Voltage, int16_t ID6_Voltage, int16_t ID7_Voltage);

//RM电机获取工作状态
float 	Get_Motor_Float_Angle(uint8_t Motor_ID);//CAN1电机浮点机械角度,单位deg
int32_t Get_Motor_Integer_Angle(uint8_t Motor_ID);//整形,单位deg
int16_t Get_Motor_Speed(uint8_t Motor_ID);//CAN1电机转速,单位RPM
float 	Get_Motor_Float_Angle2(uint8_t Motor_ID);//CAN2浮点机械角度，单位deg
int32_t Get_Motor_Integer_Angle2(uint8_t Motor_ID);
int16_t Get_Motor_Speed2(uint8_t Motor_ID);	//CAN2电机转速,单位RPM
int16_t Get_Motor_Power(uint8_t Motor_ID);
int16_t Get_Motor_Power2(uint8_t Motor_ID);
float Get_GM6020_Float_Angle(uint8_t Motor_ID);
float Get_GM6020_Float_Angle2(uint8_t Motor_ID);
int16_t Get_GM6020_Speed(uint8_t Motor_ID);
int16_t Get_GM6020_Speed2(uint8_t Motor_ID);
float Get_Motor_Rad_Angle_M3508(uint8_t Motor_ID);
float Get_Motor_Rad_Angle_M2006(uint8_t Motor_ID);
float Get_Motor_Rad_Angle2_M3508(uint8_t Motor_ID);
float Get_Motor_Rad_Angle2_M2006(uint8_t Motor_ID);

//服务接口
void RM_Motor_Data_Filter1(RCS_CAN2B_DATA_FM_RX rx_message);
void RM_Motor_Data_Filter2(RCS_CAN2B_DATA_FM_RX rx_message);

#endif



//#define _RCS_MOTOR_H_

//#include "rcs.h"		

//void Motor_Init(void);		//电机初始化
//void Motor_Init2(void);			
//void Motor_Send2(int16_t ID1_Current, int16_t ID2_Current, int16_t ID3_Current, int16_t ID4_Current);	//给3508发送数据
//void Motor_Send(int16_t ID1_Current, int16_t ID2_Current, int16_t ID3_Current, int16_t ID4_Current);
//void Motor_Send_ADD(int16_t ID5_Current, int16_t ID6_Current, int16_t ID7_Current, int16_t ID8_Current);	//给2006发送数据
//void Motor_Send2_ADD(int16_t ID5_Current, int16_t ID6_Current, int16_t ID7_Current, int16_t ID8_Current);
//void Motor_Send_6020(int16_t ID1_Current, int16_t ID2_Current, int16_t ID3_Current, int16_t ID4_Current);
//void Motor_Send2_6020(int16_t ID1_Current, int16_t ID2_Current, int16_t ID3_Current, int16_t ID4_Current);
//float 	Get_Motor_Float_Angle(uint8_t Motor_ID);//CAN1电机浮点机械角度,单位deg
//int32_t Get_Motor_Integer_Angle(uint8_t Motor_ID);//整形,单位deg
//int16_t Get_Motor_Speed(uint8_t Motor_ID);//CAN1电机转速,单位RPM
//float 	Get_Motor_Float_Angle2(uint8_t Motor_ID);//CAN2浮点机械角度，单位deg
//int32_t Get_Motor_Integer_Angle2(uint8_t Motor_ID);
//int16_t Get_Motor_Speed2(uint8_t Motor_ID);	//CAN2电机转速,单位RPM

////void Current_Data_Takepart(int16_t Current_Data, uint8_t *High_Data, uint8_t *Low_Data);
////数据拆解为高八位和低八位
//__inline void Current_Data_Takepart(int16_t Current_Data, uint8_t *High_Data, uint8_t *Low_Data)
//{
//	*High_Data = (Current_Data & 0xff00) >> 8;
//	*Low_Data = (Current_Data & 0x00ff);
//}

//#endif
