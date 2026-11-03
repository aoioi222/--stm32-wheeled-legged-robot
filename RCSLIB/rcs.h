/**
 * @filename: rcs.h
 * @brief:模板代码公用大头文件,RCSers 自写的常用功能封装
 * @date:     2012-08-17          柯国霖
 * @changlog: 2012-08-19  21:54   高震宙   添加了"RCS_pwm.h"的包含.
 * @changlog: 2019-07-12	09:03   闫锐	   添加了"RCS_OLED.h"、"RCS_OLED_ASCII.h"和"RCS_ADC.h"的包含并将定义整理归类
 * @changlog: 2020-11-21  20:38		陈志伟	 重新整理
 * @changlog: 2023-7-28   00:59   胡兴国   重新整理
 * @changlog: 2024-2-20   15:54   陈煜楷   规范代码使用方式
*/

/* ------------h header-------------------------------------*/
#ifndef _RCS_H_
#define _RCS_H_

/* -------------常用常数定义---------------------------------*/
#define pi 				3.14159265357f        //π
#define DEG2RAD         1.74532925199433e-2f  //角度变换成弧度     
#define RAD2DEG         5.72957795130823e1f		//弧度变换成角度
#define FLOAT_ZERO      1e-10f                //浮点数相等的容差
#define G               9.8f                  //重力加速度
#define DEG_TO_RAD(deg) ((deg) * pi / 180.0f) //角度转弧度宏
#define RAD_TO_DEG(rad) ((rad) * 180.0f / pi) //弧度转角度宏

/* --------------外设管理-------------------------------------*/
//定时器外设
#define HARD_STAMPER_TIMER   TIM9
#define CAN1_BUFFER_TIMER    TIM7  //驱动CAN1环形队列的定时器   
#define CAN2_BUFFER_TIMER    TIM6  //驱动CAN2环形队列的定时器  
#define GPS_X_TIM			 TIM1
#define GPS_Y_TIM			 TIM4

//外部中断

/* -------------中断优先级管理--------------------------------*/
#define RCS_CAN1_BUFFER_PRI 0x12
#define RCS_CAN2_BUFFER_PRI 0x13
#define LASER_PRI           0x02
#define GYRO_PRI            0x10
#define A1_PRI            0x10
#define ACTION_PRI          0x12
#define VISION_PRI          0x02
#define BLUETOOTH_PRI       0x01	
#define STAMP_PRI           0x01

#define R1_CALLBACK_PRI     0x02

/* --------------软件定时器位号管理----------------------------*/
#define STIM_SCARA_PICK     1
#define STIM_UPCTRL_R1      2

/* -----------------头文件------------------------------------*/
//C语言公共库
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
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
#include "RCS_Filter.h"        //滤波算法
#include "DoubleRing.h"        //双环PID算法
#include "FSM_Lite.h"          //有限状态机
#include "RCS_DataStructure.h" //常用数据结构
#include "RCS_AcceCtrl.h"

//RCSLIB模块驱动层(CAN网络功能)
#include "RCS_CAN_Interface.h" //CAN总线交互接口
#include "RCS_MOTOR.h"         //CAN总线RM电机控制
#include "RCS_VESC_MOTOR.h"    //CAN总线VESC电调控制

//RCSLIB模块驱动层(OLED屏幕)
#include "RCS_OLED.h"          //OLED驱动
#include "RCS_ASCII.h"         //OLED字库
#include "RCS_Interface.h"     //OLED接口

//RCSLIB模块驱动层(常用模块)
#include "RCS_Pwm.h"           //PWM波输出
#include "RCS_Remote.h"        //RM手柄驱动(已停产)
#include "Joystick_Ctrl.h"     //自制手柄驱动
#include "RCS_Encoder.h"       //编码器驱动
#include "Laser_Ranging.h"     //摩天激光测距驱动
#include "Gyro.h"              //维特智能陀螺仪驱动
#include "GPS_Action.h"        //东大全场定位驱动
#include "G431_GPS.h"

//RCSLIB模块驱动层(底盘控制)
#include "RCS_BaseMove.h"    //机器人全向轮底盘驱动
#include "RCS_AGV_BaseMove.h"//机器人舵轮底盘驱动

//RCSLIB机器人功能层(机器人定位)
#include "Laser_Position.h"     //融合定位算法

//RCSLIB机器人功能层(机器人控制)
#include "RCS_Ctrl_BaseMove.h"  //机器人底盘电机控制
#include "RCS_Motor_Upctrl.h"   //机器人上层电机控制
#include "RCS_Actor_Upctrl.h"   //机器人上层气缸/舵机等控制
#include "RCS_Debug.h"          //常用调试功能
#include "FSM.h"                //有限状态机
#include "RCS_RTOS.h"           //常用RTOS功能
#include "RCS_Ptl.h"            //软件通信协议

//业务逻辑层(机器人业务接口)
#include "Vision.h"   //视觉串口
#include "Ch_Ctrl.h"    //机器人全自动底盘路径
#include "Up_Ctrl.h"  //机器人上层机构功能实现
#include "R2_Upctrl.h"//R2上层机构功能实现

//业务逻辑层(任务配置)
#include "app_cfg.h"  //配置各任务的堆栈大小和优先级
#include "MainTask.h" //机器人初始化
#include "GpsTask.h"      //机器人定位(码盘)

#include "Core407_RCS12_PinMapping.h"
#include "Core407_RCS12_Debug.h"

/* ---------end of .h header-------------------------------*/
#endif
