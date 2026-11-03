/*@filename: GPS.h
 *@author     胡兴国      
 *@brief:    全场定位电机版和自制版
 *@date: 2023-7-27
*/
/* --------.h head------------------------------*/
#ifndef _GPS_H_
#define _GPS_H_

/* --------包含文件------------------------------*/
#include "rcs.h"	
#include "GPS_DSP.h"
/* --------配置宏--------------------------------*/
//#define RCS16_GPS
#define RCS13_GPS

/* --------私有宏------------------------------*/
#ifdef RCS16_GPS


#endif













#ifdef RCS13_GPS
#define ENCODER_X_DIR	 1					//编码器计数方向
#define ENCODER_Y_DIR	 -1			

#define X_ROTATE_ARM		61.3			//码盘X轮旋转臂长mm
#define Y_ROTATE_ARM		61.3			//码盘Y轮旋转臂长mm

#define	CYCLE_LINES			1024			//c,编码器线数(!!!!!!!!!!更改过，原本是1024)
#define	FREQUENCY_DIV		4					//q,分频数
#define	GPS_WHEEL_RADIUS	29.0f		//R,轮子半径mm
#define TRANSFER_CONST		0.038963112f//(1024)				//C,编码器位置/mm转换常数,C=2*pi*R/(q*c)

#define TRANSFORM_DZ	1.91//0.785398163f//0.0f//-2.35//-4.712389 //-2.35//-2.443461//-2.35619449019f//49019f// 0.785398163f//-2.35619449019f//-3.926990816987f          //码盘定位角度转换
//逆时针转+ 顺时针-


#define TRANSFORM_DX	 -27.1f//-1.695 //-27.1f     //-27.1f            //定位中心与车中心的X方向距离
#define TRANSFORM_DY		0.0f//-1.695//0.0f                 //定位中心与车中心的Y方向距离//

#define ANGLE_DX	  2.35619449019f            //电机1、2参数
#define ANGLE_DY    2.35619449019f         
#define TRANSFORM_AX	  0//4050.0f                 //电机1、2参数
#define TRANSFORM_AY	  3600.0f//3600.0f
#define TRANSFORM_X	       0.68288f             //x编码器标定参数值
#define TRANSFORM_Y	       0.68288f             //y编码器标定参数值

void GPS_Init(void);
int32_t gps_cycle_test_x(void);
int32_t gps_cycle_test_y(void);
float Get_GPS_X(void);
float Get_GPS_Y(void);
float Get_GPS_Z(void);
float Get_GPS_Ori_X(void);
float Get_GPS_Ori_Y(void);
float Get_GPS_Raw_X(void);
float Get_GPS_Raw_Y(void);
void X_Cycle(uint8_t flag);
void Y_Cycle(uint8_t flag);

void Set_GPS_X(float laser_x);
void Set_GPS_Y(float laser_y);
float Get_LaserGPS_X(void);
float Get_LaserGPS_Y(void);


#endif


#endif