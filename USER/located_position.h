/***************************************************************
基于菱形底盘激光定位服务，用于始终将底盘正面朝向篮筐
********************************************************************/
#ifndef located_position_H_
#define located_position_H_

#include "RCS_AGV_BaseMove.h"
#include "Cov_AcceLimit_Ctrl.h"
#include "RCS_dsp.h"
#include <math.h> // 包含数学库，用于tanf等函数
#include "rcs.h"
#include "RCS_Motor_Upctrl.h"
#include "RCS_PIDctrl.h"
#include "JOYSTICK_CTRL.h"
#include "RCS_VESC_MOTOR.h"
#include "Gyro.h"
/* =================车体参数定义=========================================*/
//float xchassis = 786.6; // 车身x轴长度
//float ychassis = 476;  // 车身y轴宽度
//float detx = 10;        // 激光距离x轴原点的距离
//float dety = 10;        // 车身y轴原点的距离
//float lenth = 15000;    // 场地长度
//float width = 8000;     // 场地宽度
//float baske_lenth=15000;
//float basket_width=4000; //球筐坐标
///* =================激光解算车体坐标=========================================*/
//float DT35_LASER1 = 0;
//float DT35_LASER2 = 1;
//float DT35_LASER3 = 2;
//float DT35_LASER4 = 3; // 激光1234，分别是从车体右上角开始逆时针排序
//float rotate_angl = 0; // 车身基于陀螺仪旋转角度,设逆时针旋转的角度为正

//float xwidth; // 横间距
//float ywidth; // 纵间距

//float point_location[2];
//float car_point[2];
//float towards_basket_angle;
//float target_z_from_yaxis;


void calculateWidths() ;
/* =================执行部分函数如下=========================================*/
int is_data_available() ;
void locate_DT35_POINT();//定位DT35激光的交点,交点坐标都相对于左下角坐标原点

void locate_car_point(int situation);

void caculate_towards_angle();


float get_absolute_target_z(int situation);
void towards_basket();//执行底盘转动部分

float Laser1(void);

float Laser2(void);


float get_laser(int situation);



//void towards_basket()//执行底盘转动部分
//{
//   if(is_data_available)//车体已经定位完成,执行转动对准
//	 {
//	 if(rotate_angl>0)//本身车体逆时针，顺时针回正，具体角度正负还在未知
//	 { 
//      rcsAgv0_Update_Angle_0(towards_basket_angle);
//			rcsAgv0_Update_Angle_1(towards_basket_angle);
//	 }
//	 else//本身车身顺时针，逆时针回正，具体角度正负还在未知
//	 { 
//		  rcsAgv0_Update_Angle_0(-towards_basket_angle);
//			rcsAgv0_Update_Angle_1(-towards_basket_angle);
//   }
//   }
//}

//void remote_ctrl_rotate()
//{
//	if(key_flag[0]==0)
//	{
//		towards_basket();
//	}
//}\


#endif