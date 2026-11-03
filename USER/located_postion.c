/***************************************************************
基于菱形底盘激光定位服务，用于始终将底盘正面朝向篮筐
********************************************************************/
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
#include "RCS_ADC.h"
/* =================车体参数定义=========================================*/
float xchassis = 786.6; // 车身x轴长度
float ychassis = 476;  // 车身y轴宽度
float detx = 10;        // 激光距离车身最右点的距离
float dety = 10;        // 激光距离车身最下点的距离
float lenth = 15000;    // 场地长度
float width = 8000;     // 场地宽度
float basket_lenth=15000;
float basket_width=4000; //球筐坐标
/* =================激光解算车体坐标=========================================*/
float DT35_LASER1 ;
float DT35_LASER2 ;
float DT35_LASER3 ;
float DT35_LASER4 ; // 激光1234，分别是从车体左下角开始顺时针排序
float rotate_angl ; // 车身基于陀螺仪旋转角度,设逆时针旋转的角度为正

float xwidth; // 横间距
float ywidth; // 纵间距

float point_location[2];
float car_point[2];
float towards_basket_angle;
float target_z_from_yaxis;

extern float mm_dt1;
extern float mm_dt2;
extern float mm_dt3;
extern float mm_dt4;

PID_Struct gyro_z_pid_podtion = 
{
    .P = 5.0,               // 比例系数
    .I = 0,                 // 积分系数
    .D = 0,              // 微分系数
    .Last_Error = 0.0,      // 上一次的误差
    .Limit_Output = 10000,   // 输出上限（假设电机最大转速为10弧度/秒）
    .Limit_Integral = LIMIT_ERR,// 积分项上限
    .Integral = 0.0         // 积分项
};

void calculateWidths() 
{
    xwidth = xchassis/2 - detx;
    ywidth = ychassis/2 - dety;
}
float get_laser(int situation)
{
	switch(situation){
		case 1://左下和左上激光
			DT35_LASER1=mm_dt1;
			DT35_LASER2=mm_dt2;
			break;
		case 2://左上和右上激光
			DT35_LASER1=mm_dt3;
			DT35_LASER2=mm_dt2;
			break;
		case 3://右上和右下激光
			DT35_LASER1=mm_dt3;
			DT35_LASER2=mm_dt4;
			break;
		case 4://右下和左下激光
			DT35_LASER1=mm_dt1;
			DT35_LASER2=mm_dt4;
			break;
		
	}
}
/* =================执行部分函数如下=========================================*/

void locate_DT35_POINT(int situation)//定位DT35激光的交点,交点坐标都相对于左下角坐标原点
{  
//	if(is_data_available)//车体已经定位完成
//	{
	 float now_angle=fabs(Get_Gyro_Z());//就是要保证是在0-2/pi之间的角度
	 float point_x;
	 float point_y;
	 float distance_x;
	 float distance_y;
	 int example = 1;
	 if(now_angle > 90){
		 example = 2;
	 }
	 switch(example){
		 case 1://车身旋转角度没超过90
			 distance_x=DT35_LASER1+xwidth;
			 distance_y=DT35_LASER2+ywidth;
			 break;
		 case 2://超过90
			 distance_x=DT35_LASER2+ywidth;
			 distance_y=DT35_LASER1+xwidth;
			 situation+1;
			 break;
	//	 }
	}
	 switch(situation){//此处的situation和get_laser的situation一样，都是按照从左下角开始顺时针开始算
		 case 1://激光打在左和上
			point_x = distance_x*cos(now_angle*pi/180);
			point_y = lenth - distance_y*cos(now_angle*pi/180);
			break;
		 case 2://激光打在右和上
			point_x = width - distance_x*cos(now_angle*pi/180);
			point_y = lenth - distance_y*cos(now_angle*pi/180);
			break;
		 case 3://激光打在右和下
			point_x = distance_x*cos(now_angle*pi/180);
			point_y = distance_y*cos(now_angle*pi/180);
			break;
		 case 4://激光打在左和下
			point_x = width - distance_x*cos(now_angle*pi/180);
			point_y = distance_y*cos(now_angle*pi/180);
			break;
	 }
	 car_point[0]=point_x;
	 car_point[1]=point_y;
}


void caculate_towards_angle()//计算需要旋转的角度
{
	 float now_angle=Get_Gyro_Z();//就是要保证是在0-2/pi之间的角度
	 float x_distance=fabs(car_point[0]-basket_width);
	 float y_distance=fabs(car_point[1]-basket_lenth);
	 float difference_angle=atan2f(x_distance,y_distance)*180/pi;
	if(car_point[0]<basket_width)//此处计算基于陀螺仪逆时针旋转为正，顺时针旋转为负进行计算的
	 {
		 towards_basket_angle = - (now_angle + difference_angle);
	 }
	 else
	 {
		 towards_basket_angle = - (now_angle - difference_angle);
	 } 
}


float get_absolute_target_z(int situation)
{  
	calculateWidths();
	get_laser(situation);
	locate_DT35_POINT(situation);
	caculate_towards_angle();
	return towards_basket_angle;
}




void towards_basket(int situation)//执行底盘转动部分
{
	float wheel_z_speed = PID_Normal_Ctrl(get_absolute_target_z(situation),Get_Gyro_Z(),&gyro_z_pid_podtion);
	AGV_Re_Center_Move_v3_3(0,0,-wheel_z_speed);
}


//void towards_basket()//执行底盘转动部分
//{
//   if(is_data_available)//车体已经定位完成,执行转动对准
//	 {
//	 if(now_angle>0)//本身车体逆时针，顺时针回正，具体角度正负还在未知
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
//}