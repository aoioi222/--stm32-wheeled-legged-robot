/*@filename: RCS_BaseMove.h
 *@author     胡兴国       
 *@brief:     底盘速度解算
 *@date: 2023-7-28
*/
#ifndef _RCS_BASEMOVE_H_
#define _RCS_BASEMOVE_H_

#include "rcs.h"		
/***********底盘规格(mm)************/
#define BASE_LENGTH    138.28 //底盘半长
#define BASE_WIDTH			132.15f	//半宽
#define RAD_WHEEL				28.8f	//轮子半径
#define SLOWDOWN_RATE	  19.0f	  //电机减速比
#define SLOWDOWN_RATE_2006  36.0f //2006电机减速比
#define MM_S2R_MIN			0.1772276212907689f	// 轮毂线速度(mm/s)转化成轮子转速(r/min)，计算方法：60/(2*RAD_WHEEL*pi*cos(45°))
#define MAX_MOTOR_SPEED 9000.0f	//3508电机最大转速
#define MAX_2006_SPEED  20000.0f	//2006电机最大转速

void M_BaseMove_Init(void);			//底盘初始化
void BaseMove_P2P(float move_speed_x,float move_speed_y,float rotate_speed);	
void BaseMove_Polar(float direction_angle,float run_speed,float rotate_speed);
void BaseMove_Helm_Around(float Speed1,float Speed2,float Speed3);
void BaseMove_Vision(float move_speed,float rotate_speed);																		
#endif
