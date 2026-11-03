/*@filename: DoubleRing.h
 *@author     胡兴国       
 *@brief:     存放双环PID
 *@date: 2023-7-30
*/
#ifndef _DOUBLERING_H_
#define _DOUBLERING_H_

#include "rcs.h"		

#define MAX_2006_SPEED  20000.0f	//2006电机最大转速

//float DoubleRing_Float_Ctrl(float target_angle,float now_angle,float now_speed,PID_Struct* DR_Angle_pid,PID_Struct* DR_Speed_pid);
void Run_Helm_Angle(float target_angle);
void Run_Helm_Angle_2006(float target_angle1,float target_angle2,float target_angle3);
void Run_Angle_2006(float target_angle1,float target_angle2,float target_angle3,float target_angle4);
void Run_Angle_3508(float target_angle1,float target_angle2,float target_angle3,float target_angle4);
void Run_Angle_6020(float target_angle1,float target_angle2,float target_angle3,float target_angle4);
void Run_Helm_Speed_2006(float rotate_speed1,float rotate_speed2,float rotate_speed3);
void Run_Speed_3508(float speed1,float speed2,float speed3,float speed4);
void Run_Speed_2006(float speed1,float speed2,float speed3,float speed4);
#endif
