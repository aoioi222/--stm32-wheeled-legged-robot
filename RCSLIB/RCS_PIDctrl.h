/*@filename: RCS_PIDctrl.h
 *@author     胡兴国     
 *@brief:     PID算法
 *@date: 2023-7-27
*/
#ifndef _RCS_PIDCTRL_H_
#define _RCS_PIDCTRL_H_

#include <stdint.h>
//#include "rcs.h" 宏想要跨头文件调用的话，不能include大头文件		 



#define LIMIT_ERR						10000				//积分误差限幅
#define	LIMIT_SPEED						16000				//输出电流限幅

//定义PID结构体
typedef struct {
float P;
float I;
float D;
float Last_Error;			//上次误差
float Limit_Output;  	//输出上限
float Limit_Integral; //积分项上限
float Integral; 			//积分项,存储积分误差	
}PID_Struct;


typedef struct{
    float Cur_Spd;
    float Cur_Dis;
    float Target_Spd;
    float Target_Dis;
    PID_Struct Spd_Pid;
    PID_Struct Dis_Pid;
}PID_F32_CTRL_Struct;


void PID_Init(void);		//PID初始化
PID_Struct PID_Get_RM3508_Speed_Pid(void);
PID_Struct PID_Get_RM3508_Angle_Pid(void);
PID_Struct PID_Get_RM2006_Speed_Pid(void);
PID_Struct Get_Valve_Speed_Pid(void);
PID_Struct Get_Valve_Angle_Pid(void);

//兼容PID
float PID_Normal_Ctrl(float target,float current,PID_Struct *_pid);
float PID_Angle_Ctrl(float target,float current,PID_Struct *_pid);
int32_t PID_Motor_Ctrl(float target,float current,PID_Struct *_pid);
float DoubleRing_Float_Ctrl(float target_angle,float now_angle,float now_speed,PID_Struct* DR_Angle_pid,PID_Struct* DR_Speed_pid);


#endif