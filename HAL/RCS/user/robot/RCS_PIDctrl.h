#ifndef ROBOT_PID_H
#define ROBOT_PID_H

#define LIMIT_ERR 10000

typedef struct {
float P;
float I;
float D;
float Last_Error;			//上次误差
float Limit_Output;  	//输出上限
float Limit_Integral; //积分项上限
float Integral; 			//积分项,存储积分误差	
}PID_Struct;

float PID_Normal_Ctrl(float target, float current, PID_Struct *pid);

#endif
