/*@filename: RCS_PIDctrl.c
 *@author     胡兴国       
 *@brief:     PID算法
 *@date: 2023-7-27
*/

#include "RCS_PIDctrl.h"

/*-------------常规底盘PID------------*/

/**
	@name: PID_Init
	@brief: PID初始化
**/
void PID_Init(void)
{
		
}

PID_Struct PID_Get_RM3508_Speed_Pid(void)
{
	PID_Struct speed_pid;
	//3508电机速度环PID
	speed_pid.P = 8.3;
	speed_pid.I = 0.25;
	speed_pid.D = 0.38;
	speed_pid.Last_Error = 0;			
	speed_pid.Limit_Output = 16000;  	
	speed_pid.Limit_Integral = LIMIT_ERR; 
	speed_pid.Integral = 0; 			
	return speed_pid;
}

PID_Struct PID_Get_RM3508_Angle_Pid(void)
{
	PID_Struct angle_pid;
	//3508电机角度环PID
	angle_pid.P = 20;
	angle_pid.I = 0;
	angle_pid.D = 0;
	angle_pid.Last_Error = 0;			
	angle_pid.Limit_Output = 1000;  	
	angle_pid.Limit_Integral = LIMIT_ERR; 
	angle_pid.Integral = 0; 			
	return angle_pid;
}

PID_Struct PID_Get_RM2006_Speed_Pid(void)
{
	PID_Struct speed_pid;
	//2006电机速度环PID
	speed_pid.P = 8.3;
	speed_pid.I = 0.25;
	speed_pid.D = 0.38;
	speed_pid.Last_Error = 0;			
	speed_pid.Limit_Output = 10000;  	
	speed_pid.Limit_Integral = LIMIT_ERR; 
	speed_pid.Integral = 0; 			
	return speed_pid;
}

PID_Struct Get_Valve_Speed_Pid(void)
{
	PID_Struct valve_speed_pid;			//阀门速度环

    valve_speed_pid.P = 5;
	valve_speed_pid.I = 0;
	valve_speed_pid.D = 0;
	valve_speed_pid.Last_Error = 0;			
	valve_speed_pid.Limit_Output = 5000;//LIMIT_SPEED;  	
	valve_speed_pid.Limit_Integral = LIMIT_ERR; 
	valve_speed_pid.Integral = 0; 
	return valve_speed_pid;
}

PID_Struct Get_Valve_Angle_Pid(void)
{
	PID_Struct valve_angle_pid;			//阀门角度环
	
	valve_angle_pid.P = 500;
	valve_angle_pid.I = 0;
	valve_angle_pid.D = 0;
	valve_angle_pid.Last_Error = 0;			
	valve_angle_pid.Limit_Output = 1000;//LIMIT_SPEED;  	
	valve_angle_pid.Limit_Integral = LIMIT_ERR; 
	valve_angle_pid.Integral = 0; 
	return valve_angle_pid;
}


/**
	@name: PID_Normal_Ctrl
	@brief: 通用PID函数
	@param:float target 				目标值
	@param:float current 				当前值
	@param:PID_Struct *_pid   	PID
	@return:float								浮点值
**/
float PID_Normal_Ctrl(float target,float current,PID_Struct *_pid)
{
		float err;
		float	different_err;
    float output;         
    
    err =target - current;														//误差
    _pid->Integral += err;														//积分误差
    different_err = err - _pid->Last_Error;						//微分误差
    
    if(_pid->Integral >= _pid->Limit_Integral)        //积分限幅
        _pid->Integral = _pid->Limit_Integral;
    if(_pid->Integral <= -_pid->Limit_Integral)
        _pid->Integral =-_pid->Limit_Integral;
    
    output = _pid->P * err + _pid->I * _pid->Integral + _pid->D * different_err;			//PID整合
    
    if(output >= _pid->Limit_Output)          			//输出限幅
        output = _pid->Limit_Output;
    if(output <= - _pid->Limit_Output)
        output = - _pid->Limit_Output;
    
		_pid->Last_Error = err;													//记录上次误差
		return output;
}

/**
	@name: PID_ANGLE1_Ctrl
	@brief: 角度控制PID函数
	@param:float target 				目标值
	@param:float current 				当前值
	@param:PID_Struct *_pid   	PID
	@return:float								浮点值
**/
float PID_Angle_Ctrl(float target,float current,PID_Struct *_pid)
{
		float err;
		float	different_err;
    float output;         
    
    err =target - current;														//误差
    _pid->Integral += err;														//积分误差
    different_err = err - _pid->Last_Error;						//微分误差
    
    if(_pid->Integral >= _pid->Limit_Integral)        //积分限幅
        _pid->Integral = _pid->Limit_Integral;
    if(_pid->Integral <= -_pid->Limit_Integral)
        _pid->Integral =-_pid->Limit_Integral;
    
    output = _pid->P * err + _pid->I * _pid->Integral + _pid->D * different_err;			//PID整合
    
    if(output >= _pid->Limit_Output)          			//输出限幅
        output = _pid->Limit_Output;
    if(output <= - _pid->Limit_Output)
        output = - _pid->Limit_Output;
    
		_pid->Last_Error = err;													//记录上次误差
		return output;
}

/**
	@name: PID_Motor_Ctrl
	@brief: 电机PID控制函数
	@param:float target 				目标值
	@param:float current 				当前值
	@param:PID_Struct *_pid   	PID
	@return:int32_t							电机电流值
**/
int32_t PID_Motor_Ctrl(float target,float current,PID_Struct *_pid)
{
		int32_t err;						
		int32_t	different_err;	
    int32_t output;         
    
    err =target - current;														//误差
    _pid->Integral += err;														//积分误差
    different_err = err - _pid->Last_Error;						//微分误差
    
    if(_pid->Integral >= _pid->Limit_Integral)        //积分限幅
        _pid->Integral = _pid->Limit_Integral;
    if(_pid->Integral <= -_pid->Limit_Integral)
        _pid->Integral =-_pid->Limit_Integral;
    
    output = _pid->P * err + _pid->I * _pid->Integral + _pid->D * different_err;			//PID整合
    
    if(output >= _pid->Limit_Output)          			//输出限幅
        output = _pid->Limit_Output;
    if(output <= - _pid->Limit_Output)
        output = - _pid->Limit_Output;
    
		_pid->Last_Error = err;													//记录上次误差
		return output;
}

/**
	@name:  DoubleRing_Float_Ctrl
	@brief: 浮点双环pid控制
**/
float DoubleRing_Float_Ctrl(float target_angle,float now_angle,float now_speed,PID_Struct* DR_Angle_pid,PID_Struct* DR_Speed_pid)
{
	return PID_Normal_Ctrl(PID_Normal_Ctrl(target_angle,now_angle,DR_Angle_pid),now_speed,DR_Speed_pid);
}

