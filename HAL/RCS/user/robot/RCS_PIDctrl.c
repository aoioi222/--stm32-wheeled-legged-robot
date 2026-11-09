#include "RCS_PIDctrl.h"

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
