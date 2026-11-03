#include "RCS_AcceCtrl.h"
#include "math.h"

float DacePID_Normal_Ctrl(float target,float current,DacePID_Struct *_hdl)
{
	volatile float output=PID_Normal_Ctrl(target,current,&(_hdl->pid));
	volatile float reval=output;
	volatile uint8_t direction=0;
	
	//-----------------judge direction----------------------------------
	if ((output>=0)&&(_hdl->last_output>=0))
	{
		if (output >= _hdl->last_output) direction=1; //away from 0
		if (output <= _hdl->last_output) direction=-1;//moving to 0
	}
	else if ((output<=0)&&(_hdl->last_output<=0))
	{
		if (output >= _hdl->last_output) direction=-1; //moving to 0
		if (output <= _hdl->last_output) direction=1;  //away from 0
	}
	else
	{
		direction=1;//away from 0
	}
	
	//--------------------dace limit---------------------------------------
	if (direction==1)
	{
		//away from 0 && output inc
		if (output >= _hdl->last_output)
		{
			if ((output - _hdl->last_output)>=_hdl->max_acce) 
				reval=_hdl->last_output+_hdl->max_acce;
		}
		//away from 0 && output dec
		else
		{
			if ((_hdl->last_output - output)>=_hdl->max_acce) 
				reval=_hdl->last_output - _hdl->max_acce;
		}
	}
	else if (direction==-1)
	{
		//moving to 0 && output inc
		if (output >= _hdl->last_output)
		{
			if ((output - _hdl->last_output)>=_hdl->max_dcce) 
				reval=_hdl->last_output+_hdl->max_dcce;
		}
		//moving to 0 && output dec
		else
		{
			if ((_hdl->last_output - output)>=_hdl->max_dcce) 
				reval=_hdl->last_output - _hdl->max_dcce;
		}
	}
	
	//---------------dead zone limit----------------------------
	if (fabsf(target-current)<=_hdl->dead_zone)
	{
		reval=0;
	}
	
	_hdl->last_output=reval;
	return reval;
}