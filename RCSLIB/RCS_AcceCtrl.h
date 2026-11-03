#ifndef RCS_ACCECTRL_H_
#define RCS_ACCECTRL_H_

#include "stdint.h"
#include "RCS_PIDctrl.h"

typedef struct dace_pid
{
	PID_Struct pid;
	float      max_acce;
	float      max_dcce;
	float      dead_zone;
	float      last_output;
	int        last_direction;
}DacePID_Struct;

float DacePID_Normal_Ctrl(float target,float current,DacePID_Struct *_hdl);
#endif