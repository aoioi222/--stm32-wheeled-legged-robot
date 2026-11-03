#ifndef _R2_UPCTRL_H_
#define _R2_UPCTRL_H_

#include "rcs.h"

struct valve
{
	uint8_t CAN_Group;
	uint8_t id;
	float valve_speed;
	
	int count;
};

typedef struct valve valve_para;

uint16_t Valve_Open(valve_para* Valve_data);
uint16_t Valve_Close(valve_para* Valve_data);
void valve_fsm(valve_para* Valve_data);
uint16_t Valve_Open(valve_para* Valve_data);
uint16_t Valve_Close(valve_para* Valve_data);

extern	valve_para Valve_data_1;
extern	valve_para Valve_data_2;

#endif
