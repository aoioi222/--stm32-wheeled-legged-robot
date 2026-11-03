#ifndef _VALVE_2006_H_
#define _VALVE_2006_H_

#include "RCS_Motor_Upctrl.h"


void Valve_2006_Setup(Motor_Ctrl_Node* node,uint8_t CAN_GROUP_x,uint8_t id,PID_Struct* spd_pid,DacePID_Struct* pos_pid);
uint8_t Valve_2006_Reset(Motor_Ctrl_Node* node);
uint8_t Valve_2006_Set(Motor_Ctrl_Node* node);

#endif