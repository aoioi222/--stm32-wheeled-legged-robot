#include "Valve_2006.h"


void Valve_2006_Setup(Motor_Ctrl_Node* node,uint8_t CAN_GROUP_x,uint8_t id,PID_Struct* spd_pid,DacePID_Struct* pos_pid)
{
	*spd_pid=PID_Get_RM3508_Speed_Pid();
	MotorNode_Init_C610(CAN_GROUP_x,id,node);
	MotorNode_Add_SpeedPid(node,spd_pid);
	MotorNode_Add_DaceAnglePid(node,pos_pid);
	
	node->Max_Current=5000;
	MotorNode_Add(CAN_GROUP_x,node);
}

uint8_t Valve_2006_Set(Motor_Ctrl_Node* node)
{
	return MotorNode_Update_AngleFull(0.0f,node);
}
uint8_t Valve_2006_Reset(Motor_Ctrl_Node* node)
{
	return MotorNode_Update_AngleFull(3300.0f,node);
}




