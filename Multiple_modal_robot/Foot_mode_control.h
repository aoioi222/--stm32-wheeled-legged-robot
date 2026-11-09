#ifndef FOOT_MODE_CONTROL_H
#define FOOT_MODE_CONTROL_H

#include <stdbool.h>

#define FOOT_JOINT_COUNT 12

void Foot_Mode_Init(void);
bool Foot_Mode_Control(const float joint_angle_deg[FOOT_JOINT_COUNT]);
void Foot_Mode_Stop(void);

#endif
