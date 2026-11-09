#ifndef WHEEL_MODE_CONTRL
#define WHEEL_MODE_CONTRL

#include "robot_config.h"

void Wheel_Mode_Init(void);
void Wheel_Mode_Control(const float wheel_speed[WHEEL_COUNT]);
void Wheel_Mode_Stop(void);

#endif
