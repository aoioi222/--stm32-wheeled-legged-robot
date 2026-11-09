#ifndef FOOT_MODE_CONTROL_H
#define FOOT_MODE_CONTROL_H

#include <stdbool.h>
#include "stm32f4xx_hal.h"

#define FOOT_JOINT_COUNT 12

void Foot_Mode_Init(CAN_HandleTypeDef *can);
bool Foot_Mode_Control(const float joint_angle_deg[FOOT_JOINT_COUNT]);
void Foot_Mode_Stop(void);

#endif
