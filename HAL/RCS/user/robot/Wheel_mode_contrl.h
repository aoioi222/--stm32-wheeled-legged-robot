#ifndef WHEEL_MODE_CONTRL
#define WHEEL_MODE_CONTRL

#include "robot_config.h"
#include "stm32f4xx_hal.h"

void Wheel_Mode_Init(CAN_HandleTypeDef *can);
void Wheel_Mode_Control(const float wheel_speed[WHEEL_COUNT]);
void Wheel_Mode_Stop(void);

#endif
