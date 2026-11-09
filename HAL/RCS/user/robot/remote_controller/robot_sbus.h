#ifndef ROBOT_SBUS_H
#define ROBOT_SBUS_H

#include <stdbool.h>
#include "stm32f4xx_hal.h"

bool RobotSbus_Start(UART_HandleTypeDef *uart);
bool RobotSbus_GetLiveChannels(uint16_t channels[16]);

#endif
