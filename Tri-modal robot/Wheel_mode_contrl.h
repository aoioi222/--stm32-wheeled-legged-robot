#ifndef WHEEL_MODE_CONTRL
#define WHEEL_MODE_CONTRL

#include "stm32f4xx.h"
#include "rcs.h"
#include "robot_config.h"

/* 轮式模式的全部参数（WHEEL_COUNT / WHEEL_RADIUS / WHEEL_BASE_HALF /
   WHEEL_MAX_SPEED / WHEEL_MAX_RPM / WHEEL_CAN_ID_*）统一在 robot_config.h
   的“十二、轮式模式参数”一节定义，本文件不再重复，避免同名常量出现两个值 */

void Wheel_Mode_Init(void);
void Wheel_Mode_Control(const float wheel_speed[WHEEL_COUNT]);
void Wheel_Mode_Stop(void);

#endif
