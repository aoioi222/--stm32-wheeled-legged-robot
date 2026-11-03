//@filename:Vision.h
//@author:  陈志伟
//@date:    2021-1-10
//@brief:   视觉通信

#ifndef _VISION_H_
#define _VISION_H_

#include "rcs.h"


void Vision_Init(RCS_PIN_USART USARTx_MAP);
float Vision_Get_X(void);
float Vision_Get_Y(void);
uint8_t Vision_Is_Available(void);
int Vision_identify();

#endif //_VISION_H_

