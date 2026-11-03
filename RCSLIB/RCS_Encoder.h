/*@filename: RCS_Encoder.h
 *@author     ³ÂÖ¾Î°       
 *@brief:    ±àÂëÆ÷
 *@date: 2021-12-19
*/
#ifndef _RCS_ENCODER_H_
#define _RCS_ENCODER_H_

#include "rcs.h"		

//#define GPS_Y_TIM					TIM8
//#define GPS_Y_GPIO				GPIOC
//#define GPS_Y_A_PIN				GPIO_Pin_6
//#define GPS_Y_B_PIN				GPIO_Pin_7


#define GPS_X_GPIO				GPIOE
#define GPS_X_A_PIN				GPIO_Pin_9
#define GPS_X_B_PIN				GPIO_Pin_11


#define GPS_Y_GPIO				GPIOB
#define GPS_Y_A_PIN				GPIO_Pin_6
#define GPS_Y_B_PIN				GPIO_Pin_7

void GPS_Encoder_Init(void);

#endif
