/*@filename: Laser_Ranging.h
 *@author     胡兴国       
 *@brief:     银色外壳版激光测距模块
 *@date: 2023-8-29
*/
#ifndef _LASER_RANGING_H_
#define _LASER_RANGING_H_

#include "rcs.h"	


#define LASER_BAUD				  9600			

void Laser_Ranging_Init(RCS_PIN_USART USARTx_MAP);
float Get_Laser_Data(void);
float Get_Ranging_Y(void);

#endif
