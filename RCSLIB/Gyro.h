/*@filename: Gyro_Jy901.h
 *@author     ³ÂÖ¾Î°       
 *@brief:     JY901
 *@date: 2021-1-22
*/
#ifndef _GYRO_H_
#define _GYRO_H_

#include "rcs.h"	

#define GYRO_BAUD				  115200	

void Gyro_Init(RCS_PIN_USART USARTx_MAP);
float Get_Gyro_Z(void);
float Get_Gyro_X(void);
float Get_Gyro_Y(void);
float Get_Gyro_Z_AngleSpeed(void);
#endif
