/*@filename: GPS_Action.h
 *@author     胡兴国       
 *@brief:     东大全场定位
 *@date: 2023-7-28
*/
#ifndef _GPS_ACTION_H_
#define _GPS_ACTION_H_

#include "rcs.h"	

#define ACTION_BAUD     115200

void GPS_Action_Init(RCS_PIN_USART USARTx_MAP);
float Get_GPS_Action_X(void);
float Get_GPS_Action_Y(void);
float Get_GPS_Action_Z(void);
#endif
