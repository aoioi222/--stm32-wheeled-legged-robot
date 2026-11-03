#ifndef _ROUTE_H_
#define _ROUTE_H_

#include "rcs.h"
#include "RCS_DT35.h"
#include "RCS_Stastic.h"


#define ZONE_RED  0
#define ZONE_BLUE 1


void ChCtrl_Init_DT35(void);
void ChCtrl_Test_DT35(void);
void ChCtrl_Laser_Main(void);
float ChCtrl_Laser_Get_cY(void);
float ChCtrl_Laser_Get_lX(void);
float ChCtrl_Laser_Get_rX(void);
#endif
