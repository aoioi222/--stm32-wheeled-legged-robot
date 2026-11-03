	#include "stm32f4xx.h"
	#include "rcs.h"
	#include "Overall_contrl.h"
	
uint8_t robot_statement;
typedef enum { 
	EMERGENCY=0,        //紧急情况
	RESET_CHECK,      //上电自检待机
	FLY_SLEEP,        //飞行部分睡眠
	LAND_SLEEP,      //地面部分睡眠
	WHEEL_START,      //轮式准备
  QUADRUPTED_START, //足式准备
	FLY_START,        //飞行起飞
	WHEEL_RAPID_MOVE, //轮式高速运动
  WHEEL_SLOW_MOVE,  //轮式低速运动
	FLY_MOVE,         //飞行运动
	FLY_STAY,         //飞行悬停
	FLY_LAND,         //飞行着陆
	TRANSPORT         //中转阶段
}robot_state;

void emergent_status(void)
	{	
}
	

robot_state contrl_statement=RESET_CHECK;
void statement_transport(void)
	{
  switch(contrl_statement){
		case EMERGENCY:		    
			emergent_status();
					break;
		case RESET_CHECK:
			;
				 break;
		case FLY_SLEEP:
			;
				 break;
		case LAND_SLEEP:
			;
				 break;
		case WHEEL_START:
			;
		break;				
		case QUADRUPTED_START:
				;
				 break;
		case FLY_START:
			;
					 break;
		case WHEEL_RAPID_MOVE:
					;
			 break;
		case WHEEL_SLOW_MOVE:
					;
			 break;
		case FLY_MOVE:
					;
			 break;		
	  case FLY_STAY:
				 break;	
		case FLY_LAND:
					;
			 break;	
		case TRANSPORT:
				;
				 break;		
				}	
	}