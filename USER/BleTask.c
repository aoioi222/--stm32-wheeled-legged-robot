/* ===========头文件=============================================*/
#include "rcs.h"
#include "BleTask.h"
#include "RCS_DT35.h"
#include "R2_CombCtrl.h"
/* ============私有全局变量========================================*/
static OS_STK bluetooth_task_stk[NORMAL_TASK_STK_SIZE];

/* =============用户全局变量=======================================*/
uint32_t ble_tick;
//char     oled_str[30];
//extern int8_t state_findball;
//extern float window_valid_d;
//extern float window_valid_percent;
//extern int8_t Pick_BlockBall_State;
//extern DESCARTES_AXIS Vision_Ball_DescPos;
//extern float mm_dt1;
//extern float mm_dt2;
//extern float mm_dt3;
//extern float test_X;
//extern float test_Y1;
//extern float test_Y2;
//extern uint8_t klklklk[3];
/* ============机器人调试任务=======================================*/
float printfxingxi[6];
extern Motor_Ctrl_Node agv_drv[4],agv_srv[4];
void bluetooth_task(void *p_arg)
{
	p_arg = p_arg;
	while(1)
	{
//		OLED_CLS();
//		sprintf(oled_str,"X=%d Y=%d",(int)Get_LaserGPS_X(),(int)Get_LaserGPS_Y());
//		OLED_ShowStr(0,1,oled_str,1);
//		sprintf(oled_str,"Z=%d S=%d %d",(int)Get_GPS_Z(),(int)all_route_test_state,(int)Pick_BlockBall_State);
//		OLED_ShowStr(0,2,oled_str,1);
//		sprintf(oled_str,"L_X=%d",(int)test_X);
//		OLED_ShowStr(0,3,oled_str,1);
//		sprintf(oled_str,"L_YY=%d %d",(int)test_Y1,(int)test_Y2);
//		OLED_ShowStr(0,4,oled_str,1);
//		sprintf(oled_str,"Vx=%d Vy=%d",(int)Vision_Ball_DescPos.x,(int)Vision_Ball_DescPos.y);
//		OLED_ShowStr(0,6,oled_str,1);
//		sprintf(oled_str,"If_laser=%d",(int)Laser_Reset_Or_Not());
//		OLED_ShowStr(0,5,oled_str,1);

//		
		/*----------输出调试信息---------------------------------*/
		//RCS_Shell_Main(Get_Core407_Debug_View());
		printfxingxi[0] =Get_GPS_X();
		printfxingxi[1] =Get_GPS_Y();
		JustFloat_Printf(&printfxingxi,6,USART6_MAP);
		//printfxingxi[3] =rcsSrv_Rep_RealAngle3();
		//JustFloat_Printf(&printfxingxi,6,USART3_MAP);
		
		
		/*-----------200Hz回传帧率-------------------------------*/
		delay_ms(BLE_TASK_CYCLE_MS);
	}
}

/* =============调试任务初始化======================================*/
void BleTask_Init()
{
	/*-----自定义初始化部分--------------------------------------*/		
	OLED_Init();
	
	/*-----RTOS任务初始化部分-------------------------------------*/
	OSTaskCreate(bluetooth_task,
				(void *)0,
				&bluetooth_task_stk[NORMAL_TASK_STK_SIZE - 1],
				bluetooth_task_PRIO);
}
