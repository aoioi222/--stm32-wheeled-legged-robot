/* ===========头文件=============================================*/
#include "MainTask.h"
#include "rcs.h"
#include "Core407_RCS12_PinMapping.h"
#include "R2_CombCtrl.h"
#include "Ch_Ctrl.h"
#include "Laser_Ranging.h"
#include "RCS_Motor_Reposition.h"
/* ============用户应用程序任务===================================*/
void MainTask(void *p_arg)
{
	/*------建立任务前的准备--------------------*/
	#if(OS_TASK_STAT_EN > 0)
    OSStatInit(); //重新评估CPU使用率
	#endif

	/*------全局初始化--------------------------*/
	MainTask_AllInit();

	/*------建立子任务--------------------------*/
	UpCtrlTask_Init();    //主要任务
	ChassisTask_Init();   //主要任务
//	GPS_Init(); 	//定位子任务
	//Laser_Ranging_Init(USART3_MAP);
	BleTask_Init();       //蓝牙通讯(低优先级)
	Laser_Init();
	RCS_Shell_Logs(&Core407_RCSLIB_Debug,"MainTask.c:All Task Created");

	/*------挂起当前任务,让RTOS管理其他任务------*/
	OSTaskSuspend(MainTask_PRIO);
	while(1)
	{
		OSTimeDly(1);
	}
}
/* ============机器人全局资源初始化===================================*/
void MainTask_AllInit(void)
{
	/*-------通用功能初始化-------------------------*/
	RCS_Core407_PinMap_Init();        //为底板的管脚映射赋值(管脚的复)
	RCS_Hard_Stamper_Init(HARD_STAMPER_TIMER);
	RCS_CANx_Config_With_Buffer(CAN1_MAP, CAN1_BAUD);

	/*-------机械臂相关初始化-----------------------*/

		motor_reset_init(GPIOD,GPIO_Pin_13,0b10001000);
		motor_reset_init(GPIOB,GPIO_Pin_6,0b10001000);

		ChCtrl_Init_DT35();
//	  cylinder_init();

	/*-------传感器相关初始化-----------------------*/
	Gyro_Init(USART5_MAP);
	BlueTooth_Init(USART6_MAP);


	/*-------等待初始化完成--------------------------*/
	delay_ms(2000);
	//RCS_Shell_Logs(&Core407_RCSLIB_Debug,"MainTask.c: Config Finished");
	RCS_Hard_Stamper_enable();
}
