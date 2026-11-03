/* ===========头文件=============================================*/
#include "rcs.h"
#include "ChassisTask.h"

/* ============私有全局变量========================================*/
static OS_STK chassis_task_stk[NORMAL_TASK_STK_SIZE];

/* ============用户全局变量========================================*/

/* ============机器人次要任务======================================*/
void chassis_task(void *p_arg)
{
	p_arg = p_arg;
	while(1)   
	{
		//R2_Log_Outout();
		delay_ms(5);
	}
}
/* =============次要任务初始化======================================*/
void ChassisTask_Init()
{
	/*-----自定义初始化部分--------------------------------------*/
	
	/*-----RTOS任务初始化部分-------------------------------------*/
	OSTaskCreate(chassis_task,
	           (void *)0,
	           &chassis_task_stk[NORMAL_TASK_STK_SIZE - 1],
	           chassis_task_PRIO);

}
