/* ===========头文件==============================================*/
#include "rcs.h"

/* ============私有全局变量========================================*/
static OS_STK startup_task_stk[STARTUP_TASK_STK_SIZE];
static OS_STK MainTask_stk[NORMAL_TASK_STK_SIZE];

/* ============静态函数===========================================*/
static void startup_task(void *p_arg);//二级引导程序段


/* ============一级引导程序=======================================*/
int main(void)

{
	/*--------基本硬件功能配置------------------------------*/
	//内核配置
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//Cortex-M4中断初始化
	delay_init(168);  	                           //配置时钟为168MHz
	//RTOS配置
	BSP_Init();                                    //对接硬件层和RTOS层
	OSInit();                                      //RTOS初始化

	/*--------导入二级引导程序------------------------------*/
	OSTaskCreate(startup_task,//创建引导任务
	             (void *)0,
	             &startup_task_stk[STARTUP_TASK_STK_SIZE - 1],
	             STARTUP_TASK_PRIO);
	OSStart();//开启RTOS调度
	return 0;
}

/* ============二级引导程序=======================================*/
static void startup_task(void *p_arg)
{
	/*--------创建三级引导程序----------------------------*/
	#if(OS_TASK_STAT_EN > 0)
    OSStatInit(); //重新评估CPU使用率,准备创建任务
	#endif
	
	OSTaskCreate(MainTask,//创建MainTask任务作为三级引导程序
	           (void *)0,
	           &MainTask_stk[NORMAL_TASK_STK_SIZE - 1],
	           MainTask_PRIO);

	/*------挂起当前任务,由RTOS导入三级引导程序------------*/
	OSTaskSuspend(STARTUP_TASK_PRIO);
	while (1)
	{
		OSTimeDly(1000);
	}
	
}
