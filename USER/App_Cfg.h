/**
 @brief:各任务的对栈大小以及优先级
 @tips:uC/OS-II不支持相同优先级，即每个优先级只能有一个任务，优先级值越小越优先。
**/
#ifndef  _APP_CFG_H_
#define  _APP_CFG_H_

#include "UpctrlTask.h"
#include "BleTask.h"
#include "ChassisTask.h"
/*******************任务优先级*******************/
#define STARTUP_TASK_PRIO		10
#define MainTask_PRIO			  11
#define chassis_task_PRIO		5//4
#define bluetooth_task_PRIO 8
#define upctrl_task_PRIO		4//5
#define my_gps_task_PRIO		7
/************任务栈大小（单位为 OS_STK 为）************/
#define STARTUP_TASK_STK_SIZE	256
#define NORMAL_TASK_STK_SIZE	512
/************任务执行频率****************************/
#define UPCTRL_TASK_CYCLE_MS    5
#define BLE_TASK_CYCLE_MS       5
#endif
