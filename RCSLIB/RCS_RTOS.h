#ifndef RCS_RTOS_H_
#define RCS_RTOS_H_
/*============包含库=====================================================*/
//----------C语言公共库----------------------
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

//----------BSP与中间件------------------------
#include "stm32f4xx.h"        //STM32寄存器定义
#include "stm32f4xx_conf.h"   //STM32标准库头文件
#include "bsp.h"              //uCOS底层组件
#include "includes.h"         //uCOS必备组件
#include "delay.h"            //延时函数
//----------硬件抽象层------------------------
#include "RCS_Timer.h"
//----------模块驱动层-------------------------
#include "RCS_filter.h"

/*============时间管理部分=====================================================*/
//-------------配置宏---------------------
#define RTOS_TIMETICK_US  1000000.0f/OS_TICKS_PER_SEC//一个RTOS的tick是多少微秒
#define RTOS_S_TIMETICK   OS_TICKS_PER_SEC           //一秒有几个RTOS的tick 
#define HARD_MS_TIMETICK  168000                     //一个毫秒有几个硬定时的tick
#define HARD_TICKS_PER_US 0.0005952380f              //一个硬定时tick用时多少us
#define SOFT_TICKER_LEN   200

//------------导出数据结构----------------
struct rtos_hard_timer
{
    uint8_t  enable;        //是否开启硬时间戳
    uint64_t total_tick;    //硬时间戳总数
    TIM_TypeDef* TIMx;      //产生硬时间戳的定时器
    uint8_t overflow_count; //定时器溢出次数
    uint16_t cnt;           //定时器当前计数
};
typedef struct rtos_hard_timer RCS_Hard_Stamper_T;

//------------导出函数接口----------------
void RCS_Hard_Stamper_Init(TIM_TypeDef* TIMx);
void RCS_Hard_Stamper_Disable(void);
void RCS_Hard_Stamper_enable(void);
uint64_t Hard_Get_RealTime_Stamp(void);
float Hard_Get_RealTime_Us(void);
void Hard_Delay_Us(uint32_t nus);
void RCS_Hard_Stamper_Reset(void);

uint32_t RTOS_Get_RealTime_Stamp(void);
uint32_t RTOS_Get_RealTime_Us(void);
void RTOS_Delay_Ms(uint16_t nms);
uint8_t RTOS_Count_Delay_Ms(uint8_t nms,uint8_t cycle_ms,uint8_t timer_id);
int Count_Delay_Clear(int num);
int Count_Delay(int count_num,int num);
/*============任务同步部分=====================================================*/
//-------------配置宏---------------------
#define RCS_EVENT_GROUP_COUNT 3

//------------导出数据结构----------------

typedef uint8_t RCS_MUTEX_T;
typedef uint32_t RCS_EVENT_T;
//------------导出函数接口----------------
uint8_t RCS_Pend_Mutex(RCS_MUTEX_T* mutex);
void RCS_Rel_Mutex(RCS_MUTEX_T* mutex);
void RCS_Event_Set(uint8_t event_group,uint8_t event_id);
void RCS_Event_Reset(uint8_t event_group,uint8_t event_id);
uint8_t RCS_Event_Read(uint8_t event_group,uint8_t event_id);

/*============内存管理部分=====================================================*/
//-------------配置宏---------------------
#define BLOCK_LEN    35    //块总数      
#define BLOCK_SIZE   40    //每个块的大小(byte) 
#define STK_SIZE     25     //栈空间大小
//STM32F407VG共128k RAM,平常代码会占去20k左右，因此不建议BLOCK_LEN* BLOCK_SIZE>1650
//分配过大的空间会导致L6406E错误
typedef uint8_t BLOCK_NUM_T;    
//------------导出数据结构----------------
 
struct block_type
{
    uint8_t data[BLOCK_SIZE];
};
struct stk_type
{
    uint8_t data[STK_SIZE];
};
typedef struct block_type Block_t;      
typedef struct stk_type   Stack_t;

struct soft_mmu
{
    //块空间管理
    Block_t     block_partion[BLOCK_LEN][BLOCK_SIZE]; //块空间
    uint8_t     block_ability[BLOCK_LEN]; //空间是否可用

    //栈空间管理
    Stack_t     stk_partion;//栈空间
    void*       stk_top;    //栈顶
};
typedef struct soft_mmu RCS_MMU_T;

//------------导出函数接口----------------
void* RCS_MMU_Malloc(RCS_MMU_T* mmu,uint32_t size);
void RCS_MMU_Free(RCS_MMU_T* mmu,void* addr);
uint8_t RCS_MMU_Push(RCS_MMU_T* mmu,void* data,uint32_t size);
void* RCS_MMU_Pop(RCS_MMU_T* mmu);

#endif