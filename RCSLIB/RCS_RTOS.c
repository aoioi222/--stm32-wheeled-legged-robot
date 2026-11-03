/**
 * @name:RCS_RTOS.c
 * @brief:时间管理、任务管理与内存管理的统一接口
 * @changlog:2024-3-18 CYK-Dot 完成初代代码的调试
*/
/* ========头文件==============================================*/
#include "RCS_RTOS.h"

/* ========全局变量============================================*/
RCS_MUTEX_T        MMU_mutex;
RCS_Hard_Stamper_T stamper;
RCS_EVENT_T        event_tbl[RCS_EVENT_GROUP_COUNT];

/* ========静态函数声明=========================================*/
void TimStamp_isr(void);
/* ========导出函数定义========================================*/

/**
 * @name:RCS_Hard_Stamper_Init
 * @brief:将一个定时器用于产生更为精细的tick,以在需要高精度的地方代替systick
*/
void RCS_Hard_Stamper_Init(TIM_TypeDef* TIMx)
{
	stamper.cnt=0;
	stamper.enable=0;
	stamper.overflow_count=0;
	stamper.TIMx=TIMx;
	stamper.total_tick=0;
	InitTimerInt(TIMx,65534,1000000,TimStamp_isr,STAMP_PRI);
	TIM_Cmd(TIMx,DISABLE);
}
void RCS_Hard_Stamper_Disable(void)
{
	stamper.enable=0;
	TIM_Cmd(stamper.TIMx,DISABLE);
}
void RCS_Hard_Stamper_enable(void)
{
	stamper.enable=0;
	TIM_Cmd(stamper.TIMx,ENABLE);
}
void RCS_Hard_Stamper_Reset(void)
{
	stamper.TIMx->CNT=0;
}
/**
 * @name:Hard_Get_RealTime_Stamp
 * @brief:返回硬定时器记录的时间戳(从完成初始化开始计时)
 * @tips:todo
*/
uint64_t Hard_Get_RealTime_Stamp(void)
{
    return stamper.overflow_count*65534+stamper.TIMx->CNT;
} 
/**
 * @name:Hard_Get_RealTime_Us
 * @brief:返回硬定时器记录的us数(从完成初始化开始计时)
 * @tips:todo
*/
float Hard_Get_RealTime_Us(void)
{
    return ((double)Hard_Get_RealTime_Stamp());
}
/**
 * @name:Hard_Delay_Us
 * @brief:systick阻塞延时
*/
void Hard_Delay_Us(uint32_t nus)
{
    delay_us(nus);
}
/**
 * @name:RTOS_Get_RealTime_Stamp
 * @brief:返回操作系统时间戳
*/
uint32_t RTOS_Get_RealTime_Stamp(void)
{
    return OSTimeGet();
}
/**
 * @name:RTOS_Get_RealTime_Stamp
 * @brief:返回操作系统微秒数
*/
uint32_t RTOS_Get_RealTime_Us(void)
{
    return ((uint32_t)((float)RTOS_Get_RealTime_Stamp()*RTOS_TIMETICK_US));
}
/**
 * @name:RTOS_Delay_Ms
 * @brief:挂起延时
*/
void RTOS_Delay_Ms(uint16_t nms)
{
    delay_ms(nms);
}

/**
 * @name:RTOS_Count_Delay_Ms
 * @brief:不挂起,非阻塞延时
*/
uint8_t RTOS_Count_Delay_Ms(uint8_t nms,uint8_t cycle_ms,uint8_t timer_id)
{
    return Count_Delay(nms/cycle_ms,timer_id);
}

/**
 * @name:RCS_Event_Set
 * @brief:触发某个事件
 * @param:uint8_t event_group 事件组
 * @param:uint8_t event_id    事件id
 * @tips:该功能一般用来调试，不建议在代码中对任务进行同步，否则会降低代码的可维护性
*/
void RCS_Event_Set(uint8_t event_group,uint8_t event_id)
{
    event_tbl[event_group]=event_tbl[event_group] | (1 << event_id);
}
/**
 * @name:RCS_Event_Reset
 * @brief:归零某个事件
 * @param:uint8_t event_group 事件组
 * @param:uint8_t event_id    事件id
 * @tips:该功能一般用来调试，不建议在代码中对任务进行同步，否则会降低代码的可维护性
*/
void RCS_Event_Reset(uint8_t event_group,uint8_t event_id)
{
    event_tbl[event_group]=event_tbl[event_group] & (~((RCS_EVENT_T)(1 << event_id)));
}
/**
 * @name:RCS_Event_Inquire
 * @brief:查询事件是否发生
 * @param:uint8_t event_group 事件组
 * @param:uint8_t event_id    事件id
 * @tips:该功能一般用来调试，不建议在代码中对任务进行同步，否则会降低代码的可维护性
*/
uint8_t RCS_Event_Read(uint8_t event_group,uint8_t event_id)
{
    return (event_tbl[event_group] & (1 << event_id))>> (event_id-1);
}


/**
	@name: Count_Delay
	@brief:计数延时
	@param:int count_num            			计数数
	@return	1延时完成
**/
static int count[SOFT_TICKER_LEN] = {0};

int Count_Delay(int count_num,int num)
{
	if (num >= SOFT_TICKER_LEN) return 1;
	
	count[num] ++;
	if(count[num] >=count_num)
	{
		count[num] = 0;
		return 1;
	}
	else
		return 0;
}

int Count_Delay_Clear(int num)
{
	count[num] = 0;
}



/**
 * @name:RCS_Pend_Mutex
 * @brief:请求互斥锁
*/
uint8_t RCS_Pend_Mutex(RCS_MUTEX_T* mutex)
{
    if (*mutex) return 0;
    else
    {
        *mutex=1;
        return 1;
    }
}
/**
 * @name:RCS_Rel_Mutex
 * @brief:放弃互斥锁
*/
void RCS_Rel_Mutex(RCS_MUTEX_T* mutex)
{
    *mutex=0;
}

/**
 * @name:RCS_MMU_Malloc
 * @brief:申请内存空间
*/
void* RCS_MMU_Malloc(RCS_MMU_T* mmu,uint32_t size)
{
    if (RCS_Pend_Mutex(&MMU_mutex))
    {
        if ((size > BLOCK_SIZE)) 
        {
            RCS_Rel_Mutex(&MMU_mutex);
            return NULL;
        }
        for(int i=0;i<BLOCK_LEN;i++)
        {
            if (mmu->block_ability[i]==0)
            {
                mmu->block_ability[i]=1;
                RCS_Rel_Mutex(&MMU_mutex);
                return (void*)((mmu->block_partion[i]));
            }
        }
        RCS_Rel_Mutex(&MMU_mutex);
        return NULL;
    }
    else
    {
        return NULL;
    }
}

/**
 * @name:RCS_MMU_Free
 * @brief:释放内存空间
*/
void RCS_MMU_Free(RCS_MMU_T* mmu,void* addr)
{
    uint8_t arr_addr=(addr - (void*)mmu->block_partion)/(sizeof(mmu->block_partion[0]));
    mmu->block_ability[arr_addr]=0;
}

uint8_t RCS_MMU_Push(RCS_MMU_T* mmu,void* data,uint32_t size)
{
    // uint8_t addr_remain=(&(mmu->stk_partion.data[STK_SIZE-1])-mmu->stk_top);
    // if (size>=addr_remain) return 0;
    // mmu->stk_top+=size;
    // memcpy(mmu->stk_top,data,size);
}
void* RCS_MMU_Pop(RCS_MMU_T* mmu)
{

}


/* ============中断函数=======================================*/
void TimStamp_isr(void)
{
	stamper.overflow_count++;
	if (stamper.enable==0)
		TIM_Cmd(stamper.TIMx,DISABLE);
	
	TIM_ClearITPendingBit(stamper.TIMx,TIM_IT_Update);
}
