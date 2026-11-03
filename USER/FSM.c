/**
 * @name:FSM.c
 * @brief:提供有限状态机支持
 * @changelog:2024-3-10 CYK-Dot 初次创建
*/

/* ========头文件=============================================*/
#include "FSM.h"

/* ========全局变量===========================================*/
static volatile FNCT_VOID_PVD action_skip_hook=FSM_Hook_None;
static volatile FNCT_VOID_PVD state_err_hook =FSM_Hook_None;
static volatile FNCT_VOID_PVD state_none_hook=FSM_Hook_None;
static volatile FNCT_VOID_PVD fsm_main_hook  =FSM_Hook_None;
static volatile RCS_FSM_DEBUG_T fsm_log;

/* ========静态函数===========================================*/
static inline void FSM_Log(RCS_FSM_T* fsm);
static void FSM_Assert(RCS_FSM_T* fsm);
static void FNCT_Memset(FNCT_VOID_PVD* arr,FNCT_VOID_PVD target,FSM_Num_t arr_size);
FSM_Num_t FSM_Find_State_ID(RCS_FSM_T* fsm,FNCT_VOID_PVD State);
FSM_Num_t FSM_Find_Action_ID(RCS_FSM_T* fsm,FNCT_VOID_PVD Action);

/* ========导出函数===========================================*/

/**
 * @name:FSM_Init
 * @brief:清空并初始化一个一般状态机
 * @param:RCS_FSM_T* fsm    状态机的地址
 * @param:void* Global_Var  在状态间共享的伪全局变量的指针
 * @tips:0号行为是不做行为,0号状态是什么也不做,0号事件是无事发生
 * @tips:本代码应该作为初始化的第一步而不是最后一步，放在最后一步会清空状态机中原先的内容
*/
void FSM_Init(RCS_FSM_T* fsm,void* Global_Var)
{
	//状态机清空
	FNCT_Memset(fsm->Action_Func,FSM_Action_Skip,FSM_MAX_ACTION_COUNT);
	FNCT_Memset(fsm->States_Func,FSM_Func_Error ,FSM_MAX_STATE_COUNT);
	memset(fsm->SE2A_Tbl,0,sizeof(fsm->SE2A_Tbl));
	memset(fsm->SE2S_Tbl,0,sizeof(fsm->SE2S_Tbl));

	//注册基本信息
	FSM_Add_Action(fsm,FSM_Action_Skip);
	FSM_Add_State(fsm,FSM_Func_None);
	FSM_Set_Event(fsm,FSM_NONE_EVENT_ID);
	fsm->FSM_Shared_Var=Global_Var;
}

/**
 * @name:FSM_Add_State
 * @brief:为状态机注册一个状态
 * @param:RCS_FSM_T* fsm 状态机的地址
 * @param:FNCT_VOID_PVD state 状态函数
 * @tips:如果函数名为x，应该被定义成void x(void* ipt) 
*/
void FSM_Add_State(RCS_FSM_T* fsm,FNCT_VOID_PVD state)
{
    fsm->States_Func[fsm->State_Count]=state;
    fsm->State_Count++;//队列+1
    FSM_Assert(fsm);//自检
}

/**
 * @name:FSM_Add_Action
 * @brief:为状态机注册一个行为
 * @param:RCS_FSM_T* fsm 状态机的地址
 * @param:FNCT_VOID_PVD action 行为函数
*/
void FSM_Add_Action(RCS_FSM_T* fsm,FNCT_VOID_PVD action)
{
    fsm->Action_Func[fsm->Action_Count]=action;
    fsm->Action_Count++;//队列+1
    FSM_Assert(fsm);//自检
}

/**
 * @name:FSM_Add_Event_State
 * @param:RCS_FSM_T* fsm 状态机指针
 * @param:FNCT_VOID_PVD From_State 起始状态函数。直接把函数名填进去即可
 * @param:FNCT_VOID_PVD Target_State 转移后将要进入的函数。直接把函数名填进去即可
 * @param:FSM_EVENT_t Event 发生了Event事件后，状态由From_State转移到Target_State中
 * @brief:为事件指定状态转换
*/
void FSM_Add_Event_State(RCS_FSM_T* fsm,FNCT_VOID_PVD From_State,FNCT_VOID_PVD Target_State,FSM_EVENT_t Event)
{
    fsm->SE2S_Tbl[FSM_Find_State_ID(fsm,From_State)][Event]=FSM_Find_State_ID(fsm,Target_State);
    fsm->SE2A_Tbl[FSM_Find_State_ID(fsm,From_State)][Event]=FSM_Find_Action_ID(fsm,FSM_Action_Skip);
}

/**
 * @name:FSM_Add_Event_Action
 * @param:RCS_FSM_T* fsm 状态机指针
 * @param:FNCT_VOID_PVD From_State 起始状态函数。直接把函数名填进去即可
 * @param:FNCT_VOID_PVD Trigger_Action 事件发生后要触发的退出动作
 * @param:FSM_EVENT_t Event 在From_State状态下发生了Event事件后，执行Trigger_Action函数一次。
 * @brief:为事件指定状态转移时发生的动作
*/
void FSM_Add_Event_Action(RCS_FSM_T* fsm,FNCT_VOID_PVD From_State,FNCT_VOID_PVD Trigger_Action,FSM_EVENT_t Event)
{
    fsm->SE2A_Tbl[FSM_Find_State_ID(fsm,From_State)][Event]=FSM_Find_Action_ID(fsm,Trigger_Action);
}

/**
 * @name:FSM_Set_Startup_Status
 * @brief:指定状态机的初始状态
 * @tips:本句话应晚于状态注册，否则无法定位状态函数的ID
*/
void FSM_Set_Startup_Status(RCS_FSM_T* fsm,FNCT_VOID_PVD startup_state,FSM_EVENT_t startup_event)
{
    if (FSM_Set_State(fsm,startup_state)==-1)
    {
        #ifdef FSM_DEBUG
            RCS_Shell_Logs(&Core407_RCSLIB_Debug,"ERROR:FSM.c,setup startup state earlier than state-function was registered!");
        #endif
    }
    FSM_Set_Event(fsm,startup_event);
}

/**
 * @name:FSM_Main
 * @param:RCS_FSM_T* fsm
 * @brief:状态机本体
 * @tips:请直接将该函数放在主任务upctrl_task中，将会由该函数负责触发状态函数的执行
 * @tips:
*/
void FSM_Main(RCS_FSM_T* fsm)
{
    //无事发生
    if (fsm->Current_Event == FSM_NONE_EVENT_ID)
    {
        //触发状态函数的执行
        fsm->States_Func[fsm->Current_State_ID](fsm->FSM_Shared_Var);
    }
    //事情发生
    else
    {
        //钩子函数
        #ifdef FSM_HOOK_ENABLE
            fsm_main_hook(fsm->FSM_Shared_Var);
        #endif
        //日志函数
        #ifdef FSM_ENABLE_RECORDER
            FSM_Log(fsm);
        #endif

        //触发行为函数的执行
        fsm->Action_Func[fsm->SE2A_Tbl[fsm->Current_State_ID][fsm->Current_Event]](fsm->FSM_Shared_Var);
        //状态转移
        fsm->Current_State_ID=fsm->SE2S_Tbl[fsm->Current_State_ID][fsm->Current_Event];
        //事件清零
        fsm->Current_Event = FSM_NONE_EVENT_ID;
				//触发状态函数的执行
        fsm->States_Func[fsm->Current_State_ID](fsm->FSM_Shared_Var);
    }
}

/**
 * @name:FSM_Trigger_Event
 * @brief:触发一个事件
*/
void FSM_Set_Event(RCS_FSM_T* fsm,FSM_EVENT_t Event)
{
    fsm->Current_Event=Event;
}

/**
 * @name:FSM_Set_State
 * @brief:强行指定状态机当前所处的状态
 * @tips:该函数仅用于调试，不要依赖他来进行状态跳转！
 * @reval:操作是否成功,成功返回1，否则返回-1
*/
int FSM_Set_State(RCS_FSM_T* fsm,FNCT_VOID_PVD State)
{
    FSM_Num_t state_id=FSM_Find_State_ID(fsm,State);
    if (state_id!=-1)
    {
        fsm->Current_State_ID=FSM_Find_State_ID(fsm,State);
        return 1;
    }
    else
        return -1;
}

/**
 * @name:FSM_Find_State_ID
 * @brief:查找某个状态函数被注册成的ID
*/
FSM_Num_t FSM_Find_State_ID(RCS_FSM_T* fsm,FNCT_VOID_PVD State)
{
    for(int i=0;i<fsm->State_Count;i++)
    {
        if (fsm->States_Func[i]==State) return i;
    }

    return -1;
    #ifdef FSM_DEBUG
        RCS_Shell_Logs(&Core407_RCSLIB_Debug,"Error:Can`t Find Target State");
    #endif
}

/**
 * @name:FSM_Find_Action_ID
 * @brief:查找某个行为函数被注册的ID
*/
FSM_Num_t FSM_Find_Action_ID(RCS_FSM_T* fsm,FNCT_VOID_PVD Action)
{
    for(int i=0;i<fsm->Action_Count;i++)
    {
        if (fsm->Action_Func[i]==Action) return i;
    }

    #ifdef FSM_DEBUG
        RCS_Shell_Logs(&Core407_RCSLIB_Debug,"Error:Can`t Find Target Action");
    #endif
}

/**
 * @name:FSM_Get_GlobalVar
 * @brief:返回状态机内部全局变量的指针
*/
void*  FSM_Get_GlobalVar(RCS_FSM_T* fsm)
{
    return fsm->FSM_Shared_Var;
}

/**
 * @name:FSM_Get_Event
 * @brief:获取状态机当前的事件
*/
FSM_Num_t FSM_Get_Event(RCS_FSM_T* fsm)
{
    return fsm->Current_Event;
}

/**
 * @name:FSM_Hook_On_Action_Skip
 * @brief:为默认行为函数注册钩子函数
 * @tips:如果没有为状态-事件转移注册Action,则默认会执行Action_Skip
*/
void FSM_Hook_On_Action_Skip(FNCT_VOID_PVD hook_func)
{
    #ifdef FSM_HOOK_ENABLE
        action_skip_hook=hook_func;
    #else
        #error FSM.c:试图在未启用钩子的时候把钩子函数注册进Action Skip
    #endif
}
/**
 * @name:FSM_Hook_On_Action_Skip
 * @brief:为出错的状态注册钩子函数，避免车子跑飞
 * @tips:可以把相关功能挂入出错状态，方便排查问题
*/
void FSM_Hook_On_State_Err(FNCT_VOID_PVD hook_func)
{
    #ifdef FSM_HOOK_ENABLE
        state_err_hook=hook_func;
    #else
        #error FSM.c:试图在未启用钩子的时候把钩子函数注册进State Err
    #endif
}
/**
 * @name:FSM_Hook_On_FSM_Main
 * @brief:将用户代码挂入状态机内核中
 * @tips:该钩子将会在状态转移时被触发单次
*/
void FSM_Hook_On_FSM_Main(FNCT_VOID_PVD hook_func)
{
    #ifdef FSM_HOOK_ENABLE
        fsm_main_hook=hook_func;
    #else
        #error FSM.c:试图在未启用钩子的时候把钩子函数注册进FSM_Main
    #endif
}
/**
 * @name:FSM_Get_Recorder
 * @brief:返回状态机日志输出
*/
RCS_FSM_DEBUG_T FSM_Get_Recorder(void)
{
    return fsm_log;
}
/* =================静态函数定义====================================*/
/**
 * @name:FSM_Assert
 * @brief:在配置状态机时检测是否合法
*/
static void FSM_Assert(RCS_FSM_T* fsm)
{
    if ((fsm->Action_Count >= FSM_MAX_ACTION_COUNT)||(fsm->State_Count  >= FSM_MAX_EVENT_COUNT))
    {
        #ifdef FSM_DEBUG
        RCS_Shell_Logs(&Core407_RCSLIB_Debug,"Error:FSM Action or State function overflow!");
        uint8_t err=10/0.0f;
        #endif
    }
    return;
}
/**
 * @name:
*/
static FSM_Num_t FSM_State_Trans_Assert(RCS_FSM_T* fsm)
{
    if (fsm->States_Func[fsm->SE2S_Tbl[fsm->Current_State_ID][fsm->Current_Event]]==FSM_Func_Error)
    {
        return 0;
        RCS_Shell_Logs(&Core407_RCSLIB_Debug,"Warning:FSM.c,jump to an undefined state!");
    }
    else 
        return 1;
}
/**
 * @name:FNCT_Memset
 * @brief:专门编写的memset功能
*/
static void FNCT_Memset(FNCT_VOID_PVD* arr,FNCT_VOID_PVD target,FSM_Num_t arr_size)
{
	for(int i=0;i<arr_size;i++)
	{
		arr[i]=target;
	}
}

/**
 * @name:FSM_Log
 * @brief:环形队列，记录状态机在状态转移时的相关信息，便于收集调试
*/
static inline void FSM_Log(RCS_FSM_T* fsm)
{
    fsm_log.Event_ID_Log[fsm_log.log_len]=fsm->Current_Event;
    fsm_log.State_ID_Log[fsm_log.log_len]=fsm->Current_State_ID;
    fsm_log.Action_ID_Log[fsm_log.log_len]=fsm->SE2A_Tbl[fsm->Current_State_ID][fsm->Current_Event];
    fsm_log.log_len++;
    if (fsm_log.log_len==FSM_MAX_LOG_LEN) fsm_log.log_len=0;
}
/**
 * @name:FSM_Func_Error
 * @brief:状态机内核私有的函数。当试图转入一个未定义的状态时，就会进入该函数
*/
void FSM_Func_Error(void* ipt)
{
    #ifdef FSM_DEBUG
    RCS_Shell_Logs(&Core407_RCSLIB_Debug,"Error:FSM Entered error state or action");
    #endif

    #ifndef FSM_HOOK_ENABLE
        return;
    #else
        state_err_hook(ipt);
        return;
    #endif
}
/**
 * @name:FSM_Action_Skip
 * @brief:状态机内核私有的函数。当未指定跳转动作时，默认执行该函数。支持钩子
*/
void FSM_Action_Skip(void* ipt)
{
    #ifndef FSM_HOOK_ENABLE
    return;
    #else
    action_skip_hook(ipt);
    #endif
}
/**
 * @name:FSM_Func_None
 * @brief:状态机内核私有的函数。表示什么都不做的状态函数。支持钩子
*/
void FSM_Func_None(void* ipt)
{
    #ifndef FSM_HOOK_ENABLE
    return;
    #else
    state_none_hook(ipt);
    #endif
}

void FSM_Hook_None(void* ipt)
{
    return;
}
