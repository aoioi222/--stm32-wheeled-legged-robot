#ifndef _RCS_FSM_H__
#define _RCS_FSM_H__

/* -----------------头文件------------------------------------*/
//C语言公共库
#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

//RCSLIB模块驱动层(算法层)
#include "RCS_DataStructure.h" //常用数据结构
#include "RCS_Types.h"

//RCSLIB机器人功能层
#include "RCS_Debug.h"
#include "Core407_RCS12_Debug.h"


/* ----------------配置宏-------------------------------------*/
#define FSM_MAX_STATE_COUNT     50 //状态机最大的状态数量
#define FSM_MAX_ACTION_COUNT    10 //状态机最大的动作数量
#define FSM_MAX_EVENT_COUNT     50 //状态机最大的事件数量
#define FSM_MAX_LOG_LEN         70 //最大日志记录数量

#define FSM_DEBUG                  //开启串口调试输出
#define FSM_HOOK_ENABLE            //开启钩子函数支持
#define FSM_ENABLE_RECORDER        //开启日志收集功能

/* ----------------私有宏-------------------------------------*/
#define FSM_NONE_EVENT_ID       0 //无事发生的事件ID


/* ----------------导出数据结构--------------------------------*/
typedef uint8_t FSM_Num_t;      //状态机的状态/动作数量均<255，故使用uint8_t
typedef uint16_t FSM_EVENT_t;   //状态机的事件数量有可能>255，故使用uint16_t

//基本状态机
typedef struct{
    //Public属性
    FSM_EVENT_t   Current_Event;      //当前事件的ID
    FSM_Num_t     Current_State_ID;   //当前状态函数的ID
    void*         FSM_Shared_Var;     //状态机上下文变量指针

    //Private属性
    FNCT_VOID_PVD States_Func[FSM_MAX_STATE_COUNT];
    FNCT_VOID_PVD Action_Func[FSM_MAX_ACTION_COUNT];
    FSM_Num_t     SE2S_Tbl   [FSM_MAX_STATE_COUNT][FSM_MAX_EVENT_COUNT];//输入State和Event的ID,返回转移后State的ID
    FSM_Num_t     SE2A_Tbl   [FSM_MAX_STATE_COUNT][FSM_MAX_EVENT_COUNT];//输入State和Event的ID,返回要执行的Action的ID
    FSM_Num_t     State_Count;                       //状态计数
    FSM_Num_t     Action_Count;                      //行为计数
}RCS_FSM_T;

//拓展状态机
typedef struct{
    //Public属性
    FSM_EVENT_t   Current_Event;
    FSM_Num_t     Current_State_ID;
    void*         FSM_Shared_Var;

    //Private属性
    FNCT_VOID_PVD States_Func[FSM_MAX_STATE_COUNT];
    FNCT_VOID_PVD ExiAct_Func[FSM_MAX_ACTION_COUNT];
    FNCT_VOID_PVD EntAct_Func[FSM_MAX_ACTION_COUNT];
    FSM_Num_t     SE2S_Tbl   [FSM_MAX_STATE_COUNT][FSM_MAX_EVENT_COUNT];//输入State和Event的ID,返回转移后State的ID
    FSM_Num_t     SE2A_Tbl   [FSM_MAX_STATE_COUNT][FSM_MAX_EVENT_COUNT];
    FSM_Num_t     State_Count;                       //状态计数
    FSM_Num_t     ExtAct_Count;                      //退出动作计数
    FSM_Num_t     EntAct_Count;                      //进入动作计数
}RCS_FSM_EXT_T;

//状态机日志
typedef struct 
{
    FSM_EVENT_t   Event_ID_Log[FSM_MAX_LOG_LEN];
    FSM_Num_t     State_ID_Log[FSM_MAX_LOG_LEN];
    FSM_Num_t     Action_ID_Log[FSM_MAX_LOG_LEN];
    uint16_t      log_len;
}RCS_FSM_DEBUG_T;


/* -----------------导出函数------------------------------------*/

//配置状态机
void FSM_Init(RCS_FSM_T* fsm,void* Global_Var);
void FSM_Add_State(RCS_FSM_T* fsm,FNCT_VOID_PVD state);
void FSM_Add_Action(RCS_FSM_T* fsm,FNCT_VOID_PVD action);
void FSM_Set_Startup_Status(RCS_FSM_T* fsm,FNCT_VOID_PVD startup_state,FSM_EVENT_t startup_event);

//配置状态转移表
void FSM_Add_Event_State(RCS_FSM_T* fsm,FNCT_VOID_PVD From_State,FNCT_VOID_PVD Target_State,FSM_EVENT_t Event);
void FSM_Add_Event_Action(RCS_FSM_T* fsm,FNCT_VOID_PVD From_State,FNCT_VOID_PVD Trigger_Action,FSM_EVENT_t Event);

//状态机运行
void FSM_Main(RCS_FSM_T* fsm);
void FSM_Set_Event(RCS_FSM_T* fsm,FSM_EVENT_t startup_event);

//调试辅助功能
FSM_Num_t FSM_Find_State_ID(RCS_FSM_T* fsm,FNCT_VOID_PVD State);
FSM_Num_t FSM_Find_Action_ID(RCS_FSM_T* fsm,FNCT_VOID_PVD Action);
void*     FSM_Get_GlobalVar(RCS_FSM_T* fsm);
FSM_Num_t FSM_Get_Event(RCS_FSM_T* fsm);
int       FSM_Set_State(RCS_FSM_T* fsm,FNCT_VOID_PVD State);//请勿使用该函数来跳转状态
void      FSM_Hook_On_Action_Skip(FNCT_VOID_PVD hook_func);
void      FSM_Hook_On_State_Err(FNCT_VOID_PVD hook_func);
void      FSM_Hook_On_FSM_Main(FNCT_VOID_PVD hook_func);
RCS_FSM_DEBUG_T FSM_Get_Recorder(void);

//状态机底层的常函数
void FSM_Func_None(void* ipt);
void FSM_Func_Error(void* ipt);
void FSM_Action_Skip(void* ipt);
void FSM_Hook_None(void* ipt);

/* -----------------导出宏--------------------------------------*/
#define FSM_EVENT_NONE       0
#define FSM_ERR_STATE_FUNC   FSM_Func_Error


#endif