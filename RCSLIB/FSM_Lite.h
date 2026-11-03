#ifndef _FSM_H_
#define _FSM_H_

#include <stdint.h>
#include <string.h>
//#include "rcs.h"  宏定义想要跨头文件调用的话，不能include大头文件

/*-状态机数据类型定义-*/
#define FSM_SIZE      25             //状态的数量
#define FSM_TYPE      int32_t        //32位单片机处理32位数据是最快的
#define EVENT FSM_TYPE               //返回类型为事件的函数
typedef EVENT(*FSM_VOID_Func)(void); //无输入，返回事件的函数指针
typedef EVENT(*FSM_INPT_Func)(void*);//有输入，返回事件的函数指针

/*-状态机内核事件-*/
#define EVENT_NONE    0     //事件：无事件
#define EVENT_FINISH  -1    //事件：状态函数执行完成
#define EVENT_PAUSE   -2    //事件：状态函数挂起

/*-状态机内核状态函数-*/
EVENT NOP_VOID(void);        //没东西的函数

/*-状态机本体-*/
typedef struct 
{
    FSM_TYPE Event;                    //当前事件ID 
    FSM_TYPE State;                    //当前状态ID
    FSM_TYPE nState_Table[FSM_SIZE+1];   //状态转移表,当发生EVENT_FINISH事件后,转移到下一个状态
    FSM_VOID_Func State_Func[FSM_SIZE+1];//状态函数指针数组
    FSM_VOID_Func Error_Func;
}FSM_Struct;

/*-状态机功能-*/
EVENT FSM_Server(FSM_Struct* fsm);
void FSM_Jump(FSM_Struct* fsm,int32_t new_state);
void FSM_ErrState_Init(FSM_Struct* fsm,FSM_VOID_Func func);

/*-状态函数功能*/
FSM_TYPE FSM_Get_State(FSM_Struct* fsm);



#endif