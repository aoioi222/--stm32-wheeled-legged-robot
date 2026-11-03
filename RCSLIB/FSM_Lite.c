/**
 @name:FSM_Lite.h
 @brief:阉割版有限状态机
 @readme:状态转移功能保留。不支持行为函数。不支持状态-事件转移表。
**/
#include "FSM_Lite.h"

/**
 * @name:FSM_Server
 * @brief:精简状态机的功能实现
 * @param:fsm    需要运行的状态机
 * @reval:EVENT  状态机捕获的事件
 * @readme:状态机在处理事件之前，总是会先把事件从函数中返回，此时就可以让决策机修改状态转移表，实现实时决策
**/
EVENT FSM_Server(FSM_Struct* fsm)
{
    FSM_Struct input_fsm=(*fsm); //供编译器优化的变量
    //----------处理内核事件(状态执行完成)-------------------
    if (input_fsm.Event==EVENT_FINISH)                     //仅在EVENT_FINISH事件发生时才产生状态转移
	  {
        fsm->State=input_fsm.nState_Table[fsm->State];     //状态转移
        fsm->Event=EVENT_NONE;                             //事件复位
    }
	//-----------执行状态函数--------------------------------	
	if (input_fsm.State_Func[fsm->State]!=NULL)
	{
		fsm->Event=input_fsm.State_Func[fsm->State]();    //状态执行+事件监测
	}
	else
	{
		fsm->Error_Func();       //出错
	}
	
	return fsm->Event; //总是先把事件发送出去而不是先执行
}

/**
 * @name:FSM_Jump
 * @brief:手动设置状态机的状态
 * @param:fsm   需要跳转状态的状态机
 * @param:state 需要转换到的状态
*/
void FSM_Jump(FSM_Struct* fsm,int32_t new_state)
{
    fsm->State=new_state;
}

/**
 @name: FSM_DeInit
 @brief:清除有限状态机的配置,应于状态机初始化之后执行
**/
void FSM_DeInit(FSM_Struct* fsm)
{
	for(FSM_TYPE i=0;i<=FSM_SIZE;i++)
	{
		fsm->State_Func[i]=NULL;
		fsm->nState_Table[i]=FSM_SIZE;
	}
	fsm->Event=EVENT_NONE;
	fsm->Error_Func=NOP_VOID;
}


void FSM_ErrState_Init(FSM_Struct* fsm,FSM_VOID_Func func)
{
    fsm->Error_Func=func;
}

FSM_TYPE FSM_Get_State(FSM_Struct* fsm)
{
    return fsm->State;
}

EVENT NOP_VOID(void)
{
	return EVENT_NONE;
}



