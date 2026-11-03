/**
 * @filename:RCS_Motor_Upctrl.c
 * @brief:使用统一的框架控制电机
 * @tips:请保持RM电机的ID位于1~8之内,本杰明电调的ID>8
 * @todo:6020暂未纳入该框架;CAN1下的ID1~4为底盘电机,暂为纳入该框架,仍然用Motor_Send控制
 * @contribute:CYK-dot @2024-2-2
 * ------------------------------------------------------------
 * @changelog: CYK-dot  @2024-3-25 补充了电机状态相关的接口、完善注释
 * ------------------------------------------------------------
 * @changelog: CYK-dot  @2024-5-21 补充了野指针相关的保护
*/

/* ==========头文件 ==========================================*/
#include "RCS_Motor_Upctrl.h"
#include "RCS_dsp.h"

/* ==========静态函数 ========================================*/
static inline void Reset_RMESC_OutputCurrent(uint8_t CAN_Group);
static inline void Update_RMESC_OutputCurrent(uint8_t CAN_Group,Motor_Ctrl_Node* Node);
static inline void Excute_RMESC_OutputCurrent(uint8_t CAN_Group);
static inline void Excute_VESC_Output(uint8_t CAN_Group,Motor_Ctrl_Node* Node);

static inline int8_t Judge_MotorNode_isPosProtect(Motor_Ctrl_Node* Node);
static inline int8_t Judge_MotorNode_isSpdProtect(Motor_Ctrl_Node* Node);
/* ==========全局变量 ========================================*/
Motor_Ctrl_List CtrlList[CTRL_LIST_GROUP_COUNT];    //电机节点链表
int16_t RMESC_Current[CTRL_LIST_GROUP_COUNT][9];    //3508/2006电机输出电流
int16_t GM6020_Current[CTRL_LIST_GROUP_COUNT][9];   //6020输出电流

uint8_t Teleport_Status[CTRL_LIST_GROUP_COUNT];     //RM报文输出状态
//[7~4]不使用 [3]是否需要Motor_Send [2]是否需要Motor_Send_ADD [1]是否需要GM6020_Send [0]是否需要GM6020_Send_ADD 

/* ==========函数定义 ========================================*/

/**
 * @name:MotorNode_Add
 * @brief:将电机节点加入CAN总线
 * @param:CAN_Group={CAN_GROUP_x,x=1~2}
 * @param:Motor_Ctrl_Node* Node 编写完成电机节点后,再把这个电机节点的指针送入此函数
*/
void MotorNode_Add(uint8_t CAN_Group,Motor_Ctrl_Node* Node)
{
	if (CtrlList[CAN_Group].List_Len ==CTRL_LIST_MAX_LEN) 
	{
		#ifdef RCS_MOTOR_UPCTRL_DEBUG
			RCS_Shell_Logs("ERROR:in RCS_Motor_Upctrl.c, add too many motors");
		#endif
		return;
	}
	CtrlList[CAN_Group].Node_Ctrl[CtrlList[CAN_Group].List_Len]=Node;
	CtrlList[CAN_Group].List_Len++;
}

/**
 * @name:Motor_Ctrl_Update_Current
 * @brief:更新电机节点的电流输出值
 * @param:Node 需要改变状态的电机节点
 * @tips:电流控制的电机,使用最大电流作为超功率约束
*/
void MotorNode_Update_Current(int16_t Cur,Motor_Ctrl_Node* Node)
{
	//------------未达到限幅----------------------------
	if (abs(Cur) <= Node->Max_Current)
	{
		//数值更新
		Node->Current=Cur;    //下发给电机的输出值
		Node->Out_Current=Cur;//用来观测的debug值

		//日志记录
		#ifdef RCS_MOTOR_UPCTRL_DEBUG
		if (Node->Ctrl_Type != CTRL_TYPE_CUR)
		RCS_Shell_Logs("Warning:in RCS_Motor_Upctrl.c,Tying to use current control mistakenly");
		#endif
	}
	//------------达到限幅------------------------------
	else
	{
		//数值更新
		if (Cur>0)
		{
			Node->Current=Node->Max_Current;
			Node->Out_Current=Node->Max_Current;
		}
		else 
			Node->Current=-1*Node->Max_Current;
			Node->Out_Current=Node->Current;

		//日志记录
		#ifdef RCS_MOTOR_UPCTRL_DEBUG
			RCS_Shell_Logs("Warning:in RCS_Motor_Upctrl.c,Current overload!");
			if (Node->Ctrl_Type != CTRL_TYPE_CUR)
			RCS_Shell_Logs("Warning:in RCS_Motor_Upctrl.c,Tying to use current control mistakenly");
		#endif
	}
}

/**
 * @name:Motor_Ctrl_Update_Spd
 * @brief:更新电机节点的目标速度,若电机采用电流控制,则使用速度环PID更新目标电流
 * @param:Node 需要改变状态的电机节点
 * @tips:对于转速控制的电调:需要为电调配置电机保护，否则该函数永远输出0速以保护电机
 * @tips:对于电流控制的电调:仍然使用最大电流作为超功率约束
*/
void MotorNode_Update_Spd(int Spd,Motor_Ctrl_Node* Node)
{	
		//update variables
		Node->Out_Spd=Spd;
		Node->Now_Speed=Node->Get_Node_Speed(Node->CAN_ID);
	
		//protect variables
		if (Node->Now_CtrlStatus.is_SpdProtect==1)
		if (Judge_MotorNode_isPosProtect(Node)==1)
			Node->Out_Spd=Node->Max_USpd;
		
		if (Node->Now_CtrlStatus.is_SpdProtect==1)
		if (Judge_MotorNode_isPosProtect(Node)==-1)
			Node->Out_Spd=Node->Max_LSpd;
		
		if (Node->Now_CtrlStatus.is_PosProtect==1)
		if (Judge_MotorNode_isPosProtect(Node)!=0)
			Node->Out_Spd=0;
		
		//excute for variables
		if (Node->Ctrl_Type==CTRL_TYPE_CUR)
		{
			Node->Out_Current=PID_Normal_Ctrl(Node->Out_Spd,Node->Now_Speed,Node->Motor_Speed_PID);
			MotorNode_Update_Current(Node->Out_Current,Node);
		}
		else if (Node->Ctrl_Type==CTRL_TYPE_RPM)
		{
			Node->Speed=Node->Out_Spd;
		}
}

/**
 * @name:Motor_Ctrl_Update_Angle
 * @brief:更新电机节点的目标角度
 * @param:Node 需要改变状态的电机节点
 * @tips:对于转速控制的电调:需要为电调配置电机保护，否则速度永远为0以保护电机
 * @tips:对于电流控制的电调:仍然使用最大电流作为超功率约束
*/
void MotorNode_Update_Angle(float angle,Motor_Ctrl_Node* Node)
{
	//更新目标值
	Node->Out_Angle=angle;
	//更新当前值
	Node->Now_Angle=Node->Get_Node_Angle(Node->CAN_ID);
	
	//protect
	if (Node->Now_CtrlStatus.is_PosProtect==1)
	if (Judge_MotorNode_isPosProtect(Node)==1)
		Node->Out_Angle=Node->Max_UPos;
	
	if (Node->Now_CtrlStatus.is_PosProtect==1)
	if (Judge_MotorNode_isPosProtect(Node)==-1)
		Node->Out_Angle=Node->Max_LPos;
	
	//excute
	if ((Node->Ctrl_Type==CTRL_TYPE_CUR)||(Node->Ctrl_Type==CTRL_TYPE_RPM))
	{
		//计算控制值
		Node->Out_Spd=PID_Normal_Ctrl(Node->Out_Angle,Node->Now_Angle,Node->Motor_Angle_PID);
		//更新控制值
		MotorNode_Update_Spd(Node->Out_Spd,Node);
	}
	else if (Node->Ctrl_Type==CTRL_TYPE_POS)
	{
		Node->Angle=Node->Out_Angle;
	}
}

/**
 * @name:MotorNode_Update_AngleEasy
 * @brief:更新电机节点的目标角度,以指定误差的形式更新目标电流
 * @param:angle 目标角度
 * @param:speed 移动到目标角度的速度
 * @param:tor_angle 容差,在容差以内的话让电机速度为0
 * @param:Node 需要改变状态的电机节点
 * @reval:是否抵达位置
 * @tips:适用于速度控制/电流控制类的电机
*/
uint8_t MotorNode_Update_AngleEasy(float angle,float speed,float tor_angle,Motor_Ctrl_Node* Node)
{
	//update
	Node->Out_Angle=angle;
	Node->Now_Angle=Node->Get_Node_Angle(Node->CAN_ID);
	
	//protect
	if (Node->Now_CtrlStatus.is_PosProtect==1)
	if (Judge_MotorNode_isPosProtect(Node)==1)
		Node->Out_Angle=Node->Max_UPos;
	
	if (Node->Now_CtrlStatus.is_PosProtect==1)
	if (Judge_MotorNode_isPosProtect(Node)==-1)
		Node->Out_Angle=Node->Max_LPos;
	
	//caculate & excute
	if (Node->Ctrl_Type==CTRL_TYPE_CUR)
	{
		//smaller
		if (Node->Now_Angle <= (Node->Out_Angle-tor_angle))
		{
			MotorNode_Update_Spd(speed,Node);
		}
		//bigger
		else if (Node->Now_Angle > (Node->Out_Angle-tor_angle))
		{
			MotorNode_Update_Spd(-speed,Node);
		}
		//suitable
		else
		{
			MotorNode_Update_Spd(0,Node);
		}
	}
}



uint8_t MotorNode_Update_AngleFull(float angle,Motor_Ctrl_Node* Node)
{
	//更新目标值
	Node->Out_Angle=angle;
	//更新当前值
	Node->Now_Angle=Node->Get_Node_Angle(Node->CAN_ID);
	
	//protect
	if (Node->Now_CtrlStatus.is_PosProtect==1)
	if (Judge_MotorNode_isPosProtect(Node)==1)
		Node->Out_Angle=Node->Max_UPos;
	
	if (Node->Now_CtrlStatus.is_PosProtect==1)
	if (Judge_MotorNode_isPosProtect(Node)==-1)
		Node->Out_Angle=Node->Max_LPos;
	
	//excute
	if ((Node->Ctrl_Type==CTRL_TYPE_CUR)||(Node->Ctrl_Type==CTRL_TYPE_RPM))
	{
		//计算控制值
		Node->Out_Spd=DacePID_Normal_Ctrl(Node->Out_Angle,Node->Now_Angle,Node->Motor_DaceAngle_PID);
		//更新控制值
		MotorNode_Update_Spd(Node->Out_Spd,Node);
		
		if (Node->Out_Spd==0) return 1;
	}
	else if (Node->Ctrl_Type==CTRL_TYPE_POS)
	{
		Node->Angle=Node->Out_Angle;
	}
	
	return 0;
}
/**
 * @name:MotorList_Excute
 * @brief:将所有电机节点的电流值下发给电调
 * @param:CAN_Group={CAN_GROUP_x,x=1~2}
 * @tips:该函数的目的是避免使用单独的电机控制函数，因此请放在所有业务逻辑结束后和delay_ms前，不应放在业务逻辑中
*/
void MotorList_Excute(uint8_t CAN_Group)
{
	//-----复位--------------------------------------------

	//复位多电机同报文的输出值
	Reset_RMESC_OutputCurrent(CAN_Group);
	//复位单电机单报文的输出值
	__NOP();//本杰明自己会超时停机,不需要从软件上避免控制不连续

	//-----更新--------------------------------------------

	for(int i=0;i<CtrlList[CAN_Group].List_Len;i++)
	{
		//更新多电机同报文的输出值
		Update_RMESC_OutputCurrent(CAN_Group,(CtrlList[CAN_Group].Node_Ctrl[i]));
		//更新单电机单报文的输出值
		__NOP();
	}

	//------执行-------------------------------------------

	//执行多电机单报文的输出值
	Excute_RMESC_OutputCurrent(CAN_Group);
	//执行单电机单报文的输出值
	for(int i=0;i<CtrlList[CAN_Group].List_Len;i++)
	{
		Excute_VESC_Output(CAN_Group,(CtrlList[CAN_Group].Node_Ctrl[i]));
	}
}

/**
 * @name:MotorNode_Init_C620
 * @brief:配置RM电调的参数
 * @param:uint8_t CAN_Group      填入CAN_GROUP_x
 * @param:uint8_t ID             电调ID,请配置成1~8
 * @param:Motor_Ctrl_Node* Node  需要被配置成RM电调的电机节点
*/
void MotorNode_Init_C620(uint8_t CAN_Group,uint8_t ID,Motor_Ctrl_Node* Node)
{
	Node->CAN_ID=ID;
	Node->Teleport_Type=TELEPORT_RMESC;    //RM报文
	Node->Ctrl_Type=CTRL_TYPE_CUR;         //电流控制
	Node->ESC_Type=ESC_C620;               //C620电调
	Node->Max_Current=C620_MAX_CURRENT;    //功率限制

	if (CAN_Group==CAN_GROUP_1)
	{
		Node->Get_Node_Speed=&Get_Motor_Speed;
		Node->Get_Node_Angle=&Get_Motor_Float_Angle;
	}
	else
	{
		Node->Get_Node_Speed=&Get_Motor_Speed2;
		Node->Get_Node_Angle=&Get_Motor_Float_Angle2;
	}
}

/**
 * @name:Motor_Ctrl_Init_C610_Motor
 * @brief:配置RM电调的参数
 * @param:uint8_t CAN_Group      填入CAN_GROUP_x
 * @param:uint8_t ID             电调ID,请配置成1~8
 * @param:Motor_Ctrl_Node* Node  需要被配置成RM电调的电机节点
*/
void MotorNode_Init_C610(uint8_t CAN_Group,uint8_t ID,Motor_Ctrl_Node* Node)
{
	Node->CAN_ID=ID;
	Node->Teleport_Type=TELEPORT_RMESC;    //RM报文
	Node->Ctrl_Type=CTRL_TYPE_CUR;         //电流控制
	Node->ESC_Type=ESC_C610;               //C610电调
	Node->Max_Current=C610_MAX_CURRENT;    //功率限制

	if (CAN_Group==CAN_GROUP_1)
	{
		Node->Get_Node_Speed=&Get_Motor_Speed;
		Node->Get_Node_Angle=&Get_Motor_Float_Angle;
	}
	else
	{
		Node->Get_Node_Speed=&Get_Motor_Speed2;
		Node->Get_Node_Angle=&Get_Motor_Float_Angle2;
	}
}

/**
 * @name:MotorNode_Init_GM6020_Cur
 * @brief:配置GM6020电机,并使用电流环控制
*/
void MotorNode_Init_GM6020(uint8_t CAN_Group,uint8_t ID,Motor_Ctrl_Node* Node)
{
	#ifdef RCS_MOTOR_UPCTRL_DEBUG
		if (ID <= 4) RCS_Shell_Logs("Warning:in RCS_Motor_Upctrl.c,attempt to set GM6020 ID in range of 1~4");
	#endif
	Node->CAN_ID=ID;
	Node->Teleport_Type=TELEPORT_RMESC;    //RM报文
	Node->Ctrl_Type=CTRL_TYPE_CUR;         //电流控制
	Node->ESC_Type=ESC_GM6020;             //6020电调
	Node->Max_Current=GM6020_MAX_CURRENT;  //功率限制
	
	if (CAN_Group==CAN_GROUP_1)
	{
		Node->Get_Node_Speed=&Get_GM6020_Speed;
		Node->Get_Node_Angle=&Get_GM6020_Float_Angle;
	}
	else
	{
		Node->Get_Node_Speed=&Get_GM6020_Speed2;
		Node->Get_Node_Angle=&Get_GM6020_Float_Angle2;
	}
}

/**
 * @name:Motor_Ctrl_Init_MBVESC_Motor_Cur
 * @brief:配置创客基地本杰明电调的参数,并使用本杰明电调的电流控制方式
 * @param:uint8_t CAN_Group       填入CAN_GROUP_x
 * @param:uint8_t ID              电调ID
 * @param:CTRL_TYPE CTRL_TYPE_xxx 控制类型,使用电流或者转速控制,CTRL_TYPE_CUR/CTRL_TYPE_RPM
 * @param:Motor_Ctrl_Node* Node   需要被配置成本杰明电调的电机节点
*/
void MotorNode_Init_MBVESC(uint8_t CAN_Group,uint8_t ID,CTRL_TYPE CTRL_TYPE_xxx,Motor_Ctrl_Node* Node)
{
	if (Node->Ctrl_Type == CTRL_TYPE_TOR)//@todo:暂时不懂怎么用本杰明电调控制力矩
	{
		#ifdef RCS_MOTOR_UPCTRL_DEBUG
			RCS_Shell_Logs("ERROR: in RCS_Motor_Upctrl.c,Could not init VESC with torque control type!");
			return;
		#endif
	}

	Node->CAN_ID=ID;
	Node->Teleport_Type=TELEPORT_VESC;   //本杰明报文
	Node->ESC_Type=ESC_MBVESC;           //创客基地电调
	Node->Ctrl_Type=CTRL_TYPE_xxx;       //电流控制/转速控制
	Node->Max_Current=MBVESC_MAX_CURRENT;//电流保护
	Node->Max_Power=MBVESC_MAX_POWER;    //转速的功率保护(没有保护)
	Node->Max_Speed=MBVESC_MAX_ERPM;     //极转速保护(没有保护)

	if (CAN_Group==CAN_GROUP_1)
	{
		Node->Get_Node_Speed=&Get_VESC_Speed2;
		Node->Get_Node_Angle=&Get_VESC_Pos;
	}
	else
	{
		Node->Get_Node_Speed=&Get_VESC_Speed2;
		Node->Get_Node_Angle=&Get_VESC_Pos2;
	}
}



/**
 * @name:MotorNode_Add_SpeedPid
 * @brief:为节点添加速度环pid
*/
void MotorNode_Add_SpeedPid(Motor_Ctrl_Node* Node,PID_Struct* spd_pid)
{
	Node->Motor_Speed_PID=spd_pid;
}

/**
 * @name:MotorNode_Add_AnglePid
 * @brief:为节点添加角度环pid
*/
void MotorNode_Add_AnglePid(Motor_Ctrl_Node* Node,PID_Struct* angle_pid)
{
	Node->Motor_Angle_PID=angle_pid;
}

void MotorNode_Add_DaceAnglePid(Motor_Ctrl_Node* Node,DacePID_Struct* dace_pid)
{
	Node->Motor_DaceAngle_PID=dace_pid;
}

/**
 * @name:MotorNode_Add_BldcProtect
 * @brief:配置完电调之后,为电机节点添加电机保护
 * @param:Motor_Ctrl_Node* Node 需要添加电机保护的节点的指针
 * @param:MOTOR_TYPE Motor      电机类型,目前仅支持VESC_U8_LiteL_KV110和VESC_N5065_KV140
*/
void MotorNode_Add_BldcProtect(Motor_Ctrl_Node* Node,MOTOR_TYPE Motor)
{
	//电机参数小于电调参数，则选择电机参数作为极限参数
	switch (Motor)
	{
		case VESC_U8_LiteL_KV110:
			if (Node->Max_Current > U8LiteL_KV110_MAX_CURRENT)
			{
				Node->Max_Current=U8LiteL_KV110_MAX_CURRENT;
			}
			if (Node->Max_Power > U8LiteL_KV110_MAX_POWER)
			{
				Node->Max_Power=U8LiteL_KV110_MAX_POWER;
			}
			Node->Max_Speed=(Node->Max_Speed)/U8LiteL_KV110_POLEPAIRS;//电机最大转速需要计算
		break;

		case VESC_N5065_KV140:
			if (Node->Max_Current > N5065_KV140_MAX_CURRENT)
			{
				Node->Max_Current=N5065_KV140_MAX_CURRENT;
			}
			if (Node->Max_Power > N5065_KV140_MAX_POWER)
			{
				Node->Max_Power=N5065_KV140_MAX_POWER;
			}
			Node->Max_Speed=(Node->Max_Speed)/N5065_KV140_POLEPAIRS;//电机最大转速需要计算
		break;
			
		case RMESC_M3508:
			Node->Max_Current=20000;
		break;
		
		case RMESC_M2006:
			Node->Max_Current=10000;
		break;
	}
}


void MotorNode_Add_PosProtect(Motor_Ctrl_Node* Node,float min_pos,float max_pos)
{
	(Node->Now_CtrlStatus).is_PosProtect=1;
	Node->Max_LPos=min_pos;
	Node->Max_UPos=max_pos;
}

void MotorNode_Add_SpdProtect(Motor_Ctrl_Node* Node,float n_max_spd,float p_max_spd)
{
	(Node->Now_CtrlStatus).is_SpdProtect=1;
	Node->Max_LSpd=n_max_spd;
	Node->Max_USpd=p_max_spd;
}

void MotorNode_Config_PosProtect(Motor_Ctrl_Node* Node,uint8_t new_state)
{
	(Node->Now_CtrlStatus).is_PosProtect=new_state;
}

void MotorNode_Config_SpdProtect(Motor_Ctrl_Node* Node,uint8_t new_state)
{
	(Node->Now_CtrlStatus).is_SpdProtect=new_state;
}

/**
 *@name:Motor_Ctrl_Get_Motor_Speed
 *@brief:获取电机当前的速度
**/
int16_t MotorNode_Get_Speed(Motor_Ctrl_Node* Node)
{
	if (Node->Get_Node_Speed != NULL)
		return Node->Get_Node_Speed(Node->CAN_ID);
	else
	{
		return 0;
		#ifdef RCS_MOTOR_UPCTRL_DEBUG
			RCS_Shell_Logs("Err:Get Spd From a Motor which not initialized before!");
		#endif
	}
	
}
/**
 *@name:Motor_Ctrl_Get_Motor_Angle
 *@brief:获取电机当前的角度
**/
float MotorNode_Get_Angle(Motor_Ctrl_Node* Node)
{
	if (Node->Get_Node_Angle != NULL)
		return Node->Get_Node_Angle(Node->CAN_ID);
	else
	{
		return 0;
		#ifdef RCS_MOTOR_UPCTRL_DEBUG
			RCS_Shell_Logs("Err:Get Pos From a Motor which not initialized before!");
		#endif
	}
}
/**
 *@name:Motor_Ctrl_Get_Motor_Target_Speed
 *@brief:获取电机当前的目标速度
 *@tips:请勿直接通过Node->Spd来获取目标角度，这个获取到的不是正确的目标速度，必须是Node->Out_Spd
**/
int16_t MotorNode_Get_TargetSpeed(Motor_Ctrl_Node* Node)
{
	return Node->Out_Spd;
}
/**
 *@name:Motor_Ctrl_Get_Motor_Target_Angle
 *@brief:获取电机当前的目标角度
 *@tips:请勿直接通过Node->Angle来获取目标角度，这个获取到的不是正确的目标角度，必须是Node->Out_Angle
**/
float MotorNode_Get_TargetAngle(Motor_Ctrl_Node* Node)
{
	return Node->Out_Angle;
}

/**
 * @name:MotorNode_Get_TargetCurrent
 * @brief:获取电机的目标电流
 * @tips:请勿直接通过Node->Current来获取目标角度，这个获取到的不是正确的目标电流，必须是Node->Out_Current
*/
int16_t MotorNode_Get_TargetCurrent(Motor_Ctrl_Node* Node)
{
	return Node->Out_Current;
}

/**
 * @name:Motor_Node_Get_Power
 * @brief:判断当前电机的功率
*/
int16_t Motor_Node_Get_Power(Motor_Ctrl_Node* Node)
{
	if (Node->Get_Node_Power !=NULL)
		return Node->Now_Power=Node->Get_Node_Power(Node->CAN_ID);
	else
	{
		return 0;
		#ifdef RCS_MOTOR_UPCTRL_DEBUG
			RCS_Shell_Logs("Err:Get Power From a Motor which not initialized before!");
		#endif
	}
}
/**
 * @name:MotorNode_Judge_PowerOverload
 * @brief:判断当前电机是否超过最大功率
*/
uint8_t MotorNode_Judge_PowerOverload(Motor_Ctrl_Node* Node)
{
	if (Node->Get_Node_Power !=NULL)
	{
		Node->Now_Power=Node->Get_Node_Power(Node->CAN_ID);
		if (Node->Now_Power > Node->Max_Power)
			return 1;
		else
			return 0;
	}
	else
	{
		return 1;
		#ifdef RCS_MOTOR_UPCTRL_DEBUG
		RCS_Shell_Logs("Err:Get Power From a Motor which not initialized before!");
		#endif
	}
}

uint8_t MotorNode_Judge_Selfcheck(Motor_Ctrl_Node* Node,uint8_t state)
{
	
}



/* ============静态函数=============================================*/
/**
 * @name:Reset_RMESC_OutputCurrent
 * @brief:复位RM电调的缓冲输出值
*/
static inline void Reset_RMESC_OutputCurrent(uint8_t CAN_Group)
{
	memset(RMESC_Current[CAN_Group],0,sizeof(RMESC_Current[CAN_Group]));
	memset(GM6020_Current[CAN_Group],0,sizeof(GM6020_Current[CAN_Group]));
}
/**
 * @name:Update_RMESC_OutputCurrent
 * @brief:更新RM电调的缓冲输出值
*/
static inline void Update_RMESC_OutputCurrent(uint8_t CAN_Group,Motor_Ctrl_Node* Node)
{
	//GM6020
	if (Node->ESC_Type ==ESC_GM6020)
	{
		//更新到缓冲输出值
		GM6020_Current[CAN_Group][Node->CAN_ID]=Node->Current;
		//更新输出状态机
		if (Node->CAN_ID <=4) Teleport_Status[CAN_Group] |= 0B0010;
		else                  Teleport_Status[CAN_Group] |= 0B0001;
		//用完归零，避免控制不连续
		Node->Current=0;
	}
	//RM3508 & RM2006
	else if (Node->ESC_Type ==ESC_C610 ||Node->ESC_Type ==ESC_C620)
	{
		//更新到缓冲输出值
		RMESC_Current[CAN_Group][Node->CAN_ID]=Node->Current;
		//更新输出状态机
		if (Node->CAN_ID <=4) Teleport_Status[CAN_Group] |= 0B1000;
		else                  Teleport_Status[CAN_Group] |= 0B0100;
		//用完归零，避免控制不连续
		Node->Current=0;
	}
}

/**
 * @name:Excute_RMESC_OutputCurrent
 * @brief:执行RM电调的缓冲输出值
*/
static inline void Excute_RMESC_OutputCurrent(uint8_t CAN_Group)
{
	if (CAN_Group == CAN_GROUP_1)
	{
		if ((Teleport_Status[CAN_Group] & 0b0100)>>2)
		Motor_Send_ADD(RMESC_Current[CAN_Group][5],RMESC_Current[CAN_Group][6],RMESC_Current[CAN_Group][7],RMESC_Current[CAN_Group][8]);
		if ((Teleport_Status[CAN_Group] & 0b1000)>>3)
		Motor_Send    (RMESC_Current[CAN_Group][1],RMESC_Current[CAN_Group][2],RMESC_Current[CAN_Group][3],RMESC_Current[CAN_Group][4]);
		if ((Teleport_Status[CAN_Group] & 0b0010)>>1)
		Motor_Send_6020(GM6020_Current[CAN_Group][1],GM6020_Current[CAN_Group][2],GM6020_Current[CAN_Group][3],GM6020_Current[CAN_Group][4]);
		if ((Teleport_Status[CAN_Group] & 0b0001)>>0)
		Motor_Send_6020_ADD(GM6020_Current[CAN_Group][5],GM6020_Current[CAN_Group][6],GM6020_Current[CAN_Group][7]);
	}
	else if (CAN_Group == CAN_GROUP_2)
	{
		if ((Teleport_Status[CAN_Group] & 0b1000)>>3)
		Motor_Send2    (RMESC_Current[CAN_Group][1],RMESC_Current[CAN_Group][2],RMESC_Current[CAN_Group][3],RMESC_Current[CAN_Group][4]);
		if ((Teleport_Status[CAN_Group] & 0b0100)>>2)
		Motor_Send2_ADD(RMESC_Current[CAN_Group][5],RMESC_Current[CAN_Group][6],RMESC_Current[CAN_Group][7],RMESC_Current[CAN_Group][8]);
		if ((Teleport_Status[CAN_Group] & 0b0010)>>1)
		Motor_Send2_6020   (GM6020_Current[CAN_Group][1],GM6020_Current[CAN_Group][2],GM6020_Current[CAN_Group][3],GM6020_Current[CAN_Group][4]);
		if ((Teleport_Status[CAN_Group] & 0b0001)>>0)
		Motor_Send2_6020_ADD(GM6020_Current[CAN_Group][5],GM6020_Current[CAN_Group][6],GM6020_Current[CAN_Group][7]);
	}
}
/**
 * @name:Excute_RMESC_OutputCurrent
 * @brief:更新并执行VESC电调的输出值
*/
static inline void Excute_VESC_Output(uint8_t CAN_Group,Motor_Ctrl_Node* Node)
{
	if (Node->Teleport_Type == TELEPORT_VESC)
	{
	 	//--------------电流控制的本杰明-------------------------
	 	if (Node->Ctrl_Type == CTRL_TYPE_CUR)
	 	{
			if (CAN_Group == CAN_GROUP_1)
	 			VESC_Excute_Current(CAN1,Node->CAN_ID,Node->Current);
	 		else if (CAN_Group == CAN_GROUP_2)
				VESC_Excute_Current(CAN2,Node->CAN_ID,Node->Current);

			Node->Current=0;//用完归零，避免控制不连续
	 	}
	 	//--------------转速控制的本杰明------------------------
	 	else if (Node->Ctrl_Type == CTRL_TYPE_RPM)
	 	{
			if (CAN_Group == CAN_GROUP_1)
	 			VESC_Excute_Speed(CAN1,Node->CAN_ID,Node->Speed);
			else if (CAN_Group == CAN_GROUP_2)
				VESC_Excute_Speed(CAN2,Node->CAN_ID,Node->Speed);
	 		Node->Speed=0;
	 	}
	}
}





static inline int8_t Judge_MotorNode_isPosProtect(Motor_Ctrl_Node* Node)
{
	Node->Now_Angle=Node->Get_Node_Angle(Node->CAN_ID);
	
	if ((Node->Now_Angle >= Node->Max_LPos) && (Node->Now_Angle <= Node->Max_UPos ))
	{
		Node->Now_CtrlStatus.IsrBit_PosProtect=0;
		return 0;
	}
	else if (Node->Now_Angle <= Node->Max_LPos)
	{
		Node->Now_CtrlStatus.IsrBit_PosProtect=-1;
		return -1;
	}
	else if (Node->Now_Angle >= Node->Max_UPos)
	{
		Node->Now_CtrlStatus.IsrBit_PosProtect=1;
		return 1;
	}
}

static inline int8_t Judge_MotorNode_isSpdProtect(Motor_Ctrl_Node* Node)
{
	Node->Now_Speed=Node->Get_Node_Speed(Node->CAN_ID);
	
	if ((Node->Now_Speed >= Node->Max_LSpd) && (Node->Now_Speed <= Node->Max_USpd ))
	{
		Node->Now_CtrlStatus.IsrBit_SpdProtect=0;
		return 0;
	}
	else if (Node->Now_Speed <= Node->Max_LSpd)
	{
		Node->Now_CtrlStatus.IsrBit_SpdProtect=-1;
		return -1;
	}
	else if (Node->Now_Speed >= Node->Max_USpd)
	{
		Node->Now_CtrlStatus.IsrBit_SpdProtect=1;
		return 1;
	}
}

//Motor_List_T CtrlList_CAN_1,CtrlList_CAN_2;
//Motor_List_T CtrlList_PWM_1;
//ESC_Status_t Err_Controller(M_Status_t* target,M_Status_t* now,void* param);
//ESC_Status_t None_Spd_Through_Controller(M_Status_t* target,M_Status_t* now,void* param);
//ESC_Status_t None_Cur_Through_Controller(M_Status_t* target,M_Status_t* now,void* param);
//ESC_Status_t PID_Spd_Cur_Comm_Controller(M_Status_t* target,M_Status_t* now,void* param);
//ESC_Status_t PID_Ang_Cur_Comm_Controller(M_Status_t* target,M_Status_t* now,void* param);
//static uint8_t Invalid_CtrlList_Check(Motor_List_T* CtrlList_XXX_n,Motor_Node_T* Node_tobe_Checked);
//static void Excute_Singl_Ctrl_Motor(Motor_Node_T* Node);
//static uint8_t Update_Multi_Ctrl_Motor(Motor_Node_T* Node);
//static void Excute_Multi_Ctrl_Motor(uint8_t ctrl_flag);

//void MotorNode_Init_C610_New(Motor_Node_T* Node,uint8_t id)
//{
//	//配置ID
//	Node->ESC_ID=id;

//	//配置限幅
//	Node->ESC_Limit.Current=10000;

//	//配置观测器
//	Node->ESC_Observer.Speed_Observer=RM3508_Get_Speed_Rpm;
//	
//	//配置控制器
//	Node->ESC_Controller.Cur_Ctrl_Func=None_Cur_Through_Controller;//2006电流环由C610电调负责闭环控制
//	Node->ESC_Controller.Spd_Ctrl_Func=PID_Spd_Cur_Comm_Controller;//2006速度环采用pid控制器
//	Node->ESC_Controller.Ang_Ctrl_Func=Err_Controller;//2006角度环缺省,需要根据实际情况自行添加

//	
//}

//void MotorNode_Add_New(Motor_List_T* CtrlList_XXX_x,Motor_Node_T* Node)
//{
//	//检查是否超出最大可控制电机数量
//	if (CtrlList_XXX_x->List_Len ==CTRL_LIST_MAX_LEN) 
//	{
//		#ifdef RCS_MOTOR_UPCTRL_DEBUG
//			RCS_Shell_Logs("ERROR:in RCS_Motor_Upctrl.c, add too many motors");
//		#endif
//		return;
//	}

//	//检查是否允许该电机加入该控制组
//	#ifdef RCS_MOTOR_UPCTRL_DEBUG
//		uint8_t err=Invalid_CtrlList_Check(CtrlList_XXX_x,Node);
//		if (err!=0) 
//		{
//			if (err==1) RCS_Shell_Logs("ERR:same teleport in different motor ctrl list");
//			if (err==2) RCS_Shell_Logs("ERR:add same id to list for twice");
//			return;
//		}
//	#endif

//	//将电机节点加入控制组
//	CtrlList_XXX_x->List_Node[CtrlList_XXX_x->List_Len]=Node;
//	CtrlList_XXX_x->List_Len++;
//}

//void MotorList_Excute_New(Motor_List_T* CtrlList_XXX_x)
//{
//	//step1: 要发送的多电机报文标志位
//	uint16_t multi_mesg_flag=0;

//	//step2: 遍历所有节点
//	for(int i=0;i<CtrlList_XXX_x->List_Len;i++)
//	{
//		//单电机报文：直接执行,完成执行后复位输出
//		Excute_Singl_Ctrl_Motor(CtrlList_XXX_x->List_Node[i]);
//		//多电机报文：更新输出值+更新报文标志位,完成更新后复位输出
//		multi_mesg_flag|=Update_Multi_Ctrl_Motor(CtrlList_XXX_x->List_Node[i]);
//	}

//	//step3: 根据报文标志位，发送多电机报文
//	Excute_Multi_Ctrl_Motor(multi_mesg_flag);
//}

//static void Excute_Singl_Ctrl_Motor(Motor_Node_T* Node)
//{
//	static uint8_t bus_type=Node->ESC_Output.Bus_Type;
//	static int32_t output;

//	switch(Node->ESC_Output.Ctrl_Type)
//	{
//		case CTRL_TYPE_CUR:
//			output = Node->ESC_Output.Ctrl_Msg.Current;
//			if      (bus_type==BUS_CAN_1) Node->ESC_Updater.Current_Updater(CAN1,output);
//			else if (bus_type==BUS_CAN_2) Node->ESC_Updater.Current_Updater(CAN2,output);
//		break;

//		case CTRL_TYPE_RPM:
//			output = Node->ESC_Output.Ctrl_Msg.Speed;
//			if      (bus_type==BUS_CAN_1) Node->ESC_Updater.Speed_Updater(CAN1,output);
//			else if (bus_type==BUS_CAN_2) Node->ESC_Updater.Speed_Updater(CAN2,output);
//		break;
//	}
//}
//static uint8_t Update_Multi_Ctrl_Motor(Motor_Node_T* Node)
//{
//	uint8_t reval;
//	Excute_Singl_Ctrl_Motor(Node);
//	switch(Node->ESC_Controller.Cur_Ctrl_Func)
//	{
//		case RM3508_Update_Current
//	}
//}

//static void Excute_Multi_Ctrl_Motor(uint8_t ctrl_flag)
//{

//}
////检查
//static uint8_t Invalid_CtrlList_Check(Motor_List_T* CtrlList_XXX_n,Motor_Node_T* Node_tobe_Checked)
//{
//	uint8_t bus_types=(Node_tobe_Checked->ESC_Output).Bus_Type;
//	uint8_t bus_Id=Node_tobe_Checked->ESC_ID;

//	for(int i=0;i<CtrlList_XXX_n->List_Len;i++)
//	{
//		//原则1:控制报文不在同一条总线上的电机不允许放入同一个控制链表
//		if (CtrlList_XXX_n->List_Node[i]->ESC_Output.Bus_Type != bus_types) return 1;
//		//原则2:同一个总线ID的电机不允许第二次加入链表
//		if (CtrlList_XXX_n->List_Node[i]->ESC_ID == bus_Id) return 2;
//	}

//	return 0;	
//}
////缺省控制器
//inline ESC_Status_t Err_Controller(M_Status_t* target,M_Status_t* now,void* param)
//{
//	ESC_Status_t reval;
//	reval.Ctrl_Type=CTRL_TYPE_CUR;
//	reval.Ctrl_Msg.Current=0;
//	return reval;
//}
////由电调负责控制速度,单片机不添加控制器
//inline ESC_Status_t None_Spd_Through_Controller(M_Status_t* target,M_Status_t* now,void* param)
//{
//	ESC_Status_t reval;
//	reval.Ctrl_Type=CTRL_TYPE_RPM;
//	reval.Ctrl_Msg.Speed=target->Speed;
//	return reval;
//}
////由电调负责控制相电流,单片机不添加控制器
//inline ESC_Status_t None_Cur_Through_Controller(M_Status_t* target,M_Status_t* now,void* param)
//{
//	ESC_Status_t reval;
//	reval.Ctrl_Type=CTRL_TYPE_CUR;
//	reval.Ctrl_Msg.Current=target->Current;
//	return reval;
//}
////速度-电流环PID控制器
//inline ESC_Status_t PID_Spd_Cur_Comm_Controller(M_Status_t* target,M_Status_t* now,void* param)
//{
//	ESC_Status_t reval;
//	//硬件支持电流控制
//	reval.Ctrl_Type=CTRL_TYPE_CUR;
//	//PID返回电流值
//	reval.Ctrl_Msg.Current=PID_Normal_Ctrl(target->Speed,now->Speed,(PID_Struct*)param);
//}
////角度-电流环PID控制器
//inline ESC_Status_t PID_Ang_Cur_Comm_Controller(M_Status_t* target,M_Status_t* now,void* param)
//{
//	ESC_Status_t reval;
//	//硬件支持电流控制
//	reval.Ctrl_Type=CTRL_TYPE_CUR;
//	//PID返回电流值
//	reval.Ctrl_Msg.Current=PID_Normal_Ctrl(target->Speed,now->Speed,(PID_Struct*)param);
//}
