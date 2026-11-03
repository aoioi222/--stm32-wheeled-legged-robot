#ifndef _RCS_MTR_UPCTRL_H_
#define _RCS_MTR_UPCTRL_H_

/* ------------头文件--------------------------------------*/
#include "RCS_MOTOR.h"
#include "RCS_VESC_MOTOR.h"
#include "RCS_PIDctrl.h"
#include "RCS_AcceCtrl.h"


/* ------------配置宏---------------------------------------*/
#define CTRL_LIST_MAX_LEN     30    //控制链表的最大长度
#define CTRL_LIST_GROUP_COUNT 2     //几路CAN
//#define RCS_MOTOR_UPCTRL_DEBUG

/* ------------导出宏--------------------------------------*/
typedef enum{
	TELEPORT_RMESC =0, //报文类型：机甲大师电调  (CAN标准帧)
	TELEPORT_VESC = 1, //报文类型：VESC电调     (CAN拓展帧)
}MOTOR_TELEPORT_TYPE;       

typedef enum{
	ESC_C610 =0,     //RM电调C610
	ESC_C620 =1,     //RM电调C620
	ESC_GM6020=2,    //6020电调电机一体
	ESC_MBVESC =3,   //本杰明电调创客基地Mini V6.7
}ESC_TYPE;

typedef enum motor_type{
	RMESC_M3508=0,         //M3508电机,最大电流20A
	RMESC_M2006=1,         //M2006电机,最大电流10A
	RMESC_GM602=2,         //GM6020电机,最大
	VESC_U8_LiteL_KV110=3, //U8电机
	VESC_N5065_KV140=4,    //N5065电机
}MOTOR_TYPE;

typedef enum{
	CTRL_TYPE_CUR =0, //控制类型：电流控制,RMESC/VESC均可进行电流控制
	CTRL_TYPE_RPM =1, //控制类型：速度控制,仅有VESC支持速度控制
	CTRL_TYPE_TOR =2, //控制类型：力矩控制
	CTRL_TYPE_POS =3, //控制类型：角度控制
}CTRL_TYPE;

typedef enum{
	CAN_GROUP_1 =0,  //CAN1下的电机组
	CAN_GROUP_2 =1,  //CAN2下的电机组
}MOTOR_CAN_GROUP;

typedef enum{
	MOTOR_ERR = 0,
	MOTOR_FINE= 1,
}MOTOR_SELFCHECK_REVAL;

typedef float    (*FNCT_FLOAT_CAN_U8) (CAN_TypeDef*,uint8_t);
typedef int16_t  (*FNCT_S16_CAN_U8) (CAN_TypeDef*,uint8_t);
typedef void     (*FNCT_VOID_CAN_S16) (CAN_TypeDef*,int16_t);
typedef void     (*FNCT_VOID_CAN_F32) (CAN_TypeDef*,float);
typedef void     (*FNCT_S16_F32_F32) (float,float);

/* ------------参数宏--------------------------------------*/
//配置电调的极限参数
#define MBVESC_MAX_CURRENT 60000       //创客基地VESC最大50A
#define MBVESC_MAX_POWER   1200000     //50*24W
#define MBVESC_MAX_ERPM    150000      //RPM*极对数=ERPM

#define C610_MAX_CURRENT 10000         //C610最大10A
#define C620_MAX_CURRENT 20000         //C620最大20A
#define GM6020_MAX_CURRENT 16384       //6020最大3A
#define GM6020_MAX_VOLTAGE 25000       //6020最大25V 

//配置电机的极限参数
#define U8LiteL_KV110_MAX_CURRENT      45800    //U8在180s的持续电流不得大于45.8A
#define U8LiteL_KV110_MAX_POWER        1790000  //U8最大功率1790W
#define U8LiteL_KV110_POLEPAIRS        21       //U8有36槽42极,即21对

#define N5065_KV140_MAX_CURRENT        80000    //N5065惊人80A
#define N5065_KV140_MAX_POWER          1820000  //1820W   
#define N5065_KV140_POLEPAIRS          7        //5065有14个极,即7对


/* ------------导出结构体 -----------------------------*/



typedef struct Node_Stat{
	int8_t is_PosProtect;
	int8_t is_SpdProtect;
	
	int8_t IsrBit_PosProtect;
	int8_t IsrBit_SpdProtect;
}MotorCtrl_Status;


//电机节点结构体
struct Node{
	//节点参数
	uint8_t     CAN_GROUP;
	uint8_t     CAN_ID;            //总线ID
	uint8_t     Teleport_Type;     //报文型号
	uint8_t     ESC_Type;          //电调型号
	uint8_t     Motor_Type;        //电机型号
	uint8_t     Ctrl_Type;         //控制方式
	PID_Struct* Motor_Speed_PID;   //速度环PID
	PID_Struct* Motor_Angle_PID;   //角度环PID
	DacePID_Struct* Motor_DaceAngle_PID;
	int     Max_Current;       //最大电流(直接限制)
	int     Max_Power;         //最大功率(需要在业务逻辑中应对)
	int     Max_Speed;         //最大速度(直接限制)
	float       Max_UPos;
	float       Max_LPos;
	int     Max_USpd;
	int     Max_LSpd;
	
	//节点状态
	MotorCtrl_Status Now_CtrlStatus;
	int     Now_Speed;         //当前速度
	float       Now_Angle;         //当前角度
	int     Now_Power;         //当前功率
	int     Out_Current;       //目标电流(如果不是电流控制则此项无意义)
	int     Out_Spd;           //目标速度
	float       Out_Angle;         //目标角度
	
	//节点硬件接口
	FNCT_FLOAT_U8 Get_Node_Angle;  //获取电机角度的函数
	FNCT_S16_U8   Get_Node_Speed;  //获取电机转速的函数
	FNCT_S16_U8   Get_Node_Power;  //获取电调功率的函数

	FNCT_VOID_S16 Excute_Node_Current;//CAN总线电流控制指令
	FNCT_VOID_S16 Excute_Node_Speed;  //CAN总线速度控制指令
	FNCT_VOID_F32 Excute_Node_Angle;  //CAN总线角度控制指令

	//节点私有参数(不具备对外调用意义)
	int     Current;           //待更新的输出电流
	int     Speed;             //待更新的目标速度
	float       Angle;             //待更新的目标角度
	float       Torque;            //待更新的目标扭矩
};
typedef struct Node Motor_Ctrl_Node;//把typedef和struct分开写,就能跨文件调用了

//电机控制链表结构体
struct List{
	uint8_t          List_Len;
	Motor_Ctrl_Node* Node_Ctrl[CTRL_LIST_MAX_LEN];//在RTOS下使用malloc构建链表是不安全的
};
typedef struct List Motor_Ctrl_List;

/* ------------导出函数-----------------------------------*/

//函数组1：为电机节点配置电调参数
void MotorNode_Init_C620(uint8_t CAN_Group,uint8_t ID,Motor_Ctrl_Node* Node);
void MotorNode_Init_C610(uint8_t CAN_Group,uint8_t ID,Motor_Ctrl_Node* Node);
void MotorNode_Init_GM6020(uint8_t CAN_Group,uint8_t ID,Motor_Ctrl_Node* Node);
void MotorNode_Init_MBVESC(uint8_t CAN_Group,uint8_t ID,CTRL_TYPE CTRL_TYPE_xxx,Motor_Ctrl_Node* Node);

//函数组2：为电机节点配置电机参数
void MotorNode_Add_BldcProtect(Motor_Ctrl_Node* Node,MOTOR_TYPE Motor);
void MotorNode_Add_PosProtect(Motor_Ctrl_Node* Node,float min_pos,float max_pos);
void MotorNode_Add_SpdProtect(Motor_Ctrl_Node* Node,float n_max_spd,float p_max_spd);
void MotorNode_Add_SpeedPid(Motor_Ctrl_Node* Node,PID_Struct* spd_pid);
void MotorNode_Add_AnglePid(Motor_Ctrl_Node* Node,PID_Struct* angle_pid);
void MotorNode_Add_DaceAnglePid(Motor_Ctrl_Node* Node,DacePID_Struct* dace_pid);

//函数组3：将电机节点送入控制链表
void MotorNode_Add(uint8_t CAN_Group,Motor_Ctrl_Node* Node);

//函数组4：获取电机节点的状态
int16_t MotorNode_Get_Speed(Motor_Ctrl_Node* Node);        //当前：速度
float MotorNode_Get_Angle(Motor_Ctrl_Node* Node);          //当前：角度
int16_t MotorNode_Get_TargetCurrent(Motor_Ctrl_Node* Node);//输出：电流
int16_t MotorNode_Get_TargetSpeed(Motor_Ctrl_Node* Node);  //输出：速度
float MotorNode_Get_TargetAngle(Motor_Ctrl_Node* Node);    //输出：角度
uint8_t MotorNode_Judge_PowerOverload(Motor_Ctrl_Node* Node);
uint8_t MotorNode_Judge_Selfcheck(Motor_Ctrl_Node* Node,uint8_t state);
//函数组5：更新电机节点的输出值
void MotorNode_Update_Current(int16_t Cur,Motor_Ctrl_Node* Node);//直接更新电机输出电流
void MotorNode_Update_Spd(int Spd,Motor_Ctrl_Node* Node);//指定速度并更新电流
void MotorNode_Update_Angle(float angle,Motor_Ctrl_Node* Node);//指定角度更新电流
uint8_t MotorNode_Update_AngleEasy(float angle,float speed,float tor_angle,Motor_Ctrl_Node* Node);//指定误差范围的简单角度控制
uint8_t MotorNode_Update_AngleFull(float angle,Motor_Ctrl_Node* Node);

void MotorNode_Config_PosProtect(Motor_Ctrl_Node* Node,uint8_t new_state);
void MotorNode_Config_SpdProtect(Motor_Ctrl_Node* Node,uint8_t new_state);

//函数组6：将全部控制信息下发给电调
void MotorList_Excute(uint8_t CAN_Group);	


#endif