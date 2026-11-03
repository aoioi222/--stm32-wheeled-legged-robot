/**
 *@filename:  RCS_MOTOR.c
 *@author     胡兴国、陈煜楷
 *@brief:     CAN网络上的RM电机服务
 *
 *@changlog:  2023-7-28 提供了RM6020的支持
 *@brief:     3508、2006的使用方法完全相同
 *@brief:     6020略有不同，6020的真实ID是表面ID+4
 *@brief:     3508的速度参数范围：-10000~10000
 *@brief:     2006的速度参数范围：-22000~22000
 *@brief:     6020的速度参数范围：-30000~30000
 *
 *@changlog:  2023-11-19 提供vesc电调的控制
 *@brief:     vesc电调走的CAN2.0B拓展数据帧，而RM电机走的2.0B标准帧，因此无需考虑ID重复的问题
 *@brief:     vesc电调把闭环控制做到了固件里，因此需要挂着VESC上位机调PID，单片机只负责向VESC发指令
 *
 *@changelog: 2024-1-30 提供VESC电调报文解包,为所有电机提供统一的防呆框架
 *@brief:     建议直接调用Motor_Update和Motor_Excute来向电机发送命令,而非直接使用这里的函数
 *
 *@changelog: 2024-3-5  将本杰明电调相关内容移出，并修改RCS_MOTOR.c为一个CAN服务
 *@changelog: 2024-3-23 重新
*/
/* ===============头文件=========================================*/
#include "RCS_MOTOR.h"

/* ===============私有全局变量===================================*/
static float g_encoder_start_angle[MAX_RMESC_ON_CAN];
static uint16_t g_encoder_oringin_angle[MAX_RMESC_ON_CAN];
static uint16_t g_encoder_last_angle[MAX_RMESC_ON_CAN];
static int16_t g_encoder_cycles[MAX_RMESC_ON_CAN];
volatile float g_encoder_float_angle[MAX_RMESC_ON_CAN];
volatile int32_t g_encoder_int_angle[MAX_RMESC_ON_CAN];
static int16_t g_encoder_oringin_speed[MAX_RMESC_ON_CAN];
volatile int16_t g_encoder_speed[MAX_RMESC_ON_CAN];

static float g_encoder_start_angle2[MAX_RMESC_ON_CAN];
static uint16_t g_encoder_oringin_angle2[MAX_RMESC_ON_CAN];
static uint16_t g_encoder_last_angle2[MAX_RMESC_ON_CAN];
static int16_t g_encoder_cycles2[MAX_RMESC_ON_CAN];
volatile float g_encoder_float_angle2[MAX_RMESC_ON_CAN];
volatile int32_t g_encoder_int_angle2[MAX_RMESC_ON_CAN];
static int16_t g_encoder_oringin_speed2[MAX_RMESC_ON_CAN];
volatile int16_t g_encoder_speed2[MAX_RMESC_ON_CAN];

volatile int16_t RM3508_Current_toSend[MAX_RMESC_ON_CAN][CAN_PERI_COUNT];
volatile int16_t GM6020_Current_toSend[MAX_RMESC_ON_CAN][CAN_PERI_COUNT];
volatile int16_t GM6020_Voltage_toSend[MAX_RMESC_ON_CAN][CAN_PERI_COUNT];

/* ===============静态函数声明===================================*/
static void RM_Motor_Angle_Init(CAN_TypeDef* CANx);
static void RM_Motor_Encoder_Init(CAN_TypeDef* CANx);
static void Get_Motor_Data(void);
static void Get_Motor_Data2(void);

/* ===============新接口==========================================*/

/**
 * @name:RMMotor_CAN_Init
 * @brief:初始化RM电机
 * @param:CAN_TypeDef* CANx
*/
void RMMotor_CAN_Init(CAN_TypeDef* CANx)
{
	//初始化编码器变量
	RM_Motor_Encoder_Init(CANx);
	
	//配置CAN接收回调服务
	if (CANx==CAN1)
	{
		if (RCS_Judge_CAN_Service(CAN1,RM_Motor_Data_Filter1)==0)
			RCS_CANx_Add_Handler(CAN1,RM_Motor_Data_Filter1);
	}
	else if (CANx==CAN2)
	{
		if (RCS_Judge_CAN_Service(CAN2,RM_Motor_Data_Filter2)==0)
			RCS_CANx_Add_Handler(CAN2,RM_Motor_Data_Filter2);
	}

	//CAN外设引脚映射+配置CAN外设
	if (CANx==CAN1) 
	{
		RCS_CANx_Config_With_Buffer(CAN1_MAP,CAN1_BAUD);//将CAN波特率配置为CAN1_BAUD并绑定到CAN1_MAP引脚上
	}
	else if (CANx==CAN2) 
	{
		RCS_CANx_Config_With_Buffer(CAN2_MAP,CAN2_BAUD);
	}

	//初始化电机角度变量
	delay_ms(10);
	RM_Motor_Angle_Init(CANx);
}
/**
 * @name:RM3508_Update_Current
 * @brief:更新某个3508/2006电机的电流输出值,单位mA
 * @tips:3508不得超过20A，2006不得超过10A
*/
void RM3508_Update_Current(CAN_TypeDef* CANx,uint8_t id,int16_t current)
{
	RM3508_Current_toSend[id][0]=current;
}
/**
 * @name:GM6020_Update_Current
 * @brief:更新6020的电流输出值,单位mA
 * @tips:仅新版6020支持电流控制
 * @tips:最大16384
*/
void GM6020_Update_Current(CAN_TypeDef* CANx,uint8_t id,int16_t current)
{
	if (CANx==CAN1)
		GM6020_Current_toSend[id][0]=current;
	else if (CANx==CAN2)
		GM6020_Current_toSend[id][1]=current;
}
/**
 * @name:GM6020_Update_Voltage
 * @brief:更新6020的电压输出值,单位mV
 * @tips:新旧6020均支持电压控制
 * @tips:最大25000
*/
void GM6020_Update_Voltage(CAN_TypeDef* CANx,uint8_t id,int16_t volt)
{
	if (CANx==CAN1)
		GM6020_Voltage_toSend[id][0]=volt;
	else if (CANx==CAN2)
		GM6020_Voltage_toSend[id][1]=volt;
}

/**
 * @name:RM3508_Excute_Front_Current
 * @brief:让CAN外设发送报文,控制ID1~ID4的3508/2006
*/
void RM3508_Excute_Front_Current(CAN_TypeDef* CANx)
{
	uint8_t current_data[8];
	uint8_t can_chose;

	if (CANx==CAN1)     can_chose=0;
	else if (CANx==CAN2)can_chose=1;

	Int16_to_Int8(RM3508_Current_toSend[1][can_chose], &current_data[0], &current_data[1]);
	Int16_to_Int8(RM3508_Current_toSend[2][can_chose], &current_data[2], &current_data[3]);
	Int16_to_Int8(RM3508_Current_toSend[3][can_chose], &current_data[4], &current_data[5]);
	Int16_to_Int8(RM3508_Current_toSend[4][can_chose], &current_data[6], &current_data[7]);        
	RCS_CANx_Send_STDID(CANx,RM_FRONT, 0x08, current_data);
}
/**
 * @name:RM3508_Excute_Behind_Current
 * @brief:让CAN外设发送报文,控制ID5~ID8的3508/2006
*/
void RM3508_Excute_Behind_Current(CAN_TypeDef* CANx)
{
	uint8_t current_data[8];
	uint8_t can_chose;
	
	if (CANx==CAN1)     can_chose=0;
	else if (CANx==CAN2)can_chose=1;

	Int16_to_Int8(RM3508_Current_toSend[1][can_chose], &current_data[0], &current_data[1]);
	Int16_to_Int8(RM3508_Current_toSend[2][can_chose], &current_data[2], &current_data[3]);
	Int16_to_Int8(RM3508_Current_toSend[3][can_chose], &current_data[4], &current_data[5]);
	Int16_to_Int8(RM3508_Current_toSend[4][can_chose], &current_data[6], &current_data[7]);        
	RCS_CANx_Send_STDID(CANx,RM_FRONT, 0x08, current_data);
}

/**
 * @name:GM6020_Excute_Front_Current
 * @brief:让CAN外设发送报文,控制ID1~ID4的GM6020的电流
 * @tips:不建议将6020的ID设置为1~4
*/
void GM6020_Excute_Front_Current(CAN_TypeDef* CANx)
{
//todo
}
/**
 * @name:GM6020_Excute_Behind_Current
 * @brief:让CAN外设发送报文,控制ID5~ID7的GM6020的电流
*/
void GM6020_Excute_Behind_Current(CAN_TypeDef* CANx)
{
//todo	
}
/**
 * @name:GM6020_Excute_Front_Voltage
 * @brief:让CAN外设发送报文,控制ID1~ID4的GM6020的电压
 * @tips:不建议将6020的ID设置为1~4
*/
void GM6020_Excute_Front_Voltage(CAN_TypeDef* CANx)
{
	uint8_t current_data[8];
	uint8_t can_chose;
	
	if (CANx==CAN1)     can_chose=0;
	else if (CANx==CAN2)can_chose=1;

	Int16_to_Int8(GM6020_Voltage_toSend[1][can_chose], &current_data[0], &current_data[1]);
	Int16_to_Int8(GM6020_Voltage_toSend[2][can_chose], &current_data[2], &current_data[3]);
	Int16_to_Int8(GM6020_Voltage_toSend[3][can_chose], &current_data[4], &current_data[5]);
	Int16_to_Int8(GM6020_Voltage_toSend[4][can_chose], &current_data[6], &current_data[7]);        
	RCS_CANx_Send_STDID(CANx,RM_GM6020_V_FRONT, 0x08, current_data);
}
/**
 * @name:GM6020_Excute_Behind_Voltage
 * @brief:让CAN外设发送报文,控制ID5~ID7的GM6020的电压
*/
void GM6020_Excute_Behind_Voltage(CAN_TypeDef* CANx)
{
	uint8_t current_data[8];
	uint8_t can_chose;
	
	if (CANx==CAN1)     can_chose=0;
	else if (CANx==CAN2)can_chose=1;

	Int16_to_Int8(GM6020_Voltage_toSend[1][can_chose], &current_data[0], &current_data[1]);
	Int16_to_Int8(GM6020_Voltage_toSend[2][can_chose], &current_data[2], &current_data[3]);
	Int16_to_Int8(GM6020_Voltage_toSend[3][can_chose], &current_data[4], &current_data[5]);
	Int16_to_Int8(GM6020_Voltage_toSend[4][can_chose], &current_data[6], &current_data[7]);        
	RCS_CANx_Send_STDID(CANx,RM_GM6020_V_BEHIND, 0x08, current_data);
}

/**
 * @name:RM3508_Get_Angle_Deg
 * @brief:获取3508/2006的尾部的角度制角度
 * @tips:该角度为相对角度,范围负无穷到正无穷。以主控开机时作为0度
 * @tips:该角度的更新频率为50Hz,因为电调的回传报文是50Hz
*/
float RM3508_Get_Angle_Deg(CAN_TypeDef* CANx,uint8_t id)
{
	if (CANx==CAN1)
		return g_encoder_float_angle[id - 1];
	else if (CANx==CAN2)
		return g_encoder_float_angle2[id - 1];
}
/**
 * @name:GM6020_Get_Angle_Deg
 * @brief:获取6020的角度制角度
 * @tips:该角度为绝对角度,范围负无穷到正无穷
 * @tips:该角度的更新频率为50Hz,因为电调的回传报文是50Hz
*/
float GM6020_Get_Angle_Deg(CAN_TypeDef* CANx,uint8_t id)
{
	return RM3508_Get_Angle_Deg(CANx,id+4);
}
/**
 * @name:RM3508_Get_Speed_Rpm
 * @brief:获取3508/2006的尾部的转速
 * @tips:该速度的更新频率为50Hz,因为电调的回传报文是50Hz
*/
int16_t RM3508_Get_Speed_Rpm(CAN_TypeDef* CANx,uint8_t id)
{
	if (CANx==CAN1)
		return g_encoder_speed[id - 1];
	else if (CANx==CAN2)
		return g_encoder_speed2[id - 1];
}
/**
 * @name:GM6020_Get_Speed_Rpm
 * @brief:获取6020的转速
 * @tips:该速度的更新频率为50Hz,因为电调的回传报文是50Hz
*/
int16_t GM6020_Get_Speed_Rpm(CAN_TypeDef* CANx,uint8_t id)
{
	return RM3508_Get_Speed_Rpm(CANx,id+4);
}

/**
 * @addtogroup:减速后的各个参数
*/
float RM3508_Get_Angle_Deg_Sld(CAN_TypeDef* CANx,uint8_t id)
{
	return RM3508_Get_Angle_Deg(CANx,id)/19.0f;
}
float RM2006_Get_Angle_Deg_Sld(CAN_TypeDef* CANx,uint8_t id)
{
	return RM3508_Get_Angle_Deg(CANx,id)/36.0f;
}
float RM3508_Get_Angle_Rad_Sld(CAN_TypeDef* CANx,uint8_t id)
{
	return RM3508_Get_Angle_Deg_Sld(CANx,id)*DEG2RAD;
}
float RM2006_Get_Angle_Rad_Sld(CAN_TypeDef* CANx,uint8_t id)
{
	return RM2006_Get_Angle_Deg_Sld(CANx,id)*DEG2RAD;
}
float RM3508_Get_Speed_Rpm_Sld(CAN_TypeDef* CANx,uint8_t id)
{
	return RM3508_Get_Speed_Rpm(CANx,id)/19.0f;
}
float RM2006_Get_Speed_Rpm_Sld(CAN_TypeDef* CANx,uint8_t id)
{
	return RM3508_Get_Speed_Rpm(CANx,id)/36.0f;
}


/* ===============老代码兼容接口===================================*/

/**
 * @name:Motor_Init
 * @brief:在CAN1上初始化RM电机
*/
void Motor_Init(void)
{
	//初始化编码器变量
	RM_Motor_Encoder_Init(CAN1);
	
	//配置CAN服务
	if (RCS_Judge_CAN_Service(CAN1,RM_Motor_Data_Filter1)==0)
		RCS_CANx_Add_Handler(CAN1,RM_Motor_Data_Filter1);
	
	//配置CAN外设
	RCS_CANx_Config_With_Buffer(CAN1_MAP,CAN1_BAUD);//使用软件buffer控制rm电机

	//初始化电机角度变量
	delay_ms(10);
	RM_Motor_Angle_Init(CAN1);
}

/**
 * @name:Motor_Init2
 * @brief:在CAN2上初始化RM电机
*/
void Motor_Init2(void)
{
//初始化编码器变量
	RM_Motor_Encoder_Init(CAN2);
	
	//配置CAN服务
	if (RCS_Judge_CAN_Service(CAN2,RM_Motor_Data_Filter2)==0)
		RCS_CANx_Add_Handler(CAN2,RM_Motor_Data_Filter2);
	
	//配置CAN外设
	RCS_CANx_Config_With_Buffer(CAN2_MAP,CAN2_BAUD);//使用软件buffer控制rm电机
	
	//初始化电机角度变量
	delay_ms(10);
	RM_Motor_Angle_Init(CAN2);
	
	
}

/**
 @name:Motor_Send
 @brief:通过CAN1控制M2006/3508的输出电流(ID1 ~ ID4)
**/
void Motor_Send(int16_t ID1_Current, int16_t ID2_Current, int16_t ID3_Current, int16_t ID4_Current)
{
	uint8_t current_data[8];

	Int16_to_Int8(ID1_Current, &current_data[0], &current_data[1]);
	Int16_to_Int8(ID2_Current, &current_data[2], &current_data[3]);
	Int16_to_Int8(ID3_Current, &current_data[4], &current_data[5]);
	Int16_to_Int8(ID4_Current, &current_data[6], &current_data[7]);        
	RCS_CANx_Send_STDID(CAN1,RM_FRONT, 0x08, current_data);
}

/**
 @name:Motor_Send_ADD
 @brief:通过CAN1控制M2006/3508的输出电流(ID5 ~ ID8)
**/
void Motor_Send_ADD(int16_t ID5_Current, int16_t ID6_Current, int16_t ID7_Current, int16_t ID8_Current)
{
	uint8_t current_data[8];

	Int16_to_Int8(ID5_Current, &current_data[0], &current_data[1]);
	Int16_to_Int8(ID6_Current, &current_data[2], &current_data[3]);
	Int16_to_Int8(ID7_Current, &current_data[4], &current_data[5]);
	Int16_to_Int8(ID8_Current, &current_data[6], &current_data[7]);        
	RCS_CANx_Send_STDID(CAN1,RM_BEHIND, 0x08, current_data);
}

/**
 @name:Motor_Send2
 @brief:通过CAN2控制M2006/3508的输出电流(ID1 ~ ID4)
**/
void Motor_Send2(int16_t ID1_Current, int16_t ID2_Current, int16_t ID3_Current, int16_t ID4_Current)
{
	uint8_t current_data[8];

	Int16_to_Int8(ID1_Current, &current_data[0], &current_data[1]);
	Int16_to_Int8(ID2_Current, &current_data[2], &current_data[3]);
	Int16_to_Int8(ID3_Current, &current_data[4], &current_data[5]);
	Int16_to_Int8(ID4_Current, &current_data[6], &current_data[7]);
	RCS_CANx_Send_STDID(CAN2,RM_FRONT, 0x08, current_data);
}

/**
 @name:Motor_Send2_ADD
 @brief:通过CAN2控制M2006/3508的输出电流(ID5 ~ ID8)
**/
void Motor_Send2_ADD(int16_t ID5_Current, int16_t ID6_Current, int16_t ID7_Current, int16_t ID8_Current)
{
	uint8_t current_data[8];

	Int16_to_Int8(ID5_Current, &current_data[0], &current_data[1]);
	Int16_to_Int8(ID6_Current, &current_data[2], &current_data[3]);
	Int16_to_Int8(ID7_Current, &current_data[4], &current_data[5]);
	Int16_to_Int8(ID8_Current, &current_data[6], &current_data[7]);
	RCS_CANx_Send_STDID(CAN2,RM_BEHIND, 0x08, current_data);
}

/**
 * @name:GM6020_Send
 * @brief:通过CAN1控制GM6020的电流(ID1~ID4)
 * @param:int16_t IDx_Current 范围-16384~0~16384
 * 
 * @tips:6020没有ID8
 * @tips:老版本的6020不接受电流控制,只能进行电压控制
 * @tips:会产生冲突的发送报文：6020电压控制(ID1~4) <=====>  3508/2006电流控制(ID5~8)   冲突原因：CAN报文标识符一致(0x1FF)
 * @tips:会产生冲突的回传报文: 6020回传报文(ID1~4) <=====>  3508/2006回传报文(ID5~8)   冲突原因：CAN报文标识符一致(0x200+x)
 * @tips:因此，就算6020电流控制(ID1~4)和3508/2006电流控制(ID1~4)不会产生冲突，还是推荐把6020的ID放在5~7，这样无论发送还是接收均不会产生冲突
 * @tips:迫不得已需要把6020的ID放在1~4的话，需要确保6020的发送和回传报文不会和其他电机产生冲突
*/
void GM6020_Send(int16_t ID1_Current,int16_t ID2_Current,int16_t ID3_Current,int16_t ID4_Current)
{
	uint8_t current_data[8];

	Int16_to_Int8(ID1_Current, &current_data[0], &current_data[1]);
	Int16_to_Int8(ID2_Current, &current_data[2], &current_data[3]);
	Int16_to_Int8(ID3_Current, &current_data[4], &current_data[5]);
	Int16_to_Int8(ID4_Current, &current_data[6], &current_data[7]);
	RCS_CANx_Send_STDID(CAN1,RM_GM6020_FRONT, 0x08, current_data);
}

/**
 * @name:GM6020_Send_ADD
 * @brief:通过CAN1控制GM6020的电流(ID5~ID7)
 * @param:int16_t IDx_Current 范围-16384~0~16384
 * @tips:6020没有ID8
 * @tips:老版本的6020不接受电流控制,只能进行电压控制
 * @tips:会产生冲突的发送报文：6020电压控制(ID1~4) <=====>  3508/2006电流控制(ID5~8)   冲突原因：CAN报文标识符一致(0x1FF)
 * @tips:会产生冲突的回传报文: 6020回传报文(ID1~4) <=====>  3508/2006回传报文(ID5~8)   冲突原因：CAN报文标识符一致(0x200+x)
 * @tips:因此，就算6020电流控制(ID1~4)和3508/2006电流控制(ID1~4)不会产生冲突，还是推荐把6020的ID放在5~7，这样无论发送还是接收均不会产生冲突
 * @tips:迫不得已需要把6020的ID放在1~4的话，需要确保6020的发送和回传报文不会和其他电机产生冲突
*/
void GM6020_Send_ADD(int16_t ID5_Current,int16_t ID6_Current,int16_t ID7_Current)
{
	uint8_t current_data[8];

	Int16_to_Int8(ID5_Current, &current_data[0], &current_data[1]);
	Int16_to_Int8(ID6_Current, &current_data[2], &current_data[3]);
	Int16_to_Int8(ID7_Current, &current_data[4], &current_data[5]);

	RCS_CANx_Send_STDID(CAN1,RM_GM6020_BEHIND, 0x08, current_data);
}

/**
 * @name:GM6020_Send2
 * @brief:通过CAN2控制GM6020的电流(ID1~ID4)
 * @param:int16_t IDx_Current 范围-16384~0~16384
 * 
 * @tips:6020没有ID8
 * @tips:老版本的6020不接受电流控制,只能进行电压控制
 * @tips:会产生冲突的发送报文：6020电压控制(ID1~4) <=====>  3508/2006电流控制(ID5~8)   冲突原因：CAN报文标识符一致(0x1FF)
 * @tips:会产生冲突的回传报文: 6020回传报文(ID1~4) <=====>  3508/2006回传报文(ID5~8)   冲突原因：CAN报文标识符一致(0x200+x)
 * @tips:因此，就算6020电流控制(ID1~4)和3508/2006电流控制(ID1~4)不会产生冲突，还是推荐把6020的ID放在5~7，这样无论发送还是接收均不会产生冲突
 * @tips:迫不得已需要把6020的ID放在1~4的话，需要确保6020的发送和回传报文不会和其他电机产生冲突
*/
void GM6020_Send2(int16_t ID1_Current,int16_t ID2_Current,int16_t ID3_Current,int16_t ID4_Current)
{
	uint8_t current_data[8];

	Int16_to_Int8(ID1_Current, &current_data[0], &current_data[1]);
	Int16_to_Int8(ID2_Current, &current_data[2], &current_data[3]);
	Int16_to_Int8(ID3_Current, &current_data[4], &current_data[5]);
	Int16_to_Int8(ID4_Current, &current_data[6], &current_data[7]);
	RCS_CANx_Send_STDID(CAN2,RM_GM6020_FRONT, 0x08, current_data);
}

/**
 * @name:GM6020_Send2_ADD
 * @brief:通过CAN2控制GM6020的电流(ID5~ID7)
 * @param:int16_t IDx_Current 范围-16384~0~16384
 * 
 * @tips:6020没有ID8
 * @tips:老版本的6020不接受电流控制,只能进行电压控制
 * @tips:会产生冲突的发送报文：6020电压控制(ID1~4) <=====>  3508/2006电流控制(ID5~8)   冲突原因：CAN报文标识符一致(0x1FF)
 * @tips:会产生冲突的回传报文: 6020回传报文(ID1~4) <=====>  3508/2006回传报文(ID5~8)   冲突原因：CAN报文标识符一致(0x200+x)
 * @tips:因此，就算6020电流控制(ID1~4)和3508/2006电流控制(ID1~4)不会产生冲突，还是推荐把6020的ID放在5~7，这样无论发送还是接收均不会产生冲突
 * @tips:迫不得已需要把6020的ID放在1~4的话，需要确保6020的发送和回传报文不会和其他电机产生冲突
*/
void GM6020_Send2_ADD(int16_t ID5_Current,int16_t ID6_Current,int16_t ID7_Current)
{
	uint8_t current_data[8];

	Int16_to_Int8(ID5_Current, &current_data[0], &current_data[1]);
	Int16_to_Int8(ID6_Current, &current_data[2], &current_data[3]);
	Int16_to_Int8(ID7_Current, &current_data[4], &current_data[5]);

	RCS_CANx_Send_STDID(CAN2,RM_GM6020_BEHIND, 0x08, current_data);
}

/**
 * @name:Motor_Send_6020
 * @brief:使用电压控制6020的旋转
 * @param:int16_t IDx_Voltage 范围-25000~0~25000
 * 
 * @tips:6020没有ID8
 * @tips:会产生冲突的发送报文：6020电压控制(ID1~4) <=====>  3508/2006电流控制(ID5~8)   冲突原因：CAN报文标识符一致(0x1FF)
 * @tips:会产生冲突的回传报文: 6020回传报文(ID1~4) <=====>  3508/2006回传报文(ID5~8)   冲突原因：CAN报文标识符一致(0x200+x)
 * @tips:因此，就算6020电流控制(ID1~4)和3508/2006电流控制(ID1~4)不会产生冲突，还是推荐把6020的ID放在5~7，这样无论发送还是接收均不会产生冲突
 * @tips:迫不得已需要把6020的ID放在1~4的话，需要确保6020的发送和回传报文不会和其他电机产生冲突
*/
void Motor_Send_6020(int16_t ID1_Voltage, int16_t ID2_Voltage, int16_t ID3_Voltage, int16_t ID4_Voltage)
{
	uint8_t current_data[8];

	Int16_to_Int8(ID1_Voltage, &current_data[0], &current_data[1]);
	Int16_to_Int8(ID2_Voltage, &current_data[2], &current_data[3]);
	Int16_to_Int8(ID3_Voltage, &current_data[4], &current_data[5]);
	Int16_to_Int8(ID4_Voltage, &current_data[6], &current_data[7]);        
	RCS_CANx_Send_STDID(CAN1,RM_GM6020_V_FRONT, 0x08, current_data);
}
void Motor_Send_6020_ADD(int16_t ID5_Voltage, int16_t ID6_Voltage, int16_t ID7_Voltage)
{
	uint8_t current_data[8];

	Int16_to_Int8(ID5_Voltage, &current_data[0], &current_data[1]);
	Int16_to_Int8(ID6_Voltage, &current_data[2], &current_data[3]);
	Int16_to_Int8(ID7_Voltage, &current_data[4], &current_data[5]);

	RCS_CANx_Send_STDID(CAN1,RM_GM6020_V_BEHIND, 0x08, current_data);
}

void Motor_Send2_6020(int16_t ID1_Voltage, int16_t ID2_Voltage, int16_t ID3_Voltage, int16_t ID4_Voltage)
{
	uint8_t current_data[8];

	Int16_to_Int8(ID1_Voltage, &current_data[0], &current_data[1]);
	Int16_to_Int8(ID2_Voltage, &current_data[2], &current_data[3]);
	Int16_to_Int8(ID3_Voltage, &current_data[4], &current_data[5]);
	Int16_to_Int8(ID4_Voltage, &current_data[6], &current_data[7]);
	RCS_CANx_Send_STDID(CAN2,RM_GM6020_V_FRONT, 0x08, current_data);
}
void Motor_Send2_6020_ADD(int16_t ID5_Voltage, int16_t ID6_Voltage, int16_t ID7_Voltage)
{
	uint8_t current_data[8];

	Int16_to_Int8(ID5_Voltage, &current_data[0], &current_data[1]);
	Int16_to_Int8(ID6_Voltage, &current_data[2], &current_data[3]);
	Int16_to_Int8(ID7_Voltage, &current_data[4], &current_data[5]);

	RCS_CANx_Send_STDID(CAN2,RM_GM6020_V_BEHIND, 0x08, current_data);
}


float Get_Motor_Rad_Angle_M3508(uint8_t Motor_ID)
{
	return g_encoder_float_angle[Motor_ID - 1]/19.0f*DEG2RAD;
}
float Get_Motor_Rad_Angle_M2006(uint8_t Motor_ID)
{
	return g_encoder_float_angle[Motor_ID - 1]/36.0f*DEG2RAD;
}
float Get_Motor_Rad_Angle2_M3508(uint8_t Motor_ID)
{
	return g_encoder_float_angle2[Motor_ID - 1]/19.0f*DEG2RAD;
}
float Get_Motor_Rad_Angle2_M2006(uint8_t Motor_ID)
{
	return g_encoder_float_angle2[Motor_ID - 1]/36.0f*DEG2RAD;
}

//获取CAN1浮点电机机械角度,单位“度”，启动时角度为0
float Get_Motor_Float_Angle(uint8_t Motor_ID)
{
	return g_encoder_float_angle[Motor_ID - 1];
}

//获取整形电机机械角度,单位deg
int32_t Get_Motor_Integer_Angle(uint8_t Motor_ID)
{
	return g_encoder_int_angle[Motor_ID - 1];
}

//获取电机转速,单位RPM
int16_t Get_Motor_Speed(uint8_t Motor_ID)
{
	return g_encoder_speed[Motor_ID - 1];
}

//获取CAN2电机角度,单位deg
float Get_Motor_Float_Angle2(uint8_t Motor_ID)
{
	return g_encoder_float_angle2[Motor_ID - 1];
}

//获取CAN2整形电机机械角度,单位deg
int32_t Get_Motor_Integer_Angle2(uint8_t Motor_ID)
{
	return g_encoder_int_angle2[Motor_ID - 1];
}

//获取CAN2电机转速,单位RPM
int16_t Get_Motor_Speed2(uint8_t Motor_ID)
{
	return g_encoder_speed2[Motor_ID - 1];
}
//获取6020的角度CAN1
float Get_GM6020_Float_Angle(uint8_t Motor_ID)
{
	return Get_Motor_Float_Angle(Motor_ID+4);
}
//获取6020的角度CAN2
float Get_GM6020_Float_Angle2(uint8_t Motor_ID)
{
	return Get_Motor_Float_Angle2(Motor_ID+4);
}
//获取6020速度CAN1
int16_t Get_GM6020_Speed(uint8_t Motor_ID)
{
	return Get_Motor_Speed(Motor_ID+4);
}
//获取6020速度CAN2
int16_t Get_GM6020_Speed2(uint8_t Motor_ID)
{
	return Get_Motor_Speed2(Motor_ID+4);
}

//获取RM电调功率
int16_t Get_Motor_Power(uint8_t Motor_ID)
{
	#ifdef RCS_MOTOR_DEBUG
		RCS_Shell_Logs(&Core407_RCSLIB_Debug,"Warning:RMESC`s teleport doesn`t contain power data");
	#endif
	return 0;
}
int16_t Get_Motor_Power2(uint8_t Motor_ID)
{
	#ifdef RCS_MOTOR_DEBUG
		RCS_Shell_Logs(&Core407_RCSLIB_Debug,"Warning:RMESC`s teleport doesn`t contain power data");
	#endif
	return 0;
}

/*******************CAN解包函数********************************/
//RM电机解包
void RM_Motor_Data_Filter1(RCS_CAN2B_DATA_FM_RX rx_message)
{
	if ((rx_message.IDE == CAN_Id_Standard) && (rx_message.RTR == CAN_RTR_Data) && (rx_message.DLC == 0x08)) //标准帧、数据帧长度为8
	{
		//电机号解析
		uint8_t motor_number = (rx_message.StdId & 0x000f);

		//速度数据解析
		g_encoder_oringin_speed[motor_number - 1] = (rx_message.Data[2] << 8) | (rx_message.Data[3]);
		if (g_encoder_oringin_speed[motor_number - 1] <= 3 && g_encoder_oringin_speed[motor_number - 1] >= -3)
			g_encoder_speed[motor_number - 1] = 0;
		else
			g_encoder_speed[motor_number - 1] = g_encoder_oringin_speed[motor_number - 1];

		//位置数据解析
		g_encoder_oringin_angle[motor_number - 1] = (rx_message.Data[0] << 8) | (rx_message.Data[1]);
		//跨0度线判定
		if (abs(g_encoder_oringin_angle[motor_number - 1] - g_encoder_last_angle[motor_number - 1]) > 6000)
		{
				if (g_encoder_last_angle[motor_number - 1] > 4000)g_encoder_cycles[motor_number - 1]++;
				else g_encoder_cycles[motor_number - 1]--;
			}
			g_encoder_last_angle[motor_number - 1] = g_encoder_oringin_angle[motor_number - 1];
			g_encoder_float_angle[motor_number - 1] = 360.0 * g_encoder_cycles[motor_number - 1] + g_encoder_oringin_angle[motor_number - 1] * ENCODER2ANGLE - g_encoder_start_angle[motor_number - 1];
			g_encoder_int_angle[motor_number - 1] = (int32_t)g_encoder_float_angle[motor_number - 1];
		}
}
//RM电机解包
void RM_Motor_Data_Filter2(RCS_CAN2B_DATA_FM_RX rx_message)
{
	if ((rx_message.IDE == CAN_Id_Standard) && (rx_message.RTR == CAN_RTR_Data) && (rx_message.DLC == 0x08)) //标准帧、数据帧长度为8
		{
			//电机号解析
			uint8_t motor_number = (rx_message.StdId & 0x000f);

			//速度数据解析
			g_encoder_oringin_speed2[motor_number - 1] = (rx_message.Data[2] << 8) | (rx_message.Data[3]);
			if (g_encoder_oringin_speed2[motor_number - 1] <= 3 && g_encoder_oringin_speed2[motor_number - 1] >= -3)
				g_encoder_speed2[motor_number - 1] = 0;
			else
				g_encoder_speed2[motor_number - 1] = g_encoder_oringin_speed2[motor_number - 1];

			//位置数据解析
			g_encoder_oringin_angle2[motor_number - 1] = (rx_message.Data[0] << 8) | (rx_message.Data[1]);
			//跨0度线判定
			if (abs(g_encoder_oringin_angle2[motor_number - 1] - g_encoder_last_angle2[motor_number - 1]) > 6000)
			{
				if (g_encoder_last_angle2[motor_number - 1] > 4000)g_encoder_cycles2[motor_number - 1]++;
				else g_encoder_cycles2[motor_number - 1]--;
			}
			g_encoder_last_angle2[motor_number - 1] = g_encoder_oringin_angle2[motor_number - 1];
			g_encoder_float_angle2[motor_number - 1] = 360.0 * g_encoder_cycles2[motor_number - 1] + g_encoder_oringin_angle2[motor_number - 1] * ENCODER2ANGLE - g_encoder_start_angle2[motor_number - 1];
			g_encoder_int_angle2[motor_number - 1] = (int32_t)g_encoder_float_angle2[motor_number - 1];
		}	
}


/* ===============静态函数定义===================================*/

/**
 * @name:RM_Motor_Angle_Init
 * @brief:初始化电机角度变量
*/
static void RM_Motor_Angle_Init(CAN_TypeDef* CANx)
{
	if (CANx==CAN1)
	{
		for (int i = 0; i < 8; i++)
		g_encoder_start_angle[i] = g_encoder_float_angle[i];
	}
	else if (CANx==CAN2)
	{
		for (int i = 0; i < 8; i++)
		g_encoder_start_angle2[i] = g_encoder_float_angle2[i];
	}
}

/**
 * @name:RM_Motor_Encoder_Init
 * @brief:初始化电机编码器变量
*/
static void RM_Motor_Encoder_Init(CAN_TypeDef* CANx)
{
	if (CANx==CAN1)
	{
		for (int i = 0; i < 8; i++)
		{          
			g_encoder_start_angle[i] = 0;
			g_encoder_oringin_angle[i] = 0;
			g_encoder_cycles[i] = 0;
			g_encoder_float_angle[i] = 0;
			g_encoder_int_angle[i] = 0;
			g_encoder_oringin_speed[i] = 0;
			g_encoder_speed[i] = 0;
		}
	}
	else if (CANx==CAN2)
	{
		for (int i = 0; i < 8; i++)
		{           
			g_encoder_start_angle2[i] = 0;
			g_encoder_oringin_angle2[i] = 0;
			g_encoder_cycles2[i] = 0;
			g_encoder_float_angle2[i] = 0;
			g_encoder_int_angle2[i] = 0;
			g_encoder_oringin_speed2[i] = 0;
			g_encoder_speed2[i] = 0;
		}
	}
}

/*******************CAN中断ISR（不启用CAN_Interface时才会被启用）********************************/
//CAN1接收中断
static void Get_Motor_Data(void)
{
	RCS_CAN2B_DATA_FM_RX rx_message;
	if (RCS_CAN_RXIT_FLAG(CAN1) != RESET)
	{
		RCS_CANx_Recieve(CAN1, &rx_message);
		RM_Motor_Data_Filter1(rx_message);
		Bldc_Motor_Data_Filter1(rx_message);
		
	}
}

//CAN2接收中断
static void Get_Motor_Data2(void)
{
	CanRxMsg rx_message;

	if (RCS_CAN_RXIT_FLAG(CAN2) != RESET)
	{
		RCS_CANx_Recieve(CAN2, &rx_message);
		RM_Motor_Data_Filter2(rx_message);
		Bldc_Motor_Data_Filter2(rx_message);
	}
}