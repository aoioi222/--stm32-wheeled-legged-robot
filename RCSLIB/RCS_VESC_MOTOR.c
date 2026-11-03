/**
 * @name:RCS_VESC_MOTOR
 * @brief:为CAN总线提供本杰明电调服务
 * @tips:可在本杰明源码comm_can.c中的
 * @changelog:2024-3-5 CYK-Dot 初次创建
*/

/* ==============头文件========================================*/
#include "RCS_VESC_MOTOR.h"

/* ==============私有全局变量===================================*/
volatile float   vesc_rpm[CAN_PERI_COUNT][MAX_VESC_ON_CAN];     //当前转速
volatile float   vesc_cur[CAN_PERI_COUNT][MAX_VESC_ON_CAN];     //当前电流
volatile float   vesc_pos[CAN_PERI_COUNT][MAX_VESC_ON_CAN];     //当前角度(360)
volatile int16_t vesc_power[CAN_PERI_COUNT][MAX_VESC_ON_CAN];   //当前功率

volatile float vesc_init_pos[CAN_PERI_COUNT][MAX_VESC_ON_CAN];  //初始的真实角度
volatile float vesc_last_pos[CAN_PERI_COUNT][MAX_VESC_ON_CAN];  //上次角度(360)
volatile float vesc_loop_pos[CAN_PERI_COUNT][MAX_VESC_ON_CAN];  //回环角度(正负无穷)


/* ==============静态函数声明===================================*/
static void Bldc_Variable_Memset(CAN_TypeDef* CANx);
static void Bldc_Angle_Init(CAN_TypeDef* CANx);
static inline void Bldc_AngleLoop_Update(void);
static inline void Bldc_AngleLoop_Update2(void);

/* ==============接口函数定义===================================*/

/**
 * @name:VESC_CAN_Init
 * @brief:初始化CAN外设+配置CAN管脚+为CAN配置本杰明电调服务
*/
void VESC_CAN_Init(CAN_TypeDef* CANx)
{
	//全局变量归零
	Bldc_Variable_Memset(CANx);

	//配置CAN服务
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

	//配置CAN外设
	RCS_CANx_Config_With_Buffer(CAN1_MAP,CAN1_BAUD);//使用软件buffer控制rm电机

	//赋值初始角度
	delay_ms(DLY_AFTER_CAN_RDY);
	Bldc_Angle_Init(CANx);
}

/**
 @name:VESC_Excute_Current
 @brief:通过CAN1控制VESC电调的输出电流
 @param:Id,VESC电调的ID
 @param:Current,mA电流
 @tips:由宏VESC_MAX_CURRENT负责保护电调
**/
void VESC_Excute_Current(CAN_TypeDef* CANx,uint16_t ext_id,int16_t current)
{
	volatile uint8_t vesc_cmd_data[4];
	volatile uint32_t vesc_cmd_id;
	
	vesc_cmd_id=(VESC_PACKET_SET_CURRENT<<8) | ext_id;
	Int32_to_Int8(current,vesc_cmd_data);

	RCS_CANx_Send_EXTID(CANx,vesc_cmd_id,4,vesc_cmd_data);
}

/**
 @name:VESC_Excute_Speed
 @brief:通过CAN1控制VESC电调的输出转速(单位ERPM)
**/
void VESC_Excute_Speed(CAN_TypeDef* CANx,uint16_t ext_id,int32_t speed)
{
	volatile uint8_t vesc_cmd_data[4];
	volatile uint32_t vesc_cmd_id;
	
	vesc_cmd_id=(VESC_PACKET_SET_RPM<<8) | ext_id;
	Int32_to_Int8(speed,vesc_cmd_data);
	RCS_CANx_Send_EXTID(CANx,vesc_cmd_id,4,vesc_cmd_data);
}
/**
 @name:VESC_Excute_Angle
 @brief:通过CAN1控制VESC电调的输出角度(单位Deg)
**/
void VESC_Excute_Angle(CAN_TypeDef* CANx,uint16_t ext_id,float angle)
{
	//todo
}


//获取VESC转速,单位ERPM
int16_t VESC_Get_Speed_Erpm(CAN_TypeDef* CANx,uint16_t ext_id)
{
	if (CANx==CAN1)
		return (int16_t)vesc_rpm[0][ext_id];
	else if (CANx==CAN2)
		return (int16_t)vesc_rpm[1][ext_id];
}

//获取VESC磁链角度
float Get_VESC_Pos(uint8_t Motor_ID)
{
	return vesc_pos[0][Motor_ID];
}
//获取VESC转速,单位RPM
int16_t Get_VESC_Speed2(uint8_t Motor_ID)
{
	return (int16_t)vesc_rpm[1][Motor_ID];
}
//获取VESC磁链角度
float Get_VESC_Pos2(uint8_t Motor_ID)
{
	return vesc_pos[1][Motor_ID];
}
//获取VESC功率
//todo:在报文中获取功率而不是自行用电流计算
int16_t Get_VESC_Power(uint8_t Motor_ID)
{
	return vesc_power[0][Motor_ID];
}
//获取VESC功率
//todo:在报文中获取功率而不是自行用电流计算
int16_t Get_VESC_Power2(uint8_t Motor_ID)
{
	return vesc_power[1][Motor_ID];
}


/**
 * @name:Bldc_Motor_Data_Filter1
 * @brief:在CAN1上工作的本杰明接收服务
*/
void Bldc_Motor_Data_Filter1(RCS_CAN2B_DATA_FM_RX rx_message)
{
	if ((rx_message.IDE == CAN_Id_Extended) && (rx_message.RTR == CAN_RTR_Data) && (rx_message.DLC == 0x08)) //拓展帧、数据帧长度为8
	{
		/*--------VESC报文ID解析----------------*/
		volatile uint8_t motor_number = (rx_message.ExtId & 0x00ff);
		volatile uint8_t motor_cmd = rx_message.ExtId >> 8;
		
		/*--------VESC报文内容解析--------------*/
		switch(motor_cmd)
		{
			//1号状态回传报文包括转速和当前电流
			case VESC_PACKET_STATUS:
				vesc_rpm[0][motor_number]=(rx_message.Data[0]<<24)|(rx_message.Data[1]<<16)|(rx_message.Data[2]<<8)|(rx_message.Data[3]);
				vesc_cur[0][motor_number]=((rx_message.Data[4]<<8)|(rx_message.Data[5]))*10;
				vesc_power[0][motor_number]=(int16_t)(BATTERY_VOLTAGE*vesc_cur[0][motor_number]);
			break;
			//4号状态回传报文包括当前角度
			case VESC_PACKET_STATUS_4:
				vesc_pos[0][motor_number]=((rx_message.Data[6]<<8)|(rx_message.Data[7]))*50;
			break;
		}

		/*--------VESC报文处理--------------*/
		Bldc_AngleLoop_Update();
	}	
}
/**
 * @name:Bldc_Motor_Data_Filter2
 * @brief:在CAN2上工作的本杰明接收服务
*/
void Bldc_Motor_Data_Filter2(RCS_CAN2B_DATA_FM_RX rx_message)
{
	if ((rx_message.IDE == CAN_Id_Extended) && (rx_message.RTR == CAN_RTR_Data) && (rx_message.DLC == 0x08)) //拓展帧、数据帧长度为8
	{
		/*--------VESC报文ID解析----------------*/
		volatile uint8_t motor_number = (rx_message.ExtId & 0x00ff);
		volatile uint8_t motor_cmd = rx_message.ExtId >> 8;
		
		/*--------VESC报文内容解析--------------*/
		switch(motor_cmd)
		{
			case VESC_PACKET_STATUS:
				vesc_rpm[1][motor_number]=(rx_message.Data[0]<<24)|(rx_message.Data[1]<<16)|(rx_message.Data[2]<<8)|(rx_message.Data[3]);
				vesc_cur[1][motor_number]=((rx_message.Data[4]<<8)|(rx_message.Data[5]))*10;
				vesc_power[1][motor_number]=(int16_t)(BATTERY_VOLTAGE*vesc_cur[1][motor_number]);
			break;
			
			case VESC_PACKET_STATUS_4:
				vesc_pos[1][motor_number]=((rx_message.Data[6]<<8)|(rx_message.Data[7]))*50;
			
			break;
		}

		/*--------VESC报文处理--------------*/
		Bldc_AngleLoop_Update2();
	}
}

/* ==============静态函数定义===================================*/
static void Bldc_Variable_Memset(CAN_TypeDef* CANx)
{
	if (CANx == CAN1)
	{
		memset(vesc_rpm[0],0,sizeof(vesc_rpm[0]));
		memset(vesc_pos[0],0,sizeof(vesc_pos[0]));
		memset(vesc_cur[0],0,sizeof(vesc_cur[0]));
		memset(vesc_power[0],0,sizeof(vesc_power[0]));
		memset(vesc_init_pos[0],0,sizeof(vesc_init_pos[0]));
		memset(vesc_last_pos[0],0,sizeof(vesc_last_pos[0]));
		memset(vesc_loop_pos[0],0,sizeof(vesc_loop_pos[0]));
	}
	else if (CANx == CAN2)
	{
		memset(vesc_rpm[1],0,sizeof(vesc_rpm[1]));
		memset(vesc_pos[1],0,sizeof(vesc_pos[1]));
		memset(vesc_cur[1],0,sizeof(vesc_cur[1]));
		memset(vesc_power[1],0,sizeof(vesc_power[1]));
		memset(vesc_init_pos[1],0,sizeof(vesc_init_pos[1]));
		memset(vesc_last_pos[1],0,sizeof(vesc_last_pos[1]));
		memset(vesc_loop_pos[1],0,sizeof(vesc_loop_pos[1]));
	}
}
static void Bldc_Angle_Init(CAN_TypeDef* CANx)
{
	if (CANx == CAN1)
	{
		for (int i=0;i<MAX_VESC_ON_CAN;i++)
		{
			vesc_init_pos[0][i]=Get_VESC_Pos(i+1);
		}
	}
}

static inline void Bldc_AngleLoop_Update(void)
{
	for(int i=0;i<MAX_VESC_ON_CAN;i++)
	{
		vesc_loop_pos[0][i]=vesc_loop_pos[0][i] + (vesc_pos[0][i]-vesc_last_pos[0][i]);
		vesc_last_pos[0][i]=vesc_pos[0][i];
	}
}

static inline void Bldc_AngleLoop_Update2(void)
{
	for(int i=0;i<MAX_VESC_ON_CAN;i++)
	{
		vesc_loop_pos[1][i]=vesc_loop_pos[1][i] + (vesc_pos[1][i]-vesc_last_pos[1][i]);
		vesc_last_pos[1][i]=vesc_pos[1][i];
	}
}