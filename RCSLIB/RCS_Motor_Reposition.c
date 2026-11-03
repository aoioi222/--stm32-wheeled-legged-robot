#include "RCS_Motor_Reposition.h"
#include "EventRecorder.h"
rcsEncoder_getPos_fptr Get_EncoderPos_F[PIN_NUM]={NULL};
float g_Encoder_Pos[PIN_NUM][6]={0};
float	g_Aver_Encoder[PIN_NUM]={0};
float g_Aver_Update_Encoder[PIN_NUM]={0};
void RCS_Motor_Reset(GPIO_TypeDef*	GPIOx,uint16_t	GPIO_Pin_x,uint8_t	Trigger_Mode);
int g_Reset_ir_State[PIN_NUM]={0}; 

RCS_Reset_Err_t motor_reset_init(GPIO_TypeDef*	GPIOx,uint16_t	GPIO_Pin_x,uint8_t _priority)
{
	int Reset_Pin_Mark=GetRCS_EXTI_Pin_Num(GPIO_Pin_x);//获取初始化的是哪个pin 
	if(RCS_InitEXTI(GPIOx,GPIO_Pin_x,_priority)<-1||RCS_EXTI_Register_callback(GPIOx,GPIO_Pin_x,RCS_Motor_Reset)<0)
		return RCS_Reset_EXTI_Init_Fail;
	else return RCS_Reset_OK;
}


RCS_Reset_Err_t motor_reset_ir(rcsMotor_ctrlSpd_fptr	ctrl_f,rcsEncoder_getPos_fptr	sensor_f,
																			GPIO_TypeDef*	GPIOx,uint16_t	GPIO_Pin_x,
																			float*	center_pos,uint8_t CAN_ID,float	LG_Angle_Distance)
{
	int Reset_Pin_Mark=GetRCS_EXTI_Pin_Num(GPIO_Pin_x);//获取初始化的是哪个pin 
	Get_EncoderPos_F[Reset_Pin_Mark]=sensor_f;
	switch(g_Reset_ir_State[Reset_Pin_Mark])
	{
		case 0:
			ctrl_f(Reset_Speed);
			if(g_Encoder_Pos[Reset_Pin_Mark][1]!=0)
			{
				g_Reset_ir_State[Reset_Pin_Mark]=1;
			}
		break;
		case 1:
			ctrl_f(Reset_Speed);
			if(Count_Delay(50,0))
			{
				g_Reset_ir_State[Reset_Pin_Mark]=2;
			}
		break;
		case 2:
			ctrl_f(-Reset_Speed);
			if(g_Aver_Encoder[Reset_Pin_Mark]!=0)
			{
				g_Reset_ir_State[Reset_Pin_Mark]=3;
			}
		break;
		case 3:	
			ctrl_f(-Reset_Speed);
			if(Count_Delay(50,1))
			{
				g_Reset_ir_State[Reset_Pin_Mark]=4;
			}
		break;
		case 4:
			ctrl_f(0);
			*center_pos=g_Aver_Encoder[Reset_Pin_Mark];
			Set_Rep_Agv_Angle(CAN_ID,RCS_Srv_Angle_To_Encoder(*center_pos+LG_Angle_Distance));
			g_Reset_ir_State[Reset_Pin_Mark]=5;
		break;
	}
	if(g_Reset_ir_State[Reset_Pin_Mark]==5)
		return	RCS_Reset_OK;
	else return RCS_Reset_Continue;
}


//-360~360
int check_n;
float check_distance,check_angle;
float g_Aver_Last_Update[PIN_NUM]={0};
RCS_Reset_Err_t motor_reset_angle_update(uint16_t	GPIO_Pin_x,uint8_t CAN_ID,float	LG_Angle_Distance,rcsEncoder_getPos_fptr	Rep_sensor_f)
{
	int Reset_Pin_Mark=GetRCS_EXTI_Pin_Num(GPIO_Pin_x);
	if(g_Reset_ir_State[Reset_Pin_Mark]==5&&g_Aver_Update_Encoder[Reset_Pin_Mark]!=0
				&&g_Aver_Update_Encoder[Reset_Pin_Mark]!=g_Aver_Last_Update[Reset_Pin_Mark])
	{
		g_Aver_Last_Update[Reset_Pin_Mark]=g_Aver_Update_Encoder[Reset_Pin_Mark];
		float Real_Angle=Rep_sensor_f();
		float distance=0;
		int n= (int)(Real_Angle/360); 
    if(Real_Angle < n*360) {
        n--; 
    }
		distance=-360*(n+1)+LG_Angle_Distance;

//		for(int n=0;;n++)
//		{
//			if(Real_Angle>360*n&&Real_Angle<=360*(n+1))
//			{
//					distance=-360*(n+1)+LG_Angle_Distance;
//					break;
//			}
//			n*=-1;
//			if(Real_Angle>360*n&&Real_Angle<=360*(n+1))
//			{
//					distance=-360*(n+1)+LG_Angle_Distance;
//					break;
//			}
//			n*=-1;
//		}
//		if(Real_Angle>0)
//		{
//			distance=LG_Angle_Distance-360.0f;
//		
//		}
//		else
//		{
//			distance=LG_Angle_Distance;
//		}
		Set_Rep_Agv_Angle(CAN_ID,RCS_Srv_Angle_To_Encoder(g_Aver_Update_Encoder[Reset_Pin_Mark]+distance));
		return RCS_Reset_OK;
	}
	else return RCS_Reset_Continue;
}

int g_Reset_State[PIN_NUM]={0};
float fall[4]={0},rise[4]={0};
void RCS_Motor_Reset(GPIO_TypeDef*	GPIOx,uint16_t	GPIO_Pin_x,uint8_t	Trigger_Mode)
{
	int Pin_Mark=GetRCS_EXTI_Pin_Num(GPIO_Pin_x);
		switch(g_Reset_State[Pin_Mark])
			{
			case 0:
				if(Trigger_Mode==RCS_EXTI_FALL)
				{
					g_Encoder_Pos[Pin_Mark][g_Reset_State[Pin_Mark]]=Get_EncoderPos_F[Pin_Mark]();
					g_Reset_State[Pin_Mark]=1;
				}
			break;
			case 1:
				if(Trigger_Mode==RCS_EXTI_RISE)
				{
					g_Encoder_Pos[Pin_Mark][g_Reset_State[Pin_Mark]]=Get_EncoderPos_F[Pin_Mark]();
					g_Reset_State[Pin_Mark]=2;
				}
			break;
			case 2:
				if(Trigger_Mode==RCS_EXTI_FALL)
				{
					g_Encoder_Pos[Pin_Mark][g_Reset_State[Pin_Mark]]=Get_EncoderPos_F[Pin_Mark]();
					g_Reset_State[Pin_Mark]=3;
				}
			break;
			case 3:
				if(Trigger_Mode==RCS_EXTI_RISE)
				{
					g_Encoder_Pos[Pin_Mark][g_Reset_State[Pin_Mark]]=Get_EncoderPos_F[Pin_Mark]();
					g_Aver_Encoder[Pin_Mark]
							=(g_Encoder_Pos[Pin_Mark][0]+g_Encoder_Pos[Pin_Mark][1]
								+g_Encoder_Pos[Pin_Mark][2]+g_Encoder_Pos[Pin_Mark][3])/4;
					g_Reset_State[Pin_Mark]=4;
				}
			break;
			case 4:
				if(Trigger_Mode==RCS_EXTI_FALL)
				{
					g_Encoder_Pos[Pin_Mark][g_Reset_State[Pin_Mark]]=Get_EncoderPos_F[Pin_Mark]();
					g_Aver_Update_Encoder[Pin_Mark]=0;
					g_Reset_State[Pin_Mark]=5;
				}
			break;
			case 5:
				if(Trigger_Mode==RCS_EXTI_RISE)
				{
					g_Encoder_Pos[Pin_Mark][g_Reset_State[Pin_Mark]]=Get_EncoderPos_F[Pin_Mark]();
					g_Aver_Update_Encoder[Pin_Mark]=(g_Encoder_Pos[Pin_Mark][4]+g_Encoder_Pos[Pin_Mark][5])/2;
					g_Reset_State[Pin_Mark]=4;
				}
			break;
			}
}




//int RCS_Kek_Rep_Restart(uint16_t	GPIO_Pin_x)
//{
//	int Reset_Pin_Mark=GetRCS_EXTI_Pin_Num(GPIO_Pin_x);//获取初始化的是哪个pin 
//	g_Reset_ir_State[Reset_Pin_Mark]=0;
//	g_Aver_Encoder[Reset_Pin_Mark]=0;
//	g_Encoder_Pos[Reset_Pin_Mark][1]=0;
//}