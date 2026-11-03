/**
 @filename:RCS_AGV_BaseMove.c
 @brief:舵轮底盘速度环控制
 @date:2023/11/2
 @author:陈煜楷
**/

/***************************************************************
长方形四舵轮底盘,默认CAN1通讯
电机编号、相对坐标系及电机正转时轮子的有效方向如下
        ↑Y
        |
 1 ↑----+---↑2
   |    |   |
   |    +---+----→X
   |        |长
 4 ↑--------↑3
       宽

绝对坐标系定义如下
       ↑Y
       |
       |
       |
     Z +---------→ X      
********************************************************************
三角形三舵轮底盘,默认CAN1通讯
电机编号、相对坐标系及电机正转时轮子的有效方向如下
       ↑Y
       |
       ↑ 1
      / \
     /   \    -------→ X 
    /     \
  2↑-------↑3 
绝对坐标系定义如下
       ↑Y
       |
       |
       |
     Z +---------→ X      
********************************************************************/
#include "RCS_AGV_BaseMove.h"
#include "Cov_AcceLimit_Ctrl.h"
#include "RCS_dsp.h"

/* =================全局变量=========================================*/
PID_Struct      M2006_pid_speed[4],M2006_pid_angle[4];
PID_Struct      M3508_pid_speed[4];
Motor_Ctrl_Node agv_drv[4],agv_srv[4],crazy_fly[2],shoot[2];

float tester;
float watch_angle[4];
float watch_speed[4];
int speed_reverse_flag[4] = {0,0,0,0};
DacePID_Struct AGV_3508[3];

CovDace_Handler_t AGV_CovDace;
StaySample_Handler_t AGV_VESC_Sampler[4];

/* =================中间层函数声明=======================================*/
static __INLINE AGV_Speed AGV_Calculate(Dct_Speed* input);
static __INLINE float AngleLoop(float target_angle, int ID);
void AGV_Execute(AGV_Speed* input);
static __INLINE void AGV_Execute_VESC(AGV_Speed* input);
static __INLINE float AGV_Read_X(void);
static __INLINE float AGV_Read_Y(void);
static __INLINE float AGV_Read_Angle(void);
static void AGV_PID_Init(void);
float Get_IrZero_Pos(uint8_t id);
static void IrZero_Init(void);
void AGV_Silumate_Move(float speed_x, float speed_y, float speed_z);

/* =================接口函数定义=========================================*/
/**
 * @name:AGV_PID_Init
 * @brief:舵轮PID
*/
static void AGV_PID_Init(void)
{
	PID_Struct Srv_Com_Pid[4];
	
	
	
		Srv_Com_Pid[0].P=10;		//10;//18;
		Srv_Com_Pid[0].I=1;		//1;
		Srv_Com_Pid[0].D=7;		//8;//7;
		Srv_Com_Pid[0].Integral=0;
		Srv_Com_Pid[0].Last_Error=0;
		Srv_Com_Pid[0].Limit_Integral=0;
		Srv_Com_Pid[0].Limit_Output=8000;//8000;
		
		Srv_Com_Pid[1].P=10;		//3
		Srv_Com_Pid[1].I=1;		//0.35;
		Srv_Com_Pid[1].D=7;		//0.5;2.5
		Srv_Com_Pid[1].Integral=0;
		Srv_Com_Pid[1].Last_Error=0;
		Srv_Com_Pid[1].Limit_Integral=0;
		Srv_Com_Pid[1].Limit_Output=8000;//8000;

	  
	  Srv_Com_Pid[2].P=10;		//10;//18;5.5
		Srv_Com_Pid[2].I=1;		//1;0
		Srv_Com_Pid[2].D=7;		//8;//7;2.5
		Srv_Com_Pid[2].Integral=0;
		Srv_Com_Pid[2].Last_Error=0;
		Srv_Com_Pid[2].Limit_Integral=0;
		Srv_Com_Pid[2].Limit_Output=8000;//8000;
        

	float U8_VESC_Dace=240.0f;
	AGV_CovDace.Acce_Max[0]=U8_VESC_Dace;
	AGV_CovDace.Acce_Max[1]=U8_VESC_Dace;
	AGV_CovDace.Acce_Max[2]=U8_VESC_Dace;
	AGV_CovDace.Acce_Max[3]=U8_VESC_Dace;
	AGV_CovDace.Dcce_Max[0]=-U8_VESC_Dace;
	AGV_CovDace.Dcce_Max[1]=-U8_VESC_Dace;
	AGV_CovDace.Dcce_Max[2]=-U8_VESC_Dace;
	AGV_CovDace.Dcce_Max[3]=-U8_VESC_Dace;
	
	AGV_VESC_Sampler[0].rate=3;
	AGV_VESC_Sampler[1].rate=3;
	AGV_VESC_Sampler[2].rate=3;
	AGV_VESC_Sampler[3].rate=3;
	
	M2006_pid_speed[0]=PID_Get_RM3508_Speed_Pid();
	M2006_pid_speed[0].Limit_Output=20000;
	M2006_pid_speed[1]=M2006_pid_speed[0];
	M2006_pid_speed[2]=M2006_pid_speed[0];
	M2006_pid_speed[3]=M2006_pid_speed[0];
	M2006_pid_angle[0]=Srv_Com_Pid[0];
	M2006_pid_angle[1]=Srv_Com_Pid[1];
	M2006_pid_angle[2]=Srv_Com_Pid[2];
	M2006_pid_angle[3]=Srv_Com_Pid[3];
	

	
	for(int i = 0; i<2;i++)
	{
		AGV_3508[i].pid.P = 0.5;
		AGV_3508[i].pid.I = 0;
		AGV_3508[i].pid.D = 0;
		(AGV_3508[i].pid).Limit_Output=50.0f;
		AGV_3508[i].max_acce = 1;
		AGV_3508[i].max_dcce = 50;
		AGV_3508[i].dead_zone = 10;
	}
		AGV_3508[2].pid.P = 7.0f;
		AGV_3508[2].pid.I = 0;
		AGV_3508[2].pid.D = 0;
	(AGV_3508[2].pid).Limit_Output=360.0f;		//一秒车体转一整圈为最大速度
		AGV_3508[2].max_acce = 10;
		AGV_3508[2].max_dcce = 50;
		AGV_3508[2].dead_zone = 0.5;
}

float Get_AGV_Max_Speed_X()
{
	return (AGV_3508[0].pid).Limit_Output;
}

float Get_AGV_Max_Speed_Y()
{
	return (AGV_3508[1].pid).Limit_Output;
}

/**
 * @name: AGV_Init
 * @brief:初始化舵轮底盘
*/

void AGV_Init(void)
{
    //PID初始化
    AGV_PID_Init();
    
    //底盘电机节点初始化
    for(int i=0;i<2;i++)
    {
            MotorNode_Init_MBVESC(CAN_GROUP_1,11+i,CTRL_TYPE_RPM,&agv_drv[i]);//初始化U8电机1
        #if Srv_Type == RMESC_M2006
            MotorNode_Init_C610(CAN_GROUP_1,1+i, &agv_srv[i]);
        #elif Srv_Type == RMESC_M3508
            Motor_Ctrl_Init_C620_Motor(CAN_GROUP_1,1+i, &agv_srv[i]);
        #endif
        
        //MotorNode_Add_SpeedPid(&agv_drv[i],&M3508_pid_speed[i]);
        
        agv_srv[i].Motor_Speed_PID=&M2006_pid_speed[i];
        agv_srv[i].Motor_Angle_PID=&M2006_pid_angle[i];
        agv_srv[i].Get_Node_Angle=&Get_IrZero_Pos;
        
        MotorNode_Add(CAN_GROUP_1,&(agv_drv[i]));
        MotorNode_Add(CAN_GROUP_1,&(agv_srv[i]));
        
        agv_srv[i].Angle = 0;
        
    }
		MotorNode_Init_MBVESC(CAN_GROUP_1,13,CTRL_TYPE_RPM,&crazy_fly[0]);
		MotorNode_Add(CAN_GROUP_1,&(crazy_fly[0]));		
		MotorNode_Init_MBVESC(CAN_GROUP_1,14,CTRL_TYPE_RPM,&crazy_fly[1]);
		MotorNode_Add(CAN_GROUP_1,&(crazy_fly[1]));
		
//		MotorNode_Init_MBVESC(CAN_GROUP_2,111,CTRL_TYPE_RPM,&shoot[0]);
//		MotorNode_Add(CAN_GROUP_2,&(shoot[0]));		
//		MotorNode_Init_MBVESC(CAN_GROUP_2,222,CTRL_TYPE_RPM,&shoot[1]);
//		MotorNode_Add(CAN_GROUP_2,&(shoot[1]));

    //底盘电机CAN外设初始化
    Motor_Init();
    IrZero_Init();
}

void AGV_Init2(void)
{
    MotorNode_Init_MBVESC(CAN_GROUP_2,111,CTRL_TYPE_RPM,&shoot[0]);
		MotorNode_Add(CAN_GROUP_2,&(shoot[0]));		
		MotorNode_Init_MBVESC(CAN_GROUP_2,222,CTRL_TYPE_RPM,&shoot[1]);
		MotorNode_Add(CAN_GROUP_2,&(shoot[1]));
	
    Motor_Init2();
    IrZero_Init();
}



//复位
RCS_Reset_Err_t rcsMotor_ctrlSpd_0 (float Spd)
{
	MotorNode_Update_Spd(Spd,&agv_srv[0]);
	return RCS_Reset_OK;
}
RCS_Reset_Err_t rcsMotor_ctrlSpd_1 (float Spd)
{
	MotorNode_Update_Spd(Spd,&agv_srv[1]);
	return RCS_Reset_OK;
}
RCS_Reset_Err_t rcsMotor_ctrlSpd_2 (float Spd)
{
	MotorNode_Update_Spd(Spd,&agv_srv[2]);
	return RCS_Reset_OK;
}
RCS_Reset_Err_t rcsMotor_ctrlSpd_3 (float Spd)
{
	MotorNode_Update_Spd(Spd,&agv_srv[3]);
	return RCS_Reset_OK;
}
float rcsSrv_getPos_fptr_0	(void)
{
	float Agv_Float_Angle0;
	Agv_Float_Angle0=Get_Motor_Float_Angle(1)/36.0f/Srv_Slowdown_Rate;//获得舵轮现实中的角度
	return Agv_Float_Angle0;
}
float rcsSrv_getPos_fptr_1	(void)
{
	float Agv_Float_Angle1;
	Agv_Float_Angle1=Get_Motor_Float_Angle(2)/36.0f/Srv_Slowdown_Rate;
	return Agv_Float_Angle1;
}
float rcsSrv_getPos_fptr_2	(void)
{
	float Agv_Float_Angle2;
	Agv_Float_Angle2=Get_Motor_Float_Angle(3)/36.0f/Srv_Slowdown_Rate;
	return Agv_Float_Angle2;
}
float rcsSrv_getPos_fptr_3	(void)
{
	float Agv_Float_Angle3;
	Agv_Float_Angle3=Get_Motor_Float_Angle(4)/36.0f/Srv_Slowdown_Rate;
	return Agv_Float_Angle3;
}
float rcsSrv_Rep_RealAngle0	(void)
{
	return (MotorNode_Get_Angle(&agv_srv[0])/36.0f/Srv_Slowdown_Rate);
}
	
float rcsSrv_Rep_RealAngle1	(void)
{
	return (MotorNode_Get_Angle(&agv_srv[1])/36.0f/Srv_Slowdown_Rate);
}
float rcsSrv_Rep_RealAngle2	(void)
{
	return (MotorNode_Get_Angle(&agv_srv[2])/36.0f/Srv_Slowdown_Rate);
}
	
float rcsSrv_Rep_RealAngle3	(void)
{
	return (MotorNode_Get_Angle(&agv_srv[3])/36.0f/Srv_Slowdown_Rate);
}

//void rcsAgv0_Update_Angle_3(float angle1,float angle2,float angle3)
//{
//	MotorNode_Update_Angle(angle1*36.0f*Srv_Slowdown_Rate,&agv_srv[0]);
//	MotorNode_Update_Angle(angle2*36.0f*Srv_Slowdown_Rate,&agv_srv[1]);
//	MotorNode_Update_Angle(angle3*36.0f*Srv_Slowdown_Rate,&agv_srv[2]);
//}
void rcsAgv0_Update_Angle_0(float angle1)
{
	MotorNode_Update_Angle(angle1*36.0f*Srv_Slowdown_Rate,&agv_srv[0]);

}
void rcsAgv0_Update_Angle_1(float angle2)
{
	
	MotorNode_Update_Angle(angle2*36.0f*Srv_Slowdown_Rate,&agv_srv[1]);
	
}
void rcsAgv0_Update_Angle_2(float angle3)
{
	MotorNode_Update_Angle(angle3*36.0f*Srv_Slowdown_Rate,&agv_srv[2]);
}


float RCS_Srv_Angle_To_Encoder(float Srv_angle)
{
	return (Srv_angle*36.0f*Srv_Slowdown_Rate);
}
float RCS_Encoder_To_Srv_Angle(float Encoder_angle)
{
	return (Encoder_angle/36.f/Srv_Slowdown_Rate);
}



uint8_t AGV_IrReset(void)
{
	static uint8_t Reset_State;
	float  cheat;
	switch(Reset_State)
	{
		case 0:
			MotorNode_Update_Spd(3500.0f,&agv_srv[0]);
			MotorNode_Update_Spd(3500.0f,&agv_srv[1]);
			MotorNode_Update_Spd(3500.0f,&agv_srv[2]);
			MotorNode_Update_Spd(3500.0f,&agv_srv[3]);
			cheat=MotorNode_Get_Angle(&agv_srv[0]);
			cheat=MotorNode_Get_Angle(&agv_srv[1]);
			cheat=MotorNode_Get_Angle(&agv_srv[2]);
			cheat=MotorNode_Get_Angle(&agv_srv[3]);
			if (Count_Delay(900,0)) Reset_State++;
		break;
		
		case 1:
			MotorNode_Update_Angle(0.0f,&agv_srv[0]);
			MotorNode_Update_Angle(0.0f,&agv_srv[1]);
			MotorNode_Update_Angle(0.0f,&agv_srv[2]);
			MotorNode_Update_Angle(0.0f,&agv_srv[3]);
			if (Count_Delay(900,0)) 
			{
				Reset_State=0;
				return 1;
			}
		break;
	}
	return 0;
}

/**
 @name:AGV_Move
 @brief:三、四舵轮驱动舵向电机移动
 @param:speed_x,舵轮在x方向的线速度(以场地为参考系),单位为m/min
 @param:speed_y,舵轮在y方向的线速度(以场地为参考系),单位为m/min
 @param:rotate_speed,舵轮的自转线速度,单位为m/min
**/
volatile AGV_Speed AGV_Opt_Trace;
void AGV_Move(float move_speed_x, float move_speed_y, float rotate_speed)
{
	Dct_Speed ipt_spd;
	AGV_Speed opt_sta;

	ipt_spd.Dct_vw=rotate_speed*Drv_Wheel_Radius_Rec;//半径一般是小数,和speed数量级相差较大,容易放大误差
	ipt_spd.Dct_vx=move_speed_x*Drv_Wheel_Radius_Rec;
	ipt_spd.Dct_vy=move_speed_y*Drv_Wheel_Radius_Rec;

	opt_sta=AGV_Calculate(&ipt_spd);
	AGV_Opt_Trace=opt_sta;
	AGV_Execute(&opt_sta);
}



uint8_t AGV_Srv_Move(float move_speed_x, float move_speed_y, float rotate_speed)
{
	Dct_Speed ipt_spd;
	AGV_Speed opt_sta;
	float     opt_srv_err[4];
	
	ipt_spd.Dct_vw=rotate_speed*Drv_Wheel_Radius_Rec;//半径一般是小数,和speed数量级相差较大,容易放大误差
	ipt_spd.Dct_vx=move_speed_x*Drv_Wheel_Radius_Rec;
	ipt_spd.Dct_vy=move_speed_y*Drv_Wheel_Radius_Rec;

	opt_sta=AGV_Calculate(&ipt_spd);
	opt_sta.speed[0]=0;
	opt_sta.speed[1]=0;
	opt_sta.speed[2]=0;
	opt_sta.speed[3]=0;
	AGV_Opt_Trace=opt_sta;
	AGV_Execute(&opt_sta);
	
	opt_srv_err[0]=fabsf(MotorNode_Get_Angle(&agv_srv[0])-opt_sta.angle[0]);
	opt_srv_err[1]=fabsf(MotorNode_Get_Angle(&agv_srv[1])-opt_sta.angle[1]);
	opt_srv_err[2]=fabsf(MotorNode_Get_Angle(&agv_srv[2])-opt_sta.angle[2]);
	opt_srv_err[3]=fabsf(MotorNode_Get_Angle(&agv_srv[3])-opt_sta.angle[3]);
	
	if ((opt_srv_err[0]<=100.0f)&&(opt_srv_err[1]<=100.0f)&&(opt_srv_err[2]<=100.0f)&&(opt_srv_err[3]<=100.0f))
		return 1;
	else
		return 0;
}







AGV_Speed AGV_Get_Last_Spd(void)
{
	return AGV_Opt_Trace;
}

/**
 @name:AGV_BaseMove_P2P
 @brief:三、四舵轮驱动舵向电机移动，自动校准舵向角度
 @param:speed_x,舵轮在x方向的线速度(以场地为参考系),单位为m/min
 @param:speed_y,舵轮在y方向的线速度(以场地为参考系),单位为m/min
 @param:rotate_speed,舵轮的自转线速度,单位为m/min
**/
int state_Angle_P2P;
int flag_arrive_Angle_P2P;
void AGV_BaseMove_P2P(float move_speed_x, float move_speed_y, float rotate_speed, float allow_angle_err, float target_speed_max)
{
	switch(state_Angle_P2P)
		{
			case 0:
				AGV_Angle_First(move_speed_x, move_speed_y, rotate_speed, allow_angle_err, &flag_arrive_Angle_P2P);
				if(flag_arrive_Angle_P2P)
				{
					state_Angle_P2P++;
				}
				break;
				
			case 1:
				AGV_Re_Center_Move_v3_3(move_speed_x, move_speed_y, rotate_speed);
				break;
			}
}

void AGV_Angle_First(float move_speed_x, float move_speed_y, float rotate_speed, float allow_angle_err, int8_t* flag_arrive)
{
	AGV_Speed re_center_move;
	
	float Rotate_Angle[4], velocity_x[4], velocity_y[4],R_W2RC[4], a_F, a_B, b_L, b_R	//R_W2RC: R_Wheel_to_Re_Center，轮子到指定中心的距离
		,current_angle, target_angle, composite_speed, wheel_angle[4], current_wheel_angle[4],wheel_angle2[4];	//target_angle是xy速度和与车身x轴的夹角	
	
	current_angle = AGV_Read_Angle() * DEG2RAD;
	target_angle = atan2(move_speed_y, move_speed_x);
	composite_speed = sqrt(move_speed_x * move_speed_x + move_speed_y * move_speed_y);
	
	a_F = - RC_Y + Chassis_Lenth/2.0f;
	a_B = - RC_Y - Chassis_Lenth/2.0f;
	b_L = - RC_X + Chassis_Width/2.0f;
	b_R = - RC_X - Chassis_Width/2.0f;
	
	R_W2RC[0] = sqrt(a_F * a_F + b_L * b_L)*6.0f/100.0f;
	R_W2RC[1] = sqrt(a_F * a_F + b_R * b_R)*6.0f/100.0f;
	R_W2RC[2] = sqrt(a_B * a_B + b_R * b_R)*6.0f/100.0f;
	R_W2RC[3] = sqrt(a_B * a_B + b_L * b_L)*6.0f/100.0f;
	
	Rotate_Angle[0] = atan2(-Chassis_Lenth/2.0f - RC_Y, Chassis_Width/2.0f - RC_X) - pi/2.0f;		//以指定中心展开坐标系的角度
	Rotate_Angle[1] = atan2(-Chassis_Lenth/2.0f - RC_Y, -Chassis_Width/2.0f - RC_X) - pi/2.0f;
	Rotate_Angle[2] = atan2(Chassis_Lenth/2.0f - RC_Y, -Chassis_Width/2.0f - RC_X) - pi/2.0f;
	Rotate_Angle[3] = atan2(Chassis_Lenth/2.0f - RC_Y, Chassis_Width/2.0f - RC_X) - pi/2.0f;
	
	for(int i = 0; i < 4; i++)
	{
		current_wheel_angle[i] = MotorNode_Get_Angle(&agv_srv[i])/Srv_Slowdown_Rate/36.0f;
		velocity_x[i] = composite_speed * cosf(target_angle - current_angle) + rotate_speed *DEG2RAD * R_W2RC[i] * cosf(Rotate_Angle[i]);
		velocity_y[i] = composite_speed * sinf(target_angle - current_angle) + rotate_speed *DEG2RAD * R_W2RC[i] * sinf(Rotate_Angle[i]);
		wheel_angle[i] = RAD2DEG * (atan2(velocity_y[i], velocity_x[i])-pi/2);
		//角度回环&最短路径（注意此处re_center_move.angle为上次更新的轮子角度，在此近似为轮子当前角度，可能导致结果不准确）
//		if(wheel_angle[i] - current_wheel_angle[i] > 90)
//		{
//			while(wheel_angle[i] - current_wheel_angle[i] > 90)
//			{
//				wheel_angle[i] -= 180;
//				speed_reverse_flag[i] ++;
//			}
//			re_center_move.angle[i] = wheel_angle[i];
//		}
//		else if(wheel_angle[i] - current_wheel_angle[i] < -90)
//		{
//			while(wheel_angle[i] - current_wheel_angle[i] < -90)
//			{
//				wheel_angle[i] += 180;
//				speed_reverse_flag[i] ++;
//			}
//			re_center_move.angle[i] = wheel_angle[i];
//		}
//		else
		{
			re_center_move.angle[i] = wheel_angle[i];
		}
		
		re_center_move.speed[i] = 0;
		
		
		//
//		if(wheel_angle[i] - current_wheel_angle[i] > 180)
//		{
//			while(wheel_angle[i] - current_wheel_angle[i] > 180)
//			{
//				wheel_angle[i] -= 360;
//			}
//			wheel_angle2[i] = wheel_angle[i] + 360;
//			if(fabs(wheel_angle2[i] - current_wheel_angle[i]) < fabs(wheel_angle[i] - current_wheel_angle[i]))
//			{
//				re_center_move.angle[i] = wheel_angle2[i];
//			}
//			else
//				re_center_move.angle[i] = wheel_angle[i];
//		}
//		else if(wheel_angle[i] - current_wheel_angle[i] < -180)
//		{
//			while(wheel_angle[i] - current_wheel_angle[i] < -180)
//			{
//				wheel_angle[i] += 360;
//			}
//			wheel_angle2[i] = wheel_angle[i] - 360;
//			if(fabs(wheel_angle2[i] - current_wheel_angle[i]) < fabs(wheel_angle[i] - current_wheel_angle[i]))
//			{
//				re_center_move.angle[i] = wheel_angle2[i];
//			}
//			else
//				re_center_move.angle[i] = wheel_angle[i];
//		}
//		else
//		{
//			re_center_move.angle[i] = wheel_angle[i];
//		}
	}
	AGV_Execute(&re_center_move);
	
	if(re_center_move.angle[0] - allow_angle_err < MotorNode_Get_Angle(&agv_srv[0]) /Srv_Slowdown_Rate/36.0 && re_center_move.angle[0] + allow_angle_err > MotorNode_Get_Angle(&agv_srv[0]) /Srv_Slowdown_Rate/36.0
		&&re_center_move.angle[1] - allow_angle_err < MotorNode_Get_Angle(&agv_srv[1]) /Srv_Slowdown_Rate/36.0 && re_center_move.angle[1] + allow_angle_err > MotorNode_Get_Angle(&agv_srv[1]) /Srv_Slowdown_Rate/36.0
		&&re_center_move.angle[2] - allow_angle_err < MotorNode_Get_Angle(&agv_srv[2]) /Srv_Slowdown_Rate/36.0 && re_center_move.angle[2] + allow_angle_err > MotorNode_Get_Angle(&agv_srv[2]) /Srv_Slowdown_Rate/36.0
		&&re_center_move.angle[3] - allow_angle_err < MotorNode_Get_Angle(&agv_srv[3]) /Srv_Slowdown_Rate/36.0 && re_center_move.angle[3] + allow_angle_err > MotorNode_Get_Angle(&agv_srv[3]) /Srv_Slowdown_Rate/36.0)
	{
		*flag_arrive = 1;
	}
	else
	{
		*flag_arrive = 0;
	}
}

/**
 @name:AGV_Re_Center_Move_v3_3（指定旋转中心的移动，旋转中心由define的RC_X和RC_Y决定）（角速度相同版）（让电调控制速度）
 @param:speed_x,舵轮在x方向的线速度(以场地为参考系),单位为m/min
 @param:speed_y,舵轮在y方向的线速度(以场地为参考系),单位为m/min
@param:rotate_speed,舵轮的自转角速度,单位为deg/s			//原本是deg/min，现在是deg/s，需要注意不要再给很大的自转速度！！
 @note:这里是认为各个旋转角速度相同的情况下投影的。
		和上一版不同之处在于加入了当前角度的读取
		*						
		|\						LF----------RF
		| \						|			 |
		a  \					|			 |
		|   R					LB----------RB
		|    \
		*--b--*
**/
AGV_Speed re_center_move;
AGV_Speed covdace_center_move;
AGV_Speed last_covdace_center_move;
float kokokok;
void AGV_Re_Center_Move_v3_3(float move_speed_x, float move_speed_y, float rotate_speed)
{
	
	
	float Rotate_Angle[4], velocity_x[4], velocity_y[4],R_W2RC[4], a_F, a_B, b_L, b_R	//R_W2RC: R_Wheel_to_Re_Center，轮子到指定中心的距离
		,current_angle, target_angle, composite_speed, wheel_angle[4], current_wheel_angle[2],wheel_angle2[2];	//target_angle是xy速度和与车身x轴的夹角	

	current_angle = Get_Gyro_Z() * DEG2RAD;
	target_angle = atan2(move_speed_y, move_speed_x);
	composite_speed = sqrt(move_speed_x * move_speed_x + move_speed_y * move_speed_y);
	
	a_F = - RC_Y + Chassis_Lenth/2.0f;
	a_B = - RC_Y - Chassis_Lenth/2.0f;
	b_L = - RC_X + Chassis_Width/2.0f;
	b_R = - RC_X - Chassis_Width/2.0f;
	
	R_W2RC[0] = sqrt(a_F * a_F + b_L * b_L)*6.0f/100.0f;
	R_W2RC[1] = sqrt(a_F * a_F + b_R * b_R)*6.0f/100.0f;
	R_W2RC[2] = sqrt(a_B * a_B + b_R * b_R)*6.0f/100.0f;
	R_W2RC[3] = sqrt(a_B * a_B + b_L * b_L)*6.0f/100.0f;
	
	Rotate_Angle[0] = atan2(-Chassis_Lenth/2.0f - RC_Y, Chassis_Width/2.0f - RC_X) - pi/2.0f;		//以指定中心展开坐标系的角度
	Rotate_Angle[1] = atan2(-Chassis_Lenth/2.0f - RC_Y, -Chassis_Width/2.0f - RC_X) - pi/2.0f;
	Rotate_Angle[2] = atan2(Chassis_Lenth/2.0f - RC_Y, -Chassis_Width/2.0f - RC_X) - pi/2.0f;
	Rotate_Angle[3] = atan2(Chassis_Lenth/2.0f - RC_Y, Chassis_Width/2.0f - RC_X) - pi/2.0f;

	
	if(move_speed_x <= FLOAT_ZERO && move_speed_x >= - FLOAT_ZERO
		&& move_speed_y <= FLOAT_ZERO && move_speed_y >= - FLOAT_ZERO
		&& rotate_speed <= FLOAT_ZERO && rotate_speed >= - FLOAT_ZERO)
	{
		re_center_move.speed[0] = 0;
		re_center_move.speed[1] = 0;
		re_center_move.speed[2] = 0;
		re_center_move.speed[3] = 0;
	}
	else
	{
		for(int i = 0; i < 4; i++)
		{
			current_wheel_angle[i] = MotorNode_Get_Angle(&agv_srv[i])/Srv_Slowdown_Rate/36.0f;
			velocity_x[i] = composite_speed * cosf(target_angle - current_angle) + rotate_speed *DEG2RAD * R_W2RC[i] * cosf(Rotate_Angle[i]);
			velocity_y[i] = composite_speed * sinf(target_angle - current_angle) + rotate_speed *DEG2RAD * R_W2RC[i] * sinf(Rotate_Angle[i]);
			wheel_angle[i] = RAD2DEG * (atan2(velocity_y[i], velocity_x[i])-pi/2);
			re_center_move.speed[i] = sqrt(velocity_x[i] * velocity_x[i] + velocity_y[i] * velocity_y[i]);
			//角度回环&最短路径（注意此处re_center_move.angle为上次更新的轮子角度，在此近似为轮子当前角度，可能导致结果不准确）
			if(wheel_angle[i] - current_wheel_angle[i] > 90)
			{
				while(wheel_angle[i] - current_wheel_angle[i] > 90)
				{
					wheel_angle[i] -= 180;
					re_center_move.speed[i] = - re_center_move.speed[i];
				}
				re_center_move.angle[i] = wheel_angle[i];
			}
			else if(wheel_angle[i] - current_wheel_angle[i] < -90)
			{
				while(wheel_angle[i] - current_wheel_angle[i] < -90)
				{
					wheel_angle[i] += 180;
					re_center_move.speed[i] = - re_center_move.speed[i];
				}
				re_center_move.angle[i] = wheel_angle[i];
			}
			else
			{
				re_center_move.angle[i] = wheel_angle[i];
			}
		}
	}
	AGV_Opt_Trace=re_center_move;
	AGV_Execute(&re_center_move);
}

void AGV_Re_Center_Move_v3_4(float move_speed_x, float move_speed_y, float rotate_speed)
{
	float allow_angle_err = 5;
	
	float Rotate_Angle[4], velocity_x[4], velocity_y[4],R_W2RC[4], a_F, a_B, b_L, b_R	//R_W2RC: R_Wheel_to_Re_Center，轮子到指定中心的距离
		,current_angle, target_angle, composite_speed, wheel_angle[4], current_wheel_angle[4],wheel_angle2[4];	//target_angle是xy速度和与车身x轴的夹角	
	
	current_angle = AGV_Read_Angle() * DEG2RAD;
	target_angle = atan2(move_speed_y, move_speed_x);
	composite_speed = sqrt(move_speed_x * move_speed_x + move_speed_y * move_speed_y);
	
	a_F = - RC_Y + Chassis_Lenth/2.0f;
	a_B = - RC_Y - Chassis_Lenth/2.0f;
	b_L = - RC_X + Chassis_Width/2.0f;
	b_R = - RC_X - Chassis_Width/2.0f;
	
	R_W2RC[0] = sqrt(a_F * a_F + b_L * b_L)*6.0f/100.0f;
	R_W2RC[1] = sqrt(a_F * a_F + b_R * b_R)*6.0f/100.0f;
	R_W2RC[2] = sqrt(a_B * a_B + b_R * b_R)*6.0f/100.0f;
	R_W2RC[3] = sqrt(a_B * a_B + b_L * b_L)*6.0f/100.0f;
	
	Rotate_Angle[0] = atan2(-Chassis_Lenth/2.0f - RC_Y, Chassis_Width/2.0f - RC_X) - pi/2.0f;		//以指定中心展开坐标系的角度
	Rotate_Angle[1] = atan2(-Chassis_Lenth/2.0f - RC_Y, -Chassis_Width/2.0f - RC_X) - pi/2.0f;
	Rotate_Angle[2] = atan2(Chassis_Lenth/2.0f - RC_Y, -Chassis_Width/2.0f - RC_X) - pi/2.0f;
	Rotate_Angle[3] = atan2(Chassis_Lenth/2.0f - RC_Y, Chassis_Width/2.0f - RC_X) - pi/2.0f;
	
	if(move_speed_x <= FLOAT_ZERO && move_speed_x >= - FLOAT_ZERO
		&& move_speed_y <= FLOAT_ZERO && move_speed_y >= - FLOAT_ZERO
		&& rotate_speed <= FLOAT_ZERO && rotate_speed >= - FLOAT_ZERO)
	{
		re_center_move.speed[0] = 0;
		re_center_move.speed[1] = 0;
		re_center_move.speed[2] = 0;
		re_center_move.speed[3] = 0;
	}
	else
	{
		for(int i = 0; i < 4; i++)
		{
			current_wheel_angle[i] = MotorNode_Get_Angle(&agv_srv[i])/Srv_Slowdown_Rate/36.0f;
			velocity_x[i] = composite_speed * cosf(target_angle - current_angle) + rotate_speed *DEG2RAD * R_W2RC[i] * cosf(Rotate_Angle[i]);
			velocity_y[i] = composite_speed * sinf(target_angle - current_angle) + rotate_speed *DEG2RAD * R_W2RC[i] * sinf(Rotate_Angle[i]);
			wheel_angle[i] = RAD2DEG * (atan2(velocity_y[i], velocity_x[i])-pi/2);
			re_center_move.speed[i] = sqrt(velocity_x[i] * velocity_x[i] + velocity_y[i] * velocity_y[i]);
			//角度回环&最短路径（注意此处re_center_move.angle为上次更新的轮子角度，在此近似为轮子当前角度，可能导致结果不准确）
			if(wheel_angle[i] - current_wheel_angle[i] > 90)
			{
				while(wheel_angle[i] - current_wheel_angle[i] > 90)
				{
					wheel_angle[i] -= 180;
					re_center_move.speed[i] = - re_center_move.speed[i];
				}
				re_center_move.angle[i] = wheel_angle[i];
			}
			else if(wheel_angle[i] - current_wheel_angle[i] < -90)
			{
				while(wheel_angle[i] - current_wheel_angle[i] < -90)
				{
					wheel_angle[i] += 180;
					re_center_move.speed[i] = - re_center_move.speed[i];
				}
				re_center_move.angle[i] = wheel_angle[i];
			}
			else
			{
				re_center_move.angle[i] = wheel_angle[i];
			}
		}
	}
	
	//在调2006pid时暂时注释，取消注释即可拿来跑
	if(re_center_move.angle[0] - allow_angle_err < MotorNode_Get_Angle(&agv_srv[0]) /Srv_Slowdown_Rate/36.0 && re_center_move.angle[0] + allow_angle_err > MotorNode_Get_Angle(&agv_srv[0]) /Srv_Slowdown_Rate/36.0
		&&re_center_move.angle[1] - allow_angle_err < MotorNode_Get_Angle(&agv_srv[1]) /Srv_Slowdown_Rate/36.0 && re_center_move.angle[1] + allow_angle_err > MotorNode_Get_Angle(&agv_srv[1]) /Srv_Slowdown_Rate/36.0
		&&re_center_move.angle[2] - allow_angle_err < MotorNode_Get_Angle(&agv_srv[2]) /Srv_Slowdown_Rate/36.0 && re_center_move.angle[2] + allow_angle_err > MotorNode_Get_Angle(&agv_srv[2]) /Srv_Slowdown_Rate/36.0
		&&re_center_move.angle[3] - allow_angle_err < MotorNode_Get_Angle(&agv_srv[3]) /Srv_Slowdown_Rate/36.0 && re_center_move.angle[3] + allow_angle_err > MotorNode_Get_Angle(&agv_srv[3]) /Srv_Slowdown_Rate/36.0);
	else
	{
		re_center_move.speed[0] = 0;
		re_center_move.speed[1] = 0;
		re_center_move.speed[2] = 0;
		re_center_move.speed[3] = 0;
	}
	
	AGV_Opt_Trace=re_center_move;
	AGV_Execute(&re_center_move);
}

//一阶xy控制
uint8_t AGV_Coordinate_Controlled_Curve(float start_x, float start_y, float aim_x, float aim_y,
										float start_spd_x, float start_spd_y, float aim_spd_x, float aim_spd_y,
											float z)
{
	float delta_x, delta_y, distance_x, distance_y;
	float speed_x, speed_y, speed_z;
	float now_x, now_y, now_z;
	
	now_x = AGV_Read_X();
	now_y = AGV_Read_Y();
	now_z = AGV_Read_Angle();
	
	distance_x = aim_x - start_x;
	distance_y = aim_y - start_y;
	
	delta_x = AGV_Read_X() - start_x;
	delta_y = AGV_Read_Y() - start_y;
	
	speed_x = delta_x / distance_x * (aim_spd_x - start_spd_x) + start_spd_x;
	speed_y = delta_y / distance_y * (aim_spd_y - start_spd_y) + start_spd_y;
	
	speed_z = DacePID_Normal_Ctrl(z,now_z,&AGV_3508[2]);
	
	AGV_Re_Center_Move_v3_3(speed_x, speed_y, speed_z);
	AGV_Silumate_Move(speed_x, speed_y, speed_z);
	
	if(now_x > aim_x - AGV_3508[0].dead_zone && now_x < aim_x + AGV_3508[0].dead_zone &&
		now_y > aim_y - AGV_3508[1].dead_zone && now_y < aim_y + AGV_3508[1].dead_zone &&
		now_z > z - AGV_3508[2].dead_zone && now_z < z + AGV_3508[2].dead_zone)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}



/**
 @name AGV_Arc_Move 
 @param start_x 起始点x(mm)
 @param start_y 起始点y(mm)
 @param aim_x 终止点x(mm)
 @param aim_y 终止点y(mm)
 @param central_angle 圆心角 ，没有方向性，单位为度
 @param clock_or_anti 轨迹方向，顺时针填CLOCKWISE,逆时针填ANTICLOCKWISE
 @param speed 旋转的线速度，单位为m/min
 @note 记得不要把顺逆时针搞反了噢，不然会有bug;同时要确保轮子的转速是speed的值
实际使用中还有不足，因为速度会降低
**/

int tick = 0;

uint8_t AGV_Arc_Move(float start_x, float start_y, float aim_x, float aim_y, float central_angle, uint8_t clock_or_anti, float speed)
{
	volatile float R;
	volatile float arch_length;
	volatile float time;
	volatile float angle_step;
	volatile float central_angle_rad;
	
	
	central_angle_rad = central_angle * DEG2RAD;
	
	R = sqrtf((((start_x - aim_x)*(start_x - aim_x) + (start_y - aim_y)*(start_y - aim_y)) / 2.0f) / (1.0f - cosf(central_angle_rad)));
	arch_length = R*central_angle_rad;
	
	time = (arch_length / speed) * 12.0f;		//12来自于10^(-3) * 60 * 200
	angle_step = central_angle / time;
			
	if(tick >= time)
	{
		tick = 0;
		return 1;
	}
	else
	{
		switch(clock_or_anti)
		{
			case CLOCKWISE:
				for(int i = 0; i < 4; i++)
				{
											//在此处该函数获取的是上次的角度
					MotorNode_Update_Angle(MotorNode_Get_TargetAngle(&agv_srv[i]) - angle_step*Srv_Slowdown_Rate*36.0f,&agv_srv[i]);
				}
				break;
			case ANTICLOCKWISE:
				for(int i = 0; i < 4; i++)
				{
					MotorNode_Update_Angle(MotorNode_Get_TargetAngle(&agv_srv[i]) + angle_step*Srv_Slowdown_Rate*36.0f,&agv_srv[i]);
				}
				break;
		}
		MotorNode_Update_Spd((float)AGV_Opt_Trace.speed[0]*19.0f/2.0f/pi*Drv_Wheel_Radius_Rec*(-1.0f),&agv_drv[0]);
		MotorNode_Update_Spd((float)AGV_Opt_Trace.speed[1]*19.0f/2.0f/pi*Drv_Wheel_Radius_Rec,&agv_drv[1]);
		MotorNode_Update_Spd((float)AGV_Opt_Trace.speed[2]*19.0f/2.0f/pi*Drv_Wheel_Radius_Rec,&agv_drv[2]);
		MotorNode_Update_Spd((float)AGV_Opt_Trace.speed[3]*19.0f/2.0f/pi*Drv_Wheel_Radius_Rec*(-1.0f),&agv_drv[3]);
		tick ++;
	}
	
	return 0;
}

AGV_Speed AGV_Re_Center_Move_v33_Cac(float move_speed_x, float move_speed_y, float rotate_speed)
{
	
	AGV_Speed re_center_move_cac;
	float Rotate_Angle[4], velocity_x[4], velocity_y[4],R_W2RC[4], a_F, a_B, b_L, b_R	//R_W2RC: R_Wheel_to_Re_Center，轮子到指定中心的距离
		,current_angle, target_angle, composite_speed, wheel_angle[4], current_wheel_angle[4],wheel_angle2[4];	//target_angle是xy速度和与车身x轴的夹角	
	
	current_angle = AGV_Read_Angle() * DEG2RAD;
	target_angle = atan2(move_speed_y, move_speed_x);
	composite_speed = sqrt(move_speed_x * move_speed_x + move_speed_y * move_speed_y);
	
	a_F = - RC_Y + Chassis_Lenth/2;
	a_B = - RC_Y - Chassis_Lenth/2;
	b_L = - RC_X + Chassis_Width/2;
	b_R = - RC_X - Chassis_Width/2;
	
	R_W2RC[0] = sqrt(a_F * a_F + b_L * b_L)/1000.0f;
	R_W2RC[1] = sqrt(a_F * a_F + b_R * b_R)/1000.0f;
	R_W2RC[2] = sqrt(a_B * a_B + b_R * b_R)/1000.0f;
	R_W2RC[3] = sqrt(a_B * a_B + b_L * b_L)/1000.0f;
	
	Rotate_Angle[0] = atan2(-Chassis_Lenth/2 - RC_Y, Chassis_Width/2 - RC_X) - pi/2;		//以指定中心展开坐标系的角度
	Rotate_Angle[1] = atan2(-Chassis_Lenth/2 - RC_Y, -Chassis_Width/2 - RC_X) - pi/2;
	Rotate_Angle[2] = atan2(Chassis_Lenth/2 - RC_Y, -Chassis_Width/2 - RC_X) - pi/2;
	Rotate_Angle[3] = atan2(Chassis_Lenth/2 - RC_Y, Chassis_Width/2 - RC_X) - pi/2;
	
	for(int i = 0; i < 4; i++)
	{
		current_wheel_angle[i] = Get_Motor_Float_Angle(i+1)/Srv_Slowdown_Rate/36;
		velocity_x[i] = composite_speed * cosf(target_angle - current_angle) + rotate_speed *DEG2RAD * R_W2RC[i] * cosf(Rotate_Angle[i]);
		velocity_y[i] = composite_speed * sinf(target_angle - current_angle) + rotate_speed *DEG2RAD * R_W2RC[i] * sinf(Rotate_Angle[i]);
		wheel_angle[i] = RAD2DEG * (atan2(velocity_y[i], velocity_x[i])-pi/2);
		re_center_move_cac.speed[i] = sqrt(velocity_x[i] * velocity_x[i] + velocity_y[i] * velocity_y[i]);
		//角度回环&最短路径（注意此处re_center_move.angle为上次更新的轮子角度，在此近似为轮子当前角度，可能导致结果不准确）
		if(wheel_angle[i] - current_wheel_angle[i] > 90)
		{
			while(wheel_angle[i] - current_wheel_angle[i] > 90)
			{
				wheel_angle[i] -= 180;
				re_center_move_cac.speed[i] = - re_center_move_cac.speed[i];
			}
			re_center_move_cac.angle[i] = wheel_angle[i];
		}
		else if(wheel_angle[i] - current_wheel_angle[i] < -90)
		{
			while(wheel_angle[i] - current_wheel_angle[i] < -90)
			{
				wheel_angle[i] += 180;
				re_center_move_cac.speed[i] = - re_center_move_cac.speed[i];
			}
			re_center_move_cac.angle[i] = wheel_angle[i];
		}
		else
		{
			re_center_move_cac.angle[i] = wheel_angle[i];
		}
	}
	
	return re_center_move_cac;
}
	




void AGV_Re_Center_Move_v33_Srv(float move_speed_x, float move_speed_y, float rotate_speed)
{
	AGV_Speed target_agv_spd;
	
	float Rotate_Angle[4], velocity_x[4], velocity_y[4],R_W2RC[4], a_F, a_B, b_L, b_R	//R_W2RC: R_Wheel_to_Re_Center，轮子到指定中心的距离
		,current_angle, target_angle, composite_speed, wheel_angle[4], current_wheel_angle[4],wheel_angle2[4];	//target_angle是xy速度和与车身x轴的夹角	
	
	current_angle = AGV_Read_Angle() * DEG2RAD;
	target_angle = atan2(move_speed_y, move_speed_x);
	composite_speed = sqrt(move_speed_x * move_speed_x + move_speed_y * move_speed_y);
	
	a_F = - RC_Y + Chassis_Lenth/2;
	a_B = - RC_Y - Chassis_Lenth/2;
	b_L = - RC_X + Chassis_Width/2;
	b_R = - RC_X - Chassis_Width/2;
	
	R_W2RC[0] = sqrt(a_F * a_F + b_L * b_L)/1000.0f;
	R_W2RC[1] = sqrt(a_F * a_F + b_R * b_R)/1000.0f;
	R_W2RC[2] = sqrt(a_B * a_B + b_R * b_R)/1000.0f;
	R_W2RC[3] = sqrt(a_B * a_B + b_L * b_L)/1000.0f;
	
	Rotate_Angle[0] = atan2(-Chassis_Lenth/2 - RC_Y, Chassis_Width/2 - RC_X) - pi/2;		//以指定中心展开坐标系的角度
	Rotate_Angle[1] = atan2(-Chassis_Lenth/2 - RC_Y, -Chassis_Width/2 - RC_X) - pi/2;
	Rotate_Angle[2] = atan2(Chassis_Lenth/2 - RC_Y, -Chassis_Width/2 - RC_X) - pi/2;
	Rotate_Angle[3] = atan2(Chassis_Lenth/2 - RC_Y, Chassis_Width/2 - RC_X) - pi/2;
	
	for(int i = 0; i < 4; i++)
	{
		current_wheel_angle[i] = Get_Motor_Float_Angle(i+1)/Srv_Slowdown_Rate/36;
		velocity_x[i] = composite_speed * cosf(target_angle - current_angle) + rotate_speed *DEG2RAD * R_W2RC[i] * cosf(Rotate_Angle[i]);
		velocity_y[i] = composite_speed * sinf(target_angle - current_angle) + rotate_speed *DEG2RAD * R_W2RC[i] * sinf(Rotate_Angle[i]);
		wheel_angle[i] = RAD2DEG * (atan2(velocity_y[i], velocity_x[i])-pi/2);
		target_agv_spd.speed[i] = sqrt(velocity_x[i] * velocity_x[i] + velocity_y[i] * velocity_y[i]);
		//角度回环&最短路径（注意此处target_agv_spd.angle为上次更新的轮子角度，在此近似为轮子当前角度，可能导致结果不准确）
		if(wheel_angle[i] - current_wheel_angle[i] > 90)
		{
			while(wheel_angle[i] - current_wheel_angle[i] > 90)
			{
				wheel_angle[i] -= 180;
				target_agv_spd.speed[i] = - target_agv_spd.speed[i];
			}
			target_agv_spd.angle[i] = wheel_angle[i];
		}
		else if(wheel_angle[i] - current_wheel_angle[i] < -90)
		{
			while(wheel_angle[i] - current_wheel_angle[i] < -90)
			{
				wheel_angle[i] += 180;
				target_agv_spd.speed[i] = - target_agv_spd.speed[i];
			}
			target_agv_spd.angle[i] = wheel_angle[i];
		}
		else
		{
			target_agv_spd.angle[i] = wheel_angle[i];
		}
	}
	
	target_agv_spd.speed[0]=0;
	target_agv_spd.speed[1]=0;
	target_agv_spd.speed[2]=0;
	target_agv_spd.speed[3]=0;
	
	AGV_Opt_Trace=target_agv_spd;
	AGV_Execute(&target_agv_spd);
}

void Circle_Shift_Ctrl(float target_spd,float shift_spd)
{
	AGV_Opt_Trace.angle[0]+=shift_spd;
	AGV_Opt_Trace.angle[1]+=shift_spd;
	AGV_Opt_Trace.angle[2]+=shift_spd;
	AGV_Opt_Trace.angle[3]+=shift_spd;
	AGV_Execute(&AGV_Opt_Trace);
}	



///////////////////
float Read_AGV_Target_Angle(int num)
{
	return watch_angle[num];
}
float Read_AGV_Target_Speed(int num)
{
	return watch_speed[num];
}







    //
//        if(wheel_angle[i] - current_wheel_angle[i] > 180)
//        {
//            while(wheel_angle[i] - current_wheel_angle[i] > 180)
//            {
//                wheel_angle[i] -= 360;
//            }
//            wheel_angle2[i] = wheel_angle[i] + 180;
//            if(fabs(wheel_angle2[i] - current_wheel_angle[i]) < fabs(wheel_angle[i] - current_wheel_angle[i]))
//            {
//                re_center_move.angle[i] = wheel_angle2[i];
//            }
//            else
//                re_center_move.angle[i] = wheel_angle[i];
//        }
//        else if(wheel_angle[i] - current_wheel_angle[i] < -180)
//        {
//            while(wheel_angle[i] - current_wheel_angle[i] < -180)
//            {
//                wheel_angle[i] += 360;
//            }
//            wheel_angle2[i] = wheel_angle[i] - 180;
//            if(fabs(wheel_angle2[i] - current_wheel_angle[i]) < fabs(wheel_angle[i] - current_wheel_angle[i]))
//            {
//                re_center_move.angle[i] = wheel_angle2[i];
//            }
//            else
//                re_center_move.angle[i] = wheel_angle[i];
//        }
//        else
//        {
            //re_center_mo



float read_tester()
{
	return tester;
}


/**

////////////////////////
 @name	AGV_Wheel_Angle
 @param Wheel_ID 0~3
 @note 把轮子当前角度限制在-pi到pi之内
**/
float assist_angle = 0;	//帮助维持舵角度维持一定范围的辅助角度值
float AGV_Wheel_Angle(uint8_t Wheel_ID)
{
	float current_wheel_angle;
	current_wheel_angle = Get_Motor_Float_Angle(Wheel_ID+1)/Srv_Slowdown_Rate/36 + assist_angle;
	if(current_wheel_angle > 180)
	{
		assist_angle -=360;
		current_wheel_angle = Get_Motor_Float_Angle(Wheel_ID+1)/Srv_Slowdown_Rate/36 + assist_angle;
	}
	else if(current_wheel_angle < -180)
	{
		assist_angle +=360;
		current_wheel_angle = Get_Motor_Float_Angle(Wheel_ID+1)/Srv_Slowdown_Rate/36 + assist_angle;
	}
	return current_wheel_angle;
}

/////////////////////////  2024/6/25
/**
 @name AGV_To_Coordinate
 @param destination_x(mm)
 @param destination_y(mm)
 @param destination_z(°)
**/

float jkjkj_speed_x, jkjkj_speed_y, jkjkj_speed_z;
void AGV_To_Coordinate(float destination_x, float destination_y, float destination_z, int8_t* flag_arrive)
{
	float position_x, position_y, position_z;
	
	
	position_x = AGV_Read_X();
	position_y = AGV_Read_Y();
	position_z = AGV_Read_Angle();
	
	jkjkj_speed_x = DacePID_Normal_Ctrl(destination_x,position_x,&AGV_3508[0]);
	jkjkj_speed_y = DacePID_Normal_Ctrl(destination_y,position_y,&AGV_3508[1]);
	jkjkj_speed_z = DacePID_Normal_Ctrl(destination_z,position_z,&AGV_3508[2]);
	
	AGV_Re_Center_Move_v3_4(jkjkj_speed_x,jkjkj_speed_y,jkjkj_speed_z);
	AGV_Silumate_Move(jkjkj_speed_x,jkjkj_speed_y,jkjkj_speed_z);
	
	if(jkjkj_speed_x == 0 && jkjkj_speed_y == 0 && jkjkj_speed_z == 0)
	{
		*flag_arrive = 1;
	}
	else
	{
		*flag_arrive = 0;
	}
	
}




void AGV_Drv_Debug(int16_t rpm1,int16_t rpm2,int16_t rpm3,int16_t rpm4)
{
	MotorNode_Update_Spd(rpm1,&agv_drv[0]);
	MotorNode_Update_Spd(rpm2,&agv_drv[1]);
	MotorNode_Update_Spd(rpm3,&agv_drv[2]);
	MotorNode_Update_Spd(rpm4,&agv_drv[3]);
	
}

void AGV_Srv_Debug(int16_t cur1,int16_t cur2,int16_t cur3,int16_t cur4)
{
	MotorNode_Update_Angle(cur1,&agv_srv[0]);
	MotorNode_Update_Angle(cur2,&agv_srv[1]);
	MotorNode_Update_Angle(cur3,&agv_srv[2]);
	MotorNode_Update_Angle(cur4,&agv_srv[3]);
}

/* =================中间层函数定义=========================================*/
/**
 @name:AGV_Calculate
 @brief:舵轮逆运动学解算
**/
static __INLINE AGV_Speed AGV_Calculate(Dct_Speed* input)
{
	Dct_Speed Spd_input=*input;
	AGV_Speed reval;
	float robo_angle = AGV_Read_Angle();
	static float speed_x[4] = { 0,0,0,0 };			//实际速度在x轴上的分量
	static float speed_y[4] = { 0,0,0,0 };			//实际速度在y轴上的分量
	
	#if Chassis_Type == Chassis_Type_Triple
	{
		speed_x[0] = (fmaf(rotate_speed,COS_45,(move_speed_x*cosf(robo_angle)+move_speed_y*sinf(robo_angle))));
		speed_y[0] = (fmaf(rotate_speed,-COS_45,(move_speed_y*cosf(robo_angle)-move_speed_x*sinf(robo_angle))));

		speed_x[1] = (fmaf(rotate_speed,COS_45,(move_speed_x*cosf(robo_angle)+move_speed_y*sinf(robo_angle))));
		speed_y[1] = (fmaf(rotate_speed,COS_45,(move_speed_y*cosf(robo_angle)-move_speed_x*sinf(robo_angle))));
		
		speed_x[2] = (fmaf(rotate_speed,-COS_45,(move_speed_x*cosf(robo_angle)+move_speed_y*sinf(robo_angle))));
		speed_y[2] = (fmaf(rotate_speed,COS_45,(move_speed_y*cosf(robo_angle)-move_speed_x*sinf(robo_angle))));

		speed_x[3] = (fmaf(rotate_speed,-COS_45,(move_speed_x*cosf(robo_angle)+move_speed_y*sinf(robo_angle))));
		speed_y[3] = (fmaf(rotate_speed,-COS_45,(move_speed_y*cosf(robo_angle)-move_speed_x*sinf(robo_angle))));
		
		for (int i = 0; i < 3; i++)
		{
			speed[i] = hypotf(speed_x[i], speed_y[i]); 

			angle[i] = -robo_angle - atan2f(speed_x[i], speed_y[i])* RAD2DEG;  
		}

		for (int i = 0; i < 3; i++)
		{
			out_speed[i] = PID_Normal_Ctrl(speed[i], Get_Motor_Speed(i+1), &base_speed_pid[i]);
		}
	}
	#elif Chassis_Type == Chassis_Type_Rectangle
	{
		speed_x[0] = (fmaf(Spd_input.Dct_vw,sinf(atan2f(Chassis_Lenth,Chassis_Width)),(Spd_input.Dct_vx*cosf(robo_angle)+Spd_input.Dct_vy*sinf(robo_angle))));
		speed_y[0] = (fmaf(Spd_input.Dct_vw,cosf(atan2f(Chassis_Lenth,Chassis_Width)),(Spd_input.Dct_vy*cosf(robo_angle)-Spd_input.Dct_vx*sinf(robo_angle))));

		speed_x[2] = (fmaf(Spd_input.Dct_vw,sinf(atan2f(Chassis_Lenth,Chassis_Width)),(Spd_input.Dct_vx*cosf(robo_angle)+Spd_input.Dct_vy*sinf(robo_angle))));
		speed_y[2] = (fmaf(Spd_input.Dct_vw,-cosf(atan2f(Chassis_Lenth,Chassis_Width)),(Spd_input.Dct_vy*cosf(robo_angle)-Spd_input.Dct_vx*sinf(robo_angle))));
		
		speed_x[1] = (fmaf(Spd_input.Dct_vw,-sinf(atan2f(Chassis_Lenth,Chassis_Width)),(Spd_input.Dct_vx*cosf(robo_angle)+Spd_input.Dct_vy*sinf(robo_angle))));
		speed_y[1] = (fmaf(Spd_input.Dct_vw,-cosf(atan2f(Chassis_Lenth,Chassis_Width)),(Spd_input.Dct_vy*cosf(robo_angle)-Spd_input.Dct_vx*sinf(robo_angle))));

		speed_x[3] = (fmaf(Spd_input.Dct_vw,-sinf(atan2f(Chassis_Lenth,Chassis_Width)),(Spd_input.Dct_vx*cosf(robo_angle)+Spd_input.Dct_vy*sinf(robo_angle))));
		speed_y[3] = (fmaf(Spd_input.Dct_vw,cosf(atan2f(Chassis_Lenth,Chassis_Width)),(Spd_input.Dct_vy*cosf(robo_angle)-Spd_input.Dct_vx*sinf(robo_angle))));
		
		for (int i = 0; i < 4; i++)
		{
			reval.speed[i] = hypotf(speed_x[i], speed_y[i]);     //问题2：世界坐标系√

			reval.angle[i] = -robo_angle - atan2f(speed_x[i], speed_y[i])* RAD2DEG;      //问题1：为0的情况   //问题3：角度如0到359 不如0到-1 √
		}
	}
	#endif
	return reval;
}


/* =================舵轮控制-硬件抽象层=========================================*/
/**
 @name:AGV_Execute
 @brief:将解算结果下发给舵轮底盘
**/
void AGV_Execute(AGV_Speed* input)
{
//	AGV_CovDace.Acce_Max[0]=target_acceleration_max;
//	AGV_CovDace.Acce_Max[1]=target_acceleration_max;
//	AGV_CovDace.Acce_Max[2]=target_acceleration_max;
//	AGV_CovDace.Acce_Max[3]=target_acceleration_max;
//	AGV_CovDace.Dcce_Max[0]=-target_acceleration_max;
//	AGV_CovDace.Dcce_Max[1]=-target_acceleration_max;
//	AGV_CovDace.Dcce_Max[2]=-target_acceleration_max;
//	AGV_CovDace.Dcce_Max[3]=-target_acceleration_max;
	
//	CovDace_Limit_Set_DiffData(last_covdace_center_move.speed[0],last_covdace_center_move.speed[1],last_covdace_center_move.speed[2],last_covdace_center_move.speed[3],&AGV_CovDace);
//	
//	CovDace_Limit_Get(input->speed[0],input->speed[1],input->speed[2],input->speed[3],&AGV_CovDace);
//	
//	last_covdace_center_move.speed[0]=SampleCtrl_Get(AGV_CovDace.Outp_Data[0],&AGV_VESC_Sampler[0]);
//	last_covdace_center_move.speed[1]=SampleCtrl_Get(AGV_CovDace.Outp_Data[1],&AGV_VESC_Sampler[1]);
//	last_covdace_center_move.speed[2]=SampleCtrl_Get(AGV_CovDace.Outp_Data[2],&AGV_VESC_Sampler[2]);
//	last_covdace_center_move.speed[3]=SampleCtrl_Get(AGV_CovDace.Outp_Data[3],&AGV_VESC_Sampler[3]);
	
//	MotorNode_Update_Spd(last_covdace_center_move.speed[0],&agv_drv[0]);
//	MotorNode_Update_Spd(last_covdace_center_move.speed[1],&agv_drv[1]);
//	MotorNode_Update_Spd(last_covdace_center_move.speed[2],&agv_drv[2]);
//	MotorNode_Update_Spd(last_covdace_center_move.speed[3],&agv_drv[3]);

	MotorNode_Update_Spd((float)input->speed[0]*19.0f/2.0f/pi*Drv_Wheel_Radius_Rec*(-1.0f),&agv_drv[0]);
	MotorNode_Update_Spd((float)input->speed[1]*19.0f/2.0f/pi*Drv_Wheel_Radius_Rec,&agv_drv[1]);
	
	MotorNode_Update_Angle(input->angle[0]*Srv_Slowdown_Rate*36.0f,&agv_srv[0]);
	MotorNode_Update_Angle(input->angle[1]*Srv_Slowdown_Rate*36.0f,&agv_srv[1]);
}


#define VESC_EXCUTE_COUNT 2
#define VESC_EXCUTE_TIMER 19
/**
 @name:AGV_Execute
 @brief:将解算结果下发给舵轮底盘（让电调来控制轮子速度的版本）
@note: 3.183来自于20/2/pi，再与Drv_Wheel_Radius_Rec相乘即可得到需要给电调发送的RPM，因为在vesc tool发送1500时u8的转速约为75转/min，相除得到20
**/
static __INLINE void AGV_Execute_VESC(AGV_Speed* input)
{
	static AGV_Speed last_spd;
	
	if (Count_Delay(VESC_EXCUTE_COUNT,VESC_EXCUTE_TIMER))
	{
		MotorNode_Update_Spd(input->speed[0]*3.183f*Drv_Wheel_Radius_Rec,&agv_drv[0]);
		MotorNode_Update_Spd(input->speed[1]*3.183f*Drv_Wheel_Radius_Rec,&agv_drv[1]);
		MotorNode_Update_Spd(input->speed[2]*3.183f*Drv_Wheel_Radius_Rec,&agv_drv[2]);
		MotorNode_Update_Spd(input->speed[3]*3.183f*Drv_Wheel_Radius_Rec,&agv_drv[3]);
		last_spd.speed[0]=input->speed[0];
		last_spd.speed[1]=input->speed[1];
		last_spd.speed[2]=input->speed[2];
		last_spd.speed[3]=input->speed[3];
	}
	else
	{
		MotorNode_Update_Spd(last_spd.speed[0]*3.183f*Drv_Wheel_Radius_Rec,&agv_drv[0]);
		MotorNode_Update_Spd(last_spd.speed[1]*3.183f*Drv_Wheel_Radius_Rec,&agv_drv[1]);
		MotorNode_Update_Spd(last_spd.speed[2]*3.183f*Drv_Wheel_Radius_Rec,&agv_drv[2]);
		MotorNode_Update_Spd(last_spd.speed[3]*3.183f*Drv_Wheel_Radius_Rec,&agv_drv[3]);
	}
	
	MotorNode_Update_Angle(input->angle[0]*Srv_Slowdown_Rate*36,&agv_srv[0]);
	MotorNode_Update_Angle(input->angle[1]*Srv_Slowdown_Rate*36,&agv_srv[1]);
	MotorNode_Update_Angle(input->angle[2]*Srv_Slowdown_Rate*36,&agv_srv[2]);
	MotorNode_Update_Angle(input->angle[3]*Srv_Slowdown_Rate*36,&agv_srv[3]);
}

void AGV_Test_VESC(void)
{
	MotorNode_Update_Spd(1500,&agv_drv[0]);
	MotorNode_Update_Spd(1500,&agv_drv[1]);
	MotorNode_Update_Spd(1500,&agv_drv[2]);
	MotorNode_Update_Spd(1500,&agv_drv[3]);
}


float AGV_Pid_Pos[4];
float AGV_current_pos[4];
void AGV_Test_ANGLE(void)
{
	MotorNode_Update_Angle(AGV_Pid_Pos[0],&agv_srv[0]);
	MotorNode_Update_Angle(AGV_Pid_Pos[1],&agv_srv[1]);
	MotorNode_Update_Angle(AGV_Pid_Pos[2],&agv_srv[2]);
	MotorNode_Update_Angle(AGV_Pid_Pos[3],&agv_srv[3]);
//	MotorNode_Update_Spd(AGV_Pid_Pos[0],&agv_srv[0]);
//	MotorNode_Update_Spd(AGV_Pid_Pos[1],&agv_srv[1]);
//	MotorNode_Update_Spd(AGV_Pid_Pos[2],&agv_srv[2]);
//	MotorNode_Update_Spd(AGV_Pid_Pos[3],&agv_srv[3]);
	
	AGV_current_pos[0]=MotorNode_Get_Angle(&agv_srv[0]);
	AGV_current_pos[1]=MotorNode_Get_Angle(&agv_srv[1]);
	AGV_current_pos[2]=MotorNode_Get_Angle(&agv_srv[2]);
	AGV_current_pos[3]=MotorNode_Get_Angle(&agv_srv[3]);
}
uint8_t Inited[4];
uint8_t srv_state[5];
float srv_zero[5];
float Get_IrZero_Pos(uint8_t id)
{	

	return	(Get_Motor_Float_Angle(id)-srv_zero[id]);
}

int Set_Rep_Agv_Angle(uint8_t id,float Center_Agv_Angle)
{
	srv_zero[id]=Center_Agv_Angle;
	agv_srv[id].Get_Node_Angle=&Get_IrZero_Pos;
	return 1;
}

uint8_t AGV_Init_Angle(void)
{
	srv_zero[1]=30.0f*36.0f*Srv_Slowdown_Rate;
	srv_zero[2]=-30.0f*36.0f*Srv_Slowdown_Rate;
	srv_zero[3]=30.0f*36.0f*Srv_Slowdown_Rate;
	srv_zero[4]=-30.0f*36.0f*Srv_Slowdown_Rate;
	return 1;
//	Get_IrZero_Pos(1);
//	
//	if(Inited[0] == 1 && Inited[0] == 1 && Inited[2] == 1 && Inited[3] == 1)
//	{
//		MotorNode_Update_Spd(0,&agv_srv[0]);
//		MotorNode_Update_Spd(0,&agv_srv[1]);
//		MotorNode_Update_Spd(0,&agv_srv[2]);
//		MotorNode_Update_Spd(0,&agv_srv[3]);
//		return 1;
//	}
//	else
//	{
//		MotorNode_Update_Spd(3000,&agv_srv[0]);
//		MotorNode_Update_Spd(3000,&agv_srv[1]);
//		MotorNode_Update_Spd(3000,&agv_srv[2]);
//		MotorNode_Update_Spd(3000,&agv_srv[3]);
//		return 0;
//	}
}

void Zero_CANID1_EXTI_Isr(void)
{
	srv_zero[1]=Get_Motor_Float_Angle(1);
}
void Zero_CANID2_EXTI_Isr(void)
{
	srv_zero[2]=Get_Motor_Float_Angle(2);
}
void Zero_CANID3_EXTI_Isr(void)
{
	srv_zero[3]=Get_Motor_Float_Angle(3);
}
void Zero_CANID4_EXTI_Isr(void)
{
	srv_zero[4]=Get_Motor_Float_Angle(4);
}

static void IrZero_Init(void)
{
	RCS_GPIO_Input_Init(GPIOC,GPIO_Pin_0);
	RCS_GPIO_Input_Init(GPIOC,GPIO_Pin_3);
	RCS_GPIO_Input_Init(GPIOC,GPIO_Pin_4);
	RCS_GPIO_Input_Init(GPIOC,GPIO_Pin_5);
	RCS_GPIO_Input_Init(GPIOB,GPIO_Pin_1);
//	RCS_InitEXTI(GPIOC,GPIO_Pin_0,EXTI_Trigger_Rising_Falling,Zero_CANID3_EXTI_Isr,0x01);//PC0 -- ID3
//	RCS_InitEXTI(GPIOC,GPIO_Pin_3,EXTI_Trigger_Rising_Falling,Zero_CANID2_EXTI_Isr,0x11);//PC3 -- ID2
//	RCS_InitEXTI(GPIOC,GPIO_Pin_4,EXTI_Trigger_Rising_Falling,Zero_CANID4_EXTI_Isr,0x21);//PC4 -- ID4  
//	RCS_InitEXTI(GPIOB,GPIO_Pin_1,EXTI_Trigger_Rising_Falling,Zero_CANID3_EXTI_Isr,0x31);//PB1 -- ID1  
}

float simulate_x, simulate_y, simulate_z;
void AGV_Silumate_Move(float speed_x, float speed_y, float speed_z)
{
	simulate_x += speed_x / 12.0f;
	simulate_y += speed_y / 12.0f;
	simulate_z += speed_z * 0.005f;
}

float Get_Simulate_GPS_X()
{
	return simulate_x;
}

float Get_Simulate_GPS_Y()
{
	return simulate_y;
}

float Get_Simulate_GPS_Z()
{
	return simulate_z;
}

/**
 @name AGV_Read_Angle
 @note 读取底盘当前面向的角度（换陀螺仪则该这里面的函数即可）
**/
static __INLINE float AGV_Read_Angle(void)
{
	return Get_GPS_Z();
}

static __INLINE float AGV_Read_X(void)
{
	return Get_GPS_X();
}
static __INLINE float AGV_Read_Y(void)
{
	return Get_GPS_Y();
}