/*********************************************************************
*@filename: RCS_AGV.c
*@brief:    四舵轮控制算法
*@changelog:2024-9-21 CYK-Dot 初次创建,整理自RC24赛季
---------------------------------------------------------------------
*********************************************************************/

/** 头文件引入 -----------------------------------------------------------**/
#include "RCS_AGV.h"


/** 私有宏定义 -----------------------------------------------------------**/

/** 全局变量定义 ---------------------------------------------------------**/

/** 静态函数声明 ---------------------------------------------------------**/

/** 导出函数定义 ---------------------------------------------------------**/

/**
*@name: AGV4_Kin
*@brief:送入目标(x,y,z)速度，计算四舵轮底盘的驱动轮速度和航向轮角度
*@param:chassis       |INPUT    |底盘参数
*@param:spd_dct       |INPUT    |相对场地坐标系的笛卡尔坐标系速度
*@param:pos_z_rad     |INPUT    |底盘相对场地坐标系的角度
*@reval:AGV4_KinParam |OUTPUT   |计算得到的速度和角度
**/
AGV4_Status AGV4_InvKin(AGV4_CtrlParam* chassis,AGV4_KinParam* output_spd,AGV4_DctParam* target_spd_dct,float pos_z_rad)
{
	//----------------局部变量----------------------------------------------------
	//编译器优化变量
	register AGV4_DctParam  Spd_input=*target_spd_dct;
	register AGV4_CtrlParam Chassis_input=*chassis;
	//实际速度在xy轴上的分量
	float speed_x[4] = { 0,0,0,0 };			
	float speed_y[4] = { 0,0,0,0 };		
	
	//----------------异常退出----------------------------------------------------
	if (chassis->srv_length <=0 || chassis->srv_width <=0)
	{
		//参数错误
		return AGV4_ERR_PARAM;
	}
	if (output_spd->log == AGV4_WARNN_PROC)
	{
		//线程不安全
		return AGV4_WRN_THREAD;
	}
	
	//-------------线程安全标志设立-------------------------------------------------
	output_spd->log=AGV4_WARNN_PROC;
	
	//----------使用math库优化后的逆运动学解算---------------------------------------
	speed_x[0] = (fmaf(Spd_input.z_spd_rad,sinf(atan2f(Chassis_input.srv_length,Chassis_input.srv_width)),(Spd_input.x_spd*cosf(pos_z_rad)+Spd_input.y_spd*sinf(pos_z_rad))));
	speed_y[0] = (fmaf(Spd_input.z_spd_rad,cosf(atan2f(Chassis_input.srv_length,Chassis_input.srv_width)),(Spd_input.y_spd*cosf(pos_z_rad)-Spd_input.x_spd*sinf(pos_z_rad))));

	speed_x[2] = (fmaf(Spd_input.z_spd_rad,sinf(atan2f(Chassis_input.srv_length,Chassis_input.srv_width)),(Spd_input.x_spd*cosf(pos_z_rad)+Spd_input.y_spd*sinf(pos_z_rad))));
	speed_y[2] = (fmaf(Spd_input.z_spd_rad,-cosf(atan2f(Chassis_input.srv_length,Chassis_input.srv_width)),(Spd_input.y_spd*cosf(pos_z_rad)-Spd_input.x_spd*sinf(pos_z_rad))));

	speed_x[1] = (fmaf(Spd_input.z_spd_rad,-sinf(atan2f(Chassis_input.srv_length,Chassis_input.srv_width)),(Spd_input.x_spd*cosf(pos_z_rad)+Spd_input.y_spd*sinf(pos_z_rad))));
	speed_y[1] = (fmaf(Spd_input.z_spd_rad,-cosf(atan2f(Chassis_input.srv_length,Chassis_input.srv_width)),(Spd_input.y_spd*cosf(pos_z_rad)-Spd_input.x_spd*sinf(pos_z_rad))));

	speed_x[3] = (fmaf(Spd_input.z_spd_rad,-sinf(atan2f(Chassis_input.srv_length,Chassis_input.srv_width)),(Spd_input.x_spd*cosf(pos_z_rad)+Spd_input.y_spd*sinf(pos_z_rad))));
	speed_y[3] = (fmaf(Spd_input.z_spd_rad,cosf(atan2f(Chassis_input.srv_length,Chassis_input.srv_width)),(Spd_input.y_spd*cosf(pos_z_rad)-Spd_input.x_spd*sinf(pos_z_rad))));
	
	//----------xy速度勾股定理,求解速度矢量的长度和角度--------------------------------
	for (int i = 0; i < 4; i++)
	{
		output_spd->drv_spd[i]       = hypotf(speed_x[i], speed_y[i]);     
		output_spd->srv_angle_rad[i] = -pos_z_rad - atan2f(speed_x[i], speed_y[i]) - Chassis_input.srv_start_angle_rad[i];    
	}
	
	//--------------------线程安全标志清除-----------------------------------------------
	output_spd->log=AGV4_FAULT_NONE;
	
	//------------------------解算完成----------------------------------------------------
	return AGV4_OK;
}

/**
*@name: AGV4_InvKin
*@brief:送入四舵轮底盘的驱动轮速度和航向轮角度，计算当前(x,y,z)速度
*@param:chassis       |INPUT    |底盘参数
*@param:spd_agv       |INPUT    |底盘的驱动轮速度和航向轮角度
*@param:pos_z_rad     |INPUT    |底盘相对场地坐标系的角度
*@reval:AGV4_DctParam |OUTPUT   |计算得到的速度
**/
AGV4_Status AGV4_Kin(AGV4_CtrlParam* chassis,AGV4_DctParam* output_dct,AGV4_KinParam* target_spd_agv,float pos_z_rad)
{
	//----------------局部变量----------------------------------------------------
	
	//----------------异常退出----------------------------------------------------
	
	//-------------线程安全标志设立-----------------------------------------------
	
	//-------------线程安全标志清除-----------------------------------------------
	
	//----------------函数返回----------------------------------------------------
	return AGV4_OK;
}

/**
*@name: AGV4_Optimise_AngleLoop
*@brief:优化逆运动学解算出的目标速度和角度,使其更适合用于控制舵轮底盘
*@param:chassis       |INPUT    |底盘参数
*@param:output_spd    |OUTPUT   |优化后的目标位置
*@param:target_spd    |INPUT    |目标位置
*@param:now_spd       |INPUT    |当前位置
*@reval:AGV4_Status   |OUTPUT   |优化程度反馈
*@usage:本函数采取角度回环策略进行优化,舵向轮将会以最短距离旋转到目标位置
**/
float try_angle;
float target_angle;
float now_pos;
AGV4_Status AGV4_Optimise_AngleLoop(AGV4_CtrlParam* chassis,AGV4_KinParam* output_spd,AGV4_KinParam* target_spd,AGV4_KinParam* now_spd)
{
	//----------------异常退出----------------------------------------------------
	if (chassis->srv_length <=0 || chassis->srv_width <=0)
	{
		//参数错误
		return AGV4_ERR_PARAM;
	}
	if ((output_spd->log == AGV4_WARNN_PROC)||(now_spd->log == AGV4_WARNN_PROC)||(target_spd->log == AGV4_WARNN_PROC))
	{
		//线程不安全
		return AGV4_WRN_THREAD;
	}
	
	//-------------线程安全标志设立-----------------------------------------------
	output_spd->log=AGV4_WARNN_PROC;
	now_spd->log=AGV4_WARNN_PROC;
	target_spd->log=AGV4_WARNN_PROC;
	
	//================有效代码开始================================================
	//先将目标拷贝
	memcpy((void*)output_spd,(void*)target_spd,sizeof(AGV4_KinParam));
	//对舵向轮回环
	for(int index=0;index<4;index++)
	{
		target_angle=target_spd->srv_angle_rad[index];
		now_pos     =now_spd->srv_angle_rad[index];
		
		if(target_angle - now_pos > 0.5*3.14159265359f)
			{
				while(target_angle - now_pos > 0.5*3.14159265359f)
				{
					target_angle -= 3.14159265359f;
					output_spd->drv_spd[index] = - output_spd->drv_spd[index];
				}
				output_spd->srv_angle_rad[index] = target_angle;
			}
			else if(target_angle - now_pos < -0.5*3.14159265359f)
			{
				while(target_angle - now_pos < -0.5*3.14159265359f)
				{
					target_angle += 3.14159265359f;
					output_spd->drv_spd[index] = - output_spd->drv_spd[index];
				}
				output_spd->srv_angle_rad[index] = target_angle;
			}
			else
			{
				output_spd->srv_angle_rad[index] = target_angle;
			}
		
//		try_angle   =target_spd->srv_angle_rad[index];
//		target_angle=target_spd->srv_angle_rad[index];
//		now_pos     =now_spd->srv_angle_rad[index];
//		
//		if (try_angle >= now_pos)
//		{
//			while(1)
//			{
//				try_angle-=2*3.14159265359f;
//				if (try_angle<target_angle) break;
//			}
//			try_angle+=2*3.14159265359f;
//			output_spd->srv_angle_rad[index]=try_angle;
//		}
//		else
//		{
//			while(1)
//			{
//				try_angle+=2*3.14159265359f;
//				if (try_angle>target_angle) break;
//			}
//			try_angle-=2*3.14159265359f;
//			output_spd->srv_angle_rad[index]=try_angle;
//		}
			
//		while(fabsf(output_spd->srv_angle_rad[index] - target_spd->srv_angle_rad[index]) > 2*3.1415926535898f)
//		{
//			if (output_spd->srv_angle_rad[index] > target_spd->srv_angle_rad[index])
//			{
//				output_spd->srv_angle_rad[index]-=2*3.1415926535898f;
//			}
//			else
//			{
//				output_spd->srv_angle_rad[index]+=2*3.1415926535898f;
//			}
//		}
	}
	//================有效代码结束================================================
	
	//-------------线程安全标志清除-----------------------------------------------
	output_spd->log=AGV4_FAULT_NONE;
	now_spd->log=AGV4_FAULT_NONE;
	target_spd->log=AGV4_FAULT_NONE;
	
	//----------------函数返回----------------------------------------------------
	return AGV4_OK;
}

/**
*@name: AGV4_Optimise_SlopLimit
*@brief:优化逆运动学解算出的目标速度和角度,使其更适合用于控制舵轮底盘
*@param:chassis       |INPUT    |底盘参数
*@param:output_spd    |OUTPUT   |优化后的目标速度
*@param:target_spd    |INPUT    |目标速度
*@param:now_spd       |INPUT    |当前速度
*@reval:AGV4_Status   |OUTPUT   |优化程度反馈
*@usage:本函数采取斜掠限制策略进行优化,限制舵向轮的旋转速度,以减慢移动速度为代价，避免舵向轮响应跟不上导致跑歪
**/
AGV4_Status AGV4_Optimise_SlopLimit(AGV4_CtrlParam* chassis,AGV4_KinParam* output_spd,AGV4_KinParam* target_spd,AGV4_KinParam* now_spd)
{
	//----------------异常退出----------------------------------------------------
	if (chassis->srv_length <=0 || chassis->srv_width <=0)
	{
		//参数错误
		return AGV4_ERR_PARAM;
	}
	if ((output_spd->log == AGV4_WARNN_PROC)||(now_spd->log == AGV4_WARNN_PROC)||(target_spd->log == AGV4_WARNN_PROC))
	{
		//线程不安全
		return AGV4_WRN_THREAD;
	}
	
	//-------------线程安全标志设立-----------------------------------------------
	output_spd->log=AGV4_WARNN_PROC;
	now_spd->log=AGV4_WARNN_PROC;
	target_spd->log=AGV4_WARNN_PROC;
	
	//================有效代码开始================================================
	//float   srv_pos_distance[4];
	//uint8_t srv_slope_max_id;
	//float   srv_pos_distance_max;
	float srv_spd_distance[4],angle_difference[4];
	//先将目标拷贝
	memcpy((void*)output_spd,(void*)target_spd,sizeof(AGV4_KinParam));
	//计算并求解最大斜率
	
//	for(int index=0;index<4;index++)
//	{
//		float srv_target_spd,srv_now_spd;
//		srv_target_spd=target_spd->srv_spd[index];
//		srv_now_spd=now_spd->srv_spd[index];
//		angle_difference[index]= target_spd->srv_angle_rad[index]-now_spd->srv_angle_rad[index];
//		srv_spd_distance[index]= sqrt(srv_target_spd*srv_target_spd	+srv_now_spd*srv_now_spd	-2*srv_target_spd*srv_now_spd*cosf(angle_difference[index]));
//		if(srv_spd_distance[index]>chassis->srv_limit_distance)
//		{
//			float middle_cos=(srv_now_spd*srv_now_spd+srv_spd_distance[index]*srv_spd_distance[index]-srv_target_spd*srv_target_spd)/2/srv_now_spd/srv_spd_distance[index];
//			float srv_pos_distance_max=chassis->srv_limit_distance;
//			output_spd->srv_spd[index]=sqrt(srv_now_spd*srv_now_spd+srv_pos_distance_max*srv_pos_distance_max-2*srv_pos_distance_max*srv_now_spd*middle_cos);
//			output_spd->srv_angle_rad[index]=asinf(srv_pos_distance_max	*	sqrt(1-middle_cos*middle_cos)/output_spd->srv_spd[index]);
//			
//		}

//	for(int index=0;index<4;index++)
//	{
//		if (target_spd->srv_angle_rad[index] >= now_spd->srv_angle_rad[index])
//		{
//			srv_pos_slope[index]= target_spd->srv_angle_rad[index] - now_spd->srv_angle_rad[index];
//		}
//		else
//		{
//			srv_pos_slope[index]= now_spd->srv_angle_rad[index] - target_spd->srv_angle_rad[index];
//		}

//		if (index == 0)
//		{
//			srv_pos_slope_max=srv_pos_slope[index];
//			srv_slope_max_id =0;
//		}
//		else if (srv_pos_slope[index] > srv_pos_slope_max)
//		{
//			srv_pos_slope_max=srv_pos_slope[index];
//			srv_slope_max_id =index;	
//		}
//	}
	
	//斜率约束
	
	//================有效代码结束================================================
	
	//-------------线程安全标志清除-----------------------------------------------
	output_spd->log=AGV4_FAULT_NONE;
	now_spd->log=AGV4_FAULT_NONE;
	target_spd->log=AGV4_FAULT_NONE;
	
	//----------------函数返回----------------------------------------------------
	return AGV4_OK;
}


AGV4_Status AGV4_SupportFunc_RawDeg_2_SldRad(AGV4_CtrlParam* chassis,AGV4_KinParam* raw_spd_deg_from_motor,AGV4_KinParam* output_spd)
{
	memcpy(output_spd,raw_spd_deg_from_motor,sizeof(AGV4_KinParam));
	output_spd->srv_angle_rad[0]=output_spd->srv_angle_rad[0]*1.74532925199433e-2f/(chassis->srv_slowdown_rate_rad[0]);
	output_spd->srv_angle_rad[1]=output_spd->srv_angle_rad[1]*1.74532925199433e-2f/(chassis->srv_slowdown_rate_rad[1]);
	output_spd->srv_angle_rad[2]=output_spd->srv_angle_rad[2]*1.74532925199433e-2f/(chassis->srv_slowdown_rate_rad[2]);
	output_spd->srv_angle_rad[3]=output_spd->srv_angle_rad[3]*1.74532925199433e-2f/(chassis->srv_slowdown_rate_rad[3]);
}
AGV4_Status AGV4_SupportFunc_RawRad_2_SldRad(AGV4_CtrlParam* chassis,AGV4_KinParam* raw_spd_deg_from_motor,AGV4_KinParam* output_spd)
{
	memcpy(output_spd,raw_spd_deg_from_motor,sizeof(AGV4_KinParam));
	output_spd->srv_angle_rad[0]=output_spd->srv_angle_rad[0]/(chassis->srv_slowdown_rate_rad[0]);
	output_spd->srv_angle_rad[1]=output_spd->srv_angle_rad[1]/(chassis->srv_slowdown_rate_rad[1]);
	output_spd->srv_angle_rad[2]=output_spd->srv_angle_rad[2]/(chassis->srv_slowdown_rate_rad[2]);
	output_spd->srv_angle_rad[3]=output_spd->srv_angle_rad[3]/(chassis->srv_slowdown_rate_rad[3]);
}
AGV4_Status AGV4_SupportFunc_SldRad_2_RawDeg(AGV4_CtrlParam* chassis,AGV4_KinParam* slowdown_rad_spd_from_caculate,AGV4_KinParam* output_spd)
{
	memcpy(output_spd,slowdown_rad_spd_from_caculate,sizeof(AGV4_KinParam));
	output_spd->srv_angle_rad[0]=output_spd->srv_angle_rad[0]*(chassis->srv_slowdown_rate_rad[0])*5.72957795130823e1f;
	output_spd->srv_angle_rad[1]=output_spd->srv_angle_rad[1]*(chassis->srv_slowdown_rate_rad[1])*5.72957795130823e1f;
	output_spd->srv_angle_rad[2]=output_spd->srv_angle_rad[2]*(chassis->srv_slowdown_rate_rad[2])*5.72957795130823e1f;
	output_spd->srv_angle_rad[3]=output_spd->srv_angle_rad[3]*(chassis->srv_slowdown_rate_rad[3])*5.72957795130823e1f;
}
AGV4_Status AGV4_SupportFunc_SldRad_2_RawRad(AGV4_CtrlParam* chassis,AGV4_KinParam* slowdown_rad_spd_from_caculate,AGV4_KinParam* output_spd)
{
	memcpy(output_spd,slowdown_rad_spd_from_caculate,sizeof(AGV4_KinParam));
	output_spd->srv_angle_rad[0]=output_spd->srv_angle_rad[0]*(chassis->srv_slowdown_rate_rad[0]);
	output_spd->srv_angle_rad[1]=output_spd->srv_angle_rad[1]*(chassis->srv_slowdown_rate_rad[1]);
	output_spd->srv_angle_rad[2]=output_spd->srv_angle_rad[2]*(chassis->srv_slowdown_rate_rad[2]);
	output_spd->srv_angle_rad[3]=output_spd->srv_angle_rad[3]*(chassis->srv_slowdown_rate_rad[3]);
}

