/*********************************************************************
*@filename:
*@brief:
*@changelog:
---------------------------------------------------------------------
*********************************************************************/
#ifndef RCS_AGV_H_
#define RCS_AGV_H_

/** 头文件引入 -----------------------------------------------------------**/
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <cmsis_compiler.h>

#include "RCS_PIDctrl.h"
#include "RCS_AcceCtrl.h"

/** 导出宏定义 -----------------------------------------------------------**/
typedef float    (*FNCT_AGV4) (uint8_t);

/** 导出枚举体 -----------------------------------------------------------**/

typedef enum agv4_ctrl_feedback{
	AGV4_OK        =0,
	
	AGV4_WRN_THREAD=-1,
	AGV4_ERR_PARAM =-10,
	
}AGV4_Status;

typedef enum agv4_fault{
	AGV4_FAULT_NONE    =0,
	AGV4_WARNN_PROC    =1,
	
	AGV4_FAULT_PARAM   =-1,
}AGV4_Fault;

/** 导出结构体 -----------------------------------------------------------**/

//8自由度舵轮底盘的机械参数和控制参数
typedef struct agv4_ctrl_param{
	//机械参数
	float srv_length;               //两个舵轮中心x边距离
	float srv_width;                //两个舵轮中心y边距离
	float srv_start_angle_rad[4];   //四个舵轮的起始角度
	float srv_slowdown_rate_rad[4]; //舵轮外观角度 * srv_slowdown_rate_rad = 电机实际角度
	
	//控制参数
	float srv_limit_distance;
}AGV4_CtrlParam;

//8自由度舵轮底盘各个自由度的参数
typedef struct agv4_kin_param{
	//参数
	float srv_angle_rad[4];
	float drv_spd[4];
	float srv_spd[4];
	//日志
	AGV4_Fault log;
}AGV4_KinParam;

//8自由度舵轮底盘的笛卡尔参数
typedef struct agv4_dct_param{
	float x_spd;
	float y_spd;
	float z_spd_rad;
}AGV4_DctParam;

/** 导出函数声明 ---------------------------------------------------------**/


//函数组1：4舵轮底盘解算与控制策略
AGV4_Status AGV4_InvKin            (AGV4_CtrlParam* chassis,AGV4_KinParam* output_spd,AGV4_DctParam* target_spd_dct,float pos_z_rad);
AGV4_Status AGV4_Kin               (AGV4_CtrlParam* chassis,AGV4_DctParam* output_dct,AGV4_KinParam* target_spd_agv,float pos_z_rad);
AGV4_Status AGV4_Optimise_AngleLoop(AGV4_CtrlParam* chassis,AGV4_KinParam* output_spd,AGV4_KinParam* target_spd,AGV4_KinParam* now_spd);
AGV4_Status AGV4_Optimise_SlopLimit(AGV4_CtrlParam* chassis,AGV4_KinParam* output_spd,AGV4_KinParam* target_spd,AGV4_KinParam* now_spd);
AGV4_Status AGV4_SupportFunc_Slowdown(AGV4_KinParam* raw_spd_from_motor);

AGV4_Status AGV4_SupportFunc_RawDeg_2_SldRad(AGV4_CtrlParam* chassis,AGV4_KinParam* raw_spd_deg_from_motor,AGV4_KinParam* output_spd);
AGV4_Status AGV4_SupportFunc_RawRad_2_SldRad(AGV4_CtrlParam* chassis,AGV4_KinParam* raw_spd_deg_from_motor,AGV4_KinParam* output_spd);
AGV4_Status AGV4_SupportFunc_SldRad_2_RawDeg(AGV4_CtrlParam* chassis,AGV4_KinParam* slowdown_rad_spd_from_caculate,AGV4_KinParam* output_spd);
AGV4_Status AGV4_SupportFunc_SldRad_2_RawRad(AGV4_CtrlParam* chassis,AGV4_KinParam* slowdown_rad_spd_from_caculate,AGV4_KinParam* output_spd);

#endif