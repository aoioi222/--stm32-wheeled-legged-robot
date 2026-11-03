/**
 * @filename:RCS_Ctrl_BaseMove
 * @brief:底盘综合控制
 * @changlog: 2023-7-28 胡兴国 修改整理
 * @changlog: 2024-2-20 陈煜楷 加入舵轮底盘
*/

/* --------.h header--------------------------------------------------*/
#ifndef _RCS_CTRL_BASEMOVE_H_
#define _RCS_CTRL_BASEMOVE_H_

/* --------头文件-----------------------------------------------------*/
#include "RCS_PIDctrl.h"
#include "RCS_MOTOR.h"	

/* --------配置相关宏--------------------------------------------------*/

//底盘配置
//#define USE_OMNI_CHASSIS    //使用全向轮底盘,全向轮底盘参数在RCS_BaseMove.h
#define USE_AGV_CHASSIS     //使用舵轮底盘,舵轮底盘参数在RCS_AGV_BaseMove.h

//自动控制参数
#define POINT_NUM       200		//贝塞尔曲线点数
#define LEAD_POINT_NUM  4.0f	//贝塞尔曲线牵引点超前数
#define SLOW_POINT      150.0f	//减速的临界点

#define BIG_ANGLE           5.0  //大角度
#define START_SPEED         500  //启动速度
#define GET_HELM_ANGLE      0.2f //舵轮角度到达判断


/* ---------导出数据结构-----------------------------------------------*/
typedef struct Point{
	float x;
	float y;
	float z;
}Coord_Point;//坐标点结构体

typedef struct Param{
	float ACCE;//加速度
	float DECE;//减速度
	float L_ANGLE;//角度到达容差_左
	float R_ANGLE;//角度到达容差_右
	float run_speed;//跑点速度
	PID_Struct rotate_angle_pid;
	
}Trace_Param;//胡兴国跑点算法参数

typedef struct Speed_Vector{
	float value;									
	float direction;							
}Route_BridgeSpeed;


typedef struct{
	struct Speed_Vector bridge;			//桥接速度
	struct Speed_Vector speed[POINT_NUM];					//控制整个路线中的行车速度
	struct Point ctrl_point[2];			//0为近点、1为远点			//三阶贝赛尔曲线需要两个控制点
	struct Point point[POINT_NUM];	//路线点
	char  direction;								//主方向为'x'或'y'(看方向角，绝对值在45°~135°之间的为'x',否则为'y')
	uint8_t begin_speed_ctrl;				//起点出发是否需要加速，1是，0不是
	uint8_t end_speed_ctrl;					//到达终点是否需要减速，1是，0不是
}Route_Point;//贝赛尔曲线上各点姿态结构体


/* ---------导出函数------------------------------------------------------*/

//初始化
void Ctrl_BaseMove_Init(void); 
//底盘距离控制
void Point2Point_ER(Coord_Point* cp,Trace_Param* tp,int *ER_flag);
void Point2Point(float target_x,float target_y,float target_z,PID_Struct distance_x_pid,PID_Struct distance_y_pid,PID_Struct rotate_angle_pid);
void Bessel_P2P(Route_Point *rpoint,int *index,int AUTO_SPEED,float target_z,PID_Struct distance_x_pid,PID_Struct distance_y_pid,PID_Struct rotate_angle_pid,PID_Struct angle_pid);

//底盘位姿判定
int Judge_Pos(float target_x,float target_y,float target_z);		//坐标达到判定
int Judge_Pos_B(int index);																			//点控曲线点位到达判定

#endif