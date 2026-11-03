/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _RCS_SCARA_H_
#define _RCS_SCARA_H_

/* Includes ------------------------------------------------------------------*/
#include "RCS_MOTOR.h"

/* Scara Configuration -------------------------------------------------------*/
typedef double SCARA_FLOAT_T;      //使用双精度浮点计算(15位有效数字)
#define SCARA_COS cos              //对应精度的数学函数
#define SCARA_SIN sin

#define SCARA_PINV_COMMON 0.001

/* Exported Define & types----------------------------------------------------*/
typedef struct scara_angle{
	SCARA_FLOAT_T joint_angle_main;
	SCARA_FLOAT_T joint_angle_end;
}SCARA_AXIS;//关节角度向量

typedef struct descartes{
	SCARA_FLOAT_T x;
	SCARA_FLOAT_T y;
}DESCARTES_AXIS;//笛卡尔坐标向量

typedef struct scara_param{
	SCARA_FLOAT_T Main_Len;   //大臂长度
	SCARA_FLOAT_T End_Len;    //小臂长度
	SCARA_FLOAT_T Pinv_Epsilon;//奇异矩阵正则化系数
	
	SCARA_FLOAT_T Main_SlDn;  //大臂减速比(大臂角度/电机输出轴角度)(must >0)
	SCARA_FLOAT_T End_SlDn;   //小臂减速比(小臂角度/电机输出轴角度)(must >0)
	
	SCARA_AXIS    Start_Pos;  //rad
}SCARA_PARAM;//机械臂参数

/* Exported functions --------------------------------------------------------*/

SCARA_PARAM Scara_Param_Init(SCARA_FLOAT_T Main_Len,         SCARA_FLOAT_T End_Len,
                             SCARA_FLOAT_T Main_Slowdown,    SCARA_FLOAT_T End_Slowdown,
														 SCARA_FLOAT_T Main_StartPos_Deg,SCARA_FLOAT_T End_StartPos_Deg);

SCARA_AXIS Scara_Spd_Ctrl(SCARA_FLOAT_T spd_x,            SCARA_FLOAT_T spd_y,
                          SCARA_FLOAT_T Main_MotorPos_Rad,SCARA_FLOAT_T End_MotorPos_Rad,
                          SCARA_PARAM* scara);

SCARA_AXIS Scara_Pos_Ctrl(SCARA_FLOAT_T desc_pos_x,SCARA_FLOAT_T desc_pos_y,
                          SCARA_FLOAT_T Main_MotorPos_Rad,SCARA_FLOAT_T End_MotorPos_Rad,
                          SCARA_PARAM* scara);

SCARA_AXIS Scara_Pos_Ctrl_Direct(SCARA_FLOAT_T Main_Pos_Deg,SCARA_FLOAT_T End_Pos_Deg,SCARA_PARAM* scara);

SCARA_AXIS Scara_Get_Scara_Pos(SCARA_FLOAT_T Main_MotorPos_Rad,SCARA_FLOAT_T End_MotorPos_Rad,SCARA_PARAM* scara);
DESCARTES_AXIS Scara_Get_Desc_Pos(SCARA_FLOAT_T Main_MotorPos_Rad,SCARA_FLOAT_T End_MotorPos_Rad,SCARA_PARAM* scara);

//正逆运动学
DESCARTES_AXIS Scara_Kin(SCARA_AXIS* jonit_vector,SCARA_PARAM* scara);
SCARA_AXIS Scara_InvKin(DESCARTES_AXIS* target_vector,SCARA_AXIS* now_axis,SCARA_PARAM* scara);
//正逆微分运动学
DESCARTES_AXIS Scara_Jcb_Kin(SCARA_AXIS* jonit_vector,SCARA_AXIS* now_axis,SCARA_PARAM* scara);
SCARA_AXIS Scara_Jcb_InvKin(DESCARTES_AXIS* target_speed,SCARA_AXIS* now_axis,SCARA_PARAM* scara);
SCARA_AXIS Scara_Jcb_PInvKin(DESCARTES_AXIS* target_speed,SCARA_AXIS* now_axis,SCARA_PARAM* scara);
//减速比换算
SCARA_AXIS Scara_Trans_Motor_Pos(SCARA_AXIS* jonit_vector,SCARA_PARAM* scara);
SCARA_AXIS Motor_Trans_Scara_Pos(SCARA_AXIS* motor_vector,SCARA_PARAM* scara);
SCARA_AXIS Scara_Trans_Motor_Spd(SCARA_AXIS* jonit_vector,SCARA_PARAM* scara);
SCARA_AXIS Motor_Trans_Scara_Spd(SCARA_AXIS* motor_vector,SCARA_PARAM* scara);
//死点点位判断
SCARA_FLOAT_T Judge_Scara_Jcb_InvKin(DESCARTES_AXIS* target_speed,SCARA_AXIS* now_axis,SCARA_PARAM* scara);
uint8_t Judge_Scara_InvKin(DESCARTES_AXIS* target_pos,SCARA_AXIS* now_axis,SCARA_PARAM* scara);



#endif
