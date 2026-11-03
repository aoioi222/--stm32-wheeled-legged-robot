#ifndef RCS_AGV_BASEMOVE_H_
#define RCS_AGV_BASEMOVE_H_

/* =================头文件=========================================*/
#include "rcs.h"

/* =================结构体=========================================*/
typedef struct dercartes_speed_vector{
    float_t Dct_vx;
    float_t Dct_vy;
    float_t Dct_vw;
}Dct_Speed;

typedef struct agv_status_vector{
    float_t angle[4];
    float_t speed[4];
}AGV_Speed;


/* =================参数配置=========================================*/

#define CLOCKWISE 1
#define ANTICLOCKWISE -1


//底盘光电门引脚配置
#define LG1_GPIO GPIOA
#define LG1_Pin GPIO_Pin_1

#define LG2_GPIO GPIOA
#define LG2_Pin GPIO_Pin_1

#define LG3_GPIO GPIOA
#define LG3_Pin GPIO_Pin_1

#define LG4_GPIO GPIOA
#define LG4_Pin GPIO_Pin_1

//底盘基本参数配置
#define Chassis_Lenth 0.1f                    //底盘长度
#define Chassis_Width 500.0f                    //底盘宽度
#define Chassis_Radius 230.0f	//230.0f				//底盘半径（正三角舵轮底盘的参数，舵轮中心到底盘中心的距离）
#define RC_X 0.0f						//neg->right to ahead		//Rotate_Center_X（以几何中心为坐标原点，前进方向为y正方向，右侧为x正方向）
#define RC_Y 0.0f								//Rotate_Center_Y


#define Srv_Slowdown_Rate 3.7142857142f         //舵电机输出轴->舵轮的减速比
#define Srv_Type          RMESC_M2006

#define Drv_Wheel_Radius_Rec 14.92537f              //驱动轮半径(单位为m)的倒数
#define Drv_MAX_POWER        U8LiteL_KV110_MAX_POWER
#define Drv_MAX_CURRENT      U8LiteL_KV110_MAX_CURRENT

//底盘控制控制配置
#define Srv_Tor_Angle 8.0f                      //舵电机角度差多少的时候开始让驱动轮转动

// <<< Use Configuration Wizard in Context Menu >>>
// <h> 配置底盘基本参数,底盘的浮点参数请直接在.h文件中修改
    #define Chassis_Type_Triple 0
    #define Chassis_Type_Rectangle 1
	// <o> 底盘类型
		//<0=> Chassis_Type_Triple
		//<1=> Chassis_Type_Rectangle
	#define Chassis_Type 1
// </h>

// <h> 配置驱动轮参数,轮系的浮点参数请直接在.h文件中修改
    #define Dri_Type_3508 0
    #define Dri_Type_Vesc 1
    #define Dri_Type_Odrive 2
	// <o> 驱动电机类型
		//<0=> Dri_Type_3508
		//<1=> Dri_Type_Vesc
		//<2=> Dri_Type_Odrive
	#define AGV_Dri_type 1
// </h>
// <<< end of configuration section >>>

/* =================接口声明=========================================*/
float AGV_Wheel_Angle(uint8_t Wheel_ID);
void AGV_Init(void);
void AGV_Init2(void);
void AGV_Init_3(void);
RCS_Reset_Err_t rcsMotor_ctrlSpd_0 (float Spd);
RCS_Reset_Err_t rcsMotor_ctrlSpd_1 (float Spd);
RCS_Reset_Err_t rcsMotor_ctrlSpd_2 (float Spd);
RCS_Reset_Err_t rcsMotor_ctrlSpd_3 (float Spd);

float rcsSrv_getPos_fptr_0	(void);
float rcsSrv_getPos_fptr_1	(void);
float rcsSrv_getPos_fptr_2	(void);
float rcsSrv_getPos_fptr_3	(void);
float rcsSrv_Rep_RealAngle0	(void);
float rcsSrv_Rep_RealAngle1	(void);
float rcsSrv_Rep_RealAngle2	(void);
float rcsSrv_Rep_RealAngle3	(void);

//void rcsAgv0_Update_Angle_3(float angle1,float angle2,float angle3);
void rcsAgv0_Update_Angle_0(float angle1);
void rcsAgv0_Update_Angle_1(float angle2);
void rcsAgv0_Update_Angle_2(float angle3);

float RCS_Srv_Angle_To_Encoder(float Srv_angle);
float RCS_Encoder_To_Srv_Angle(float Encoder_angle);
	

uint8_t AGV_IrReset(void);
void AGV_Move(float move_speed_x, float move_speed_y, float rotate_speed);
void AGV_Cost_Move(float move_speed_x, float move_speed_y, float rotate_speed);
uint8_t AGV_Srv_Move(float move_speed_x, float move_speed_y, float rotate_speed);

void AGV_Drv_Debug(int16_t rpm1,int16_t rpm2,int16_t rpm3,int16_t rpm4);
void AGV_Srv_Debug(int16_t cur1,int16_t cur2,int16_t cur3,int16_t cur4);
void AGV_Test_VESC(void);
void AGV_Test_ANGLE(void);
uint8_t AGV_Init_Angle(void);

void AGV_BaseMove_P2P(float move_speed_x, float move_speed_y, float rotate_speed, float allow_angle_err, float target_speed_max);
void AGV_Re_Center_Move(float move_speed_x, float move_speed_y, float rotate_speed);
void AGV_Re_Center_Move_v2(float move_speed_x, float move_speed_y, float rotate_speed);
void AGV_Re_Center_Move_v3(float move_speed_x, float move_speed_y, float rotate_speed);

void AGV_Re_Center_Move_v3_2(float move_speed_x, float move_speed_y, float rotate_speed);

float Get_IrZero_Pos(uint8_t id);
int Set_Rep_Agv_Angle(uint8_t id,float Center_Agv_Angle);
void AGV_To_Coordinate(float destination_x, float destination_y, float destination_z, int8_t* flag_arrive);
uint8_t AGV_Arc_Move(float start_x, float start_y, float aim_x, float aim_y, float central_angle, uint8_t clock_or_anti, float speed);
uint8_t AGV_Coordinate_Controlled_Curve(float start_x, float start_y, float aim_x, float aim_y,
										float start_spd_x, float start_spd_y, float aim_spd_x, float aim_spd_y,
											float z);
										
void AGV_Re_Center_Move_v3_3(float move_speed_x, float move_speed_y, float rotate_speed);
void Equilateral_Triangle_AGV_Move(float move_speed_x, float move_speed_y, float rotate_speed);
float read_tester();
										
AGV_Speed AGV_Re_Center_Move_v33_Cac(float move_speed_x, float move_speed_y, float rotate_speed);
void AGV_Re_Center_Move_v33_Srv(float move_speed_x, float move_speed_y, float rotate_speed);

void AGV_Re_Center_Move_v3_4(float move_speed_x, float move_speed_y, float rotate_speed);

void Circle_Shift_Ctrl(float target_spd,float shift_spd);
AGV_Speed AGV_Get_Last_Spd(void);
void AGV_Execute(AGV_Speed* input);
float Get_AGV_Max_Speed_X();
float Get_AGV_Max_Speed_Y();


void AGV_Angle_First(float move_speed_x, float move_speed_y, float rotate_speed, float allow_angle_err, int8_t* flag_arrive);
float Read_AGV_Target_Angle(int num);
float Read_AGV_Target_Speed(int num);
										
										
void AGV_Silumate_Move(float speed_x, float speed_y, float speed_z);
float Get_Simulate_GPS_X();
float Get_Simulate_GPS_Y();
float Get_Simulate_GPS_Z();


#endif