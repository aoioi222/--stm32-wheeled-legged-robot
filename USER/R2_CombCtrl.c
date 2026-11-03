#include "R2_CombCtrl.h"
#include "Ch_Ctrl.h"

/*********************************************************************
 * 
 * @addtogroup:宏定义
 * 
 * *******************************************************************/

/*********************************************************************
 * 
 * @addtogroup:全局变量
 * 
 * *******************************************************************/

//桶框的位置
volatile float Pos_Silo[5][2][3];//[silo][zone][x/y/z]
//大臂找球PID
PID_Struct     FindBall_Main_Pid;
//底盘找球PID
DacePID_Struct FindBall_Chassis_Pid;
//底盘跑点PID
DacePID_Struct     agv_z_pid;
DacePID_Struct     agv_y_pid;
DacePID_Struct     agv_x_pid;

//传入底盘进行速度控制的值
float agv_target_x;
float agv_target_y;
float agv_target_z;

float agv_start_x;
float agv_start_y;

AGV_Speed HAL_AGV_Last_Output;

//复合动作日志
LogServer_t Combo_Log;

//上一次的输出值
float target_scara_x=450.0f;
float target_scara_y=0.0f;
float target_gantry;
float target_actor;
uint8_t punk_status;
uint8_t swiv_status;
float safe_x;
float safe_y;

//日志
int8_t scara_err;
int8_t comm_err;

//lasered xy
volatile float test_X;
volatile float test_Y1;
volatile float test_Y2;

//奥比中光返回的球位置的滑动窗口
volatile Comm_WindowData_t Vision_Cup_Window_X;
volatile Comm_WindowData_t Vision_Cup_Window_Y;
volatile Comm_WindowData_t Vision_Cup_Window_D;

volatile Comm_WindowData_t Vision_Pick_Window_X;
volatile Comm_WindowData_t Vision_Pick_Window_Y;

//奥比中光相机返回数据，经处理过后得到有效参数
float valid_cup_var_x;
float valid_cup_var_y;
float valid_cup_mean_x;
float valid_cup_mean_y;
float valid_cup_mean_d;
float valid_cup_percent;
float valid_vision_cup_xyd[3][60];

int8_t Pick_IsolateBall_State;

/*********************************************************************
 * 
 * @addtogroup:初始化
 * 
 * *******************************************************************/

void CombCtrl_Param_Init(RCS_PIN_USART USARTx_MAP)
{
	//距离环PID参数
	FindBall_Main_Pid.P=-3.0f;
	FindBall_Main_Pid.Limit_Output=3200.0f;
	
	FindBall_Chassis_Pid.pid.P=10.0f;
	FindBall_Chassis_Pid.pid.Limit_Output=10000.0f;
	FindBall_Chassis_Pid.dead_zone=15.0f;
	FindBall_Chassis_Pid.max_acce=100.0f;
	FindBall_Chassis_Pid.max_dcce=200.0f;

	agv_x_pid.pid.P=0.1f;//0.2f;//0.1f;//
	agv_x_pid.pid.Limit_Output=150.0f;//330.0f;//600.0f;//330.0f;//60.0f;//
	agv_x_pid.max_acce=1.0f;//2.0f;//1.0f;//
	agv_x_pid.max_dcce=agv_x_pid.pid.Limit_Output*0.8;
	agv_x_pid.dead_zone=50.0f;
	
	agv_y_pid.pid.P=0.1f;//0.2f;//0.1f;//
	agv_y_pid.pid.Limit_Output=150.0f;//330.0f;//600.0f;//330.0f;//60.0f;//
	agv_y_pid.max_acce=1.0f;//2.0f;//1.0f;//
	agv_y_pid.max_dcce=agv_y_pid.pid.Limit_Output*0.8;
	agv_y_pid.dead_zone=50.0f;
	
	agv_z_pid.pid.P=6.0f;
	agv_z_pid.pid.Limit_Output=200.0f;//300.0f;//200.0f;
	agv_z_pid.max_acce=13.0f;
	agv_z_pid.max_dcce=50.0f;
	agv_z_pid.dead_zone=0.5f;

	
	//速度环PID参数

	//滑动窗口
	WindowFloat_Init_Slide(&Vision_Cup_Window_X,10);
	WindowFloat_Init_Slide(&Vision_Cup_Window_Y,10);
	WindowFloat_Init_Slide(&Vision_Cup_Window_D,10);
	WindowFloat_Init_Slide(&Vision_Pick_Window_X,15);
	WindowFloat_Init_Slide(&Vision_Pick_Window_Y,15);

	//日志系统
	LogServer_Init(&Combo_Log,USARTx_MAP.USARTx,"Combo\0");

}

void CombCtrl_Param_ReInit()
{
	agv_x_pid.pid.P=0.1f;//
	agv_x_pid.pid.Limit_Output=290.0f;//
	agv_x_pid.max_acce=1.0f;//
	agv_x_pid.max_dcce=agv_x_pid.pid.Limit_Output*0.8;
	agv_x_pid.dead_zone=50.0f;
	
	agv_y_pid.pid.P=0.1f;//
	agv_y_pid.pid.Limit_Output=290.0f;//
	agv_y_pid.max_acce=1.0f;//
	agv_y_pid.max_dcce=agv_y_pid.pid.Limit_Output*0.8;
	agv_y_pid.dead_zone=50.0f;
	
	agv_z_pid.pid.P=6.0f;
	agv_z_pid.pid.Limit_Output=200.0f;
	agv_z_pid.max_acce=13.0f;
	agv_z_pid.max_dcce=50.0f;
	agv_z_pid.dead_zone=0.5f;
}

/*********************************************************************
 * 
 * @addtogroup:HAL层-底盘
 * 
 * *******************************************************************/
//底盘控制函数根据此函数获取X坐标
float HAL_Chassis_Get_X(void)
{
	//return ChCtrl_DT35_Get_X();
	return Get_LaserGPS_X();
}
//底盘控制函数根据此函数获取Y坐标
float HAL_Chassis_Get_Y(void)
{
	return Get_LaserGPS_Y();
}
//底盘控制函数根据此函数获取Z坐标
float HAL_Chassis_Get_Z(void)
{
	return Get_GPS_Z();
}
//设置底盘上次的速度输出，一般用于速度控制转距离控制，避免控制不连续
void HAL_Chassis_Reset(float x_last_output,float y_last_output,float z_last_output)
{
	agv_x_pid.last_output=x_last_output;
	agv_y_pid.last_output=y_last_output;
	agv_z_pid.last_output=z_last_output;
}

uint8_t HAL_Chassis_Ctrl(float param_x,float param_y,float param_z)
{
        
	//==============锁z轴的xy速度控制============================================

	agv_target_z=PID_Normal_Ctrl(param_z,HAL_Chassis_Get_Z(),&agv_z_pid);
	AGV_Re_Center_Move_v3_3(param_x,param_y,agv_target_z);
	HAL_AGV_Last_Output=AGV_Get_Last_Spd();
	if (agv_target_z==0) return 2;
	else if (fabsf(param_z-HAL_Chassis_Get_Z())<=1.5f) return 1;
	else return 0;
}


