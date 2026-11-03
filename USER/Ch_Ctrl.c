#include "Ch_Ctrl.h"

DT35_Handler_t dt35_1;
DT35_Handler_t dt35_2;
DT35_Handler_t dt35_3;
DT35_Handler_t dt35_4;
Comm_WindowData_t dt351_window;
Comm_WindowData_t dt352_window;
Comm_WindowData_t dt353_window;
Comm_WindowData_t dt354_window;

#define Chassis_L 650.0f
#define Chassis_W 400.0f


void ChCtrl_Init_DT35(void)
{
	DT35_Init(&dt35_4,50.f, 14000.0f, LASER_ADC_ONE,LASER_CHANNEL_FOUR,LASER_GPIO_FOUR,LASER_PIN_FOUR);
	DT35_Init(&dt35_3,0.0f+0.5f*Chassis_L,  5350.0f+0.5f*Chassis_L,  LASER_ADC_ONE,LASER_CHANNEL_THREE,LASER_GPIO_THREE,LASER_PIN_THREE);//right
	DT35_Init(&dt35_2,50.f, 14000.0f, LASER_ADC_ONE,LASER_CHANNEL_TWO,LASER_GPIO_TWO,LASER_PIN_TWO); //yellow small
	DT35_Init(&dt35_1,50.f, 14000.0f, LASER_ADC_ONE,LASER_CHANNEL_ONE,LASER_GPIO_ONE,LASER_PIN_ONE);//left 
	DT35_Setup_KalmanFilter(&dt35_1,1.0f,0.0001,0.05f,0.0f);
	DT35_Setup_KalmanFilter(&dt35_2,1.0f,0.0001,0.05f,0.0f);
	DT35_Setup_KalmanFilter(&dt35_3,1.0f,0.0001,0.05f,0.0f);
	DT35_Setup_KalmanFilter(&dt35_4,1.0f,0.0001,0.05f,0.0f);
	WindowFloat_Init_Slide(&dt351_window,40);
	WindowFloat_Init_Slide(&dt352_window,40);
	WindowFloat_Init_Slide(&dt353_window,40);
	WindowFloat_Init_Slide(&dt354_window,40);
}
volatile float dt_data[4];
volatile float window_var;
void ChCtrl_Test_DT35(void)
{
		dt_data[0]=DT35_Get_Kalman(&dt35_1);
		dt_data[1]=DT35_Get_Kalman(&dt35_2);
		dt_data[2]=DT35_Get_Kalman(&dt35_3);
		dt_data[3]=DT35_Get_Kalman(&dt35_4);
		JustFloat_Printf(dt_data,4,USART1_MAP);
	
		WindowFloat_Update_Member(&dt351_window,dt_data[1]);
		window_var=WindowFloat_Get_Var(&dt353_window);
}


volatile float mm_dt1;
volatile float mm_dt2;
volatile float mm_dt3;
volatile float mm_dt4;

void ChCtrl_Laser_Main(void)
{
	mm_dt1=DT35_Get_Kalman(&dt35_1);
	mm_dt2=DT35_Get_Kalman(&dt35_2);
	mm_dt3=DT35_Get_Kalman(&dt35_3);
	mm_dt4=DT35_Get_Kalman(&dt35_4);
}

float ChCtrl_Laser_Get_cY(void)
{
	return DT35_Get_Kalman(&dt35_2);
}

float ChCtrl_Laser_Get_lX(void)
{
	return DT35_Get_Kalman(&dt35_1);
}

float ChCtrl_Laser_Get_rX(void)
{
	return DT35_Get_Kalman(&dt35_3);
}

uint8_t Judge_XL_Available(uint8_t zone)
{
	
	if (zone==ZONE_RED)
	{
		if (fabsf(Get_Gyro_Z()-0.0f)<=3.0f)
		if (Get_LaserGPS_Y()<=7500.0f)
			return 1;
	}
	if (zone==ZONE_BLUE)
	{
		if (Get_LaserGPS_Y()>=7900.0f)
			return 1;
	}
	
	return 0;
}

uint8_t Judge_XR_Available(uint8_t zone)
{
	if (zone==ZONE_RED)
	{
		if (Get_LaserGPS_Y()>=7500.0f)
			return 1;
	}
	
	if (zone==ZONE_BLUE)
	{
		if (fabsf(Get_Gyro_Z()-0.0f)<=3.0f)
		if (Get_LaserGPS_Y()<=7900.0f)
	
		return 1;
	}
	
	return 0;
}

uint8_t Judge_Y_Available(uint8_t zone)
{
	
}

float ChCtrl_DT35_Get_X(void)
{
	return DT35_Get_Kalman(&dt35_3);
}
