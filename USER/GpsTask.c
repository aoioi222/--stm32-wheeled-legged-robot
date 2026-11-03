/*@filename: GPS.c
 *@author     胡兴国      
 *@brief:     全场定位电机版和自制版
 *@brief:     码盘的帧率大于1000Hz    
 *@brief:     定位帧率小于20Hz时底盘会跑不稳
 *@date: 2023-8-31
*/

/* =========头文件=================================*/
#include "GpsTask.h"
#include "G431_GPS.h"
/* =========静态函数定义============================*/
static OS_STK my_gps_task_stk[NORMAL_TASK_STK_SIZE];
void my_gps_task(void *p_arg);

volatile float GPS_Start_X;
volatile float GPS_Start_Y;

#ifdef RCS16_GPS
/* =========私有全局变量=============================*/
RCS_Spd_T Now_Encoder_Spd,Now_Base_Spd;
RCS_Pos_T Now_Pos,Start_Pos;

/* =========导出函数接口=============================*/
/**
	@name: GPS_Init
	@brief:自制全场定位初始化
**/
void GPS_Init(void)
{	
	GPS_Encoder_Init();							//编码器初始化
	OSTaskCreate(my_gps_task,					//创建信息处理任务
	           (void *)0,
	           &my_gps_task_stk[NORMAL_TASK_STK_SIZE - 1],
	           my_gps_task_PRIO);
}

/**
	@name: my_gps_task
	@brief:GPS信息处理任务，速度积分版
**/
void my_gps_task(void *p_arg)
{
	p_arg = p_arg;
	while(1)   
	{
		//根据编码器里程得到编码器速度
		gpos_diff_gspd(&Now_Encoder_Spd);
		//根据编码器速度得到机器人速度
		gspd_Kin_spd(&Now_Encoder_Spd,Get_Gyro_Z(),&Now_Base_Spd);
		//机器人速度积分
		bspd_cumtrupz_bpos(&Now_Base_Spd,Start_Pos,&Now_Pos);
		//200Hz运算帧率
		delay_ms(5);
	}
}

#endif


#ifdef RCS13_GPS
static int32_t gps_cycle_x = 0;			//编码器溢出圈数
static int32_t gps_cycle_y = 0;			
 float gps_origin_x = 0;				//原始编码器数据
 float gps_origin_y = 0;
 float gps_x = 0;			//输出XY值
 float gps_y = 0;
static volatile float raw_gps_x = 0;			//输出XY值
static volatile float raw_gps_y = 0;
static float raw_z = 0;				
static float k1=0;
static float k2 = 0;
static float delta_zangle;						//分割后的偏航角增量
static float rad_zangle;							//偏航角（弧度制）
static float gps_sum_zangle = 0;			//拓展后的偏航角
static float gps_last_sum_zangle=0;
static float last_zangle = 0;				//上一时刻的偏航角
static float angle_error=0,angle_move=0; 	
static int cycle_z = 0;				//z的圈数	
static float last_org_x=0,last_org_y =0;		//上一时刻X、Y编码器里程信息
static double delta_org[3]={0,0,0};		//里程增量

/**
	@name: GPS_Init
	@brief:自制全场定位初始化
**/
void GPS_Init(void)
{	
	//GPS_Encoder_Init();	//编码器初始化
	G431GPS_Init(USART6_MAP,ACTION_PRI);
	OSTaskCreate(my_gps_task,					//创建信息处理任务
	           (void *)0,
	           &my_gps_task_stk[NORMAL_TASK_STK_SIZE - 1],
	           my_gps_task_PRIO);

}

//float TRANSFORM_DZ=1.91;

/**
	@name: my_gps_task
	@brief:GPS信息处理任务，码盘版
**/
void my_gps_task(void *p_arg)
{
	p_arg = p_arg;
	while(1)   
	{
		raw_z =Get_Gyro_Z();  //获取偏航角
		rad_zangle = raw_z * DEG2RAD;   //偏航角弧度制
		if(last_zangle >= 160.0f && raw_z <= -160.0f)
			cycle_z ++;
		if(last_zangle <= -160.0f && raw_z >= 160.0f)
			cycle_z --;
		
		gps_sum_zangle = cycle_z * 360.0f + raw_z;	//拓展角度至正负无穷		

    //码盘定位
//		gps_origin_y = -((Get_Motor_Float_Angle(1)/36.0f)/3600.0f)*(2*pi*GPS_WHEEL_RADIUS);//(gps_cycle_x * 65536.0 + GPS_X_TIM->CNT) * TRANSFER_CONST * ENCODER_X_DIR;	//获取编码器脉冲值并转化为里程值
//		gps_origin_x = ((Get_Motor_Float_Angle(4)/36.0f)/3600.0f)*(2*pi*GPS_WHEEL_RADIUS);//(gps_cycle_y * 65536.0 + GPS_Y_TIM->CNT) * TRANSFER_CONST * ENCODER_Y_DIR;
			
		gps_origin_x =G431GPS_Get_X() * TRANSFER_CONST * ENCODER_X_DIR;//(gps_cycle_x * 65536.0 + GPS_X_TIM->CNT) * TRANSFER_CONST * ENCODER_X_DIR;	//获取编码器脉冲值并转化为里程值
		gps_origin_y =G431GPS_Get_Y() * TRANSFER_CONST * ENCODER_X_DIR;//(gps_cycle_y * 65536.0 + GPS_Y_TIM->CNT) * TRANSFER_CONST * ENCODER_Y_DIR;
		
		delta_org[0] = gps_origin_x - last_org_x;				//角度值转换成里程值，与车轮半径有关
		delta_org[1] = gps_origin_y - last_org_y;

		raw_gps_x += delta_org[0] * cos(rad_zangle) - delta_org[1] * sin(rad_zangle);  //坐标变换之前的数值
		raw_gps_y += delta_org[0] * sin(rad_zangle) + delta_org[1] * cos(rad_zangle);
		

 	//保存这一时刻里程值
		last_org_x = gps_origin_x;								//保存上一时刻的里程
		last_org_y = gps_origin_y;
		last_zangle = raw_z;											//保存上一时刻偏航角
		
    //码盘定位
		gps_x = raw_gps_x * cos(TRANSFORM_DZ) + raw_gps_y * sin(TRANSFORM_DZ) + TRANSFORM_DX * (1 - cos(rad_zangle));
		gps_y =-raw_gps_x * sin(TRANSFORM_DZ) + raw_gps_y * cos(TRANSFORM_DZ) - sin(rad_zangle) * TRANSFORM_DX;
		
		delay_ms(10);
	}
}

///**
//	@name: my_gps_task
//	@brief:GPS信息处理任务，电机版
//**/
//void my_gps_task(void *p_arg)
//{
//	p_arg = p_arg;
//	while(1)   
//	{
//		
//		raw_z = Get_Gyro_Z();  //获取偏航角
//		rad_zangle = raw_z * DEG2RAD;   //偏航角弧度制
//		if(last_zangle >= 160.0f && raw_z <= -160.0f)
//			cycle_z ++;
//		if(last_zangle <= -160.0f && raw_z >= 160.0f)
//			cycle_z --;
//		
//		
//		gps_sum_zangle = cycle_z * 360.0f + raw_z;	//拓展角度至正负无穷		
//			
//		gps_origin_x = Get_Motor_Float_Angle(1);	//获取编码器脉冲值并转化为里程值
//		gps_origin_y = -Get_Motor_Float_Angle(2);
//		
//		
//		delta_org[0] = (gps_origin_x - last_org_x)*TRANSFORM_X;				//角度值转换成里程值，与车轮半径有关
//		delta_org[1] = (gps_origin_y - last_org_y)*TRANSFORM_Y;
//		
//		raw_gps_x += delta_org[0] * cos(rad_zangle) - delta_org[1] * sin(rad_zangle);  //坐标变换之前的数值
//		raw_gps_y += delta_org[0] * sin(rad_zangle) + delta_org[1] * cos(rad_zangle);
//		

// 	//保存这一时刻里程值
//		last_org_x = gps_origin_x;								//保存上一时刻的里程
//		last_org_y = gps_origin_y;
//		last_zangle = raw_z;											//保存上一时刻偏航角
//		
//	//坐标空间变换，定位仪-》车中心
//		gps_x =-raw_gps_x * cos(ANGLE_DX) - raw_gps_y * sin(ANGLE_DY) + sin(rad_zangle) * TRANSFORM_AX;  //TRANSFORM_AX/AY是车中心与定位中心的距离
//		gps_y = raw_gps_x * sin(ANGLE_DX) - raw_gps_y * cos(ANGLE_DY) + TRANSFORM_AY * (1 - cos(rad_zangle));
//		
//		gps_x = gps_x/10.0f;
//		gps_y = gps_y/10.0f;
//		
//		
//		delay_ms(2);
//	}
//}

volatile float refer_gps_x;
volatile float refer_gps_y;
volatile float true_laser_x;
volatile float true_laser_y;
void Set_GPS_X(float laser_x)
{
	refer_gps_x=Get_GPS_X();
	true_laser_x=laser_x;
}
void Set_GPS_Y(float laser_y)
{
	refer_gps_y=Get_GPS_Y();
	true_laser_y=laser_y;
}

float Get_LaserGPS_X(void)
{
	return true_laser_x+Get_GPS_X()-refer_gps_x;
}

float Get_LaserGPS_Y(void)
{
	return true_laser_y+Get_GPS_Y()-refer_gps_y;
}

/**
	@name: Get_GPS_X
	@brief:获取X坐标
**/
float Get_GPS_X(void)
{
//	return Get_GPS_Action_X();
	return gps_x;
}

/**
	@name: Get_GPS_Y
	@brief:获取Y坐标
**/
float Get_GPS_Y(void)
{
//	return Get_GPS_Action_Y();
	return gps_y;
}

/**
	@name: Get_GPS_Z
	@brief:获取扩展偏航角
**/
float Get_GPS_Z(void)
{
//	return Get_GPS_Action_Z();
	return gps_sum_zangle;
}

/**
	@name: Get_GPS_Ori_X
	@brief:返回从动轮转过路程
**/
float Get_GPS_Ori_X(void)
{
	return gps_origin_x;
}

/**
	@name: Get_GPS_Ori_Y
	@brief:返回从动轮转过路程
**/
float Get_GPS_Ori_Y(void)
{
	return gps_origin_y;
}

/**
	@name: Get_GPS_Ori_X
	@brief:返回从动轮转过路程
**/
float Get_GPS_Raw_X(void)
{
	return raw_gps_x;
}

/**
	@name: Get_GPS_Ori_Y
	@brief:返回从动轮转过路程
**/
float Get_GPS_Raw_Y(void)
{
	return raw_gps_y;
}

int32_t gps_cycle_test_x(void)
{
	return gps_cycle_x;
}

int32_t gps_cycle_test_y(void)
{
	return gps_cycle_y;
}

/**
	@name: X_Cycle
	@brief: X编码器圈数自加
	@param:uint8_t flag  自加还是自减信号
**/
void X_Cycle(uint8_t flag)
{
	if(flag)
		gps_cycle_x++;
	else 
		gps_cycle_x--;
}
/**
	@name: Y_Cycle
	@brief: Y编码器圈数自加
	@param:uint8_t flag  自加还是自减信号
**/
void Y_Cycle(uint8_t flag)
{
	if(flag)
		gps_cycle_y++;
	else 
		gps_cycle_y--;
}
#endif