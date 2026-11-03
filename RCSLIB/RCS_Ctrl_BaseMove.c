/**
 * @filename:RCS_PROVINCE-COMPETITION
 * @brief:底盘综合控制
 * @changlog: 2023-7-28 胡兴国 修改整理
 * @changlog: 2024-2-20 陈煜楷 加入舵轮底盘
*/

/* =================头文件=================================*/
#include "RCS_Ctrl_BaseMove.h"

/* =================私有全局变量============================*/
static int angle_flag = 0;
static int distance_flag = 0;
static float dece_distance = 0;
static int count[100] = {0};
static int i = 0;
static int acce_time;
static float acce_speed,dece_speed;

/* ==================外部导入================================*/
extern __inline void RCS_USART_Send_Char( USART_TypeDef *_USARTx, uint8_t _character);
extern int rocker_lx,rocker_ly;  //左摇杆控制运动
extern int rocker_rx,rocker_ry;  //右遥感控制旋转
extern int direction_key[4];

/* ==================静态函数================================*/
static double Calculate_Direction(Route_Point *rpoint,int index,int lead_point_num);
static float Speed_Limit(float speed,float max_speed,float min_speed);
static int Point_Limit(int num);
static float EndSpeedCalculate(int index,float max_speed);

/* ==================函数实现================================*/

/**
 * @name: Ctrl_BaseMove_Init
 * @brief: 底盘控制初始化
**/
void Ctrl_BaseMove_Init(void)
{
	#ifdef USE_OMNI_CHASSIS
		M_BaseMove_Init();		//麦轮底盘初始化
	#endif 
	
	#ifdef USE_AGV_CHASSIS 
		AGV_Init();           //舵轮底盘初始化
	#endif
}



/**
	@name: Point2Point_ER
	@brief: 胡兴国最佳直线跑点算法
	@param: Coord_Point* cp  目标点,包括xyz,详见RCS_Ctrl_BaseMove.h
	@param: Trace_Param* tp  跑点参数,详见RCS_Ctrl_BaseMove.h
	@param: int *ER_flag     完成跑点的标志位
**/
void Point2Point_ER(Coord_Point* cp,Trace_Param* tp,int *ER_flag)
{
	Coord_Point input_cp=*cp;//用于编译器优化
	Trace_Param input_tp=*tp;

	float x,y,z;
	float direction,speed,rotate_speed;
	float distance;
	
	//获取当前坐标
	x = Get_GPS_Action_X();
	y = Get_GPS_Action_Y();
	z = Get_GPS_Z();
	
	direction = -z -(atan2((input_cp.x - x),(input_cp.y - y))) * RAD2DEG;      //调整当前运动方向
	distance = sqrt((x - input_cp.x)*(x - input_cp.x) + (y - input_cp.y)*(y - input_cp.y));   //判断离终点距离
	dece_distance = (input_tp.run_speed * input_tp.run_speed)/(2 * input_tp.DECE);    //计算开始减速时的距离
	acce_time = (int)(input_tp.run_speed/input_tp.ACCE);    //加速次数/时间
	
	//加速过程
	if(count[i] < acce_time)
	{
		acce_speed = count[i] * input_tp.run_speed / acce_time;
		count[i] ++;
	}
	
	//判断是否到达终点
	if(distance <5)
	{
		distance_flag = 1;
		dece_speed = 0;
	}
	else
	{
		distance_flag = 0;
		
		//快速运动范围
		if(distance > dece_distance)
		{
			dece_speed = input_tp.run_speed;
		}
		//减速
		else
		{
			dece_speed = sqrt(2 * input_tp.DECE * distance);
//			dece_speed = (2 * DECE * distance)*(2 * DECE * distance);
//			dece_speed = run_speed * distance / dece_distance;;
		}
	}
	
	//判断是加速还是减速
	if(acce_speed >= dece_speed)
	{
		speed = dece_speed;
	}
	else
	{
		speed = acce_speed;
	}
	
	//旋转
	rotate_speed = PID_Angle_Ctrl(input_cp.z,Get_GPS_Z(),&tp->rotate_angle_pid);
	if(((input_cp.z-Get_GPS_Z())>=-(input_tp.R_ANGLE))&&((input_cp.z-Get_GPS_Z())<=input_tp.L_ANGLE))
	{
		angle_flag = 1;
	}
	else
	{
		angle_flag = 0;
	}	
	
	BaseMove_Polar(direction,speed,rotate_speed);
  
	//判断是否到达
	if((distance_flag == 1)&&(angle_flag == 1))
	{
		distance_flag = 0;
		angle_flag = 0;
		i++;
		*ER_flag = 1;
//		upspeed_flag = 0;
	}
	else
	{
		*ER_flag =  0;
	}
	return;
}



/**
	@name: Point2Point
	@brief: 点对点移动自动控制
	@param: float target_x		目标X坐标
	@param: float target_y		目标Y坐标
	@param: float target_z		目标偏航角
**/
void Point2Point(float target_x,float target_y,float target_z,PID_Struct distance_x_pid,PID_Struct distance_y_pid,PID_Struct rotate_angle_pid)
{
	float speed[3] = {0,0,0};
	float direction,run_speed;
	
	speed[0] = PID_Normal_Ctrl(target_x,Get_GPS_X(),&distance_x_pid);
	speed[1] = PID_Normal_Ctrl(target_y,Get_GPS_Y(),&distance_y_pid);
	speed[2] = PID_Normal_Ctrl(target_z,Get_GPS_Z(),&rotate_angle_pid);
	
	direction = -Get_GPS_Z() -(atan2(speed[0],speed[1])) * RAD2DEG;
	run_speed = sqrt(speed[0] * speed[0] + speed[1] * speed[1]);
	if(run_speed>500)
	{
		run_speed = 500;
	}
	
	BaseMove_Polar(direction,run_speed,speed[2]);
}

/**
	@name: Bessel_P2P
	@brief: 贝赛尔曲线点对点移动自动控制
	@param: Route_Point rpoint		贝塞尔曲线点位信息
	@return: 当前曲线的点位数
	@explain:路径规划一定要合理，角度绝对值在45°~135°之间的用'x'，以外用'y',并且曲线一定要尽可能单调
**/
void Bessel_P2P(Route_Point *rpoint,int *index,int AUTO_SPEED,float target_z,PID_Struct distance_x_pid,PID_Struct distance_y_pid,PID_Struct rotate_angle_pid,PID_Struct angle_pid)
{
	float x,y,z;
	float direction,run_speed,rotate_speed;
	float temp[2],nearest_dis;
	float speed[2];
	float nearest_point[2]; 
	int int_lead[2];
	
  //获取当前坐标
	x = Get_GPS_X();
	y = Get_GPS_Y();
	z = Get_GPS_Z();
	
	temp[0] = rpoint->point[POINT_NUM-1].x - rpoint->point[0].x;
	temp[1] = rpoint->point[POINT_NUM-1].y - rpoint->point[0].y;
	
	if((temp[0] == 0) && (temp[1]== 0))								//原地旋转
	{
		//计算当前点位
		nearest_point[0] = POINT_NUM * ((z - rpoint->point[0].z)/(rpoint->point[POINT_NUM-1].z - rpoint->point[0].z));
		//牵引点限幅
		int_lead[0] = Point_Limit(nearest_point[0])+10;
		if(Judge_Pos_B(int_lead[0]))
		{
			*index =  int_lead[0];
			BaseMove_P2P(0,0,0);
			return;
		}
		if(((rpoint->point[POINT_NUM-1].z) - (rpoint->point[0].z))>0)
		{
			rotate_speed = 100.0f;
		}
		else
		{
			rotate_speed = -100.0f;
		}
		BaseMove_P2P(0,0,rotate_speed);
		*index =  int_lead[0];					//传出最近点
		return;
	}
	
	else if(temp[0] == 0)					//沿Y轴方向移动
	{
		nearest_point[1] = POINT_NUM * ((y - rpoint->point[0].y)/temp[1]);
		int_lead[0] = Point_Limit(nearest_point[1]);
	}
	else if(temp[1] == 0)				//沿X轴方向移动
	{
		nearest_point[0] = POINT_NUM * ((x - rpoint->point[0].x)/temp[0]);
		int_lead[0] = Point_Limit(nearest_point[0]);
	}
	else										//正常情况最近点判定（一定要是单调曲线）
	{
		nearest_point[0] = POINT_NUM * ((x - rpoint->point[0].x)/temp[0]);		//X固定下Y方向的最近点
		if(nearest_point[0] > POINT_NUM-1)
			nearest_point[0] = POINT_NUM-1;
		if(nearest_point[0] < 0)
			nearest_point[0] = 0;		
		int_lead[0] = (int)(nearest_point[0]);
		
		nearest_point[1] = POINT_NUM * ((y - rpoint->point[0].y)/temp[1]);	  //Y固定下X方向的最近点
		if(nearest_point[1] > POINT_NUM-1)
			nearest_point[1] = POINT_NUM-1;
		if(nearest_point[1] < 0)
			nearest_point[1] = 0;		
		int_lead[1] = (int)(nearest_point[1]);
		
		temp[0] = nearest_point[0] - nearest_point[1];												//两个最近点取差
		if(temp[0] != 0)													//若两者算出的最近点相同则最近点不变，反之则取二者的线性组合：(cos2θ+1)/2 = △n = nx-nearest
		{
			temp[1] = atan2(rpoint->point[int_lead[0]].x - rpoint->point[int_lead[1]].x,rpoint->point[int_lead[1]].y - rpoint->point[int_lead[0]].y);
			int_lead[0] =(int)(nearest_point[0] - (cos(2*temp[1])+1.0f) * temp[0] /2.0f);
			int_lead[0] = Point_Limit(int_lead[0]);
		}
	}
	
	if(Judge_Pos_B(int_lead[0]))
	{
		*index =  int_lead[0];
		BaseMove_Polar(0,0,0);
		return;
	}
	temp[0] = Calculate_Direction(rpoint,int_lead[0],1);  //弧度制
	
	//判断沿x方向还是y方向运动
	if(fabsf(rpoint->speed[POINT_NUM-1].direction * DEG2RAD) < pi/4.0f*3.0f && fabsf(rpoint->speed[POINT_NUM-1].direction * DEG2RAD) > pi/4.0f)
		rpoint->direction = 'x';
	else
		rpoint->direction = 'y';
	
	//计算校准点 
	if(rpoint->direction == 'x')
	{
		nearest_dis = (y - rpoint->point[int_lead[0]].y) * sin(temp[0]);
		run_speed = PID_Normal_Ctrl(0,nearest_dis,&distance_y_pid);
	}
	else
	{
		nearest_dis = (x - rpoint->point[int_lead[0]].x) * cos(temp[0]);
		run_speed = PID_Normal_Ctrl(0,nearest_dis,&distance_x_pid);
	}
	
	//路线法向整定
	direction = temp[0] - pi/2.0f;
	speed[0] = -run_speed * sin(direction);
	speed[1] =  run_speed * cos(direction);
	
	if(rpoint->end_speed_ctrl == 0)
	{
		//不减速
		speed[0] = speed[0] - sin(temp[0]) * AUTO_SPEED;
		speed[1] = speed[1] + cos(temp[0]) * AUTO_SPEED;
	}
	else              
	{
		//靠近终点时减速
		speed[0] = speed[0] - sin(temp[0]) * EndSpeedCalculate(int_lead[0],AUTO_SPEED);
		speed[1] = speed[1] + cos(temp[0]) * EndSpeedCalculate(int_lead[0],AUTO_SPEED);
	}
	
	//换算成极坐标
	direction = atan2(-speed[0],speed[1]) * RAD2DEG - z;
	run_speed = sqrt(speed[0] * speed[0] + speed[1] * speed[1]);
	
	//偏航角度整定
	rotate_speed = PID_Normal_Ctrl(rpoint->point[int_lead[0]].z,z,&angle_pid);
	
	BaseMove_Polar(direction,run_speed,rotate_speed);
	*index =  int_lead[0];					//传出最近点
	
	return;
}

/**
	@name: EndSpeedCalculate
	@brief: 速度计算
	@param: float max_speed 最大速度
	@param:	int index 路线点索引
	@return: 速度值
**/
static float EndSpeedCalculate(int index,float max_speed)
{
	if(index <= SLOW_POINT)
		return max_speed;
	else
		return ((max_speed-400.0f) * (1.0f - (index-SLOW_POINT)/(POINT_NUM-SLOW_POINT)))+400.0f;			//kx+b
}

/**
	@name: Calculate_Direction
	@brief: 方向角计算
	@param: Route_Point  *rpoint	曲线
	@param: int index							索引
	@param:	int lead_point_num		超前点
	@return: 索引点的方向角度（角度制）
**/
static double Calculate_Direction(Route_Point *rpoint,int index,int lead_point_num)
{
	float temp[2];
	if(index == POINT_NUM)
	{
		temp[0] = rpoint->ctrl_point[1].x - rpoint->point[POINT_NUM-1].x;
		temp[1] = rpoint->point[POINT_NUM-1].y - rpoint->ctrl_point[1].y;
	}
	else if(index == 0)
	{
		temp[0] = rpoint->point[0].x - rpoint->ctrl_point[0].x;
		temp[1] = rpoint->ctrl_point[0].y - rpoint->point[0].y;
	}
	else
	{
		temp[0] = rpoint->point[index-1].x - rpoint->point[Point_Limit(index-1+lead_point_num)].x;
		temp[1] = rpoint->point[Point_Limit(index-1+lead_point_num)].y - rpoint->point[index-1].y;
	}
	return (atan2(temp[0],temp[1]));
}


/**
	@name: Judge_Pos
	@brief: 位置到达判定
	@param: float target_x					目标X
	@param: float target_y					目标Y
	@param: float target_z					目标Z
	@return: 1到达，0未到达
**/
int Judge_Pos(float target_x,float target_y,float target_z)
{
		float err_xy,err_z,err_speed;
		
		err_xy = fabsf(Get_KalmanFilter_X()-target_x)+fabsf(Get_KalmanFilter_Y()-target_y);
		err_z = fabsf(Get_Ave_Z()-target_z);
		for(int i =0;i<4;i++)
			err_speed = abs(Get_Motor_Speed(i+1));
	
		if((err_xy < 20) && (err_speed < 400) && (err_z < 1 ))
			return 1;
		else
			return 0;
}

/**
	@name: Judge_Pos_B
	@brief: 位置到达判定(贝塞尔)
	@param: int index					当前贝塞尔曲线点位
	@return: 1到达，0未到达
**/
int Judge_Pos_B(int index)
{
	if(index >= (POINT_NUM - 1))
		return 1;
	else
		return 0;
}

/**
	@name: Point_Limit
	@brief: 点数限幅
	@param: int num				点数
	@return: int 					限幅后点数
**/
static int Point_Limit(int num)
{
	if(num > POINT_NUM-1)
		return POINT_NUM-1;
	else if(num <0)
		return 0;
	else
		return num;
}

/**
	@name: Speed_Limit
	@brief: 速度限幅
	@param: float speed				速度
	@param: float max_speed		最大速度
	@param: float min_speed		最小速度
	@return: float 						限幅后速度
**/
static float Speed_Limit(float speed,float max_speed,float min_speed)
{
	if(speed > max_speed)
		speed = max_speed;
	if(speed < min_speed)
		speed = min_speed;
	return speed;
}
