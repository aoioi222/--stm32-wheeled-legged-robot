/*@filename: RCS_BaseMove.c
 *@author     胡兴国       
 *@brief:     底盘速度解算
 *@date: 2023-7-28
*/
#include "RCS_BaseMove.h"

/*******************
X型长方形麦轮底盘，默认CAN1通讯
电机编号、相对坐标系及电机正转时轮子的有效方向如下
        ↑Y
        |
 1↗----+---↘2
   |    |   |
   |    +---+----→X
   |        |长
 4↖--------↙3
       宽

绝对坐标系定义如下
       ↑Y
       |
       |
       |
     Z +---------→ X      
********************/

/********全局变量********/
static float wheel_dir[4];		   //电机正转时轮子运动的正方向
static float wheel_distance;		 //轮子和底盘中心得距离
static float wheel_w_dir[4];		 //底盘自转时自转线速度在各轮子上的方向
PID_Struct base_speed_pid[4];

/********静态函数********/
static void Wheel_Init(void);


/************************/


/**
	@name: M_BaseMove_Init
	@brief: 麦轮底盘初始化
**/
void M_BaseMove_Init(void)
{
		base_speed_pid[0]=PID_Get_RM2006_Speed_Pid();
		base_speed_pid[1]=PID_Get_RM2006_Speed_Pid();
		base_speed_pid[2]=PID_Get_RM2006_Speed_Pid();
		base_speed_pid[3]=PID_Get_RM2006_Speed_Pid();
		Motor_Init();		//电机初始化,是在can1shan
		Wheel_Init();		//轮子参数初始化
}

/**
	@name: BaseMove_P2P
	@brief: 麦轮底盘核心算法(直角坐标式)
	@param:float move_speed_x             横向平动速度(r/min)
	@param:float move_speed_y							纵向平动速度(r/min)
	@param:float rotate_speed             自转模式(rad/s逆时针为正,手动控制时开启)
**/
void BaseMove_P2P(float move_speed_x,float move_speed_y,float rotate_speed)
{
    float target_speed[4] = {0,0,0,0};			//目标转速
		float max_speed= 0;
		int32_t out[4]={0,0,0,0};

    for(int i=0;i<4;i++)
		{
				target_speed[i] += move_speed_x * cos(-pi/2.0f - wheel_dir[i]) + move_speed_y * cos( 0 - wheel_dir[i]); //平动速度在轮毂正方向投影
				target_speed[i] += rotate_speed * cos(wheel_w_dir[i] - wheel_dir[i]);									//转动线速度在轮毂正方向投影
				target_speed[i] *= MM_S2R_MIN;				//将轮毂正方向线速度转化为轮子转速
				target_speed[i] *= SLOWDOWN_RATE;		  //将轮子转速转化为编码器转速以便于PID调速
				if(fabsf(target_speed[i]) > max_speed)
					max_speed = fabsf(target_speed[i]);
		}
		
		if(max_speed > MAX_MOTOR_SPEED)
		{
			for(int i=0;i<4;i++)
				target_speed[i] = target_speed[i] * MAX_MOTOR_SPEED / max_speed;
		}
		
		for(int i= 0;i<4;i++)
			out[i] = PID_Motor_Ctrl(target_speed[i],Get_Motor_Speed(i+1),&base_speed_pid[i]); //速度环
		Get_Motor_Speed(1);

		Motor_Send(out[0], out[1], out[2], out[3]);
	
}

/**
	@name: BaseMove_Polar
	@brief:麦轮底盘核心算法(极坐标式)
	@param:float direction_angle	行车方向
	@param:float run_speed				行车速度
	@param:float rotate_speed		自转速度
**/
void BaseMove_Polar(float direction_angle,float run_speed,float rotate_speed)
{
    float target_speed[4] = {0};			//目标转速
		float max_speed = 0;							//最大速度限制
		int32_t out[4];

    for(int i=0;i<4;i++)
		{
				target_speed[i] += run_speed * cos(direction_angle * DEG2RAD - wheel_dir[i]); 			//平动速度在轮毂正方向投影
				target_speed[i] += rotate_speed * cos(wheel_w_dir[i] - wheel_dir[i]);								//转动线速度在轮毂正方向投影
				if(fabsf(target_speed[i]) > max_speed)
					max_speed = fabsf(target_speed[i]);
		}
		if(max_speed > MAX_MOTOR_SPEED)
		{
			for(int i=0;i<4;i++)
				target_speed[i] = target_speed[i] * MAX_MOTOR_SPEED / max_speed;
		}
		for(int i= 0;i<4;i++)
			out[i] = PID_Motor_Ctrl(target_speed[i],Get_Motor_Speed(i+1),&base_speed_pid[i]); //速度环

		Motor_Send(out[0], out[1], out[2], out[3]);
}


/**
	@name: BaseMove_Vision
	@brief:麦轮底盘核心算法(视觉式)
	@param:float move_speed     	       	视觉方向平动速度
	@param:float rotate_speed             自转速度
**/
void BaseMove_Vision(float move_speed,float rotate_speed)
{
    float target_speed[4] = {0,0,0,0};			//目标转速
		float max_speed= 0;	
		int32_t out[4]={0,0,0,0};

    for(int i=0;i<4;i++)
		{
				target_speed[i] += move_speed * cos(-wheel_dir[i]); //平动速度在轮毂正方向投影
				target_speed[i] += rotate_speed * cos(wheel_w_dir[i] - wheel_dir[i]);									//转动线速度在轮毂正方向投影
				target_speed[i] *= MM_S2R_MIN;				//将轮毂正方向线速度转化为轮子转速
				target_speed[i] *= SLOWDOWN_RATE;		  //将轮子转速转化为编码器转速以便于PID调速
				if(fabsf(target_speed[i]) > max_speed)
					max_speed = fabsf(target_speed[i]);
		}
		if(max_speed > MAX_MOTOR_SPEED)
		{
			for(int i=0;i<4;i++)
				target_speed[i] = target_speed[i] * MAX_MOTOR_SPEED / max_speed;
		}
		for(int i= 0;i<4;i++)
			out[i] = PID_Motor_Ctrl(target_speed[i],Get_Motor_Speed(i+1),&base_speed_pid[i]); //速度环

		Motor_Send(out[0], out[1], out[2], out[3]);
}

/**
	@name: Wheel_Init
	@brief:轮子参数初始化
**/
static void Wheel_Init(void)
{
	  //设置轮毂正方向（见最上方图例）
		wheel_dir[0] = -pi/4.0f;			
		wheel_dir[1] = 3 * wheel_dir[0];
		wheel_dir[2] = -wheel_dir[1];
		wheel_dir[3] = -wheel_dir[0];
		//设置自转时自转线速度在各轮子上的方向
		wheel_w_dir[0] = atan2( BASE_WIDTH, BASE_LENGTH) + pi/2.0f;
		wheel_w_dir[1] = atan2(-BASE_WIDTH, BASE_LENGTH) + pi/2.0f;
		wheel_w_dir[2] = atan2(-BASE_WIDTH,-BASE_LENGTH) + pi/2.0f;
		wheel_w_dir[3] = atan2( BASE_WIDTH,-BASE_LENGTH) + pi/2.0f;
		//计算轮子与底盘中心距离
		wheel_distance = sqrt(BASE_LENGTH * BASE_LENGTH + BASE_WIDTH * BASE_WIDTH) / 2.0f;
}
