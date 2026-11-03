/*@filename: Laser_Positioning.h
 *@author     胡兴国       
 *@brief:     激光定位
 *@date: 2023-7-27
*/

#ifndef _LASER_POSITION_H_
#define _LASER_POSITION_H_

#include "rcs.h"

#define LASER_EQUIVALENT                                0.8544921875		//(3.6m)         0.2197265625f(1m)		// 量程差/4096.0
#define LASER_MIN_DIS																		100.0f					//最小标定距离			
#define LASER_MAX_DIS																		10000.0f				//最大标定距离

#define LASER3X4																				160.0f					//激光3到激光4的安装的横向距离
#define MIDPOSX																					5500.0f					//中场X坐标

#define BUFFERLEN																				5							//滤波队列长度
#define BUFFERLEN_VISION																20						//视觉队列滤波长度
#define TIMES                                           100            //一次中位数滤波对象个数

void Laser_Init(void);																									//激光初始化
void Laser_Task(void);
float Get_Laser_GPS_X(void);
float Get_Laser_GPS_Y(void);
float Get_Laser_GPS_Z(void);
float Get_Laser_Distance_X(void);
float Get_Laser_Distance_Y(void);
float Get_Laser_Z(void);																								//获取激光偏航角
void Laser_Get_Pos(float *now_pos_x,float *now_pos_y,float now_pos_z);	//激光定位
#endif
