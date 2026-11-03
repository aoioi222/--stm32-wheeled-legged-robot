/*@filename:  Filter.h
 *@author     陈志伟       
 *@brief:     滤波算法
 *@date: 2020-12-11
*/

#ifndef _Filter_H_
#define _Filter_H_

#include "rcs.h"

void Kalman_Filter(void);
float Get_KalmanFilter_X(void);
float Get_KalmanFilter_Y(void);
float Get_Ave_Z(void);
float KalmanFilter_x(float inData);
float KalmanFilter_y(float inData);
float Get_Median(float *data,int num);
float Get_Average(float data[],int num);
float Recursive_Filter(float data[],int num,int begin_index);
float Median_Ave_Filter(float data[],int num);
int Count_Delay(int count_num,int num);
void Int2Str(int num,char str[]);
void Float2Str(float num, char str[], uint8_t length, uint8_t decimal_length);

#define GPS_COV				10.0f				//系统噪声协方差
#define	LASER_COV			10.0f				//测量值的噪声协方差

#endif
