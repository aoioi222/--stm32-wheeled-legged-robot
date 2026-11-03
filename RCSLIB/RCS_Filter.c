/*@filename: RCS_Filter.c
 *@author     陈志伟       
 *@brief:    滤波及数学运算函数
 *@date: 2020-11-20
 *@author     胡兴国     
 *@brief:    卡尔曼滤波
 *@date: 2023-5-5
*/


#include "RCS_Filter.h"

/************静态全局变量*************/
static float filter_x = 0;	//Kalman滤波后的坐标			
static float filter_y = 0;
static float xy_cov = 0;		//最优预测值协方差
static float filter_z = 0;	//均值滤波后的偏航角

/*************静态函数****************/
static float Average_Filter_Z(float gps_z,float laser_z,float gps_weight);

/**************************************/


/**
	@name: Kalman_Filter
	@brief: kalman滤波算法（滤波目标X、Y坐标）
	@explain:融合码盘坐标和激光坐标，暂未使用
**/
void Kalman_Filter(void)
{
	
	float rcs_pos_x = 0;			//全场定位坐标
	float rcs_pos_y = 0;	
	
	float rcs_delta_x;				//坐标增量
	float rcs_delta_y;
	
	float temp_filter_x = 0;	//预测值坐标（预测值）
	float temp_filter_y = 0;
	
	float kalman_gen = 0;			//卡尔曼增益
	float temp_cov = 0;				//临时预测值协方差
	
	static float rcs_last_x;	//上一时刻全场定位坐标增量
	static float rcs_last_y;
	
	static float laser_x = 0;	//激光坐标（测量值）
	static float laser_y = 0;
	
	rcs_pos_x = Get_GPS_X();	//获取全场定位坐标
	rcs_pos_y = Get_GPS_Y();

	filter_z = Get_GPS_Z();//0.9f * filter_z + 0.1f * Average_Filter_Z(RCS_Get_Z(),Get_Laser_Z(),0.7);		//滞后均值滤波偏航角
	Laser_Get_Pos(&laser_x,&laser_y,filter_z);				//获取激光定位坐标	

	rcs_delta_x = rcs_pos_x - rcs_last_x;	//获取坐标增量
	rcs_delta_y = rcs_pos_y - rcs_last_y;	
	
	temp_filter_x = filter_x + rcs_delta_x;	//预测值
	temp_filter_y = filter_y + rcs_delta_y;
	
	temp_cov = xy_cov + GPS_COV;						//预测值协方差
	kalman_gen = temp_cov / (temp_cov + LASER_COV);	//更新卡尔曼增益
	
	
	filter_x = temp_filter_x + kalman_gen * (laser_x - temp_filter_x);	//更新最优化预测值
	filter_y = temp_filter_y + kalman_gen * (laser_y - temp_filter_y);

	rcs_last_x = rcs_pos_x;			//记录上一时刻全场定位坐标
	rcs_last_y = rcs_pos_y;
	
	xy_cov = (1.0f - kalman_gen) * temp_cov;	//更新最优化预测协方差
}

/**
	@name: Get_KalmanFilter_X
	@brief: 获取卡尔曼滤波后的x坐标
	@return: 滤波后的X坐标值
**/
float Get_KalmanFilter_X(void)
{
	return Get_GPS_X();//filter_x;
}

/**
	@name: Get_KalmanFilter_Y
	@brief: 获取卡尔曼滤波后的y坐标
	@return: 滤波后的Y坐标值
**/
float Get_KalmanFilter_Y(void)
{
	return Get_GPS_Y();;//filter_y;
}

/**
	@name: Get_Ave_Z
	@brief: 获取均值滤波后的偏航角
	@return: 滤波后的Z坐标值
**/
float Get_Ave_Z(void)
{
	return Get_GPS_Z();
}

/**
	@name: KalmanFilter_x
	@brief:卡尔曼滤波算法，根据应用场景调节p q r参数
	@param:p误差协方差初始值
	@param:q过程噪声，影响曲线平滑程度，q越小越平滑，噪声影响越小，q越大动态响应越好。Q值越大，
         代表越信任测量值，Q值无穷大，代表只用测量值，Q值越小，代表越信任模型预测值，Q值为0，代表只用模型预测值
  @param:r观测噪声，传感器精度越高r越小，调整滤波后的曲线与实测曲线的相近程度，r越小越相信实际值，r越大越相信预测值，r无限大只相信预测值，r越大动态响应越慢
**/
float KalmanFilter_x(float inData)
{
  static float prevData = 0;                                 //上一个数据
//  static float p = 1.0f, q = 0.001f, r = 0.05f, kGain = 0;      //华科参数
//	static float p = 1.0f, q = 0.5f, r = 0.01f, kGain = 0; 
	static float p = 1.0f, q = 0.001f, r = 0.05f, kGain = 0;
  p = p + q;
  kGain = p / (p + r);          //计算卡尔曼增益
  inData = prevData + (kGain * (inData - prevData));    //准确位置=上一次预测位置+K*（实际数据-上一次预测位置）
  p = (1 - kGain) * p;           //更新测量方差
  prevData = inData;
  return inData;                 //返回估计值
}

/**
	@name: KalmanFilter_y
	@brief:卡尔曼滤波算法，根据应用场景调节p q r参数
	@param:误差协方差初始值
	@param:q过程噪声，影响曲线平滑程度，q越小越平滑，q越大动态响应越好。Q值越大，
         代表越信任测量值，Q值无穷大，代表只用测量值，Q值越小，代表越信任模型预测值，Q值为0，代表只用模型预测值
  @param:r观测噪声，调整滤波后的曲线与实测曲线的相近程度，r越小越相信实际值，r越大越相信预测值，r无限大只相信预测值，r越大动态响应越慢
**/
float KalmanFilter_y(float inData)
{
  static float prevData = 0;                                 //上一个数据
	static float p = 1.0f, q = 0.001f, r = 0.05f, kGain = 0;
  p = p + q;
  kGain = p / (p + r);                                      //计算卡尔曼增益
  inData = prevData + (kGain * (inData - prevData));      //计算本次滤波估计值
  p = (1 - kGain) * p;                                      //更新测量方差
  prevData = inData;
  return inData;                                             //返回估计值
}


/**
	@name: Get_Median
	@brief:队列中位数滤波
	@param:float *data          		  数组
	@param:int num            			数组大小
	@return	中位数
**/
float Get_Median(float data[],int num)
{
	float buf[num];
	float temp;
	
	for(int i =0;i<num;i++)
		buf[i] = data[i];
	
	for(int i =0;i<=num-2;i++)
	{
		for(int j = 0;j<=num-2;j++)
		{
			if(buf[j] > buf[j+1])
			{
				temp = buf[j];
				buf[j] = buf[j+1];
				buf[j+1] = temp;
			}
		}
	}
	
	if(num%2 == 1)
		return buf[(num-1)/2];
	else
		return ((buf[num/2-1] + buf[num/2])/2.0f);
}


/**
	@name: Get_Average
	@brief:队列平均值滤波
	@param:float *data          		  数组
	@param:int num            			数组大小
	@return	平均值
**/
float Get_Average(float data[],int num)
{
	float sum = 0;
	for(int i=0;i<num;i++)
		sum += data[i];
	return sum/((float)num);
}


/**
	@name: Average_Filter_Z
	@brief:偏航角加权平均滤波
	@param:float gps_z          		  全场定位偏航角
	@param:float laser_z            	激光定位偏航角
	@param:float gps_weight           全场定位的比重（0~1）
	@return	平均值
**/
static float Average_Filter_Z(float gps_z,float laser_z,float gps_weight)
{
	return (gps_weight * gps_z + (1.0f - gps_weight) * laser_z);
}

/**
	@name: Recursive_Filter
	@brief:加权递推均值滤波
	@param:float *data          		  数组
	@param:int num            			数组大小
	@param:int begin_index          最新值在数组中位置
	@return	滤波值
**/
float Recursive_Filter(float data[],int num,int begin_index)
{
	int index;
	float sum = 0;
	float sum_i = 0;
	index = begin_index;

	for(int i=num; i>0 ; i--)
	{
		sum += data[index] * i;
		sum_i += i;
		index --;
		if(index <0)
			index = num-1;
	}
	
	sum /= sum_i;
	return sum;
}


/**
	@name: Recursive_Filter
	@brief:中值滤波
	@param:float *data          		  数组
	@param:int num            			数组大小
	@param:int begin_index          最新值在数组中位置
	@return	滤波值
**/
float Median_Ave_Filter(float data[],int num)
{
	int max_num,min_num;
	float sum;
	max_num = data[0];
	min_num = data[0];
	sum = data[0];
	for(int i =1;i<num;i++)
	{
		sum += data[i];
		if(data[i] > max_num)
			max_num = data[i];
		if(data[i] < min_num)
			min_num = data[i];
	}
	sum = (sum - max_num - min_num)/(num-2);
	return sum;
}


/**
	@name: Count_Delay
	@brief:整型转字符串
	@param:int num           			整数
	@return	char str[].字符串指针
**/
void Int2Str(int num,char str[])
{
	char temp[16];
	int index,len;
	int temp_num = num;
	int dir_flag = 0;
	
	len = sizeof(str);
	if(temp_num == 0)
	{
		str[0] = '0';
		for(index = 1;index<len-1;index++)
			str[index] = ' ';
		str[len-1] = '\0';
	}
	else
	{
		if(temp_num <0)
		{
			str[0] = '-';
			dir_flag = 1;
			temp_num = -temp_num;
		}
		for(index = 0;temp_num != 0;index++)
		{
			temp[index] = '0'+ temp_num % 10;
			temp_num = temp_num/10;
		}
		temp_num = index-1;
		for(index = 0;index<=temp_num;index++)
			str[index+dir_flag] = temp[temp_num - index];
		for(index = temp_num+1;index<len-1;index++)
			str[index] = ' ';
		str[len-1] = '\0';
	}	
}

/**
	@name: Float2Str
	@brief:浮点型转字符串
	@param:float 		num           		浮点数
	@param:uint8_t  length          	字符串长度
	@param:uint8_t  decimal_length    保留的小数位数
	@return	char str[].字符串指针
**/
void Float2Str(float num, char str[], uint8_t length, uint8_t decimal_length)
{
	int temp_num;
	uint8_t tmp[10];
	int i = 0, j = 0;

	temp_num = (int)(num * pow(10, decimal_length));
	if (temp_num < 0)
	{
		str[j++] = '-';
		temp_num = -temp_num;
	}
	do
	{
		tmp[i++] = temp_num % 10 + '0';
		temp_num /= 10;
	}
	while (i < decimal_length);
	tmp[i++] = '.';
	do
	{
		tmp[i++] = temp_num % 10 + '0';
		temp_num /= 10;
	}
	while (temp_num);
	while (i)
		str[j++] = tmp[--i];
	while (j < length)
		str[j++] = ' ';
	str[j] = '\0';
}
