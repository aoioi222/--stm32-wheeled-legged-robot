#ifndef __KALMAN_FILTER_H
#define __KALMAN_FILTER_H


#include "stm32f4xx.h"                  // 芯片头文件
#include "robot_math.h"
#include "I2C_ICM_42688_P_control.h"


/*************************** 结构体定义 ***************************/

// 每个姿态轴独立使用一套卡尔曼滤波器实例，三轴分开运算
typedef struct
{
    // 滤波器状态
    float angle;    // 姿态角度，单位rad
    float bias;     // 陀螺仪估计零偏，单位rad/s
    
    // 噪声参数
    float Q_angle;  // 过程噪声-角度
    float Q_gyro;   // 过程噪声-陀螺零偏
    float R_angle;  // 观测噪声-加速度计角度
    
    // 2阶误差协方差矩阵
    float P[2][2];
    
} kalman_filter_typedef;


// IMU融合输出姿态结构体
typedef struct
{
    // 卡尔曼融合输出姿态角，单位rad
    float roll;
    float pitch;
    float   yaw;
    
}Angles;


/*************************** 外部变量声明 ***************************/
extern Angles final_angles;


/*************************** 函数声明 ***************************/
void Kalman_Filter_Init(kalman_filter_typedef *kf, 
                        float angle_0, float bias_0, 
                        float p00, float p01, float p10, float p11, 
                        float R_angle,
                        float Q_angle, float Q_gyro);
float Kalman_Filter_Calculate(kalman_filter_typedef *kf, float newAngle, float newGyro, float dt);


bool IMU_Fusion_Init(I2C_HandleTypeDef *i2c);
Angles * fusion_algorithm(float dt);
Angles * Get_Angle(void);


#endif
