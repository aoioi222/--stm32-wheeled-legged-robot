#include "Kalman_Filter.h"
#include <math.h>
#define DEGTORAD (3.14159265358979323846f / 180.0f)


Angles final_angles = {0.0f, 0.0f, 0.0f};



static kalman_filter_typedef kf_roll;
static kalman_filter_typedef kf_pitch;
static kalman_filter_typedef kf_yaw;


/**
 * @brief  卡尔曼滤波器初始化
 * @param  kf        指向单个轴卡尔曼滤波器结构体
 * @param  angle_0   初始角度值(rad)
 * @param  bias_0    初始陀螺仪零偏值(rad/s)
 * @param  p00 p01 p10 p11  初始协方差矩阵各个元素
 * @param  R_angle   角度观测噪声协方差，表征加速度计输出噪声水平
 * @param  Q_angle   过程噪声协方差-角度，模型预测的角度不确定度
 * @param  Q_gyro    过程噪声协方差-陀螺，模型预测的陀螺零偏不确定度
 * @note   1.p01与p10代表角度和零偏之间的互协方差，普通IMU应用初始可置0
 *         2.该滤波器为动态零偏估计，不需要预先做离线陀螺标定，运行中在线估计陀螺漂移
 *         3.参数调优经验：振动大场景R适当调大；陀螺噪声大则Q_gyro调大；加速度计噪声大R_angle调大
 */

void Kalman_Filter_Init(kalman_filter_typedef *kf, 
                        float angle_0, float bias_0, 
                        float p00, float p01, float p10, float p11, 
                        float R_angle,
                        float Q_angle, float Q_gyro)
{
    kf->angle = angle_0;  // 初始化角度状态
    kf->bias = bias_0;    // 初始化陀螺零偏


    // 初始化误差协方差矩阵P
    kf->P[0][0] = p00;
    kf->P[0][1] = p01;
    kf->P[1][0] = p10;
    kf->P[1][1] = p11;
    
    // 赋值过程噪声Q、观测噪声R，滤波器调参核心参数
    kf->Q_angle = Q_angle;
    kf->Q_gyro = Q_gyro;
    kf->R_angle = R_angle;
}


/**
 * @brief  卡尔曼滤波器迭代计算
 * @param  kf        指向单轴卡尔曼滤波器结构体
 * @param  newAngle  观测角度输入，来自加速度计解算(rad)
 * @param  newGyro   陀螺仪测量值(rad/s)，已经扣除外部零偏
 * @param  dt        本次滤波迭代时间间隔，单位秒(s)
 * @retval 滤波输出最优估计角度(rad)
 * @note   1.调用周期必须和dt严格匹配
 *         2.单位约定：newGyro单位rad/s，dt单位s，newAngle单位rad；
 *           原始IMU读出为角度制，调用该函数前务必转换为弧度
 */

float Kalman_Filter_Calculate(kalman_filter_typedef *kf, float newAngle, float newGyro, float dt)
{
    // 读取上一次的状态量
    float bias = kf->bias;
    float angle = kf->angle;

    // 状态预测：陀螺积分预测角度
    angle = angle - bias * dt + newGyro * dt;

    // 协方差矩阵预测更新
    kf->P[0][0] = kf->P[0][0] + kf->Q_angle - (kf->P[0][1] + kf->P[1][0]) * dt;
    kf->P[0][1] = kf->P[0][1] - kf->P[1][1] * dt;
    kf->P[1][0] = kf->P[1][0] - kf->P[1][1] * dt;
    kf->P[1][1] = kf->P[1][1] + kf->Q_gyro;

    // 计算卡尔曼增益K
    float K0 = kf->P[0][0] / (kf->P[0][0] + kf->R_angle);
    float K1 = kf->P[1][0] / (kf->P[0][0] + kf->R_angle);

    // 状态修正，利用加速度计观测残差修正角度与陀螺零偏
    kf->angle = angle + K0 * (newAngle - angle);
    kf->bias = bias + K1 * (newAngle - angle);

    // 更新后验协方差矩阵
    kf->P[0][0] = kf->P[0][0] - K0 * kf->P[0][0];
    kf->P[0][1] = kf->P[0][1] - K0 * kf->P[0][1];
    kf->P[1][0] = kf->P[1][0] - K1 * kf->P[0][0];
    kf->P[1][1] = kf->P[1][1] - K1 * kf->P[0][1];

    // 返回卡尔曼滤波输出角度
    return kf->angle;
}


bool IMU_Fusion_Init(I2C_HandleTypeDef *i2c)
{
    ICM42688P_InitConfigTypeDef imu_config = {.Hi2c = i2c, .ICM42688P_Addr = 0x68u};
    if (ICM42688P_Init(&imu_config, ICM42688P_CALIB_SAMPLES) != 0u)
        return false;

    Kalman_Filter_Init(&kf_roll,  0, 0, 1, 0, 0, 1, 0.001f, 0.003f, 0.03f);
    Kalman_Filter_Init(&kf_pitch, 0, 0, 1, 0, 0, 1, 0.001f, 0.003f, 0.03f);
    Kalman_Filter_Init(&kf_yaw,   0, 0, 1, 0, 0, 1, 0.001f, 0.003f, 0.03f);
    return true;
}


/**
 * @brief  IMU姿态融合主函数
 * @param  dt  两次调用之间时间间隔，单位s
 * @retval 返回融合后的姿态角度结构体指针
 * @note   单位统一：陀螺仪输入rad/s，角度输出rad；
 *         Yaw航向无加速度计绝对观测，仅做陀螺积分和零偏估计，会发生漂移
 */
Angles * fusion_algorithm(float dt)
{
    ICM42688P_Data_t *imu;
    float acc_roll, acc_pitch;
    float gyro_x, gyro_y, gyro_z;
    float acc_norm;


    /* 从ICM42688P FIFO读取刷新原始数据 */
    if (ICM42688P_UpdateData() != 0U)
        return &final_angles;


    /* 获取经过驱动转换后的物理量 */
    imu = ICM42688P_GetData();


    /* 加速度计解算roll、pitch，输出弧度 */
    acc_roll  = atan2f(-imu->acc[1], imu->acc[2]);
    acc_norm  = sqrtf(imu->acc[1] * imu->acc[1] + imu->acc[2] * imu->acc[2]);
    acc_pitch = atan2f(imu->acc[0], acc_norm);


    /* 陀螺仪：角度制(deg/s)转换为弧度制(rad/s) */
    gyro_x = imu->gyro[0] * DEGTORAD;
    gyro_y = imu->gyro[1] * DEGTORAD;
    gyro_z = imu->gyro[2] * DEGTORAD;


    /* Roll Pitch：加速度计观测 + 陀螺仪预测 卡尔曼融合 */
    final_angles.roll  = Kalman_Filter_Calculate(&kf_roll,  acc_roll,  gyro_x, dt);
    final_angles.pitch = Kalman_Filter_Calculate(&kf_pitch, acc_pitch, gyro_y, dt);


    /* Yaw航向：无加速度计绝对参考，使用自身上一时刻角度做观测，仅积分+零偏估计，存在漂移 */
    final_angles.yaw   = Kalman_Filter_Calculate(&kf_yaw, kf_yaw.angle, gyro_z, dt);


    return &final_angles;
}


/**
 * @brief  获取融合之后姿态角度
 * @retval 姿态结构体指针，roll pitch yaw单位rad
 */
Angles * Get_Angle(void)
{
    return &final_angles;
}
