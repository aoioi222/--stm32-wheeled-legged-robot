/**
 * @file sync_pid.cpp
 * @author 陈煜楷
 * @brief 同步PID的面向对象实现，提供多种保护措施
 * @version 0.1
 * @date 2025-05-03
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "sync_pid.h"
#include <cmath> 

/**
 * @brief 同步PID控制器构造函数
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param max_output 输出限幅，防止输出过大
 * @param max_integral 积分限幅，防止积分饱和
 * @param filter_d 微分滤波系数，0~1之间，1为不滤波
 * @param dead_zone 死区，只有误差超过这个范围才进行PID计算
 * @param threshold_intergral 积分阈值，只有误差超过这个阈值才进行积分
 */
sync_pid::sync_pid(float kp, float ki, float kd, float max_output, float max_integral, float filter_d, float dead_zone,float threshold_intergral)
    : kp(kp), ki(ki), kd(kd), max_output(max_output), max_integral(max_integral),
      filter_d(filter_d), dead_zone(dead_zone), threshold_intergral(threshold_intergral),
      last_error(0.0f), last_output(0.0f), integral(0.0f), d_term(0.0f)
{
}

/**
 * @brief 计算函数
 * @param target 目标值
 * @param current 当前值
 * @return 输出值
 */
float sync_pid::calculate(float target, float current)
{
    float error = target - current;

    // 死区处理
    if (std::fabs(error) < dead_zone)
        error = 0.0f;

    // 积分项计算，限幅
    integral += error;
    if (integral > max_integral)
        integral = max_integral;
    else if (integral < -max_integral)
        integral = -max_integral;

    // 微分项使用一阶滤波器
    float raw_derivative = error - last_error;
    d_term = (1.0f - filter_d) * d_term + filter_d * raw_derivative;

    // PID 输出
    float output = kp * error + ki * integral + kd * d_term;

    // 输出限幅
    if (output > max_output)
        output = max_output;
    else if (output < -max_output)
        output = -max_output;

    last_error = error;
    last_output = output;

    return output;
}

/**
 * @brief 设置比例系数
 * @param val 比例系数
 * @return 错误码
 */
rcs_err_t sync_pid::set_kp(float val)
{ 
    kp = val; 
    return RCS_OK; 
}

/**
 * @brief 设置积分系数
 * @param val 积分系数
 * @return 错误码
 */
rcs_err_t sync_pid::set_ki(float val)
{ 
    ki = val; 
    return RCS_OK; 
}

/**
 * @brief 设置微分系数
 * @param val 微分系数
 * @return 错误码
 */ 
rcs_err_t sync_pid::set_kd(float val)
{ 
    kd = val; return RCS_OK; 
}

/**
 * @brief 设置输出限幅
 * @param val 输出限幅
 * @return 错误码
 */
rcs_err_t sync_pid::set_max_output(float val) 
{ 
    max_output = val; return RCS_OK; 
}

/**
 * @brief 设置积分限幅
 * @param val 积分限幅
 * @return 错误码
 */
rcs_err_t sync_pid::set_max_integral(float val)
{ 
    max_integral = val; return RCS_OK; 
}

/**
 * @brief 设置微分滤波系数  
 * @param val 微分滤波系数
 * @return 错误码
 */
rcs_err_t sync_pid::set_filter_d(float val)
{ 
    filter_d = val; 
    return RCS_OK; 
}

/**
 * @brief 设置死区
 * @param val 死区
 * @return 错误码
 */
rcs_err_t sync_pid::set_dead_zone(float val)  
{ 
    dead_zone = val; 
    return RCS_OK; 
}

/**
 * @brief 设置积分阈值
 * 
 * @param threshold 
 * @return rcs_err_t 
 */
rcs_err_t sync_pid::set_threshold_i(float threshold)
{
    threshold_intergral = threshold;
    return RCS_OK;
}

