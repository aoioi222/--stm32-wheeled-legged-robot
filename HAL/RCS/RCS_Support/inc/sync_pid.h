/**
 * @file sync_pid.h
 * @author 陈煜楷
 * @brief 同步PID的面向对象实现，提供多种保护措施
 * @version 0.1
 * @date 2025-05-03
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once

#include "rcs_private_config.h"
#include "rcs_types.h"

/**
 * @brief 之前调过的勉强能用的PID参数
 * @note 使用方式：在定义对象时传入构造函数，比如sync_pid my_pid(RCS_ASYN_PID_DEFAULT_RM3508_SPPED_PID);
 * @note 如果运行效果不好，请精修
 */
#define RCS_ASYN_PID_DEFAULT_RM3508_SPPED_PID 8.3f  ,0.25f   ,0.38f  ,16000.0f  ,10000.0f ,0.0f ,0.0f ,0.0f
#define RCS_ASYN_PID_DEFAULT_RM3508_ANGLE_PID 20.0f ,0.0f    ,0.0f   ,1000.0f   ,1000.0f  ,0.0f ,0.0f ,0.0f
#define RCS_ASYN_PID_DEFAULT_RM2006_SPPED_PID 8.3f  ,0.25f   ,0.38f  ,16000.0f  ,10000.0f ,0.0f ,0.0f ,0.0f

/**
 * @brief 同步PID控制器
 * @note 使用方式：在定义对象时传入构造函数，比如sync_pid my_pid(RCS_ASYN_PID_DEFAULT_RM3508_SPPED_PID);
 * @note 如果运行效果不好，请精修
 */
class sync_pid
{
    public:
        sync_pid(float kp,float ki,float kd,float max_output,float max_integral,float filter_d,float dead_zone,float threshold_intergral);
        
        float calculate(float target,float current);
        
        rcs_err_t set_kp(float kp);
        rcs_err_t set_ki(float ki);
        rcs_err_t set_kd(float kd);

        rcs_err_t set_max_output(float max_output);
        rcs_err_t set_max_integral(float max_integral);
        rcs_err_t set_filter_d(float filter_d);
        rcs_err_t set_threshold_i(float threshold);
        rcs_err_t set_dead_zone(float dead_zone);

    private:
        float kp;
        float ki;
        float kd;
        float max_output;
        float max_integral;
        float filter_d;
        float dead_zone;
        float threshold_intergral;

        float last_error;
        float last_output;
        float integral;
        float d_term;
};
        

        
