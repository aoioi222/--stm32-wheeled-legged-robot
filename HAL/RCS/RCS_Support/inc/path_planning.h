/**
 * @file path_planning.h
 * @author 陈煜楷
 * @brief 常用的路径规划器，可以串联到PID的前端，提供平滑输出
 * @version 0.1
 * @date 2025-06-03
 * @copyright Copyright XMU-RCS (c) 2025
 * -----------------------------------------------------------------
 * @changelog 2025-06-03
 * 1. 实现了1维梯形规划器。尚未移植陈志伟的2维贝塞尔规划器。
 * -----------------------------------------------------------------
 */

#pragma once
#include <cmath>

/**
 * @brief 梯形路径规划器
 * 
 * @note 使用场景：需要限制加速度的场合，比如底盘跑点时，不能让速度突变，否则全场定位容易打滑，造成精度误差。
 *       又比如是对电机的输出进行平滑，避免胀紧套因加速度过大打滑、松动。
 * 
 * @note 使用方式：建议串联在PID控制器的前端，如下所示
 * 
 *       当前值---------------------+
 *                                 |
 *                                 v
 *       目标值--->梯形规划器--->PID控制器--->输出值---->下发给电机
 * 
 * @note 上述流程的伪代码：
 * 
 * static rectangle_plan planer(0.1,0.1);
 * static sync_pid pid(RCS_ASYN_PID_DEFAULT_RM3508_SPPED_PID);
 * static 电机类 电机A;
 * 
 * void rectangle_pid_test(float target)
 * {
 *   float current = 电机A.获取转速();
 *   float smooth_target = planer.plan(target);
 *   float output = pid.calculate(current,smooth_target);
 *   电机A.更新电流(output);
 * }
 * 
 */
class rectangle_plan
{
    public:
        rectangle_plan(float max_slope_rise,float max_slope_fall);
        float plan(float target);
        void reset(float last_output);
    private:
        enum direction{
            RISE,
            FALL
        };
        float max_rise_k;
        float max_fall_k;
        float last_output;
};