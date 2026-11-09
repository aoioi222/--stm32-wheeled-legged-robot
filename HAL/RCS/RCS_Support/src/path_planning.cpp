/**
 * @file path_planning.cpp
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
#include "path_planning.h"

/**
 * @brief 梯形规划器的构造函数
 * 
 * @param max_slope_rise 最大上升斜率
 * @param max_slope_fall 最大下降斜率
 * 
 * @warning 梯形规划器需要保存过去信息，因此请将梯形规划区对象定义为全局的，避免被释放。
 */
rectangle_plan::rectangle_plan(float max_slope_rise,float max_slope_fall)
{
    max_rise_k = max_slope_rise;
    max_fall_k = max_slope_fall;
    last_output = 0;
}

/**
 * @brief 执行梯形规划
 * 
 * @param target 目标值
 * @return float 规划后的目标值
 */
float rectangle_plan::plan(float target)
{
    direction dir;
    // 远离0，称为rise
    if ((last_output>0 && target>last_output) || (last_output<0 && target<last_output)) {
        dir = RISE;
    }
    // 靠近0，称为fall
    else if ((last_output>0 && target<last_output) || (last_output<0 && target>last_output)) {
        dir = FALL;
    }
    // 跨越0，称为rise
    else {
        dir = RISE;
    }
    // 根据上升下降情况，使用梯形平滑输出
    switch(dir) {
        case RISE:
            if (target <= last_output)
                last_output = std::fmaxf(last_output-max_rise_k,target);
            else
                last_output = std::fminf(last_output+max_rise_k,target);
        break;

        case FALL:
            if (target <= last_output)
                last_output = std::fmaxf(last_output-max_fall_k,target);
            else
                last_output = std::fminf(last_output+max_fall_k,target);
        break;
    }
    // 返回规划后的目标点
    return last_output;
}

/**
 * @brief 重置梯形规划器
 * 
 * @param last_output 上一次的输出值
 * @warning 梯形规划器需要连续工作，如果中间有一段时间停了，在恢复工作时，需要重置梯形规划器，否则会计算错误。
 */
void rectangle_plan::reset(float last_output)
{
    this->last_output = last_output;
}