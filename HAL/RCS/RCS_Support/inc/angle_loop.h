/**
 * @file angle_loop.h
 * @author 陈煜楷
 * @brief 角度运算的回环操作，包含两种操作：规范和实现
 *        1. 规范：将位于[-∞,+∞]的角度转为[-180,180)或[0,360)。也即从无限到有限。
 *        2. 实现：给定一个规范化的角度，再给定一个参考角度，计算一个最接近参考角度的角度。也即从有限到无限。
 * @note 应用场景
 *        1. 舵轮角度回环。因为舵轮需要连续转动，在经过边界(正负180)时需要计算最短路，否则会卡住过不去。
 *        2. 对电机做类似机械时钟那样的角度控制。也即用户只指定相对角度，但电机获取到的是绝对角度。
 * ----------------------------------------------------------------------------------------------------
 * @version 0.1
 * @date 2025-06-03
 * @changelog 初次创建并完成代码逻辑测试
 * ----------------------------------------------------------------------------------------------------
 * 
 */

#pragma once
#include <cmath>

class angle_loop
{
    public:
        /**
         * @brief 把负无穷到正无穷的角度转为[-180,180)
         * 
         * @param angle_deg 
         * @retval 规范化后的角度 
         */
        static inline double regular_from_inf_to_0(double angle_deg)
        {
            double retval = angle_deg;
            retval = fmod(retval,360.0);
            if (retval < 0) {
                retval += 360.0;
            }
            if (retval > 180.0) {
                retval -= 360.0;
            }
            if (retval == 180.0) {
                retval = -180.0;
            }
            return retval;
        }
        /**
         * @brief 把负无穷到正无穷的角度转为[0,360)
         * 
         * @param angle_deg 
         * @retval 规范化后的角度 
         */
        static inline double regular_from_inf_to_180(double angle_deg)
        {
            double retval = angle_loop::regular_from_inf_to_0(angle_deg);
            if (retval < 0){
                retval += 360.0;
            }
            return retval;
        }
        /**
         * @brief 给定[-180,180)的目标角度、[-∞,+∞]的当前角度，计算抵达目标的最近角度
         * 
         * @param angle_regular_deg  取值在[-180,180)的原始角度
         * @param ref_inf_deg        取值在[-∞,+∞]的参考角度
         * @retval 最近角度。其含义是：将原始角度加上k个360度后，使得结果最接近参考角度。k是整数，不一定是自然数。
         */
        static inline double normalize_from_0_to_inf(double angle_regular_deg, double ref_inf_deg)
        {
            double diff = ref_inf_deg - angle_regular_deg;
            double k = std::round(diff / 360.0);
            return angle_regular_deg + k * 360.0;
        }
};