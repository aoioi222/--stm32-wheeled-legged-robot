#pragma once
#include "rcs_types.h"
/**
 * @brief 四舵轮底盘计算器(含重心修正)
 * -----------------------------------------------------------
 * 
 * @note 平动正方向和ID定义：
 *       y
 *       ^
 *       |
 *       |  3号-----------------2号
 *       |   |                   |
 *       |   |                 (短边)
 *       |   |                   |
 *       |  0号------(长边)------1号  
 *       | 
 *       +-----------------------> x
 * 
 * @note 转动正方向定义：
 * 1. 与X轴重合，视为0度。
 * 2. 逆时针旋转，角度增加。
 * 
 * -------------------------------------------------------------
 */
class rcs_agv4
{
    public:
        // 导出类型
        struct wheel {
            float angle[4];
            float speed[4];
        };
        struct chassis {
            float x;
            float y;
            float z;
        };
        // 构造方法
        rcs_agv4(float chassis_length, float chassis_width,float bias_x, float bias_y);
        // 导出方法
        wheel inv_kin(float target_spd_x,float target_spd_y,float target_spd_z,float chassis_z_rad);
        chassis kin(wheel status);
        wheel find_nearest(wheel target,wheel current);
    private:
        float len;
        float wid;
        float bias_x;
        float bias_y;
};


/**
 * @brief 四全向轮底盘/四麦轮底盘解算
 * @tparam chassis_length 相邻两个全向轮的距离
 * -----------------------------------------------------------
 * 
 * @note 平动正方向和ID定义：
 *       y
 *       ^
 *       |
 *       |  3号--------------2号
 *       |   |               |
 *       |   |            (正方形)
 *       |   |               |
 *       |  0号---(正方形)---1号  
 *       | 
 *       +-----------------------> x
 * 
 * @note 转动正方向定义：
 * 1. 与X轴重合，视为0度。
 * 2. 逆时针旋转，角度增加。
 * 
 * -------------------------------------------------------------
 */
class rcs_omni4
{
    public:
        // 导出类型
        struct wheel {
            float speed[4];
        };
        struct chassis {
            float x;
            float y;
            float z;
        };
        // 构造函数
        rcs_omni4(float chassis_width);

        // 导出方法
        wheel inv_kin(float target_spd_x,float target_spd_y,float target_spd_z,float chassis_z_rad);
        chassis kin(wheel current_status);
    private:
        float wid;
};