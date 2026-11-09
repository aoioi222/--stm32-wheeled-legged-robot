#include <cmath>
#include <cstdint>

#include "rcs_types.h"
#include "rcs_os.h"
#include "angle_loop.h"

#include "kin_chassis.h"

/**
 * @brief 四舵轮底盘计算器的构造函数
 * 
 * @param chassis_length 长边相邻两个舵轮的距离，单位mm
 * @param chassis_width  短边相邻两个舵轮的距离，单位mm
 * @param bias_x 重心距离几何中心的x轴偏移量，单位mm
 * @param bias_y 重心距离几何中心的y轴偏移量，单位mm
 * 
 * @note 底盘正方向详见.h文件中rcs_agv4类的声明
 * @warning 暂未实现重心偏移，如有需要，可以参考老代码中的3v3函数
 */
rcs_agv4::rcs_agv4(float chassis_length, float chassis_width,float bias_x, float bias_y)
{
    this->len = chassis_length;
    this->wid = chassis_width;
    this->bias_x = bias_x;
    this->bias_y = bias_y;
}

/**
 * @brief 四舵轮底盘计算器的逆运动学函数，给定底盘相对大地坐标系的xyz速度和z轴角度，计算每个舵轮的参数
 * 
 * @param target_spd_x 底盘相对于大地坐标系的x轴速度，单位取决于电机转速
 * @param target_sped_y 底盘相对于大地坐标系的y轴速度，单位取决于电机转速
 * @param target_spd_z  底盘相对于大地坐标系的z轴速度，单位取决于电机转速
 * @param chassis_z_rad 底盘当前相对于大地坐标系的角度，范围(-∞,+∞)，单位rad
 * @return rcs_agv4::wheel 四个舵轮的转速和角度
 * 
 * @note 使用方式：本函数是纯软件的，在获取到转速和角度后，请自行使用电机控制代码下发
 */
rcs_agv4::wheel rcs_agv4::inv_kin(float target_spd_x,float target_spd_y,float target_spd_z,float chassis_z_rad)
{
    rcs_agv4::wheel retval;
    float temp_speed_x;
    float temp_speed_y;

    // 将世界坐标系转为车体坐标系
    float speed_x = target_spd_x*cosf(chassis_z_rad) + target_spd_y*sinf(chassis_z_rad);
    float speed_y = -target_spd_x*sinf(chassis_z_rad) + target_spd_y*cosf(chassis_z_rad);

    // 车轮相对于车体中心的坐标（底盘坐标系）
    float half_length = this->len / 2.0f;
    float half_width  = this->wid  / 2.0f;

    // 车轮0 (左下 RL)
    {
        //获取旋转分量
        float x = -half_length;
        float y = half_width;
        float rot_vx = -target_spd_z * y;
        float rot_vy =  target_spd_z * x;
        //加上平移分量
        temp_speed_x = speed_x + rot_vx;
        temp_speed_y = speed_y + rot_vy;
        //得到最终结果
        retval.speed[0] = sqrtf(temp_speed_x * temp_speed_x + temp_speed_y * temp_speed_y);
        retval.angle[0] = atan2f(temp_speed_y, temp_speed_x);
    }

    // 车轮1 (右下 RR)
    {
        //获取旋转分量
        float x = -half_length;
        float y = -half_width;
        float rot_vx = -target_spd_z * y;
        float rot_vy =  target_spd_z * x;
        //加上平移分量
        temp_speed_x = speed_x + rot_vx;
        temp_speed_y = speed_y + rot_vy;
        //得到最终结果
        retval.speed[1] = sqrtf(temp_speed_x * temp_speed_x + temp_speed_y * temp_speed_y);
        retval.angle[1] = atan2f(temp_speed_y, temp_speed_x);
    }

    // 车轮2 (右上 FR)
    {
        float x = half_length;
        float y = -half_width;
        float rot_vx = -target_spd_z * y;
        float rot_vy =  target_spd_z * x;
        temp_speed_x = speed_x + rot_vx;
        temp_speed_y = speed_y + rot_vy;
        retval.speed[2] = sqrtf(temp_speed_x * temp_speed_x + temp_speed_y * temp_speed_y);
        retval.angle[2] = atan2f(temp_speed_y, temp_speed_x);
    }

    // 车轮3 (左上 FL)
    {
        float x = half_length;
        float y = half_width;
        float rot_vx = -target_spd_z * y;
        float rot_vy =  target_spd_z * x;
        temp_speed_x = speed_x + rot_vx;
        temp_speed_y = speed_y + rot_vy;
        retval.speed[3] = sqrtf(temp_speed_x * temp_speed_x + temp_speed_y * temp_speed_y);
        retval.angle[3] = atan2f(temp_speed_y, temp_speed_x);
    }
    
    return retval;
}

/**
 * @brief 四舵轮底盘计算器的正运动学函数，给定四个舵轮的参数，计算底盘相对大地坐标系的xyz速度和z轴角度
 * 
 * @param status 四个舵轮的转速和角度，类型为rcs_agv4::wheel
 * @return rcs_agv4::chassis 
 * 
 * @note 使用方式：可以用于估计底盘在某个时刻的速度。由于本函数是纯软件的，status需要自行使用电机监测代码获取
 */
rcs_agv4::chassis rcs_agv4::kin(rcs_agv4::wheel status)
{
    // 没有此需求，暂未实现
    rcs_agv4::chassis retval;
    retval.x = 0;
	retval.y = 0;
	retval.z = 0;
    return retval;
}

/**
 * @brief 舵轮单元角度回环
 * 
 * @param target 目标角度，获取自inv_kin函数
 * @param current 当前角度，获取自电机监测代码
 * 
 * @retval rcs_agv4::wheel 通过回环得到的最短移动路线
 * 
 * @warning 注意，本函数只对角度进行回环，不对速度不进行回环。因此速度一定要是正数。
 *          真实速度有可能是负数，在此情况下，请利用负数减速比，将其扭回正数。
 * 
 * @warning 如果直接把inv_kin得到的值下发给电机，是不能让底盘边走边转的。
 *          因为当底盘在360度和180度附近时，会挑选距离更远的路线，使得轮子擦地卡死。
 *          因此正确的做法是，先用inv_kin得到理论目标值，再使用find_nearest得到最短路线，最后将最短路线下发给电机。
 */
rcs_agv4::wheel rcs_agv4::find_nearest(rcs_agv4::wheel target,rcs_agv4::wheel current)
{
    rcs_agv4::wheel retval = target;
    // 参数是否合规
    if (current.speed[0]<ZERO || current.speed[1]<ZERO || current.speed[2]<ZERO && current.speed[3]<ZERO) {
        rcs_kprintf("agv:speed invalid\n",portMAX_DELAY);
    }
    // 先将 位于(-∞,+∞)的目标值 规范化到 [-pi,pi)
    for(uint8_t i=0;i<4;i++) {
        retval.angle[i] = angle_loop::regular_from_inf_to_0(retval.angle[i]);
    }
    // 将规范化目标值 实现到 临近当前值的一个最近值，也即求解最短路
    for(uint8_t i=0;i<4;i++) {
        retval.angle[i] = angle_loop::normalize_from_0_to_inf(retval.angle[i],current.angle[i]);
    }
    // 得到最短路
    return retval;
}

rcs_omni4::rcs_omni4(float chassis_width)
{
    this->wid = chassis_width;
}

rcs_omni4::wheel rcs_omni4::inv_kin(float target_spd_x,float target_spd_y,float target_spd_z,float chassis_z_rad)
{
	rcs_omni4::wheel retval;
    // 将世界坐标系转为车体坐标系
    float chassis_spd_x = target_spd_x*cosf(chassis_z_rad) + target_spd_y*sinf(chassis_z_rad);
    float chassis_spd_y = -target_spd_x*sinf(chassis_z_rad) + target_spd_y*cosf(chassis_z_rad);
	return retval;
}