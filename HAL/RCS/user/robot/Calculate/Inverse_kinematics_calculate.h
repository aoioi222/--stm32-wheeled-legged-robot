/**
 ******************************************************************************
 * @file    Inverse_kinematics_calculate.h
 * @brief   四足机器人单腿逆运动学头文件
 *          提供机体坐标系足端位置 -> 三个关节角度(舵机)的IK求解接口
 ******************************************************************************
 */

#ifndef INVERSE_KINEMATICS_CALCULATE_H
#define INVERSE_KINEMATICS_CALCULATE_H

#include "Foot_structure.h"
#include "robot_math.h"

/* 坐标系定义
            /\ z轴向上
            |
            | / y轴向左
            |/                  机头
            +-----> x轴向前
            O(机体几何中心)

左后腿(LEG_L_B)3                左前腿(LEG_L_F)1

右后腿(LEG_R_B)2                右前腿(LEG_R_F)0
*/

// 腿编号定义：F=前腿，B=后腿，R=右腿，L=左腿

// 数学极小值EPSILON统一放置在robot_config.h

// 空指针判断宏
#define IS_NULL_PTR(ptr) ((ptr) == NULL)

/**
 * @brief 逆运动学求解状态枚举
 */
typedef enum
{
    IK_SUCCESS = 0,             // 求解成功，输出关节角度有效
    IK_ERROR_INVALID_PARAM = 1, // 非法入参：空指针/腿ID超出范围
    IK_ERROR_JOINT1_LIMIT = 2,  // Joint1(髋侧摆关节)角度超出机械限位
    IK_ERROR_JOINT2_LIMIT = 3,  // Joint2(髋俯仰关节)角度超出机械限位
    IK_ERROR_JOINT3_LIMIT = 4,  // Joint3(膝关节)角度超出机械限位
    IK_ERROR_WORKSPACE = 5      // 目标足端位置超出机器人可达工作空间
} IK_Status_t;

/************************ 对外API接口声明 ************************/

/**
 * @brief 单腿完整逆运动学求解
 * @param  leg_id             腿编号，LEG_R_F / LEG_L_F / LEG_R_B / LEG_L_B
 * @param  body_foot_pos      输入：机体坐标系下目标足端位置，单位 m
 * @param  out_joint_angles   输出：求解得到的3个关节角度，单位 °，含有效性标志位
 * @retval IK_Status_t        IK求解状态码
 * @note  该函数内部不做姿态旋转变换，输入直接使用机体坐标系坐标
 *        求解顺序 Joint1 -> Joint3 -> Joint2，依次校验可达工作空间
 */
IK_Status_t foot_ik_solve_all(Leg_ID_t leg_id, const FootPosition_t *body_foot_pos,
                              LegJointAngles_t *out_joint_angles);

#endif // INVERSE_KINEMATICS_CALCULATE_H
