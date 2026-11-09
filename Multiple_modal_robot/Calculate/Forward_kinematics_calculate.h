#ifndef FORWARD_KINEMATICS_CALCULATE_H
#define FORWARD_KINEMATICS_CALCULATE_H

#include "rcs.h"
#include "Foot_structure.h"

/*机体坐标系            /\z【上】
                        |
                        |  /y【左】
                        | /                      狗头
                        |/____>x【前】
                       O(机体中心)    
       左后腿(LEG_L_B)              左前腿(LEG_L_F)·
              
   右后腿(LEG_R_B)             右前腿(LEG_R_F)·           

*/


// ====================== 函数和数据结构定义 ======================

// 正运动学计算返回状态枚举（补充关节超限位的细分状态）
typedef enum {
    FK_SUCCESS = 0,                  // 计算成功
    FK_ERROR_HIP_ABD_LIMIT,      // 髋展关节超限位
    FK_ERROR_HIP_ROT_LIMIT,      // 髋转关节超限位
    FK_ERROR_KNEE_LIMIT,          // 膝关节超限位
    FK_ERROR_INVALID_PARAM       // 无效参数
} FK_Status_t;

// 定义4x4矩阵类型
typedef float Matrix4x4[4][4];


/**
 * @brief 单腿正运动学几何解算（纯计算，无内部状态）
 * @param[in] leg_id 腿编号（LEG_R_F/LEG_L_F/LEG_R_B/LEG_L_B）
 * @param[in] joint_angles ：该腿的关节角度结构体（单位：度）
 * @param[out] out_foot_pos：指向该腿足端位置的指针（解算层仅修改该指针指向的内容，单位：米）
 * @return FK_Status_t 解算状态枚举：
 */
FK_Status_t leg_forward_kinematics_matrix(Leg_ID_t leg_id, LegJointAngles_t joint_angles,
                                          FootPosition_t *out_foot_pos);


#endif // FORWARD_KINEMATICS_CALCULATE_H
