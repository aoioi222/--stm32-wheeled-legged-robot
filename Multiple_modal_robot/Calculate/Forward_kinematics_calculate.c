/******************************************************************************
 * @file    Forward_kinematics_calculate.c
 * @brief   四足机器人腿部正运动学计算工具
 * @version V3.0
 * @date    2026-02-18
 ******************************************************************************
 * @attention
 *
 * 1. LEG_ID枚举默认按"右前-左前-右后-左后"顺序定义，需与硬件腿序一一对应
 * 3. 角度单位统一为度(°)，坐标单位统一为米(m)，计算前自动完成角度转弧度
 * 5. 正解计算需确保关节角度在限位范围内，超出时返回错误码并终止计算
 * 6.髋展关节（joint1）旋转轴为前后，髋转关节（joint2）旋转轴为左右，膝关节（joint3）旋转轴为左右
 * 7.定义从关节的旋转轴方向的外侧看过去，关节顺时针旋转为正角度，逆时针旋转为负角度
 ******************************************************************************
 */

#include "Forward_kinematics_calculate.h"

// 核心函数：纯几何解析法
FK_Status_t leg_forward_kinematics_matrix(Leg_ID_t leg_id, LegJointAngles_t joint_angles, FootPosition_t *out_foot_pos)
{

    // 参数合法性校验
    if (leg_id < LEG_R_F || leg_id > LEG_L_B || out_foot_pos == NULL)
    {
        return FK_ERROR_INVALID_PARAM;
    }

    // 角度限位校验
    if (joint_angles.joint1_deg < JOINT1_MIN ||
        joint_angles.joint1_deg > JOINT1_MAX)
    {
        return FK_ERROR_HIP_ABD_LIMIT;
    }
    if (joint_angles.joint2_deg < JOINT2_MIN ||
        joint_angles.joint2_deg > JOINT2_MAX)
    {
        return FK_ERROR_HIP_ROT_LIMIT;
    }
    if (joint_angles.joint3_deg < JOINT3_MIN ||
        joint_angles.joint3_deg > JOINT3_MAX)
    {
        return FK_ERROR_KNEE_LIMIT;
    }
    // 右腿髋展角对称修正
    float hip_abd_deg = joint_angles.joint1_deg;
    switch (leg_id)
    {
    case LEG_R_F: // 右前腿
                  // 无需变换
        break;
    case LEG_L_F: // 左前腿
        hip_abd_deg = -hip_abd_deg;
        break;
    case LEG_R_B: // 右后腿
        break;
        // 无需变换
        break;
    case LEG_L_B: // 左后腿
        hip_abd_deg = -hip_abd_deg;
        break;
    }

    // 右前左后为反
    //  角度转弧度【根据硬件取正负】
    float theta1_rad = DEG_TO_RAD(hip_abd_deg);              // 髋展角（绕机体X轴）
    float theta2_rad = DEG_TO_RAD(joint_angles.joint2_deg);  // 髋转角
    float theta3_rad = -DEG_TO_RAD(joint_angles.joint3_deg); // 膝转角

    float local_x = 0.0f, local_y = 0.0f, local_z = 0.0f;

    float proj_len = THIGH_LENGTH * cosf(theta2_rad) + CALF_LENGTH * cosf(theta2_rad + theta3_rad);
    // 计算腿部在Y-Z平面上的投影长度
    local_x = THIGH_LENGTH * sinf(theta2_rad) + CALF_LENGTH * sinf(theta2_rad + theta3_rad); // X轴（前+）
    local_y = HIP_ABD_LENGTH * cosf(theta1_rad) + sinf(theta1_rad) * proj_len;               // Y轴（左+）
    local_z = -(proj_len * cosf(theta1_rad) + HIP_ABD_LENGTH * sinf(theta1_rad));            // Z轴（上+）

    // 叠加全局偏移
    switch (leg_id)
    {
    case LEG_R_F:
        out_foot_pos->foot_pos_x_m = HIP_OFFSET_X_R_F + local_x;
        out_foot_pos->foot_pos_y_m = HIP_OFFSET_Y_R_F - local_y;
        out_foot_pos->foot_pos_z_m = HIP_OFFSET_Z_R_F + local_z;
        break;
    case LEG_L_F:
        out_foot_pos->foot_pos_x_m = HIP_OFFSET_X_L_F + local_x;
        out_foot_pos->foot_pos_y_m = HIP_OFFSET_Y_L_F + local_y;
        out_foot_pos->foot_pos_z_m = HIP_OFFSET_Z_L_F + local_z;
        break;
    case LEG_R_B:
        out_foot_pos->foot_pos_x_m = HIP_OFFSET_X_R_B + local_x;
        out_foot_pos->foot_pos_y_m = HIP_OFFSET_Y_R_B - local_y;
        out_foot_pos->foot_pos_z_m = HIP_OFFSET_Z_R_B + local_z;
        break;
    case LEG_L_B:
        out_foot_pos->foot_pos_x_m = HIP_OFFSET_X_L_B + local_x;
        out_foot_pos->foot_pos_y_m = HIP_OFFSET_Y_L_B + local_y;
        out_foot_pos->foot_pos_z_m = HIP_OFFSET_Z_L_B + local_z;
        break;
    default:
        break;
    }
    return FK_SUCCESS;
}