#include "Jacobian_matrix.h"

/* ----------------------------------------------------------------
   启用开关：本文件从 check_joint_limit 到 leg_ik_jacobian_solve 的内容
   只有在 JACOBIAN_IK_ENABLE 非 0 时才编译。
   默认 0 —— 实际解算一律走几何法 foot_ik_solve_all()，这里保留的是
   雅可比矩阵、伪逆、速度/力/加速度映射与迭代IK求解，当前没有任何调用者，
   但会被链接进镜像占约 1.5KB Flash。
   需要恢复时：把 Jacobian_matrix.h 里的 JACOBIAN_IK_ENABLE 改为 1，
   并把 leg_ik_solve() 里的 IK_MODE_JACOBIAN / IK_MODE_AUTO 分支接回去。
   ---------------------------------------------------------------- */
#if JACOBIAN_IK_ENABLE

/************************* 轻量化工具函数 *************************/

static void check_joint_limit(LegJointAngles_t *joint_angles)
{
    if (joint_angles == NULL)
        return;

    if (joint_angles->joint1_deg < JOINT1_MIN)
        joint_angles->joint1_deg = JOINT1_MIN;
    if (joint_angles->joint1_deg > JOINT1_MAX)
        joint_angles->joint1_deg = JOINT1_MAX;
    if (joint_angles->joint2_deg < JOINT2_MIN)
        joint_angles->joint2_deg = JOINT2_MIN;
    if (joint_angles->joint2_deg > JOINT2_MAX)
        joint_angles->joint2_deg = JOINT2_MAX;
    if (joint_angles->joint3_deg < JOINT3_MIN)
        joint_angles->joint3_deg = JOINT3_MIN;
    if (joint_angles->joint3_deg > JOINT3_MAX)
        joint_angles->joint3_deg = JOINT3_MAX;
}

static void get_hip_offset(Leg_ID_t leg_id, float *offset_x, float *offset_y, float *offset_z)
{
    if (offset_x == NULL || offset_y == NULL || offset_z == NULL)
        return;
    switch (leg_id)
    {
    case LEG_R_F:
        *offset_x = HIP_OFFSET_X_R_F;
        *offset_y = HIP_OFFSET_Y_R_F;
        *offset_z = HIP_OFFSET_Z_R_F;
        break;
    case LEG_L_F:
        *offset_x = HIP_OFFSET_X_L_F;
        *offset_y = HIP_OFFSET_Y_L_F;
        *offset_z = HIP_OFFSET_Z_L_F;
        break;
    case LEG_R_B:
        *offset_x = HIP_OFFSET_X_R_B;
        *offset_y = HIP_OFFSET_Y_R_B;
        *offset_z = HIP_OFFSET_Z_R_B;
        break;
    case LEG_L_B:
        *offset_x = HIP_OFFSET_X_L_B;
        *offset_y = HIP_OFFSET_Y_L_B;
        *offset_z = HIP_OFFSET_Z_L_B;
        break;
    default:
        *offset_x = 0.0f;
        *offset_y = 0.0f;
        *offset_z = 0.0f;
        break;
    }
}

/************************* 轻量化伪逆 *************************/
void matrix_pinv_lightweight(JacobianMat_t *J, float J_pinv[3][3])
{
    if (J == NULL || J_pinv == NULL)
        return;
    memset(J_pinv, 0, sizeof(float) * 3 * 3);

    // 取出雅可比里真正存在的元素
    float J01 = J->data[0][1]; // vx对joint2的偏导
    float J02 = J->data[0][2]; // vx对joint3的偏导
    float J10 = J->data[1][0]; // vy对joint1的偏导
    float J20 = J->data[2][0]; // vz对joint1的偏导
    float J21 = J->data[2][1]; // vz对joint2的偏导
    float J22 = J->data[2][2]; // vz对joint3的偏导

    const float lam = 0.01f; // 阻尼
    const float lam2 = lam * lam;

    // --------------------------
    // 最简 J.T * J + λI
    // --------------------------
    // JTJ的非零项（对称矩阵，只算上三角）
    float a = J10 * J10 + J20 * J20 + lam2; // JTJ[0][0]
    float b = J20 * J21;                    // JTJ[0][1]
    float c = J20 * J22;                    // JTJ[0][2]
    float d = J01 * J01 + J21 * J21 + lam2; // JTJ[1][1]
    float e = J01 * J02 + J21 * J22;        // JTJ[1][2]
    float f = J02 * J02 + J22 * J22 + lam2; // JTJ[2][2]

    // 行列式
    float det = a * (d * f - e * e) - b * (b * f - c * e) + c * (b * e - d * c);

    if (fabs(det) < JACOBIAN_EPSILON)
    {
        // 奇异保护，简单对角返回
        J_pinv[0][1] = J10 / (J10 * J10 + lam2);
        J_pinv[0][2] = J20 / (J20 * J20 + lam2);
        J_pinv[1][0] = J01 / (J01 * J01 + lam2);
        J_pinv[1][2] = J21 / (J21 * J21 + lam2);
        J_pinv[2][0] = J02 / (J02 * J02 + lam2);
        J_pinv[2][2] = J22 / (J22 * J22 + lam2);
        return;
    }

    float inv_det = 1.0f / det;

    // --------------------------
    // inv(JTJ) 伴随矩阵
    // --------------------------
    float adj00 = (d * f - e * e); // 余子式C00
    float adj01 = (c * e - b * f); // 余子式C01（= C10）
    float adj02 = (b * e - d * c); // 余子式C02（= C20）
    float adj11 = (a * f - c * c); // 余子式C11
    float adj12 = (b * c - a * e); // 余子式C12（= C21）
    float adj22 = (a * d - b * b); // 余子式C22

    // 计算伪逆 J^+ = inv(JTJ) · J.T（仅非零项）

    // joint1（行0）：仅响应vy、vz
    J_pinv[0][1] = adj00 * J10 * inv_det;                               // J^+[0][1] = (adj00*J10)/det
    J_pinv[0][2] = (adj00 * J20 + adj01 * J21 + adj02 * J22) * inv_det; // J^+[0][2] = (adj00*J20 + adj01*J21 + adj02*J22)/det

    // joint2（行1）：仅响应vx、vz
    J_pinv[1][0] = (adj01 * J10 + adj11 * J01 + adj12 * J02) * inv_det; // J^+[1][0] = (adj01*J10 + adj11*J01 + adj12*J02)/det
    J_pinv[1][2] = (adj01 * J20 + adj11 * J21 + adj12 * J22) * inv_det; // J^+[1][2] = (adj01*J20 + adj11*J21 + adj12*J22)/det

    // joint3（行2）：仅响应vx、vz
    J_pinv[2][0] = (adj02 * J10 + adj12 * J01 + adj22 * J02) * inv_det; // J^+[2][0] = (adj02*J10 + adj12*J01 + adj22*J02)/det
    J_pinv[2][2] = (adj02 * J20 + adj12 * J21 + adj22 * J22) * inv_det; // J^+[2][2] = (adj02*J20 + adj12*J21 + adj22*J22)/det
}

void leg_jacobian_calc(Leg_ID_t leg_id, const LegJointAngles_t *joint_angles, JacobianMat_t *J)
{
    if (J == NULL || joint_angles == NULL)
        return;
    memset(J->data, 0, sizeof(float) * 3 * 3);

    // 髋展角右腿取反
    float hip_abd_deg = joint_angles->joint1_deg;
    if (leg_id == LEG_R_F || leg_id == LEG_R_B)
        hip_abd_deg = -hip_abd_deg;

    float theta1_rad = DEG_TO_RAD(hip_abd_deg);              // 髋展角
    float theta2_rad = DEG_TO_RAD(joint_angles->joint2_deg); // 髋转角
    float theta3_rad = DEG_TO_RAD(joint_angles->joint3_deg); // 膝转角
    float theta23_rad = theta2_rad + theta3_rad;

    // 预计算三角函数
    float sin_theta1 = sinf(theta1_rad), cos_theta1 = cosf(theta1_rad);
    float sin_theta2 = sinf(theta2_rad), cos_theta2 = cosf(theta2_rad);
    float sin_theta23 = sinf(theta23_rad), cos_theta23 = cosf(theta23_rad);

    // 计算Y-Z平面投影长度
    float proj_len = THIGH_LENGTH * cos_theta2 + CALF_LENGTH * cos_theta23;

    // 仅计算非零元素（无leg_dir，完全适配正运动学）
    //  行0（vx：前后）：仅列1、列2非零
    J->data[0][1] = THIGH_LENGTH * cos_theta2 + CALF_LENGTH * cos_theta23;
    J->data[0][2] = CALF_LENGTH * cos_theta23;

    // 行1（vy：侧向）：仅列0非零
    J->data[1][0] = -HIP_ABD_LENGTH * sin_theta1 - proj_len * cos_theta1;

    // 行2（vz：垂直）：列0、列1、列2非零
    J->data[2][0] = -HIP_ABD_LENGTH * cos_theta1 + proj_len * sin_theta1;
    J->data[2][1] = -THIGH_LENGTH * sin_theta2 - CALF_LENGTH * sin_theta23;
    J->data[2][2] = -CALF_LENGTH * sin_theta23;
}

/************************* 速度/力/加速度映射 *************************/
void jacobian_vel_forward_map(const JacobianMat_t *J, const float joint_vel_deg[3], FootVelocity_t *foot_vel)
{
    if (J == NULL || joint_vel_deg == NULL || foot_vel == NULL)
        return;
    memset(foot_vel, 0, sizeof(FootVelocity_t));

    // 仅转换需要的关节角速度（弧度）
    float j1_rad = DEG_TO_RAD(joint_vel_deg[0]);
    float j2_rad = DEG_TO_RAD(joint_vel_deg[1]);
    float j3_rad = DEG_TO_RAD(joint_vel_deg[2]);

    // 仅非零项计算（无多余乘法）
    foot_vel->foot_vel_x_mps = J->data[0][1] * j2_rad + J->data[0][2] * j3_rad;
    foot_vel->foot_vel_y_mps = J->data[1][0] * j1_rad;
    foot_vel->foot_vel_z_mps = J->data[2][0] * j1_rad + J->data[2][1] * j2_rad + J->data[2][2] * j3_rad;
}

void jacobian_vel_inverse_map(Leg_ID_t leg_id, const LegJointAngles_t *joint_angles, const FootVelocity_t *target_foot_vel, float joint_vel_deg[3])
{
    if (joint_angles == NULL || target_foot_vel == NULL || joint_vel_deg == NULL)
        return;
    memset(joint_vel_deg, 0, sizeof(float) * 3);

    // 计算当前姿态的雅各比矩阵
    JacobianMat_t J;
    leg_jacobian_calc(leg_id, joint_angles, &J);

    // 计算雅各比伪逆
    float J_pinv[3][3];
    matrix_pinv_lightweight(&J, J_pinv);

    // 提取足端目标速度（m/s）
    float vx = target_foot_vel->foot_vel_x_mps;
    float vy = target_foot_vel->foot_vel_y_mps;
    float vz = target_foot_vel->foot_vel_z_mps;

    // 核心：J^+ · 足端速度 → 关节角速度（弧度/秒）
    float j1_rad = J_pinv[0][1] * vy + J_pinv[0][2] * vz; // 关节1仅受y/z影响
    float j2_rad = J_pinv[1][0] * vx + J_pinv[1][2] * vz; // 关节2仅受x/z影响
    float j3_rad = J_pinv[2][0] * vx + J_pinv[2][2] * vz; // 关节3仅受x/z影响

    // 弧度/秒 → 角度/秒（适配电机控制）
    joint_vel_deg[0] = RAD_TO_DEG(j1_rad);
    joint_vel_deg[1] = RAD_TO_DEG(j2_rad);
    joint_vel_deg[2] = RAD_TO_DEG(j3_rad);
}

void jacobian_force_map(const JacobianMat_t *J, const float foot_force[3], float joint_torque[3])
{
    if (J == NULL || foot_force == NULL || joint_torque == NULL)
        return;
    memset(joint_torque, 0, sizeof(float) * 3);

    // 仅计算非零元素乘法（力映射=雅各比转置×力）
    joint_torque[0] = J->data[1][0] * foot_force[1] + J->data[2][0] * foot_force[2];
    joint_torque[1] = J->data[0][1] * foot_force[0] + J->data[2][1] * foot_force[2];
    joint_torque[2] = J->data[0][2] * foot_force[0] + J->data[2][2] * foot_force[2];
}

void jacobian_acc_forward_simple(const JacobianMat_t *J, const float joint_acc_deg[3], FootAcceleration_t *foot_acc)
{
    if (J == NULL || joint_acc_deg == NULL || foot_acc == NULL)
        return;
    memset(foot_acc, 0, sizeof(FootAcceleration_t));

    // 弧度转换
    float ddq1 = DEG_TO_RAD(joint_acc_deg[0]);
    float ddq2 = DEG_TO_RAD(joint_acc_deg[1]);
    float ddq3 = DEG_TO_RAD(joint_acc_deg[2]);

    // 直接把加速度当速度乘J
    foot_acc->foot_acc_x_mps2 = J->data[0][1] * ddq2 + J->data[0][2] * ddq3;
    foot_acc->foot_acc_y_mps2 = J->data[1][0] * ddq1;
    foot_acc->foot_acc_z_mps2 = J->data[2][0] * ddq1 + J->data[2][1] * ddq2 + J->data[2][2] * ddq3;
}

void jacobian_acc_inverse_simple(Leg_ID_t leg_id, const LegJointAngles_t *joint_angles, const FootAcceleration_t *target_acc, float joint_acc_deg[3])
{
    JacobianMat_t J;
    leg_jacobian_calc(leg_id, joint_angles, &J);

    float J_pinv[3][3];
    matrix_pinv_lightweight(&J, J_pinv);

    float ax = target_acc->foot_acc_x_mps2;
    float ay = target_acc->foot_acc_y_mps2;
    float az = target_acc->foot_acc_z_mps2;

    // 直接用伪逆映射加速度
    float ddq1 = J_pinv[0][1] * ay + J_pinv[0][2] * az;
    float ddq2 = J_pinv[1][0] * ax + J_pinv[1][2] * az;
    float ddq3 = J_pinv[2][0] * ax + J_pinv[2][2] * az;

    joint_acc_deg[0] = RAD_TO_DEG(ddq1);
    joint_acc_deg[1] = RAD_TO_DEG(ddq2);
    joint_acc_deg[2] = RAD_TO_DEG(ddq3);
}

/************************* 奇异检测 *************************/
JacobianSingularStatus_t jacobian_singularity_check(const JacobianMat_t *J)
{
    if (J == NULL)
        return JACOBIAN_SINGULAR_FULL;

    // 仅用非零项计算行列式（匹配雅可比结构）
    float det = -J->data[0][1] * (J->data[1][0] * J->data[2][2]) + J->data[0][2] * (J->data[1][0] * J->data[2][1]);
    det = fabsf(det);

    if (det < JACOBIAN_SINGULAR_FULL_VAL)
    {
        return JACOBIAN_SINGULAR_FULL;
    }
    else if (det < JACOBIAN_SINGULAR_NEAR_VAL)
    {
        return JACOBIAN_SINGULAR_NEAR;
    }
    else
    {
        return JACOBIAN_SINGULAR_NONE;
    }
}

/************************* 逆解求解 *************************/
IKJacobianError_t leg_ik_jacobian_solve(Leg_ID_t leg_id, const FootPosition_t *target_pos,
                                        const LegJointAngles_t *init_angles, LegJointAngles_t *result_angles)
{
    if (target_pos == NULL || init_angles == NULL || result_angles == NULL ||
        leg_id < LEG_R_F || leg_id > LEG_L_B)
    {
        result_angles->valid = 0;
        return IK_JACOBIAN_ERR_INVALID_PARAM;
    }

    memcpy(result_angles, init_angles, sizeof(LegJointAngles_t));
    result_angles->valid = 0;
    float offset_x, offset_y, offset_z;
    get_hip_offset(leg_id, &offset_x, &offset_y, &offset_z);

    // 目标位置补偿髋偏移
    float tar_x = target_pos->foot_pos_x_m - offset_x;
    float tar_y = target_pos->foot_pos_y_m - offset_y;
    float tar_z = target_pos->foot_pos_z_m - offset_z;

    // 迭代求解
    for (int iter = 0; iter < IK_JACOBIAN_ITER_MAX; iter++)
    {
        // 正运动学计算当前足端位置
        FootPosition_t current_pos;
        FK_Status_t fk_status = leg_forward_kinematics_matrix(leg_id, *result_angles, &current_pos);
        if (fk_status != FK_SUCCESS)
        {
            result_angles->valid = 0;
            return IK_JACOBIAN_ERR_FK_FAILED;
        }

        // 当前位置补偿髋偏移
        float cur_x = current_pos.foot_pos_x_m - offset_x;
        float cur_y = current_pos.foot_pos_y_m - offset_y;
        float cur_z = current_pos.foot_pos_z_m - offset_z;

        // 位置误差（仅计算必要项）
        float dx = tar_x - cur_x;
        float dy = tar_y - cur_y;
        float dz = tar_z - cur_z;
        float err_sq = dx * dx + dy * dy + dz * dz;

        // 误差达标则退出
        if (err_sq < IK_JACOBIAN_POS_TOL * IK_JACOBIAN_POS_TOL)
        {
            result_angles->valid = 1;
            return IK_JACOBIAN_SUCCESS;
        }

        // 计算雅各比 + 奇异检测
        JacobianMat_t J;
        leg_jacobian_calc(leg_id, result_angles, &J);
        if (jacobian_singularity_check(&J) == JACOBIAN_SINGULAR_FULL)
        {
            result_angles->valid = 0;
            return IK_JACOBIAN_ERR_SINGULAR;
        }

        // 计算伪逆
        float J_pinv[3][3];
        matrix_pinv_lightweight(&J, J_pinv);

        // 动态步长（仅必要判断）
        float step = (err_sq > IK_JACOBIAN_ERROR_THRESH * IK_JACOBIAN_ERROR_THRESH)
                         ? IK_JACOBIAN_STEP_LARGE
                         : IK_JACOBIAN_STEP_SMALL;

        // 关节增量计算（仅非零项，弧度转角度）
        float dq1 = RAD_TO_DEG(J_pinv[0][1] * dy + J_pinv[0][2] * dz) * step;
        float dq2 = RAD_TO_DEG(J_pinv[1][0] * dx + J_pinv[1][2] * dz) * step;
        float dq3 = RAD_TO_DEG(J_pinv[2][0] * dx + J_pinv[2][2] * dz) * step;

        // 更新关节角度 + 限位
        result_angles->joint1_deg += dq1;
        result_angles->joint2_deg += dq2;
        result_angles->joint3_deg += dq3;
        check_joint_limit(result_angles);
    }
    return IK_JACOBIAN_ERR_ITER_LIMIT;
}

#endif /* JACOBIAN_IK_ENABLE */


/**
 * @brief  几何法求解状态码转雅可比错误码
 * @param  status foot_ik_solve_all 返回的 IK_Status_t
 * @retval 对应的 IKJacobianError_t
 * @note   两套枚举的数值并不一致(几何法里 IK_ERROR_JOINT1_LIMIT=2、IK_ERROR_JOINT3_LIMIT=4)，
 *         直接赋值会让关节超限被当成"雅可比奇异"或"参数非法"，故障定位失真，故显式转换。
 *         关节触限与工作空间越界归为奇异，其余求解失败归为迭代未收敛。
 */
static IKJacobianError_t ik_status_to_jacobian_err(IK_Status_t status)
{
    switch (status)
    {
    case IK_SUCCESS:
        return IK_JACOBIAN_SUCCESS;
    case IK_ERROR_JOINT1_LIMIT:
    case IK_ERROR_JOINT2_LIMIT:
    case IK_ERROR_JOINT3_LIMIT:
    case IK_ERROR_WORKSPACE:
        return IK_JACOBIAN_ERR_SINGULAR;
    case IK_ERROR_INVALID_PARAM:
        return IK_JACOBIAN_ERR_INVALID_PARAM;
    default:
        return IK_JACOBIAN_ERR_ITER_LIMIT;
    }
}

/**
 * @brief  IK统一入口：几何法解算 + 状态码转换
 * @param  leg_id        腿编号
 * @param  target_pos    目标足端位置（机体坐标系）
 * @param  result_angles 输出关节角
 * @param  mode          仅支持 IK_MODE_GEOMETRY，其余模式返回 IK_JACOBIAN_ERR_INVALID_MODE
 * @retval IKJacobianError_t 求解状态
 * @note   雅可比迭代链路已由 JACOBIAN_IK_ENABLE 停用（见本文件开头说明），
 *         因此 IK_MODE_JACOBIAN / IK_MODE_AUTO 不再可用。
 */
IKJacobianError_t leg_ik_solve(Leg_ID_t leg_id, const FootPosition_t *target_pos, LegJointAngles_t *result_angles, IKMode_t mode)
{
    if (target_pos == NULL || result_angles == NULL || leg_id < LEG_R_F || leg_id > LEG_L_B)
    {
        return IK_JACOBIAN_ERR_INVALID_PARAM;
    }

    IK_Status_t geo_status;
    switch (mode)
    {
    case IK_MODE_GEOMETRY:
        // 几何法无迭代过程，初值不参与求解，直接转换状态码返回
        geo_status = foot_ik_solve_all(leg_id, target_pos, result_angles);
        return ik_status_to_jacobian_err(geo_status);

#if JACOBIAN_IK_ENABLE
    case IK_MODE_JACOBIAN:
    case IK_MODE_AUTO:
        /* 迭代IK已停用：若恢复 Jacobian_matrix.h 的 JACOBIAN_IK_ENABLE，
           需要在这里把 prev_angles 缓存与 leg_ik_jacobian_solve() 的降级逻辑接回来 */
        return IK_JACOBIAN_ERR_INVALID_MODE;
#endif

    default:
        return IK_JACOBIAN_ERR_INVALID_MODE; // 无效模式
    }
}