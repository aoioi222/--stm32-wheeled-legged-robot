#include "Inverse_kinematics_calculate.h"

/* IK 错误打印：错误分支触发，但异常时会在控制周期内连续输出，默认关闭 */
#if ROBOT_DEBUG_PRINT
#define LOG_IK(...) printf(__VA_ARGS__)
#else
#define LOG_IK(...) ((void)0)
#endif

/******************************************************************************************
 * 单腿逆运动学解算实现
 ******************************************************************************************/

/*
 * @brief 弧度归一化，最终映射至 [-π/2, π/2] 区间
 * @param rad 输入弧度值
 * @return 归一化后的弧度值
 */
static float rad_normalize(float rad)
{
    // 第一步：将弧度归一化到 [-π, π] 主值区间
    rad = fmodf(rad, 2.0f * pi);
    if (rad > pi)
        rad -= 2.0f * pi;
    if (rad < -pi)
        rad += 2.0f * pi;

    // 第二步：折叠到 sin/cos 等效的 [-π/2, π/2] 区间
    if (rad > pi / 2)
    {
        // 大于90度，利用余弦对称性折叠到第一象限
        rad = pi - rad;
    }
    else if (rad < -pi / 2)
    {
        // 小于-90度，利用余弦对称性折叠到第四象限
        rad = -pi - rad;
    }

    return rad;
}

/**
 * @brief float型数值限幅函数，将值约束在[min, max]范围内
 */
static inline float float_clamp(float val, float min, float max)
{
    if (val < min)
        return min;
    if (val > max)
        return max;
    return val;
}

// /*
//  * @brief 确保角度为负值，正值取反
//  */
// static float get_negative_angle(float angle)
// {
//     if (angle > 0.0f)
//         return -angle;
//     else
//         return angle;
// }

// /*
//  * @brief 确保角度为正值，负值取反
//  */
// static float get_positive_angle(float angle)
// {
//     if (angle < 0.0f)
//         return -angle;
//     else
//         return angle;
// }

/**
 * @brief 根据腿ID获取髋关节相对于机体中心的偏移量
 */
static void get_hip_offset(Leg_ID_t leg_id, float *offset_x, float *offset_y, float *offset_z)
{
    if (IS_NULL_PTR(offset_x) || IS_NULL_PTR(offset_y) || IS_NULL_PTR(offset_z))
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
        *offset_x = *offset_y = *offset_z = 0.0f;
        break;
    }
}

/*
 * 单腿逆运动学主解算函数：输入机体坐标系下足端位置，输出三个关节角度
 */
IK_Status_t foot_ik_solve_all(Leg_ID_t leg_id, const FootPosition_t *body_foot_pos, LegJointAngles_t *out_joint_angles)
{
    // 入参合法性校验：空指针/腿ID越界
    if (IS_NULL_PTR(body_foot_pos) || IS_NULL_PTR(out_joint_angles) ||
        leg_id < LEG_R_F || leg_id > LEG_L_B)
    {
        return IK_ERROR_INVALID_PARAM;
    }

    // 初始化输出角度，默认标记为无效
    out_joint_angles->joint1_deg = 0.0f;
    out_joint_angles->joint2_deg = 0.0f;
    out_joint_angles->joint3_deg = 0.0f;
    out_joint_angles->valid = 0;

    // ************************* 第一步：机体坐标系 → 腿部局部坐标系 转换
    float local_x, local_y, local_z;
    float hip_x, hip_y, hip_z;
    get_hip_offset(leg_id, &hip_x, &hip_y, &hip_z);
    local_x = body_foot_pos->foot_pos_x_m - hip_x;
    local_y = body_foot_pos->foot_pos_y_m - hip_y;
    local_z = body_foot_pos->foot_pos_z_m - hip_z;
    // printf("leg_id:%d\n", leg_id);
    //************************* 第二步：求解joint1（髋侧摆关节），几何法双解 + 腿型匹配 + 限位校验

    float r = sqrtf(local_y * local_y + local_z * local_z); // Y-Z平面内足端到原点的距离

    // 奇异点保护：距离过小无法解算
    if (r < EPSILON)
    {
        return IK_ERROR_WORKSPACE;
    }

    // 计算alpha角（三角关系补偿角）
    float cos_comp = float_clamp(HIP_ABD_LENGTH / r, -1.0f, 1.0f);
    float alpha = acosf(cos_comp);
    float phi = atan2f(local_z, -local_y);

    // 计算两个候选解，归一化后转角度
    float theta1_plus_deg = RAD_TO_DEG(rad_normalize(alpha + phi));
    float theta1_minus_deg = RAD_TO_DEG(rad_normalize(-alpha + phi));

    switch (leg_id) // 根据腿的位置选择对应构型的解
    {
    case LEG_L_F: // 左前腿
        out_joint_angles->joint1_deg = -theta1_minus_deg;
        break;
    case LEG_L_B: // 左后腿
        out_joint_angles->joint1_deg = theta1_minus_deg;
        break;
    case LEG_R_F: // 右前腿
        out_joint_angles->joint1_deg = theta1_plus_deg;
        break;
    case LEG_R_B: // 右后腿
        out_joint_angles->joint1_deg = -theta1_plus_deg;
        break;
    }

    /* joint1角度误差补偿校准（实测拟合）
     理论值→实际修正值：
     30°→30.5f°, 25°→27.6f°, 20°→21.7f°, 15°→15.9f°, 10°→10.4f°, 5°→5.1f°, 0°→0°
     -5°→-4.9f°, -10°→-9.6f°, -15°→-14.2f°, -20°→-18.6f°, -25°→-22.8f°
     采用线性拟合修正公式，补偿机械安装与舵机零点偏差
    */

    out_joint_angles->joint1_deg = out_joint_angles->joint1_deg - 0.005f * out_joint_angles->joint1_deg + 0.5f; // 线性补偿修正
    if (out_joint_angles->joint1_deg == 0.5f)
    {
        out_joint_angles->joint1_deg = 0.0f; // 线性补偿修正
    }

    // printf("计算得到joint1为=%.1f°（%.1f~%.1f°）",
    //        out_joint_angles->joint1_deg, JOINT1_MIN, JOINT1_MAX);

    // joint1限位校验，超出则返回错误
    if (out_joint_angles->joint1_deg < JOINT1_MIN)
    {
        out_joint_angles->joint1_deg = JOINT1_MIN;
        return IK_ERROR_JOINT1_LIMIT;
    }
    if (out_joint_angles->joint1_deg > JOINT1_MAX)
    {
        out_joint_angles->joint1_deg = JOINT1_MAX;
        return IK_ERROR_JOINT1_LIMIT;
    }

    //********************* 第三步：求解joint3（膝关节），余弦定理 + 符号匹配 + 限位校验

    float joint1_rad = DEG_TO_RAD(out_joint_angles->joint1_deg);
    float O2_z = 0.0f;
    if (leg_id == LEG_R_B || leg_id == LEG_L_B)
        O2_z = HIP_ABD_LENGTH * sinf(-joint1_rad);
    else
        O2_z = HIP_ABD_LENGTH * sinf(joint1_rad);

    float O2_y = HIP_ABD_LENGTH * cosf(joint1_rad); // 髋侧摆末端O2点在Y-Z平面的投影坐标
    if (leg_id == LEG_R_F || leg_id == LEG_R_B)
        O2_y = -O2_y; // 右腿Y方向取反
    float delta_x = local_x - 0.0f;
    float delta_y = local_y - O2_y;
    float delta_z = local_z - O2_z; // 足端相对于O2点（joint2旋转中心）的坐标差

    float D = sqrtf(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z); // O2到足端的直线距离

    // 距离奇异点校验
    if (D < EPSILON)
    {
        LOG_IK("[Workspace Error] 足端距离过小，奇异点 D=%.4f m\n", D);
        return IK_ERROR_WORKSPACE;
    }

    float D_min = fabsf(THIGH_LENGTH - CALF_LENGTH);
    float D_max = THIGH_LENGTH + CALF_LENGTH;

    // 工作空间校验：距离超出腿长可达范围
    if (D < D_min - 0.01f)
    {
        LOG_IK("[Workspace Error] 足端距离过小，超出工作空间 D=%.4f m\n", D);
        return IK_ERROR_WORKSPACE;
    }
    else if (D > D_max + 0.01f)
    {
        LOG_IK("[Workspace Error] 足端距离过大，超出工作空间 D=%.4f m\n", D);
        return IK_ERROR_WORKSPACE;
    }

    // 余弦定理求解膝关节角度，cosθ3 = (L1? + L2? - D?) / (2 * L1 * L2)
    // 余弦定理求解膝关节内角，范围 0 ~ π（0°~180°）
    float cos_theta3 = (THIGH_LENGTH * THIGH_LENGTH + CALF_LENGTH * CALF_LENGTH - D * D) / (2 * THIGH_LENGTH * CALF_LENGTH);
    cos_theta3 = float_clamp(cos_theta3, -1.0f, 1.0f);
    float theta3_rad = acosf(cos_theta3);

    // 转换为自定义关节角度：伸直为0°，右腿弯曲为负，左腿弯曲为正
    // 右腿：范围 -180° ~ 0°；左腿：范围 0° ~ 180°
    float joint3_rad;
    switch (leg_id)
    {
    case LEG_R_F:
    case LEG_R_B:
        joint3_rad = theta3_rad - pi; // 内角180°→0°，内角0°→-180°
        break;
    case LEG_L_F:
    case LEG_L_B:
        joint3_rad = pi - theta3_rad; // 内角180°→0°，内角0°→180°
        break;
    default:
        joint3_rad = 0.0f;
        break;
    }

    out_joint_angles->joint3_deg = RAD_TO_DEG(joint3_rad);
    // printf("joint3为%.1f°", out_joint_angles->joint3_deg);

    // joint3限位校验
    switch (leg_id)
    {
    case LEG_L_F: // 左腿
    case LEG_L_B: //[0到170度]
        if (out_joint_angles->joint3_deg < JOINT3_MIN)
        {
            out_joint_angles->joint3_deg = JOINT3_MIN;
            LOG_IK("1joint3角度超出范围，角度太小！return IK_ERROR_JOINT3_LIMIT\n");
            return IK_ERROR_JOINT3_LIMIT;
        }
        if (out_joint_angles->joint3_deg > JOINT3_MAX)
        {
            out_joint_angles->joint3_deg = JOINT3_MAX;
            LOG_IK("2joint3角度超出范围，角度太大！return IK_ERROR_JOINT3_LIMIT\n");
            return IK_ERROR_JOINT3_LIMIT;
        }
        break;
    case LEG_R_B: // 右腿
    case LEG_R_F: //[-170到0度]
        if (out_joint_angles->joint3_deg < (-JOINT3_MAX))
        {
            out_joint_angles->joint3_deg = (-JOINT3_MAX);
            LOG_IK("3joint3角度超出范围，角度太大！return IK_ERROR_JOINT3_LIMIT\n");
            return IK_ERROR_JOINT3_LIMIT;
        }
        if (out_joint_angles->joint3_deg > JOINT3_MIN)
        {
            out_joint_angles->joint3_deg = JOINT3_MIN;
            LOG_IK("4joint3角度超出范围，角度太小！return IK_ERROR_JOINT3_LIMIT\n");
            return IK_ERROR_JOINT3_LIMIT;
        }
        break;
    }
    // ************************* 第四步：求解joint2（髋俯仰关节），辅助角公式法
    // -------------------------- 辅助角公式构建方程 --------------------------
    // 方程形式：A*sinθ + B*cosθ = C
    float theta1 = joint1_rad;                               // 已解出的关节1角度
    float A = delta_x;                                       // 方程系数A，对应X方向分量
    float B = delta_z * cos(theta1) - delta_y * sin(theta1); // 方程系数B，投影到矢状面
    float d2 = THIGH_LENGTH;                                 // 大腿长度 d2
    float d3 = CALF_LENGTH;                                  // 小腿长度 d3
    float d4 = D;                                            // O2到足端距离 d4
    float C = (d2 * d2 + d4 * d4 - d3 * d3) / (2.0f * d2);    // 方程常数项C
    if (fabsf(C) > sqrtf(A * A + B * B))
    {
        LOG_IK("[IK Error] 方程无解，超出joint2工作空间\n");
        LOG_IK("[IK Error] fabsf(C)=%.4f - sqrtf(A*A + B*B)=%.4f\n== %.4f\n", fabsf(C), sqrtf(A * A + B * B), fabsf(C) - sqrtf(A * A + B * B));
        return IK_ERROR_WORKSPACE;
    }

    // 1.f 计算振幅R与相位角alpha
    float R = sqrtf(A * A + B * B);
    if (R < EPSILON)
    {
        LOG_IK("[IK Error] 振幅过小，奇异点，无法解算joint2\n");
        return IK_ERROR_WORKSPACE;
    }
    float alpha_1 = atan2f(A, B); // 相位角，由A、B的比值决定

    // 2.f 计算phi角，得到两个候选解
    float cos_phi = float_clamp(C / R, -1.0f, 1.0f);
    float phi_1 = acosf(cos_phi);
    // 3.f 得到joint2的两个候选解：theta2 = alpha ± phi
    float joint2_rad_plus = alpha_1 + phi_1; // plus候选解
    joint2_rad_plus = rad_normalize(joint2_rad_plus);
    float joint2_rad_minus = alpha_1 - phi_1; // minus候选解
    joint2_rad_minus = rad_normalize(joint2_rad_minus);

    // static float joint2_pre_deg[4] = {0};
    float joint2_rad = 0.0f;
    switch (leg_id)
    {
    case LEG_L_F: // 左前腿
    case LEG_L_B: // 左后腿
        joint2_rad = joint2_rad_plus;
        break;
    case LEG_R_F: // 右前腿
    case LEG_R_B: // 右后腿
        joint2_rad = -joint2_rad_plus;
        break;
    }
    // -------------------------- 角度归一化与腿型符号匹配 --------------------------
    // 转换为角度输出
    out_joint_angles->joint2_deg = RAD_TO_DEG(joint2_rad);
    // joint2_pre_deg[leg_id] = out_joint_angles->joint2_deg;

    // #define JOINT2_MIN -70.0f #define JOINT2_MAX 20.0f
    // joint2限位校验
    switch (leg_id)
    {
    case LEG_L_F: // 左腿
    case LEG_L_B:
        // printf("joint2转角为=%.1f°(%.1f~%.1f°)\n",
        //        out_joint_angles->joint2_deg, JOINT2_MIN, JOINT2_MAX);
        if (out_joint_angles->joint2_deg < JOINT2_MIN)
        {
            out_joint_angles->joint2_deg = JOINT2_MIN;
            LOG_IK("1joint2角度超出范围，角度太大！return IK_ERROR_JOINT2_LIMIT\n");
            return IK_ERROR_JOINT2_LIMIT;
        }
        if (out_joint_angles->joint2_deg > JOINT2_MAX)
        {
            out_joint_angles->joint2_deg = JOINT2_MAX;
            LOG_IK("2joint2角度超出范围，角度太小！return IK_ERROR_JOINT2_LIMIT\n");
            return IK_ERROR_JOINT2_LIMIT;
        }
        break;
    case LEG_R_B: // 右腿
    case LEG_R_F:
        // printf("计算得到joint2转角为=%.1f°(joint2限位:%.1f~%.1f°)\n",
        //        out_joint_angles->joint2_deg, (-JOINT2_MAX), (-JOINT2_MIN));
        if (out_joint_angles->joint2_deg < (-JOINT2_MAX))
        {
            out_joint_angles->joint2_deg = (-JOINT2_MAX);
            LOG_IK("3joint2角度超出范围，角度太大！return IK_ERROR_JOINT2_LIMIT\n");
            return IK_ERROR_JOINT2_LIMIT;
        }
        if (out_joint_angles->joint2_deg > (-JOINT2_MIN)) //>20°
        {
            out_joint_angles->joint2_deg = (-JOINT2_MIN);
            LOG_IK("4joint2角度超出范围，角度太小！return IK_ERROR_JOINT2_LIMIT\n");
            return IK_ERROR_JOINT2_LIMIT;
        }
        break;
    }

    out_joint_angles->valid = 1;
    // printf("[IK Success] 解算完成，valid = %d\n", out_joint_angles->valid);
    //  解算成功返回
    return IK_SUCCESS;
}
