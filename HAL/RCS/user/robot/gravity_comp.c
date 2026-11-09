/**
 * @file    gravity_comp.c
 * @brief   单腿重力矩补偿，作为关节位置前馈
 * @details 按 RF 腿规范约定计算三关节（髋外展/髋摆动/膝）重力矩。
 *          膝角采用 FK 约定 theta3 = -joint3_deg。
 *          质量为占位值，需实测标定。
 */
#include "gravity_comp.h"
#include "robot_config.h"
#include <math.h>

void leg_gravity_torque(float q1_deg, float q2_deg, float q3_deg, float tau_nm[3])
{
    float q1 = DEG_TO_RAD(q1_deg);
    float q2 = DEG_TO_RAD(q2_deg);
    float q3 = -DEG_TO_RAD(q3_deg); /* FK 膝角符号 */

    float c1 = cosf(q1);
    float s1 = sinf(q1);
    float c2 = cosf(q2);
    float s2 = sinf(q2);
    float c23 = cosf(q2 + q3);
    float s23 = sinf(q2 + q3);

    /* 髋外展重力矩 */
    tau_nm[0] = G * s1 * (HIP_ABD_MASS * HIP_ABD_LENGTH / 2.0f + THIGH_MASS * (HIP_ABD_LENGTH + THIGH_LENGTH / 2.0f * c2) + CALF_MASS * (HIP_ABD_LENGTH + THIGH_LENGTH * c2 + CALF_LENGTH / 2.0f * c23));

    /* 髋摆动重力矩 */
    tau_nm[1] = G * c1 * ((THIGH_MASS / 2.0f + CALF_MASS) * THIGH_LENGTH * s2 + CALF_MASS * CALF_LENGTH / 2.0f * s23);

    /* 膝重力矩 */
    tau_nm[2] = G * c1 * CALF_MASS * CALF_LENGTH / 2.0f * s23;
}
