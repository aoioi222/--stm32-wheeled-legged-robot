#ifndef GRAVITY_COMP_H
#define GRAVITY_COMP_H

/* 单腿重力矩补偿 */
/* 输入三关节角度（度，RF 腿规范约定），输出对应关节重力矩（N·m） */
/* tau_nm[0] 髋外展矩，tau_nm[1] 髋摆动矩，tau_nm[2] 膝矩 */
void leg_gravity_torque(float q1_deg, float q2_deg, float q3_deg, float tau_nm[3]);

#endif

