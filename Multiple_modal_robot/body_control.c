#include "body_control.h"
#include <math.h>
#include <stdio.h>


// 默认PD姿态参数，宏定义来自robot_config.h
const AttitudePDConfig_t ATTITUDE_PD_DEFAULT = {
    .kp_pitch = PD_KP_PITCH,
    .kd_pitch = PD_KD_PITCH,
    .kp_roll = PD_KP_ROLL,
    .kd_roll = PD_KD_ROLL,
    .max_correction = PD_MAX_CORRECTION,
    .kp_yaw = PD_KP_YAW,
    .ki_yaw = PD_KI_YAW,
    .kd_yaw = PD_KD_YAW,
    .max_yaw_step = PD_MAX_YAW_STEP,
    .max_yaw_integral = PD_MAX_YAW_INTEGRAL,
    .dead_zone_pitch_rad = PD_DZ_PITCH_RAD,
    .dead_zone_roll_rad = PD_DZ_ROLL_RAD,
    .dead_zone_yaw_rad = PD_DZ_YAW_RAD};


// 行走模式姿态PD参数
const AttitudePDConfig_t ATTITUDE_PD_WALK = {
    .kp_pitch = PD_WALK_KP_PITCH,
    .kd_pitch = PD_WALK_KD_PITCH,
    .kp_roll = PD_WALK_KP_ROLL,
    .kd_roll = PD_WALK_KD_ROLL,
    .max_correction = PD_WALK_MAX_CORRECTION,
    .kp_yaw = PD_WALK_KP_YAW,
    .ki_yaw = PD_WALK_KI_YAW,
    .kd_yaw = PD_WALK_KD_YAW,
    .max_yaw_step = PD_WALK_MAX_YAW_STEP,
    .max_yaw_integral = PD_WALK_MAX_YAW_INTEGRAL,
    .dead_zone_pitch_rad = PD_WALK_DZ_PITCH_RAD,
    .dead_zone_roll_rad = PD_WALK_DZ_ROLL_RAD,
    .dead_zone_yaw_rad = PD_WALK_DZ_YAW_RAD};


AttitudePDConfig_t g_attitude_pd;


PID_Struct g_pitch_pid;
PID_Struct g_roll_pid;
PID_Struct g_yaw_pid;


float g_current_pitch_correction = 0.0f;
float g_current_roll_correction = 0.0f;


float g_yaw_target = 0.0f;
float g_current_yaw_correction = 0.0f;
float g_yaw_manual_offset = 0.0f;
uint8_t g_yaw_target_initialized = 0;
float g_pitch_target = 0.0f; // 机体俯仰目标角度(rad)，正值抬头，负值低头


/**
 * @brief  机体姿态控制器初始化
 */
void body_attitude_control_init(void)
{
    float dt = TIME_STEP / 1000.0f;


    g_attitude_pd = ATTITUDE_PD_WALK;


    g_pitch_pid.P = g_attitude_pd.kp_pitch;
    g_pitch_pid.I = 0.0f;
    g_pitch_pid.D = g_attitude_pd.kd_pitch / dt;
    g_pitch_pid.Last_Error = 0.0f;
    g_pitch_pid.Limit_Output = g_attitude_pd.max_correction;
    g_pitch_pid.Limit_Integral = LIMIT_ERR;
    g_pitch_pid.Integral = 0.0f;


    g_roll_pid.P = g_attitude_pd.kp_roll;
    g_roll_pid.I = 0.0f;
    g_roll_pid.D = g_attitude_pd.kd_roll / dt;
    g_roll_pid.Last_Error = 0.0f;
    g_roll_pid.Limit_Output = g_attitude_pd.max_correction;
    g_roll_pid.Limit_Integral = LIMIT_ERR;
    g_roll_pid.Integral = 0.0f;


    g_yaw_pid.P = g_attitude_pd.kp_yaw;
    g_yaw_pid.I = g_attitude_pd.ki_yaw * dt;
    g_yaw_pid.D = g_attitude_pd.kd_yaw / dt;
    g_yaw_pid.Last_Error = 0.0f;
    g_yaw_pid.Limit_Output = g_attitude_pd.max_yaw_step;
    g_yaw_pid.Limit_Integral = g_attitude_pd.max_yaw_integral / dt;
    g_yaw_pid.Integral = 0.0f;


    g_current_yaw_correction = 0.0f;
    g_yaw_manual_offset = 0.0f;
    g_yaw_target_initialized = 0;
    g_pitch_target = 0.0f;
}


/**
 * @brief  机体姿态PD控制更新，支持航向Z轴稳定
 * @param  leg_sm[4] 四条腿状态机指针数组
 * @note   参数leg_sm当前未使用，姿态修正在foot_lerp之后执行
 */
void body_attitude_control_update(LegStateMachine_t *leg_sm[4])
{
    (void)leg_sm; // 未使用参数，姿态修正逻辑放在foot_lerp之后


    if (g_robot_state.mode != ROBOT_MODE_QUADRUPED)
        return;


    // 姿态误差计算：目标俯仰 g_pitch_target，0代表机体水平
    float pitch_error = g_pitch_target - g_robot_state.body_pitch;
    float roll_error = 0.0f - g_robot_state.body_roll;

    // 俯仰死区处理，小于死区阈值误差清零
    if (fabsf(pitch_error) < g_attitude_pd.dead_zone_pitch_rad)
        pitch_error = 0.0f;
    else
        pitch_error -= (pitch_error > 0.0f ? g_attitude_pd.dead_zone_pitch_rad : -g_attitude_pd.dead_zone_pitch_rad);

    // 横滚死区处理，小于死区阈值误差清零
    if (fabsf(roll_error) < g_attitude_pd.dead_zone_roll_rad)
        roll_error = 0.0f;
    else
        roll_error -= (roll_error > 0.0f ? g_attitude_pd.dead_zone_roll_rad : -g_attitude_pd.dead_zone_roll_rad);

    // PD输出腿部高度修正量，PID内部自带输出限幅与误差保存
    float pitch_correction = PID_Normal_Ctrl(pitch_error, 0.0f, &g_pitch_pid);
    float roll_correction = PID_Normal_Ctrl(roll_error, 0.0f, &g_roll_pid);


    g_current_pitch_correction = pitch_correction;
    g_current_roll_correction = roll_correction;


    // printf("[姿态调试] pitch=%.4f rad | roll=%.4f rad | pitch_corr=%.4f | roll_corr=%.4f\n",
    //     g_robot_state.body_pitch, g_robot_state.body_roll,
    //     pitch_correction, roll_correction);


    // 航向稳定逻辑：速度模式目标航向 = 手动偏置 + 运动方向角
    if (!g_yaw_target_initialized)
    {
        g_yaw_target = g_robot_state.body_yaw;
        g_yaw_manual_offset = g_robot_state.body_yaw;
        g_yaw_target_initialized = 1;
    }


    if (fabsf(g_robot_state.cmd_vel_yaw) > 1e-6f)
    {
        // 手动旋转模式：Q/E控制转向，关闭航向稳定PID
        g_current_yaw_correction = 0.0f;
        g_yaw_manual_offset = g_robot_state.body_yaw;
        g_yaw_target = g_robot_state.body_yaw;
        g_yaw_pid.Integral = 0.0f;
        g_yaw_pid.Last_Error = 0.0f;
        // printf("[航向]手动旋转模式, offset=%.3f rad\n", g_yaw_manual_offset);
    }
    else
    {
        // 速度模式，根据xy运动速度计算期望前进朝向
        float desired_heading = 0.0f;
        if (fabsf(g_robot_state.cmd_vel_x) > 1e-6f)
        {
            desired_heading = atan2f(g_robot_state.cmd_vel_y, g_robot_state.cmd_vel_x);
        }
        g_yaw_target = g_yaw_manual_offset + desired_heading;


        float yaw_error_raw = g_yaw_target - g_robot_state.body_yaw;
        // 角度误差归算到 [-pi, pi]
        while (yaw_error_raw > pi)
            yaw_error_raw -= 2.0f * pi;
        while (yaw_error_raw < -pi)
            yaw_error_raw += 2.0f * pi;


        // 航向死区处理，处理后送入PID，PID内部完成积分微分限幅
        float yaw_error = yaw_error_raw;
        if (fabsf(yaw_error) < g_attitude_pd.dead_zone_yaw_rad)
            yaw_error = 0.0f;
        else
            yaw_error -= (yaw_error > 0.0f ? g_attitude_pd.dead_zone_yaw_rad : -g_attitude_pd.dead_zone_yaw_rad);


        g_current_yaw_correction = PID_Normal_Ctrl(yaw_error, 0.0f, &g_yaw_pid);


        // printf("[航向PID] target=%.3f desired=%.3f offset=%.3f actual=%.3f err=%.4f I=%.4f out=%.4f\n",
        //         g_yaw_target, desired_heading, g_yaw_manual_offset, g_robot_state.body_yaw, yaw_error, g_yaw_pid.Integral, g_current_yaw_correction);
    }

}


/**
 * @brief  将姿态修正量应用到支撑腿，foot_lerp之后、IK求解之前调用
 * @param  sm 单条腿状态机指针
 */
void body_apply_to_stance_leg(LegStateMachine_t *sm)
{
    if (sm == NULL || sm->current_state != LEG_STATE_STANCE)
        return;


    float height_correction = 0.0f;


    // 俯仰修正：前腿、后腿高度反向补偿
    if (sm->leg_id == LEG_R_F || sm->leg_id == LEG_L_F)
        height_correction -= g_current_pitch_correction;
    else
        height_correction += g_current_pitch_correction;


    // 横滚修正：右腿、左腿高度反向补偿
    if (sm->leg_id == LEG_R_F || sm->leg_id == LEG_R_B)
        height_correction += g_current_roll_correction;
    else
        height_correction -= g_current_roll_correction;


    // 机体俯仰目标带来几何Z偏移补偿，Z ≈ pitch * hip_x
    float pitch_geom_z = g_pitch_target * FOOT_OFFSET_X;
    if (sm->leg_id == LEG_R_F || sm->leg_id == LEG_L_F)
        height_correction -= pitch_geom_z;
    else
        height_correction += pitch_geom_z;


    sm->current_target_pos.foot_pos_z_m += height_correction;
}


/**
 * @brief  获取当前姿态PD输出修正量
 * @param  pitch_correction 传出俯仰高度修正
 * @param  roll_correction  传出横滚高度修正
 */
void body_get_correction(float *pitch_correction, float *roll_correction)
{
    if (pitch_correction != NULL)
        *pitch_correction = g_current_pitch_correction;
    if (roll_correction != NULL)
        *roll_correction = g_current_roll_correction;
}


/**
 * @brief  获取航向稳定输出修正量
 * @retval yaw修正输出
 */
float body_get_yaw_correction(void)
{
    return g_current_yaw_correction;
}


/**
 * @brief 重置机体俯仰目标角度为水平
 */
void body_reset_pitch_target(void)
{
    g_pitch_target = 0.0f;
}


/**
 * @brief  计算ZMP零力矩点与稳定裕度
 * @param  zmp_out ZMP计算结果输出结构体
 * @param  leg_sm[4] 四条腿状态机数组
 */
void zmp_calculate(ZMPData_t *zmp_out, LegStateMachine_t *leg_sm[4])
{
    float total_force = 0.0f;
    float zmp_x = 0.0f;
    float zmp_y = 0.0f;


    float min_x = 100.0f;
    float max_x = -100.0f;
    float min_y = 100.0f;
    float max_y = -100.0f;


    // 遍历四条支撑腿，加权计算ZMP，此处默认各腿受力相等
    for (int i = 0; i < 4; i++)
    {
        if (leg_sm[i]->current_state == LEG_STATE_STANCE)
        {
            float x = leg_sm[i]->current_target_pos.foot_pos_x_m;
            float y = leg_sm[i]->current_target_pos.foot_pos_y_m;
            float force = 1.0f; // 简化实现，默认支撑腿受力均等


            total_force += force;
            zmp_x += x * force;
            zmp_y += y * force;


            // 更新支撑多边形边界
            min_x = fminf(min_x, x);
            max_x = fmaxf(max_x, x);
            min_y = fminf(min_y, y);
            max_y = fmaxf(max_y, y);
        }
    }


    // 无支撑腿，判定为不稳定
    if (total_force < 0.1f)
    {
        zmp_out->is_stable = false;
        zmp_out->stability_margin = 0.0f;
        zmp_out->x = 0.0f;
        zmp_out->y = 0.0f;
        return;
    }


    // 归一化得到ZMP坐标
    zmp_x /= total_force;
    zmp_y /= total_force;


    zmp_out->x = zmp_x;
    zmp_out->y = zmp_y;


    // 判断ZMP点是否落在支撑多边形内部
    zmp_out->is_stable = (zmp_x >= min_x && zmp_x <= max_x) && (zmp_y >= min_y && zmp_y <= max_y);


    // 计算稳定裕度：取到多边形边界的最小距离，裕度越大越稳定
    float margin_x = fminf(zmp_x - min_x, max_x - zmp_x);
    float margin_y = fminf(zmp_y - min_y, max_y - zmp_y);
    zmp_out->stability_margin = fminf(margin_x, margin_y);
}
