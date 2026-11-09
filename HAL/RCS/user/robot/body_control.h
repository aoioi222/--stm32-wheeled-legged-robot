#ifndef BODY_CONTROL_H
#define BODY_CONTROL_H


#include "include_head.h"
#include "leg_state_machine.h"
#include "robot_state.h"
#include "RCS_PIDctrl.h"


// 姿态PD控制器参数结构体
typedef struct
{
    float kp_pitch;             // 俯仰比例系数
    float kd_pitch;             // 俯仰微分系数
    float kp_roll;              // 横滚比例系数
    float kd_roll;              // 横滚微分系数
    float max_correction;        // 姿态最大腿部Z方向修正量(m)
    float kp_yaw;                // 航向比例系数(m/rad)
    float ki_yaw;                // 航向积分系数(m/(rad*s))，用于速度模式航向锁定
    float kd_yaw;                // 航向微分系数(m*s/rad)
    float max_yaw_step;          // 航向PID输出限幅(m)
    float max_yaw_integral;      // 航向积分限幅，抗积分饱和
    float dead_zone_pitch_rad;   // 俯仰死区(rad)，小于阈值不输出控制
    float dead_zone_roll_rad;    // 横滚死区(rad)
    float dead_zone_yaw_rad;     // 航向死区(rad)
} AttitudePDConfig_t;


// ZMP零力矩点输出结构体
typedef struct
{
    float x;                    // ZMP点x坐标(m)
    float y;                    // ZMP点y坐标(m)
    bool is_stable;             // 是否处于稳定状态
    float stability_margin;    // 稳定裕度(m)，数值越大稳定性越高
} ZMPData_t;


// 姿态控制器初始化
void body_attitude_control_init(void);


// 姿态PD控制更新，每个控制周期调用
void body_attitude_control_update(LegStateMachine_t *leg_sm[4]);


// 将姿态修正量施加给支撑腿，调用顺序 foot_lerp之后，IK求解之前
void body_apply_to_stance_leg(LegStateMachine_t *sm);


// 获取当前姿态PD修正输出
void body_get_correction(float *pitch_correction, float *roll_correction);


// 获取航向稳定修正输出
float body_get_yaw_correction(void);


// 将机体俯仰目标重置为水平
void body_reset_pitch_target(void);


// 航向目标初始化标志，外部可读写，用于开机航向初始化
extern uint8_t g_yaw_target_initialized;
extern float g_yaw_manual_offset;
extern float g_pitch_target; // 机体俯仰目标角度(rad)


extern PID_Struct g_pitch_pid;
extern PID_Struct g_roll_pid;
extern PID_Struct g_yaw_pid;


// ZMP零力矩点计算
void zmp_calculate(ZMPData_t *zmp_out, LegStateMachine_t *leg_sm[4]);


#endif
