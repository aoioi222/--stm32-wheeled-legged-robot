/**
 ******************************************************************************
 * @file    gait_generator.h
 * @brief   四足机器人步态发生器模块
 *          负责根据机身速度指令生成各腿支撑/摆动目标、步态相位规划
 *          支持Trot/Walk/Pronk/Pace/Bound多种经典步态
 ******************************************************************************
 */

#ifndef GAIT_GENERATOR_H
#define GAIT_GENERATOR_H

#include "include_head.h"
#include "leg_state_machine.h"
#include "robot_state.h"
// 腿ID索引：LEG_R_F=0, LEG_L_F=1, LEG_R_B=2, LEG_L_B=3
//  步态类型枚举
typedef enum
{
    GAIT_TROT,  // 对角小跑（右前+左后 / 左前+右后 对角同步，最常用动态步态）
    GAIT_WALK,  // 慢走四拍步态（依次抬腿，静态稳定步态）
    GAIT_PRONK, // 四腿同时蹦跳（四腿同相位，腾空时间长）
    GAIT_PACE,  // 同侧步态（左侧两腿 / 右侧两腿 同步运动）
    GAIT_BOUND  // 前后分组跳（前腿一组 / 后腿一组 同步运动）
} GaitType_t;

/**
 * @brief 步态生成配置参数
 * @note  用于定义步态周期、步长限制、摆动高度等运动约束
 */
typedef struct
{
    float cycle_time;  // 完整步态周期，单位 s
    float swing_ratio; // 单腿摆动相在一个周期内的占比 (0~1)
    float step_height; // 足端默认摆动抬升高度，单位 m//和lift_z有关
    float max_step_x;  // X轴（前后）最大单步步长限制，单位 m
    float max_step_y;  // Y轴（左右）最大单步步长限制，单位 m
} GaitGenConfig_t;

/**
 * @brief 单腿步态输出结果
 * @note  输出给单腿轨迹控制器的目标点与状态标志
 */
typedef struct
{
    FootPosition_t swing_start;  // 摆动起点 = 支撑终点（抬脚点）
    FootPosition_t swing_end;    // 摆动终点 = 支撑起点（落地点）
    FootPosition_t stance_start; // 支撑起点 = 摆动终点
    FootPosition_t stance_end;   // 支撑终点 = 摆动起点
    float stance_duration_s;     // 支撑相持续时间 s
    float swing_duration_s;      // 摆动相持续时间 s
    uint8_t need_swing;          // 1=当前相位应进入摆动相
} LegGaitOutput_t;

/**
 * @brief 全局步态发生器上下文
 * @note  保存当前步态类型、配置参数、全局相位及四条腿的输出目标
 */
typedef struct
{
    GaitType_t gait_type;          // 当前选用的步态类型
    GaitGenConfig_t config;        // 步态运动参数配置
    float global_phase;            // 全局归一化相位 [0,1)，由仿真时间与周期计算
    LegGaitOutput_t leg_output[4]; // 四条腿的步态目标输出，索引对应腿ID
} GaitGenerator_t;

/**
 * @brief  步态发生器初始化
 * @param  gen 步态发生器句柄
 * @return 无
 * @note   初始化默认使用对角小跑(TROT)参数，清空上下文
 */
void gait_gen_init(GaitGenerator_t *gen);

/**
 * @brief  步态核心更新函数
 * @param  gen   步态发生器句柄
 * @param  robot 机器人全局状态（包含仿真时间、机身高度、速度指令）
 * @return 无
 * @note   计算全局相位→根据速度计算步长→按步态类型分配相位→输出各腿目标
 */
void gait_gen_update(GaitGenerator_t *gen, const RobotGlobalState_t *robot);

/**
 * @brief  获取指定腿的步态目标
 * @param  gen     步态发生器句柄
 * @param  leg_id  腿编号（LEG_R_F/LEG_L_F/LEG_R_B/LEG_L_B）
 * @param  out     输出单腿步态目标
 * @return 无
 */
void gait_gen_get_leg_target(GaitGenerator_t *gen, Leg_ID_t leg_id, LegGaitOutput_t *out);

float gait_gen_get_single_phase(GaitGenerator_t *gen, int leg_id);

// 各步态默认配置（定义在 gait_generator.c，供运行时步态切换使用）
extern const GaitGenConfig_t TROT_CONFIG;
extern const GaitGenConfig_t WALK_CONFIG;
extern const GaitGenConfig_t PRONK_CONFIG;

// 运行时切换步态类型（更新 gait_type 和 config）
void gait_gen_switch_gait(GaitGenerator_t *gen, GaitType_t new_type);

#endif
