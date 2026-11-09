/**
 ******************************************************************************
 * @file    bezier_foot_tra.h
 * @brief   机械狗单腿足端轨迹生成模块头文件
 *          基于三次贝塞尔曲线实现足端摆动轨迹离线生成、实时插值与关节指令解算
 ******************************************************************************
 */

#ifndef BEZIER_FOOT_TRA_H
#define BEZIER_FOOT_TRA_H

#include "Jacobian_matrix.h" // 间接包含足端、关节、腿ID等基础结构体定义
#include "robot_math.h"

// 轨迹配置宏定义已集中到 robot_config.h
// MAX_TRAJECTORY_POINTS、TIME_EPSILON、CONFIG_DEFAULT_LIFT_HEIGHT 见 robot_config.h

/**
 * @brief 轨迹运行状态枚举
 */
typedef enum
{
    TRAJ_STATUS_OK = 0,      // 轨迹正常运行中
    TRAJ_STATUS_END,         // 轨迹已执行完成
    TRAJ_STATUS_ERROR_INPUT, // 执行异常（参数非法）
    TRAJ_STATUS_ERROR_IK     // 逆运动学求解失败
} TrajStatus_t;

/**
 * @brief 贝塞尔轨迹配置参数结构体
 */
typedef struct
{
    float cycle_time;    // 单步摆动周期，单位：s
    float lift_height_m; // 足端最大抬升高度，单位：m，推荐范围0.05~0.025m【调试参数】要考虑后续加上一些动态的调整
} BezierTrajConfig_t;

/**
 * @brief 轨迹运行时状态结构体
 * @note  用于保存上一周期数据，实现差分计算、IK失败保护
 */
typedef struct
{
    FootPosition_t last_foot_pos;       // 上一控制周期足端位置（机体坐标系）
    LegJointAngles_t last_joint_angles; // 上一控制周期关节角度
    float last_joint_vel[3];            // 上一控制周期关节角速度，顺序[j1,j2,j3]，单位：°/s
    float last_time;                    // 上一控制周期时间戳
} BezierTrajRuntime_t;

/**
 * @brief 离线预生成轨迹数据结构体
 */
typedef struct
{
    uint16_t point_num;                               // 实际生成的轨迹离散点数
    FootPosition_t point_list[MAX_TRAJECTORY_POINTS]; // 离散轨迹点数组
} BezierTrajData_t;

/**
 * @brief 单腿轨迹控制器总上下文
 * @note  包含单条腿轨迹的配置、运行状态、离线轨迹数据
 */
typedef struct
{
    Leg_ID_t leg_id;               // 对应机械狗腿ID
    BezierTrajConfig_t config;     // 轨迹配置参数
    BezierTrajRuntime_t runtime;   // 运行时状态
    BezierTrajData_t data;         // 离线预生成轨迹数据
    uint8_t is_trajectory_running; // 轨迹运行标志（预留，暂未使用）
} LegTrajContext_t;

// 对外接口函数声明
/**
 * @brief  单腿轨迹控制器初始化
 * @param  ctx      轨迹控制器上下文指针
 * @param  leg_id   对应机械狗腿ID
 * @param  config   轨迹配置参数（周期、抬升高度）
 * @retval 0=初始化成功，-1=参数非法/初始化失败
 */
int bezier_traj_controller_init(LegTrajContext_t *ctx, Leg_ID_t leg_id, BezierTrajConfig_t config);

/**
 * @brief  离线生成单腿足端三次贝塞尔摆动轨迹
 * @param  ctx        轨迹控制器上下文指针
 * @param  start_pos  摆动起点足端位置（机体坐标系）
 * @param  end_pos    摆动终点足端位置（机体坐标系）
 * @retval 无
 */
void generate_leg_bezier_traj(LegTrajContext_t *ctx, FootPosition_t start_pos, FootPosition_t end_pos);

/**
 * @brief  实时获取关节角度、角速度、角加速度指令
 * @param  ctx           轨迹控制器上下文指针
 * @param  current_time  当前摆动时间（从摆动开始计时），单位：s
 * @param  joint_angles  输出参数：目标关节角度
 * @param  joint_vel     输出参数：目标关节角速度
 * @param  joint_acc     输出参数：目标关节角加速度（当前固定为0）
 * @retval TrajStatus_t  轨迹运行状态
 */
TrajStatus_t get_leg_joint_cmd(LegTrajContext_t *ctx, float current_time,
                               LegJointAngles_t *joint_angles, float joint_vel[3], float joint_acc[3]);

/**
 * @brief  轨迹控制器去初始化（清零上下文）
 * @param  ctx  轨迹控制器上下文指针
 * @retval 无
 */
void bezier_traj_controller_deinit(LegTrajContext_t *ctx);

#endif // BEZIER_FOOT_TRA_H
