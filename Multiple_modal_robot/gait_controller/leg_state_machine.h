/**
 ******************************************************************************
 * @file    leg_state_machine.h
 * @brief   单腿状态机头文件
 ******************************************************************************
 */

#ifndef LEG_STATE_MACHINE_H
#define LEG_STATE_MACHINE_H

#include "include_head.h"
#include "bezier_foot_tra.h"
#include "robot_state.h"
#include "gait_generator.h"

/**
 * @brief 腿状态错误码
 */
typedef enum
{
    LEG_ERR_NONE = 0,
    LEG_ERR_TRAJ_INIT_FAIL,
    LEG_ERR_INVALID_TRANS,
    LEG_ERR_STANCE_IK_FAIL,
    LEG_ERR_SWING_TRAJ_FAIL,
} LegErrorCode_t;

/**
 * @brief 单腿状态枚举
 */
typedef enum
{
    LEG_STATE_HOLD = 0,  // 保持/待机
    LEG_STATE_STANCE,    // 支撑相
    LEG_STATE_SWING,     // 摆动相
    LEG_STATE_WHEEL,     // 轮式模式
    LEG_STATE_FOLD,      // 腿部折叠
    LEG_STATE_UNFOLD,    // 腿部展开
    LEG_STATE_ERROR      // 错误状态
} LegState_t;

/**
 * @brief 单腿状态机结构体
 */
typedef struct
{
    Leg_ID_t leg_id;

    LegState_t current_state;
    LegState_t last_state;

    float state_start_time_s;
    float state_duration_s;

    float phase;         // 当前状态内部相位 0~1
    float swing_phase;   // 摆动相相位缓存
    float stance_phase;  // 支撑相相位缓存

    LegTrajContext_t traj_ctx;

    // 摆动相起点终点
    FootPosition_t swing_start_pos;
    FootPosition_t swing_end_pos;

    // 支撑相起点终点
    FootPosition_t stance_start_pos;
    FootPosition_t stance_end_pos;

    FootPosition_t current_target_pos;
    LegJointAngles_t current_joint_angles;

    // 关节空间目标姿态：轮式/折叠状态在 run 回调里朝它插值
    LegJointAngles_t target_joint_angles;

    int ik_fail_count;
    int is_trajectory_finished;
    TrajStatus_t last_traj_status;
    LegErrorCode_t last_error;

    FootPosition_t stance_world_pos;
    uint8_t first_swing_done;
} LegStateMachine_t;

int leg_state_machine_init(LegStateMachine_t *sm, Leg_ID_t leg_id, BezierTrajConfig_t config);

int leg_state_switch(LegStateMachine_t *sm, LegState_t new_state,
                     float duration_s, const FootPosition_t *end_pos,
                     float current_time_s);

/**
 * @brief 关节空间状态跳转：轮式/折叠/展开用，目标为关节角而非足端位置
 * @param sm 状态机
 * @param new_state 目标状态（WHEEL / FOLD / UNFOLD）
 * @param duration_s 过渡时长，run 回调按 (now-start)/duration 从起始角线性插值到目标角
 * @param target_joint_angles 目标关节角（IK规范角，未做安装方向重映射）
 * @param current_time_s 当前系统时间
 * @return 0成功，-1不允许跳转/空指针/目标状态不是关节空间状态
 * @note  起始角取当前 current_joint_angles，与原先在主循环里记录 “_start_angles” 等价
 */
int leg_state_switch_joint(LegStateMachine_t *sm, LegState_t new_state,
                           float duration_s, const LegJointAngles_t *target_joint_angles,
                           float current_time_s);

int leg_state_machine_update(LegStateMachine_t *sm, float current_time_s);

void leg_get_target_angles(const LegStateMachine_t *sm, LegJointAngles_t *out_angles);

void leg_get_target_pos(const LegStateMachine_t *sm, FootPosition_t *out_pos);

const char *leg_state_get_name(LegState_t state);

const char *leg_error_str(LegErrorCode_t e);

void leg_state_machine_print(const LegStateMachine_t *sm);

void leg_state_machine_reset(LegStateMachine_t *sm);

/**
 * @brief 是否四足行走模式（支撑/摆动）
 */
static inline uint8_t leg_is_quadruped_mode(const LegStateMachine_t *sm)
{
    return (sm->current_state == LEG_STATE_STANCE || sm->current_state == LEG_STATE_SWING);
}

/**
 * @brief 轮式/折叠/站立三类姿态的目标关节角（IK规范角，未做安装方向重映射）
 * @note  原先定义在 UpctrlTask.c 的 WHEEL/STAND/FOLD_TARGET_ANGLES，
 *        现由状态机统一提供，保证同一种姿态只有一个数据源
 */
static inline LegJointAngles_t leg_wheel_mode_angles(Leg_ID_t leg_id)
{
    static const LegJointAngles_t t[4] = {
        [LEG_R_F] = {0.0f,  WHEEL_J2, -WHEEL_J3, 1},
        [LEG_L_F] = {0.0f, -WHEEL_J2,  WHEEL_J3, 1},
        [LEG_R_B] = {0.0f,  WHEEL_J2, -WHEEL_J3, 1},
        [LEG_L_B] = {0.0f, -WHEEL_J2,  WHEEL_J3, 1},
    };
    return t[leg_id];
}

static inline LegJointAngles_t leg_stand_mode_angles(Leg_ID_t leg_id)
{
    static const LegJointAngles_t t[4] = {
        [LEG_R_F] = {STAND_J1,  STAND_J2, -STAND_J3, 1},
        [LEG_L_F] = {STAND_J1, -STAND_J2,  STAND_J3, 1},
        [LEG_R_B] = {STAND_J1,  STAND_J2, -STAND_J3, 1},
        [LEG_L_B] = {STAND_J1, -STAND_J2,  STAND_J3, 1},
    };
    return t[leg_id];
}

static inline LegJointAngles_t leg_fold_mode_angles(Leg_ID_t leg_id)
{
    static const LegJointAngles_t t[4] = {
        [LEG_R_F] = {-FOLD_J1, FOLD_J2, -FOLD_J3, 1},
        [LEG_L_F] = {FOLD_J1, -FOLD_J2, FOLD_J3, 1},
        [LEG_R_B] = {FOLD_J1, FOLD_J2, -FOLD_J3, 1},
        [LEG_L_B] = {-FOLD_J1, -FOLD_J2, FOLD_J3, 1},
    };
    return t[leg_id];
}

/**
 * @brief 是否轮式模式
 */
static inline uint8_t leg_is_wheeled_mode(const LegStateMachine_t *sm)
{
    return (sm->current_state == LEG_STATE_WHEEL);
}

/**
 * @brief 是否折叠（飞行模式）
 */
static inline uint8_t leg_is_flight_mode(const LegStateMachine_t *sm)
{
    return (sm->current_state == LEG_STATE_FOLD);
}

/**
 * @brief 腿是否正在运动
 */
static inline uint8_t leg_is_moving(const LegStateMachine_t *sm)
{
    return (sm->current_state == LEG_STATE_SWING ||
            sm->current_state == LEG_STATE_UNFOLD ||
            sm->current_state == LEG_STATE_FOLD);
}

/**
 * @brief 状态跳转许可表，定义合法状态迁移
 */
static inline uint8_t leg_state_switch_allowed(LegState_t current, LegState_t target)
{
    if (current == LEG_STATE_ERROR)
        return (target == LEG_STATE_HOLD) ? 1 : 0;
    if (current == LEG_STATE_HOLD)
        return 1;
    if ((current == LEG_STATE_STANCE || current == LEG_STATE_SWING) &&
        (target == LEG_STATE_STANCE || target == LEG_STATE_SWING || target == LEG_STATE_HOLD || target == LEG_STATE_ERROR || target == LEG_STATE_WHEEL))
        return 1;
    if (current == LEG_STATE_WHEEL &&
        (target == LEG_STATE_HOLD || target == LEG_STATE_UNFOLD || target == LEG_STATE_STANCE ||
         target == LEG_STATE_FOLD))
        return 1;
    if (current == LEG_STATE_FOLD &&
        (target == LEG_STATE_HOLD || target == LEG_STATE_UNFOLD || target == LEG_STATE_WHEEL))
        return 1;
    if (current == LEG_STATE_UNFOLD &&
        (target == LEG_STATE_STANCE || target == LEG_STATE_HOLD))
        return 1;
    return 0;
}

#endif
