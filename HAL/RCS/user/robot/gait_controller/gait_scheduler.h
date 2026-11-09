/**
 ******************************************************************************
 * @file    gait_scheduler.h
 * @brief   四足机器人步态调度器模块
 *          作为步态发生器与腿部状态机的中间衔接层，根据步态相位
 *          自动控制单腿支撑/摆动状态的有序切换，实现步态闭环运行
 ******************************************************************************
 */

#ifndef GAIT_SCHEDULER_H
#define GAIT_SCHEDULER_H

#include "gait_generator.h"
#include "leg_state_machine.h"
#include "robot_state.h"

/**
 * @brief 步态调度器上下文结构体
 * @note  持有腿部状态机与步态发生器的指针，实现两者联动控制
 */
typedef struct
{
    LegStateMachine_t *leg_sm[4]; // 四条腿的状态机指针数组，索引对应腿ID
    GaitGenerator_t *gait_gen;    // 步态发生器指针，用于获取足端目标与摆动指令
} GaitScheduler_t;

/**
 * @brief  初始化步态调度器
 * @param  sch     步态调度器句柄
 * @param  leg_sm  四条腿状态机指针数组
 * @param  gen     步态发生器句柄
 * @retval 0=初始化成功，-1=参数空指针异常
 * @note   绑定状态机与步态发生器，清空调度器上下文
 */
int gait_scheduler_init(GaitScheduler_t *sch, LegStateMachine_t *leg_sm[4], GaitGenerator_t *gen);

/**
 * @brief  步态调度器主更新函数（控制周期调用）
 * @param  sch  步态调度器句柄
 * @note   仅在四足模式下工作；
 *         遍历四条腿，根据 gait_out.need_swing 标志自动切换支撑/摆动状态
 *         摆动：支撑 → 摆动
 *         支撑：摆动完成 → 支撑
 */
void gait_scheduler_update(GaitScheduler_t *sch);

#endif
