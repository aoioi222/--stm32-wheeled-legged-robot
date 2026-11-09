/**
 * @brief 四足机器人步态调度器实现
 * @details 连接步态发生器与腿部状态机，根据步态计算结果控制腿部状态切换
 */

#include "gait_scheduler.h"

/**
 * @brief 初始化步态调度器
 * @param sch 步态调度器上下文指针
 * @param leg_sm 四条腿的状态机指针数组
 * @param gen 步态发生器指针
 * @return 0-初始化成功，-1-参数错误
 */
int gait_scheduler_init(GaitScheduler_t *sch, LegStateMachine_t *leg_sm[4], GaitGenerator_t *gen)
{
    if (sch == NULL || leg_sm == NULL || gen == NULL)
        return -1;

    memset(sch, 0, sizeof(GaitScheduler_t));
    sch->gait_gen = gen; // 建立步态调度器与步态发生器的关联

    for (int i = 0; i < 4; i++)
    {
        if (leg_sm[i] == NULL)
            return -1;
        sch->leg_sm[i] = leg_sm[i]; // 建立步态调度器与腿部状态机的关联
    }
    return 0;
}

void gait_scheduler_update(GaitScheduler_t *sch)
{
    if (g_robot_state.mode != ROBOT_MODE_QUADRUPED)
        return;

    for (int i = 0; i < 4; i++)
    {
        LegStateMachine_t *sm = sch->leg_sm[i];
        LegGaitOutput_t gait_out;
        gait_gen_get_leg_target(sch->gait_gen, i, &gait_out);

        // 支撑 → 摆动：仅当前为支撑相时触发
        if (gait_out.need_swing)
        {
            if (sm->current_state == LEG_STATE_STANCE)
            {
                // 传入摆动时长和摆动终点
                leg_state_switch(sm, LEG_STATE_SWING,
                                 gait_out.swing_duration_s,
                                 &gait_out.swing_end,
                                 g_robot_state.sim_time_s);
            }
        }
        // 摆动 → 支撑：轨迹完成后触发
        else
        {
            if (sm->current_state == LEG_STATE_SWING)
            {
                // 传入支撑时长和支撑终点（即下次摆动起点）
                leg_state_switch(sm, LEG_STATE_STANCE,
                                 gait_out.stance_duration_s,
                                 &gait_out.stance_end,
                                 g_robot_state.sim_time_s);
            }
        }
    }
}
