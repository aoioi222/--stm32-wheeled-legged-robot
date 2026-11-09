#include "gait_controller.h"
#include "includes.h"
#include "body_control.h"

/* 步态切换提示打印：由遥控触发，属控制周期内阻塞输出，默认关闭 */
#if ROBOT_DEBUG_PRINT
#define LOG_GAIT(...) printf(__VA_ARGS__)
#else
#define LOG_GAIT(...) ((void)0)
#endif

GaitGenerator_t gait_gen;
GaitScheduler_t gait_scheduler;
LegStateMachine_t leg_fsm[4];
FootPosition_t foot_target_pos[4];

/**
 * @brief 步态控制器初始化
 *        初始化步态生成器、贝塞尔轨迹、单腿状态机、步态调度器
 */
void gait_controller_init(void)
{
    // 步态生成器配置
    GaitGenConfig_t gait_cfg = {
        .cycle_time = CYCLE_TIME,
        .swing_ratio = SWING_RATIO,
        .step_height = STEP_HEIGHT,
        .max_step_x = MAX_STEP_X,
        .max_step_y = MAX_STEP_Y};

    gait_gen_init(&gait_gen);
    gait_gen.gait_type = GaitType;
    gait_gen.config = gait_cfg;

    // 贝塞尔轨迹配置：摆动相总时间 = 步态周期 × 摆动占空比
    BezierTrajConfig_t traj_cfg = {
        .cycle_time = CYCLE_TIME * SWING_RATIO,
        .lift_height_m = STEP_HEIGHT};

    // 四条腿状态机初始化 + 足端初始落脚点设置
    for (int i = 0; i < 4; i++)
    {
        leg_state_machine_init(&leg_fsm[i], (Leg_ID_t)i, traj_cfg);

        // 机体坐标系初始足端位置
        foot_target_pos[i].foot_pos_x_m = (i == LEG_L_F || i == LEG_R_F) ? FOOT_OFFSET_X : (-FOOT_OFFSET_X);
        foot_target_pos[i].foot_pos_y_m = (i == LEG_L_F || i == LEG_L_B) ? FOOT_OFFSET_Y : (-FOOT_OFFSET_Y);
        foot_target_pos[i].foot_pos_z_m = -FOOT_OFFSET_Z;

        // 将初始位置写入状态机各个位置缓存
        leg_fsm[i].current_target_pos = foot_target_pos[i];
        leg_fsm[i].swing_start_pos = foot_target_pos[i];
        leg_fsm[i].stance_start_pos = foot_target_pos[i];
        leg_fsm[i].stance_end_pos = foot_target_pos[i];
    }

    // 步态调度器绑定4条腿状态机与步态生成器
    LegStateMachine_t *sm_ptr[4] = {&leg_fsm[0], &leg_fsm[1], &leg_fsm[2], &leg_fsm[3]};
    gait_scheduler_init(&gait_scheduler, sm_ptr, &gait_gen);

    g_robot_state.sim_time_s = 0.0f;
    gait_controller_enter_stance(g_robot_state.sim_time_s);
}

void gait_controller_enter_stance(float now)
{
    gait_gen_update(&gait_gen, &g_robot_state);
    float stance_time = gait_gen.config.cycle_time * (1.0f - gait_gen.config.swing_ratio);
    for (int i = 0; i < 4; i++)
        leg_state_switch(&leg_fsm[i], LEG_STATE_STANCE, stance_time,
                         &gait_gen.leg_output[i].stance_end, now);
}

/**
 * @brief 步态控制器周期更新函数
 * @param abs_time 当前绝对时间，单位 s
 * @note 一般放在1kHz/500Hz主循环中调用
 */
void gait_controller_update(float abs_time)
{
    float swing_ratio = gait_gen.config.swing_ratio;

    for (int leg_id = 0; leg_id < 4; leg_id++)
    {
        LegStateMachine_t *sm = &leg_fsm[leg_id];

        // 获取该腿在整个步态周期内的归一化相位 [0~1]
        float leg_phase = gait_gen_get_single_phase(&gait_gen, leg_id);

        // 根据当前状态映射为本状态内部phase：摆动相0~1 / 支撑相0~1
        if (sm->current_state == LEG_STATE_SWING)
        {
            sm->phase = leg_phase / swing_ratio;
        }
        else if (sm->current_state == LEG_STATE_STANCE)
        {
            sm->phase = (leg_phase - swing_ratio) / (1.0f - swing_ratio);
        }
        else
        {
            sm->phase = 0.0f;
        }
        // 相位钳位，防止浮点漂移超出区间
        sm->phase = fminf(fmaxf((float)sm->phase, 0.0f), 1.0f);

        // 单腿状态机内部更新：贝塞尔轨迹插值、状态跳转判断
        leg_state_machine_update(sm, abs_time);

        // 足端位置和关节角均由腿状态机提供
        leg_get_target_pos(sm, &foot_target_pos[leg_id]);
    }
}

/**
 * @brief 步态切换接口：切换 Trot / Walk / Pronk
 * @param new_gait_type 目标步态类型枚举
 * @note 切换时会清零姿态PID积分，摆动腿强制落地，同步更新轨迹参数
 */
void gait_controller_switch_gait(GaitType_t new_gait_type)
{
    gait_gen_switch_gait(&gait_gen, new_gait_type);

    /* 步态切换清零姿态PID积分，抑制切换瞬间震荡 */
    g_pitch_pid.Integral = 0.0f;
    g_pitch_pid.Last_Error = 0.0f;
    g_roll_pid.Integral = 0.0f;
    g_roll_pid.Last_Error = 0.0f;
    g_yaw_pid.Integral = 0.0f;
    g_yaw_pid.Last_Error = 0.0f;
    g_yaw_target_initialized = 0;

    /* 将正在摆动的腿强制切为支撑相，避免步态切换时悬空失配 */
    float sync_stance_time = gait_gen.config.cycle_time * (1.0f - gait_gen.config.swing_ratio);
    for (int i = 0; i < 4; i++)
    {
        if (leg_fsm[i].current_state == LEG_STATE_SWING)
        {
            leg_state_switch(&leg_fsm[i], LEG_STATE_STANCE, sync_stance_time,
                             &leg_fsm[i].current_target_pos, g_robot_state.sim_time_s);
        }
    }

    // 更新机器人状态机子模式标记
    switch (new_gait_type)
    {
    case GAIT_TROT:
        g_robot_state.quad_sub_mode = QUAD_MODE_TROT;
        break;
    case GAIT_WALK:
        g_robot_state.quad_sub_mode = QUAD_MODE_WALK;
        break;
    case GAIT_PRONK:
        g_robot_state.quad_sub_mode = QUAD_MODE_PRONK;
        break;
    default:
        break;
    }

    // 更新所有腿贝塞尔轨迹参数：摆动周期、抬腿高度跟随新步态配置
    float new_traj_cycle = gait_gen.config.cycle_time * gait_gen.config.swing_ratio;
    for (int i = 0; i < 4; i++)
    {
        leg_fsm[i].traj_ctx.config.cycle_time = new_traj_cycle;
        leg_fsm[i].traj_ctx.config.lift_height_m = gait_gen.config.step_height;
    }

    const char *name = (new_gait_type == GAIT_TROT) ? "Trot" : (new_gait_type == GAIT_WALK) ? "Walk" : "Pronk";
    LOG_GAIT("[步态切换] -> %s (T=%.2fs, swing=%.0f%%, step_h=%.3fm)\n",
           name, gait_gen.config.cycle_time,
           gait_gen.config.swing_ratio * 100.0f,
           gait_gen.config.step_height);
}
