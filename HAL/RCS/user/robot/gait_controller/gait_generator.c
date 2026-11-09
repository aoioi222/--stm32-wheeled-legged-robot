#include "gait_generator.h"
#include "body_control.h"
// 根据机器人速度，算出每条腿这一周期该迈多大步、什么时候抬腿、什么时候落地，输出给状态机去执行。
// 对角小跑（TROT）默认运动参数（值定义在 robot_config.h）
const GaitGenConfig_t TROT_CONFIG = {
    .cycle_time = TROT_CYCLE_TIME,
    .swing_ratio = TROT_SWING_RATIO,
    .step_height = TROT_STEP_HEIGHT,
    .max_step_x = TROT_MAX_STEP_X,
    .max_step_y = TROT_MAX_STEP_Y};

// 慢走四拍步态（WALK）默认参数
const GaitGenConfig_t WALK_CONFIG = {
    .cycle_time = WALK_CYCLE_TIME,
    .swing_ratio = WALK_SWING_RATIO,
    .step_height = WALK_STEP_HEIGHT,
    .max_step_x = WALK_MAX_STEP_X,
    .max_step_y = WALK_MAX_STEP_Y};

// 四腿同时跳（PRONK）默认参数
const GaitGenConfig_t PRONK_CONFIG = {
    .cycle_time = PRONK_CYCLE_TIME,
    .swing_ratio = PRONK_SWING_RATIO,
    .step_height = PRONK_STEP_HEIGHT,
    .max_step_x = PRONK_MAX_STEP_X,
    .max_step_y = PRONK_MAX_STEP_Y};

//  各步态相位偏移表（0-RF, 1-LF, 2-RB, 3-LB）
//                              RF    LF    RB    LB
const float trot_phase[4] = {0.0f, 0.5f, 0.5f, 0.0f};
const float walk_phase[4] = {0.0f, 0.25f, 0.75f, 0.5f};
const float pronk_phase[4] = {0.0f, 0.0f, 0.0f, 0.0f};
const float pace_phase[4] = {0.0f, 0.5f, 0.0f, 0.5f};
const float bound_phase[4] = {0.0f, 0.0f, 0.5f, 0.5f};

void gait_gen_init(GaitGenerator_t *gen)
{
    memset(gen, 0, sizeof(GaitGenerator_t));
    gen->config = TROT_CONFIG;
}

void gait_gen_update(GaitGenerator_t *gen, const RobotGlobalState_t *robot)
{
    // 全局归一化相位 [0,1)：把当前时间对周期取余，除以周期 → 归一化为 0 ~1
    gen->global_phase = fmodf(robot->sim_time_s, gen->config.cycle_time) / gen->config.cycle_time;

    // 计算单步步长（速度 × 周期）并限幅
    float step_x = robot->cmd_vel_x * gen->config.cycle_time;
    float step_y = robot->cmd_vel_y * gen->config.cycle_time;
    step_x = fminf(fmaxf(step_x, -gen->config.max_step_x), gen->config.max_step_x);
    step_y = fminf(fmaxf(step_y, -gen->config.max_step_y), gen->config.max_step_y);

    /* rate-limit step length to avoid foot landing jump on abrupt cmd_vel change */
    static float prev_step_x = 0.0f;
    static float prev_step_y = 0.0f;
    float dstep_x = step_x - prev_step_x;
    float dstep_y = step_y - prev_step_y;
    if (dstep_x > STEP_RATE_LIMIT) dstep_x = STEP_RATE_LIMIT;
    if (dstep_x < -STEP_RATE_LIMIT) dstep_x = -STEP_RATE_LIMIT;
    if (dstep_y > STEP_RATE_LIMIT) dstep_y = STEP_RATE_LIMIT;
    if (dstep_y < -STEP_RATE_LIMIT) dstep_y = -STEP_RATE_LIMIT;
    step_x = prev_step_x + dstep_x;
    step_y = prev_step_y + dstep_y;
    prev_step_x = step_x;
    prev_step_y = step_y;

    // 计算支撑/摆动时长
    float swing_time = gen->config.cycle_time * gen->config.swing_ratio;
    // t1=T*rate
    float stance_time = gen->config.cycle_time - swing_time;
    // t2=T-t1

    // 偏航差步：前馈（转向速率指令）+ 反馈（航向保持PD）
    float yaw_feedforward = robot->cmd_vel_yaw * FOOT_OFFSET_Y * gen->config.cycle_time;
    float yaw_feedback = body_get_yaw_correction();
    float yaw_diff = yaw_feedforward + yaw_feedback;

    const float *phase_offset;
    switch (gen->gait_type)
    {
    case GAIT_TROT:
        phase_offset = trot_phase;
        break;
    case GAIT_WALK:
        phase_offset = walk_phase;
        break;
    case GAIT_PRONK:
        phase_offset = pronk_phase;
        break;
    case GAIT_PACE:
        phase_offset = pace_phase;
        break;
    case GAIT_BOUND:
        phase_offset = bound_phase;
        break;
    default:
        phase_offset = trot_phase;
        break;
    }

    // 遍历四条腿生成目标
    for (int i = 0; i < 4; i++)
    {
        LegGaitOutput_t *out = &gen->leg_output[i];
        float leg_phase = gait_gen_get_single_phase(gen, i);
        // 基准点：髋关节正下方
        FootPosition_t base_pos;
        base_pos.foot_pos_x_m = (i == LEG_R_F || i == LEG_L_F) ? FOOT_OFFSET_X : -FOOT_OFFSET_X;
        base_pos.foot_pos_y_m = (i == LEG_R_F || i == LEG_R_B) ? -FOOT_OFFSET_Y : FOOT_OFFSET_Y;
        base_pos.foot_pos_z_m = HIP_OFFSET_Z_R_B - FOOT_OFFSET_Z; // 调整参数

        // 偏航差步：正差步 → 右腿迈更大、左腿更小 → 左转（纠正右偏）
        // 注意：CMD_V_X 需保证 step_x 不饱和 max_step_x，否则 yaw 修正会被 clamp 压回
        float leg_step_x = step_x;
        if (i == LEG_R_F || i == LEG_R_B)
            leg_step_x -= yaw_diff; // 右腿：正差步→迈更大→左转
        else
            leg_step_x += yaw_diff; // 左腿：正差步→迈更小→左转
        leg_step_x = fminf(fmaxf(leg_step_x, -gen->config.max_step_x), gen->config.max_step_x);

        // 对称步长分配：落点在前半步长，抬脚点在后半步长
        out->swing_end = base_pos;
        out->swing_end.foot_pos_x_m += 0.5f * leg_step_x;
        out->swing_end.foot_pos_y_m += 0.5f * step_y;

        // 摆动起点（抬脚点）= 基准 - 半步长（后方）
        out->swing_start = base_pos;
        out->swing_start.foot_pos_x_m -= 0.5f * leg_step_x;
        out->swing_start.foot_pos_y_m -= 0.5f * step_y;
        // 支撑相起止点与摆动相闭合
        out->stance_start = out->swing_end; // 支撑起点 = 摆动终点（落地）
        out->stance_end = out->swing_start; // 支撑终点 = 摆动起点（抬脚）

        out->swing_duration_s = swing_time;
        out->stance_duration_s = stance_time;

        // 相位 < 摆动占比 → 应处于摆动相
        out->need_swing = (leg_phase < gen->config.swing_ratio) ? 1 : 0;
    }
}

void gait_gen_get_leg_target(GaitGenerator_t *gen, Leg_ID_t leg_id, LegGaitOutput_t *out)
{
    if (out == NULL || leg_id >= 4)
        return;
    *out = gen->leg_output[leg_id];
}

float gait_gen_get_single_phase(GaitGenerator_t *gen, int leg_id)
{
    if (gen == NULL || leg_id < 0 || leg_id >= 4)
        return 0.0f;

    const float *phase_offset;
    switch (gen->gait_type)
    {
    case GAIT_TROT:
        phase_offset = trot_phase;
        break;
    case GAIT_WALK:
        phase_offset = walk_phase;
        break;
    case GAIT_PRONK:
        phase_offset = pronk_phase;
        break;
    case GAIT_PACE:
        phase_offset = pace_phase;
        break;
    case GAIT_BOUND:
        phase_offset = bound_phase;
        break;
    default: // 兜底 Trot
        phase_offset = trot_phase;
        break;
    }
    return fmodf(gen->global_phase + phase_offset[leg_id], 1.0f);
}

void gait_gen_switch_gait(GaitGenerator_t *gen, GaitType_t new_type)
{
    if (!gen)
        return;
    gen->gait_type = new_type;
    switch (new_type)
    {
    case GAIT_TROT:
        gen->config = TROT_CONFIG;
        break;
    case GAIT_WALK:
        gen->config = WALK_CONFIG;
        break;
    case GAIT_PRONK:
        gen->config = PRONK_CONFIG;
        break;
    default:
        break;
    }
}
