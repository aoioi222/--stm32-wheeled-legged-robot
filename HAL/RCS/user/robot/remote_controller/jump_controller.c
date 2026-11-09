#include "jump_controller.h"

/* 跳跃阶段提示打印：每次阶段切换一条阻塞式串口打印，默认关闭 */
#if ROBOT_DEBUG_PRINT
#define LOG_JUMP(...) printf(__VA_ARGS__)
#else
#define LOG_JUMP(...) ((void)0)
#endif

// 跳跃阶段时长已集中到 robot_config.h（JUMP_CROUCH_DUR 等）

// 站立姿态关节角度（恢复目标，值定义在 robot_config.h）
static const LegJointAngles_t STAND_ANGLES[4] = {
    [LEG_R_F] = {STAND_J1,  STAND_J2, -STAND_J3, 1},
    [LEG_L_F] = {STAND_J1, -STAND_J2,  STAND_J3, 1},
    [LEG_R_B] = {STAND_J1,  STAND_J2, -STAND_J3, 1},
    [LEG_L_B] = {STAND_J1, -STAND_J2,  STAND_J3, 1},
};

// 下蹲蓄力：前腿深蹲前倾，后腿浅蹲保持后高 → 重心前移
static const LegJointAngles_t CROUCH_ANGLES[4] = {
    [LEG_R_F] = {0.0f,  JUMP_CROUCH_J2_F, -JUMP_CROUCH_J3_F, 1},
    [LEG_L_F] = {0.0f, -JUMP_CROUCH_J2_F,  JUMP_CROUCH_J3_F, 1},
    [LEG_R_B] = {0.0f,  JUMP_CROUCH_J2_R, -JUMP_CROUCH_J3_R, 1},
    [LEG_L_B] = {0.0f, -JUMP_CROUCH_J2_R,  JUMP_CROUCH_J3_R, 1},
};

// 蹬地前冲：后腿爆发伸直向后下方蹬 → 反作用力推身体前上方
static const LegJointAngles_t PUSH_OFF_ANGLES[4] = {
    [LEG_R_F] = {0.0f,  JUMP_PUSH_J2_F, -JUMP_PUSH_J3_F, 1},
    [LEG_L_F] = {0.0f, -JUMP_PUSH_J2_F,  JUMP_PUSH_J3_F, 1},
    [LEG_R_B] = {0.0f,  JUMP_PUSH_J2_R, -JUMP_PUSH_J3_R, 1},
    [LEG_L_B] = {0.0f, -JUMP_PUSH_J2_R,  JUMP_PUSH_J3_R, 1},
};

// 空中摆腿：前腿前伸准备落地，后腿收拢
static const LegJointAngles_t AERIAL_ANGLES[4] = {
    [LEG_R_F] = {0.0f,  JUMP_AERIAL_J2_F, -JUMP_AERIAL_J3_F, 1},
    [LEG_L_F] = {0.0f, -JUMP_AERIAL_J2_F,  JUMP_AERIAL_J3_F, 1},
    [LEG_R_B] = {0.0f,  JUMP_AERIAL_J2_R, -JUMP_AERIAL_J3_R, 1},
    [LEG_L_B] = {0.0f, -JUMP_AERIAL_J2_R,  JUMP_AERIAL_J3_R, 1},
};

// 落地缓冲：前腿深蹲吸震（先触地），后腿中度缓冲
static const LegJointAngles_t LANDING_ANGLES[4] = {
    [LEG_R_F] = {0.0f,  JUMP_LANDING_J2_F, -JUMP_LANDING_J3_F, 1},
    [LEG_L_F] = {0.0f, -JUMP_LANDING_J2_F,  JUMP_LANDING_J3_F, 1},
    [LEG_R_B] = {0.0f,  JUMP_LANDING_J2_R, -JUMP_LANDING_J3_R, 1},
    [LEG_L_B] = {0.0f, -JUMP_LANDING_J2_R,  JUMP_LANDING_J3_R, 1},
};

JumpController_t g_jump_ctrl;

// 两角度线性插值
static void angles_lerp(const LegJointAngles_t *a, const LegJointAngles_t *b,
                        float t, LegJointAngles_t *out)
{
    out->joint1_deg = a->joint1_deg + t * (b->joint1_deg - a->joint1_deg);
    out->joint2_deg = a->joint2_deg + t * (b->joint2_deg - a->joint2_deg);
    out->joint3_deg = a->joint3_deg + t * (b->joint3_deg - a->joint3_deg);
    out->valid = 1;
}

// 获取当前阶段的起止角度
static void get_phase_angles(JumpPhase_t phase,
                             const LegJointAngles_t **start,
                             const LegJointAngles_t **end)
{
    switch (phase)
    {
    case JUMP_CROUCH:
        *start = STAND_ANGLES;
        *end = CROUCH_ANGLES;
        break;
    case JUMP_PUSH_OFF:
        *start = CROUCH_ANGLES;
        *end = PUSH_OFF_ANGLES;
        break;
    case JUMP_AERIAL:
        *start = PUSH_OFF_ANGLES;
        *end = AERIAL_ANGLES;
        break;
    case JUMP_LANDING:
        *start = AERIAL_ANGLES;
        *end = LANDING_ANGLES;
        break;
    case JUMP_RECOVER:
        *start = LANDING_ANGLES;
        *end = STAND_ANGLES;
        break;
    default:
        *start = STAND_ANGLES;
        *end = STAND_ANGLES;
        break;
    }
}

static float get_phase_duration(JumpPhase_t phase)
{
    switch (phase)
    {
    case JUMP_CROUCH:
        return JUMP_CROUCH_DUR;
    case JUMP_PUSH_OFF:
        return JUMP_PUSH_OFF_DUR;
    case JUMP_AERIAL:
        return JUMP_AERIAL_DUR;
    case JUMP_LANDING:
        return JUMP_LANDING_DUR;
    case JUMP_RECOVER:
        return JUMP_RECOVER_DUR;
    default:
        return 0.0f;
    }
}

void jump_controller_init(JumpController_t *jc)
{
    memset(jc, 0, sizeof(JumpController_t));
    jc->phase = JUMP_IDLE;
    jc->active = 0;
    jc->completed = 0;
    for (int i = 0; i < 4; i++)
        jc->output_joints[i] = STAND_ANGLES[i];
}

void jump_controller_trigger(JumpController_t *jc, float current_time)
{
    if (jc->active)
        return; // 已在跳跃中，忽略重复触发
    jc->phase = JUMP_CROUCH;
    jc->phase_start_time = current_time;
    jc->active = 1;
    jc->completed = 0;
    LOG_JUMP("[跳跃] 触发！开始下蹲蓄力...\n");
}

void jump_controller_update(JumpController_t *jc, float current_time)
{
    if (!jc->active || jc->phase == JUMP_DONE)
        return;

    float elapsed = current_time - jc->phase_start_time;
    float duration = get_phase_duration(jc->phase);

    if (elapsed >= duration)
    {
        // 进入下一阶段
        switch (jc->phase)
        {
        case JUMP_CROUCH:
            jc->phase = JUMP_PUSH_OFF;
            LOG_JUMP("[跳跃] 蹬地腾空！\n");
            break;
        case JUMP_PUSH_OFF:
            jc->phase = JUMP_AERIAL;
            LOG_JUMP("[跳跃] 空中调整...\n");
            break;
        case JUMP_AERIAL:
            jc->phase = JUMP_LANDING;
            LOG_JUMP("[跳跃] 落地缓冲！\n");
            break;
        case JUMP_LANDING:
            jc->phase = JUMP_RECOVER;
            LOG_JUMP("[跳跃] 恢复站立...\n");
            break;
        case JUMP_RECOVER:
            jc->phase = JUMP_DONE;
            jc->active = 0;
            jc->completed = 1;
            LOG_JUMP("[跳跃] 完成！\n");
            return;
        default:
            break;
        }
        jc->phase_start_time = current_time;
        elapsed = 0.0f;
        duration = get_phase_duration(jc->phase);
    }

    // 插值计算当前帧的目标角度
    float t = (duration > 0.0f) ? (float)(elapsed / duration) : 1.0f;
    t = fminf(fmaxf(t, 0.0f), 1.0f);

    const LegJointAngles_t *start, *end;
    get_phase_angles(jc->phase, &start, &end);
    for (int i = 0; i < 4; i++)
        angles_lerp(&start[i], &end[i], t, &jc->output_joints[i]);
}

uint8_t jump_controller_is_active(const JumpController_t *jc)
{
    return jc->active && jc->phase != JUMP_DONE;
}
