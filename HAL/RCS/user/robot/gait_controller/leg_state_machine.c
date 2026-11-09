#include "leg_state_machine.h"
#include "robot_state.h"
#include "body_control.h"

/* 状态切换日志：每次状态切换一条阻塞式串口打印，是控制周期内的周期负载，默认关闭 */
#if ROBOT_DEBUG_PRINT
#define LOG_STATE(sm, type, msg)                                 \
    printf("[LEG:%d][%s] %s | %s\n",                             \
           sm->leg_id, leg_state_get_name(sm->current_state), type, msg)
#else
#define LOG_STATE(sm, type, msg) ((void)0)
#endif

/* 诊断打印：只在人工调用诊断函数时输出，不随控制周期执行 */
#if ROBOT_DEBUG_PRINT
#define LOG_LEG_DIAG(...) printf(__VA_ARGS__)
#else
#define LOG_LEG_DIAG(...) ((void)0)
#endif

// ======================== Private Function Declare ========================
static void foot_lerp(const FootPosition_t *a, const FootPosition_t *b, float t, FootPosition_t *out);

static void leg_state_on_enter_hold(LegStateMachine_t *sm);
static void leg_state_on_run_hold(LegStateMachine_t *sm, float dt_s);
static void leg_state_on_exit_hold(LegStateMachine_t *sm);

static void leg_state_on_enter_stance(LegStateMachine_t *sm);
static void leg_state_on_run_stance(LegStateMachine_t *sm, float dt_s);
static void leg_state_on_exit_stance(LegStateMachine_t *sm);

static void leg_state_on_enter_swing(LegStateMachine_t *sm);
static void leg_state_on_run_swing(LegStateMachine_t *sm, float dt_s);
static void leg_state_on_exit_swing(LegStateMachine_t *sm);

static void leg_state_on_enter_error(LegStateMachine_t *sm);
static void leg_state_on_run_error(LegStateMachine_t *sm, float dt_s);
static void leg_state_on_exit_error(LegStateMachine_t *sm);

static void leg_state_on_enter_wheel(LegStateMachine_t *sm);
static void leg_state_on_run_wheel(LegStateMachine_t *sm, float dt_s);
static void leg_state_on_exit_wheel(LegStateMachine_t *sm);

static void leg_state_on_enter_fold(LegStateMachine_t *sm);
static void leg_state_on_run_fold(LegStateMachine_t *sm, float dt_s);
static void leg_state_on_exit_fold(LegStateMachine_t *sm);

static void leg_state_on_enter_unfold(LegStateMachine_t *sm);
static void leg_state_on_run_unfold(LegStateMachine_t *sm, float dt_s);
static void leg_state_on_exit_unfold(LegStateMachine_t *sm);

// 关节空间过渡辅助（轮式/折叠/展开共用）
static float leg_joint_transition_t(const LegStateMachine_t *sm, float now);
static void leg_joint_lerp_to_target(LegStateMachine_t *sm, float t);

// ======================== Public API ========================

/**
 * @brief 单腿状态机初始化
 * @param sm 状态机实例
 * @param leg_id 腿编号
 * @param config 贝塞尔摆动轨迹配置
 * @return 0成功，-1失败
 */
int leg_state_machine_init(LegStateMachine_t *sm, Leg_ID_t leg_id, BezierTrajConfig_t config)
{
    if (!sm || leg_id < LEG_R_F || leg_id > LEG_L_B)
        return -1;

    memset(sm, 0, sizeof(LegStateMachine_t));
    sm->leg_id = leg_id;
    sm->traj_ctx.leg_id = leg_id;
    sm->first_swing_done = 0;

    sm->current_state = LEG_STATE_HOLD;
    sm->last_state = LEG_STATE_HOLD;

    if (bezier_traj_controller_init(&sm->traj_ctx, leg_id, config) != 0)
    {
        sm->last_error = LEG_ERR_TRAJ_INIT_FAIL;
        return -1;
    }

    // 设置各腿上电默认站立关节角度
    switch (leg_id)
    {
    case LEG_R_F:
        sm->current_joint_angles = (LegJointAngles_t){STAND_J1, STAND_J2, -STAND_J3, 1};
        break;
    case LEG_L_F:
        sm->current_joint_angles = (LegJointAngles_t){STAND_J1, -STAND_J2, STAND_J3, 1};
        break;
    case LEG_R_B:
        sm->current_joint_angles = (LegJointAngles_t){STAND_J1, STAND_J2, -STAND_J3, 1};
        break;
    case LEG_L_B:
        sm->current_joint_angles = (LegJointAngles_t){STAND_J1, -STAND_J2, STAND_J3, 1};
        break;
    }

    // 初始足端目标位置
    sm->current_target_pos.foot_pos_x_m = 0.0f;
    sm->current_target_pos.foot_pos_y_m = 0.0f;
    sm->current_target_pos.foot_pos_z_m = -0.15f;

    sm->last_error = LEG_ERR_NONE;
    printf("[LEG:%d] leg sm init ok\n", leg_id);
    return 0;
}

/**
 * @brief 状态跳转接口：执行状态退出→切换→状态进入
 * @param sm 状态机
 * @param new_state 目标状态
 * @param duration_s 本状态持续时间
 * @param end_pos 本状态结束足端位置，可为NULL
 * @param current_time_s 当前系统时间
 * @return 0成功，-1不允许跳转/空指针
 */
int leg_state_switch(LegStateMachine_t *sm, LegState_t new_state,
                     float duration_s, const FootPosition_t *end_pos,
                     float current_time_s)
{
    if (!sm)
        return -1;
    if (!leg_state_switch_allowed(sm->current_state, new_state))
    {
        sm->last_error = LEG_ERR_INVALID_TRANS;
        return -1;
    }
    if (new_state == sm->current_state)
        return 0;

    // 执行旧状态退出回调
    switch (sm->current_state)
    {
    case LEG_STATE_HOLD:
        leg_state_on_exit_hold(sm);
        break;
    case LEG_STATE_STANCE:
        leg_state_on_exit_stance(sm);
        break;
    case LEG_STATE_SWING:
        leg_state_on_exit_swing(sm);
        break;
    case LEG_STATE_WHEEL:
        leg_state_on_exit_wheel(sm);
        break;
    case LEG_STATE_FOLD:
        leg_state_on_exit_fold(sm);
        break;
    case LEG_STATE_UNFOLD:
        leg_state_on_exit_unfold(sm);
        break;
    case LEG_STATE_ERROR:
        leg_state_on_exit_error(sm);
        break;
    }

    // 更新状态机元数据
    sm->last_state = sm->current_state;
    sm->current_state = new_state;
    sm->state_start_time_s = current_time_s;
    sm->state_duration_s = duration_s;
    sm->phase = 0.0f;

    // 摆动相：记录摆动起点、终点
    if (new_state == LEG_STATE_SWING)
    {
        sm->swing_start_pos = sm->current_target_pos;
        if (end_pos != NULL)
            sm->swing_end_pos = *end_pos;
        else
            sm->swing_end_pos = sm->current_target_pos;
    }
    // 支撑相：记录支撑起点、终点
    else if (new_state == LEG_STATE_STANCE)
    {
        sm->stance_start_pos = sm->current_target_pos;
        if (end_pos != NULL)
            sm->stance_end_pos = *end_pos;
        else
            sm->stance_end_pos = sm->current_target_pos;
    }

    // 执行新状态进入回调
    switch (new_state)
    {
    case LEG_STATE_HOLD:
        leg_state_on_enter_hold(sm);
        break;
    case LEG_STATE_STANCE:
        leg_state_on_enter_stance(sm);
        break;
    case LEG_STATE_SWING:
        leg_state_on_enter_swing(sm);
        break;
    case LEG_STATE_WHEEL:
        leg_state_on_enter_wheel(sm);
        break;
    case LEG_STATE_FOLD:
        leg_state_on_enter_fold(sm);
        break;
    case LEG_STATE_UNFOLD:
        leg_state_on_enter_unfold(sm);
        break;
    case LEG_STATE_ERROR:
        leg_state_on_enter_error(sm);
        break;
    }

    return 0;
}

/**
 * @brief 关节空间状态跳转：轮式/折叠/展开
 * @param sm 状态机
 * @param new_state 目标状态（WHEEL / FOLD / UNFOLD）
 * @param duration_s 过渡时长
 * @param target_joint_angles 目标关节角（IK规范角）
 * @param current_time_s 当前系统时间
 * @return 0成功，-1不允许跳转/空指针/目标不是关节空间状态
 * @note  起始角就是 sm->current_joint_angles，因此反复切换会从当前姿态续接，
 *        不会像原先那样每次都要重新捕捉一次起始角
 */
int leg_state_switch_joint(LegStateMachine_t *sm, LegState_t new_state,
                           float duration_s, const LegJointAngles_t *target_joint_angles,
                           float current_time_s)
{
    if (!sm || !target_joint_angles)
        return -1;
    if (new_state != LEG_STATE_WHEEL && new_state != LEG_STATE_FOLD && new_state != LEG_STATE_UNFOLD)
        return -1;

    if (leg_state_switch(sm, new_state, duration_s, NULL, current_time_s) != 0)
        return -1;

    sm->target_joint_angles = *target_joint_angles;
    sm->current_joint_angles.valid = 1;
    return 0;
}

/**
 * @brief 状态机周期更新，放在控制循环调用
 * @param sm 状态机
 * @param current_time_s 当前绝对时间
 * @return 0
 */
int leg_state_machine_update(LegStateMachine_t *sm, float current_time_s)
{
    if (!sm)
        return -1;

    // 保存当前phase到对应状态缓存
    if (sm->current_state == LEG_STATE_SWING)
        sm->swing_phase = sm->phase;
    else if (sm->current_state == LEG_STATE_STANCE)
        sm->stance_phase = sm->phase;

    float dt_s = g_robot_state.dt_s;

    // 运行当前状态运行回调
    switch (sm->current_state)
    {
    case LEG_STATE_HOLD:
        leg_state_on_run_hold(sm, dt_s);
        break;
    case LEG_STATE_STANCE:
        leg_state_on_run_stance(sm, dt_s);
        break;
    case LEG_STATE_SWING:
        leg_state_on_run_swing(sm, dt_s);
        break;
    case LEG_STATE_WHEEL:
        leg_state_on_run_wheel(sm, dt_s);
        break;
    case LEG_STATE_FOLD:
        leg_state_on_run_fold(sm, dt_s);
        break;
    case LEG_STATE_UNFOLD:
        leg_state_on_run_unfold(sm, dt_s);
        break;
    case LEG_STATE_ERROR:
        leg_state_on_run_error(sm, dt_s);
        break;
    }

    return 0;
}

/**
 * @brief 获取状态机输出目标关节角度
 */
void leg_get_target_angles(const LegStateMachine_t *sm, LegJointAngles_t *out)
{
    if (sm && out)
        *out = sm->current_joint_angles;
}

/**
 * @brief 获取状态机输出目标足端位置
 */
void leg_get_target_pos(const LegStateMachine_t *sm, FootPosition_t *out)
{
    if (sm && out)
        *out = sm->current_target_pos;
}

/**
 * @brief 状态枚举转字符串，打印调试
 */
const char *leg_state_get_name(LegState_t s)
{
    switch (s)
    {
    case LEG_STATE_HOLD:
        return "HOLD";
    case LEG_STATE_STANCE:
        return "STANCE";
    case LEG_STATE_SWING:
        return "SWING";
    case LEG_STATE_WHEEL:
        return "WHEEL";
    case LEG_STATE_FOLD:
        return "FOLD";
    case LEG_STATE_UNFOLD:
        return "UNFOLD";
    case LEG_STATE_ERROR:
        return "ERROR";
    default:
        return "???";
    }
}

/**
 * @brief 错误码转可读字符串
 */
const char *leg_error_str(LegErrorCode_t e)
{
    static const char *table[] = {
        [LEG_ERR_NONE] = "-",
        [LEG_ERR_TRAJ_INIT_FAIL] = "traj init fail",
        [LEG_ERR_INVALID_TRANS] = "no trans",
        [LEG_ERR_STANCE_IK_FAIL] = "Stance IK fail",
        [LEG_ERR_SWING_TRAJ_FAIL] = "Swing traj err",
    };
    if (e >= sizeof(table) / sizeof(table[0]) || !table[e])
        return "?";
    return table[e];
}

/**
 * @brief 打印单腿完整状态，用于调试
 */
void leg_state_machine_print(const LegStateMachine_t *sm)
{
    if (!sm)
    {
        LOG_LEG_DIAG("sm null\n");
        return;
    }

    LOG_LEG_DIAG("=== LEG %d ===\n", sm->leg_id);
    LOG_LEG_DIAG("State: %s  Ph:%.2f\n",
                 leg_state_get_name(sm->current_state), sm->phase);
    LOG_LEG_DIAG("Foot: %.2f %.2f %.2f\n",
                 sm->current_target_pos.foot_pos_x_m,
                 sm->current_target_pos.foot_pos_y_m,
                 sm->current_target_pos.foot_pos_z_m);
    LOG_LEG_DIAG("Joint: %.1f %.1f %.1f\n",
                 sm->current_joint_angles.joint1_deg,
                 sm->current_joint_angles.joint2_deg,
                 sm->current_joint_angles.joint3_deg);
    LOG_LEG_DIAG("Msg: %s\n", leg_error_str(sm->last_error));
    LOG_LEG_DIAG("===============\n");
}

/**
 * @brief 复位状态机到 HOLD
 */
void leg_state_machine_reset(LegStateMachine_t *sm)
{
    if (sm)
        leg_state_switch(sm, LEG_STATE_HOLD, 0.0f, NULL, g_robot_state.sim_time_s);
}

// ======================== Private Helpers ========================
/**
 * @brief 关节空间过渡的归一化进度
 * @param sm 状态机
 * @param now 当前绝对时间(s)
 * @return 0~1 的插值系数，未开始返回0，超时返回1
 */
static float leg_joint_transition_t(const LegStateMachine_t *sm, float now)
{
    if (sm->state_duration_s <= 0.0f)
    {
        return 1.0f;
    }
    float t = (float)((now - sm->state_start_time_s) / sm->state_duration_s);
    if (t > 1.0f)
    {
        t = 1.0f;
    }
    if (t < 0.0f)
    {
        t = 0.0f;
    }
    return t;
}

/**
 * @brief 三个关节角按系数线性插值
 * @param sm 状态机
 * @param t  插值系数 0~1
 */
static void leg_joint_lerp_to_target(LegStateMachine_t *sm, float t)
{
    float s1 = sm->current_joint_angles.joint1_deg;
    float s2 = sm->current_joint_angles.joint2_deg;
    float s3 = sm->current_joint_angles.joint3_deg;
    const LegJointAngles_t *d = &sm->target_joint_angles;

    sm->current_joint_angles.joint1_deg = s1 + t * (d->joint1_deg - s1);
    sm->current_joint_angles.joint2_deg = s2 + t * (d->joint2_deg - s2);
    sm->current_joint_angles.joint3_deg = s3 + t * (d->joint3_deg - s3);
    sm->current_joint_angles.valid = 1;
}

/**
 * @brief 足端线性插值，支撑相使用，附带Z向补偿
 * @param a 起点
 * @param b 终点
 * @param t [0~1]插值系数
 * @param out 输出足端位置
 */
static void foot_lerp(const FootPosition_t *a, const FootPosition_t *b, float t, FootPosition_t *out)
{
    out->foot_pos_x_m = a->foot_pos_x_m + t * (b->foot_pos_x_m - a->foot_pos_x_m);
    out->foot_pos_y_m = a->foot_pos_y_m + t * (b->foot_pos_y_m - a->foot_pos_y_m);
    out->foot_pos_z_m = a->foot_pos_z_m + t * (b->foot_pos_z_m - a->foot_pos_z_m) + t * STANCE_Z_COMPENSATION;
}

// ======================== HOLD 待机保持状态 ========================
static void leg_state_on_enter_hold(LegStateMachine_t *sm)
{
    LOG_STATE(sm, "ENTER", "hold");
    memset(&sm->traj_ctx, 0, sizeof(LegTrajContext_t));
    sm->is_trajectory_finished = 1;
    sm->ik_fail_count = 0;
    sm->swing_phase = 0.0f;
    sm->stance_phase = 0.0f;
    sm->last_error = LEG_ERR_NONE;
}

static void leg_state_on_run_hold(LegStateMachine_t *sm, float dt_s)
{
    // HOLD状态：保持当前关节角度，不做轨迹运动
}

static void leg_state_on_exit_hold(LegStateMachine_t *sm)
{
    LOG_STATE(sm, "EXIT", "hold");
}

// ======================== STANCE 支撑相 ========================
static void leg_state_on_enter_stance(LegStateMachine_t *sm)
{
    LOG_STATE(sm, "ENTER", "stance");
    sm->is_trajectory_finished = 0;
    sm->ik_fail_count = 0;
    sm->stance_phase = 0.0f;
}

static void leg_state_on_run_stance(LegStateMachine_t *sm, float dt_s)
{
    float phase = (float)fmin(sm->phase, 1.0f);

    // 支撑相足端线性插值
    foot_lerp(&sm->stance_start_pos, &sm->stance_end_pos, phase, &sm->current_target_pos);
    // 叠加机身姿态修正（滚转俯仰）
    body_apply_to_stance_leg(sm);

    LegJointAngles_t prev_angles = sm->current_joint_angles;
    IKJacobianError_t err = leg_ik_solve(sm->leg_id, &sm->current_target_pos,
                                        &sm->current_joint_angles, IK_MODE_GEOMETRY);

    if (err != IK_JACOBIAN_SUCCESS || sm->current_joint_angles.valid != 1)
    {
        // IK求解失败，保持上一帧角度
        sm->current_joint_angles = prev_angles;
        sm->ik_fail_count++;
        sm->last_error = LEG_ERR_STANCE_IK_FAIL;
    }
    else
    {
        sm->ik_fail_count = 0;
    }

    if (sm->phase >= 0.99f)
        sm->is_trajectory_finished = 1;
}

static void leg_state_on_exit_stance(LegStateMachine_t *sm)
{
    LOG_STATE(sm, "EXIT", "stance -> swing");
}

// ======================== SWING 摆动相（贝塞尔轨迹） ========================
static void leg_state_on_enter_swing(LegStateMachine_t *sm)
{
    LOG_STATE(sm, "ENTER", "swing");
    FootPosition_t traj_start;

    if (sm->first_swing_done == 0)
    {
        // 第一次摆动：从当前实际足端位置开始
        traj_start = sm->current_target_pos;
        sm->first_swing_done = 1;
    }
    else
    {
        // 后续摆动：使用状态切换保存的摆动起点
        traj_start = sm->swing_start_pos;
    }

    // 生成贝塞尔摆动轨迹
    generate_leg_bezier_traj(&sm->traj_ctx, traj_start, sm->swing_end_pos);

    sm->is_trajectory_finished = 0;
    sm->ik_fail_count = 0;
    sm->swing_phase = 0.0f;
    sm->traj_ctx.is_trajectory_running = 1;
    sm->last_error = LEG_ERR_NONE;
}

static void leg_state_on_run_swing(LegStateMachine_t *sm, float dt_s)
{
    float t = (float)(sm->phase * sm->state_duration_s);
    float v[3] = {0}, a[3] = {0};

    TrajStatus_t status = get_leg_joint_cmd(&sm->traj_ctx, t,
                                            &sm->current_joint_angles, v, a);

    sm->current_target_pos = sm->traj_ctx.runtime.last_foot_pos;
    sm->last_traj_status = status;

    if (status == TRAJ_STATUS_END)
    {
        sm->is_trajectory_finished = 1;
        sm->current_target_pos = sm->swing_end_pos;
        LOG_STATE(sm, "INFO", "swing done");
    }
    else if (status == TRAJ_STATUS_ERROR_IK || status == TRAJ_STATUS_ERROR_INPUT)
    {
        sm->ik_fail_count++;
        sm->last_error = LEG_ERR_SWING_TRAJ_FAIL;
    }
    else
    {
        sm->ik_fail_count = 0;
    }
}

static void leg_state_on_exit_swing(LegStateMachine_t *sm)
{
    sm->traj_ctx.is_trajectory_running = 0;
    LOG_STATE(sm, "EXIT", "swing -> stance");
}

// ======================== ERROR 错误状态 ========================
static void leg_state_on_enter_error(LegStateMachine_t *sm)
{
    LOG_STATE(sm, "FAULT", "ERROR state");
    memset(&sm->traj_ctx, 0, sizeof(LegTrajContext_t));
    sm->is_trajectory_finished = 1;
    sm->ik_fail_count = 0;
    sm->last_error = LEG_ERR_NONE;
}

static void leg_state_on_run_error(LegStateMachine_t *sm, float dt_s)
{
    // 错误状态，停止运动，等待切回HOLD复位
}

static void leg_state_on_exit_error(LegStateMachine_t *sm)
{
    LOG_STATE(sm, "RESET", "exit error");
    sm->ik_fail_count = 0;
    sm->last_error = LEG_ERR_NONE;
}

// ======================== WHEEL 轮式模式 ========================
static void leg_state_on_enter_wheel(LegStateMachine_t *sm)
{
    LOG_STATE(sm, "ENTER", "wheel");
    sm->target_joint_angles = leg_wheel_mode_angles(sm->leg_id);
    sm->current_joint_angles.valid = 1;
    sm->last_error = LEG_ERR_NONE;
}

static void leg_state_on_run_wheel(LegStateMachine_t *sm, float dt_s)
{
    leg_joint_lerp_to_target(sm, leg_joint_transition_t(sm, g_robot_state.sim_time_s));
}

static void leg_state_on_exit_wheel(LegStateMachine_t *sm)
{
    LOG_STATE(sm, "EXIT", "wheel");
}

// ======================== FOLD 腿部折叠 ========================
static void leg_state_on_enter_fold(LegStateMachine_t *sm)
{
    LOG_STATE(sm, "ENTER", "fold");
    sm->target_joint_angles = leg_fold_mode_angles(sm->leg_id);
    sm->current_joint_angles.valid = 1;
    sm->last_error = LEG_ERR_NONE;
}

static void leg_state_on_run_fold(LegStateMachine_t *sm, float dt_s)
{
    leg_joint_lerp_to_target(sm, leg_joint_transition_t(sm, g_robot_state.sim_time_s));
}

static void leg_state_on_exit_fold(LegStateMachine_t *sm)
{
    LOG_STATE(sm, "EXIT", "fold");
}

// ======================== UNFOLD 腿部展开 ========================
static void leg_state_on_enter_unfold(LegStateMachine_t *sm)
{
    LOG_STATE(sm, "ENTER", "unfold");
    sm->target_joint_angles = leg_stand_mode_angles(sm->leg_id);
    sm->phase = 0.0f;
    sm->current_joint_angles.valid = 1;
    sm->last_error = LEG_ERR_NONE;
}

static void leg_state_on_run_unfold(LegStateMachine_t *sm, float dt_s)
{
    float t = leg_joint_transition_t(sm, g_robot_state.sim_time_s);
    leg_joint_lerp_to_target(sm, t);
    sm->phase = t;

    if (t >= 1.0f)
    {
        LOG_STATE(sm, "INFO", "unfold done -> STANCE");

        bool front = sm->leg_id == LEG_R_F || sm->leg_id == LEG_L_F;
        bool left = sm->leg_id == LEG_L_F || sm->leg_id == LEG_L_B;
        FootPosition_t stance_end = {
            front ? FOOT_OFFSET_X : -FOOT_OFFSET_X,
            left ? FOOT_OFFSET_Y : -FOOT_OFFSET_Y,
            -FOOT_OFFSET_Z
        };
        sm->current_target_pos = stance_end;

        leg_state_switch(sm, LEG_STATE_STANCE, 0.5f, &stance_end, g_robot_state.sim_time_s);
    }
}

static void leg_state_on_exit_unfold(LegStateMachine_t *sm)
{
    LOG_STATE(sm, "EXIT", "unfold");
}
