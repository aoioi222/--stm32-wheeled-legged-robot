#include "bezier_foot_tra.h"

/* 轨迹生成调试打印：每次进入摆动相触发一次，属于控制周期内负载，默认关闭 */
#if ROBOT_DEBUG_PRINT
#define LOG_TRAJ(...) printf(__VA_ARGS__)
#else
#define LOG_TRAJ(...) ((void)0)
#endif

// 更新运行时状态（内部工具函数）
static inline void update_runtime(LegTrajContext_t *ctx, FootPosition_t *pos,
                                  LegJointAngles_t *angles, float time)
{
    if (!ctx || !pos || !angles)
        return;

    ctx->runtime.last_foot_pos = *pos;
    ctx->runtime.last_joint_angles = *angles;
    ctx->runtime.last_time = time;
}

// 三次贝塞尔曲线单轴插值
static float bezier3(float p0, float p1, float p2, float p3, float t)
{
    float t2 = t * t;
    float t3 = t2 * t;
    float mt = 1.0f - t;
    float mt2 = mt * mt;
    float mt3 = mt2 * mt;
    return mt3 * p0 + 3.0f * mt2 * t * p1 + 3.0f * mt * t2 * p2 + t3 * p3;
}

/**
 * @brief  轨迹控制器初始化
 * @return 0成功，-1失败
 */
int bezier_traj_controller_init(LegTrajContext_t *ctx, Leg_ID_t leg_id, BezierTrajConfig_t config)
{
    if (ctx == NULL || leg_id < LEG_R_F || leg_id > LEG_L_B || config.cycle_time <= TIME_EPSILON)
        return -1;

    memset(ctx, 0, sizeof(LegTrajContext_t));
    ctx->leg_id = leg_id;
    ctx->config = config;
    return 0;
}

/**
 * @brief  离线生成三次贝塞尔摆动轨迹
 * @param  ctx        轨迹上下文
 * @param  start_pos  起点足端位置（机体坐标系）
 * @param  end_pos    终点足端位置（机体坐标系）
 * @out 离线生成的轨迹点
 */
void generate_leg_bezier_traj(LegTrajContext_t *ctx, FootPosition_t start_pos, FootPosition_t end_pos)
{
    if (ctx == NULL)
        return;

    // 上一帧IK失败则从上次有效位置开始，避免轨迹跳变
    if (ctx->runtime.last_joint_angles.valid == 1)
    {
        start_pos = ctx->runtime.last_foot_pos;
        ctx->runtime.last_joint_angles.valid = 0;
    }

    memset(&ctx->runtime, 0, sizeof(BezierTrajRuntime_t));
    ctx->runtime.last_foot_pos = start_pos;

    uint16_t N = MAX_TRAJECTORY_POINTS;
    ctx->data.point_num = N;

    // 计算贝塞尔控制点
    float mid_x = 0.5f * (start_pos.foot_pos_x_m + end_pos.foot_pos_x_m);
    float mid_y = 0.5f * (start_pos.foot_pos_y_m + end_pos.foot_pos_y_m);
    float lift_z = CONFIG_DEFAULT_LIFT_HEIGHT; // 控制抬腿高度，后续可以添加足端距离的动态控制

    // X轴控制点：前后方向平滑过渡
    float px[4] = {
        start_pos.foot_pos_x_m,
        mid_x,
        mid_x,
        end_pos.foot_pos_x_m};
    // Y轴控制点：左右方向平滑过渡
    float py[4] = {
        start_pos.foot_pos_y_m,
        mid_y,
        mid_y,
        end_pos.foot_pos_y_m};
    // Z轴控制点：拱形抬升，两端切向水平，落地无冲击
    float pz[4] = {
        start_pos.foot_pos_z_m,
        start_pos.foot_pos_z_m + lift_z,
        end_pos.foot_pos_z_m + lift_z,
        end_pos.foot_pos_z_m};

    // 生成离散轨迹点
    for (int i = 0; i < N; i++)
    {
        float t = (float)i / (N - 1); // 归一化时间 0~1

        ctx->data.point_list[i].foot_pos_x_m = bezier3(px[0], px[1], px[2], px[3], t);
        ctx->data.point_list[i].foot_pos_y_m = bezier3(py[0], py[1], py[2], py[3], t);
        ctx->data.point_list[i].foot_pos_z_m = bezier3(pz[0], pz[1], pz[2], pz[3], t);
    }

    // 打印调试信息
    LOG_TRAJ("\n[贝塞尔轨迹生成完成] 点数: %d\n", N);
    LOG_TRAJ("起点: %.3f, %.3f, %.3f m\n", start_pos.foot_pos_x_m, start_pos.foot_pos_y_m, start_pos.foot_pos_z_m);
    LOG_TRAJ("终点: %.3f, %.3f, %.3f m\n", end_pos.foot_pos_x_m, end_pos.foot_pos_y_m, end_pos.foot_pos_z_m);
    LOG_TRAJ("抬升高度: %.3f m\n", lift_z);
}

/**
 * @brief  运行时获取关节指令（位置+速度）
 * @param  ctx          轨迹上下文
 * @param  current_time 当前时间（单位：s，从摆动开始计时）
 * @param  joint_angles 输出：目标关节角度（度）
 * @param  joint_vel    输出：目标关节角速度（°/s）
 * @param  joint_acc    输出：目标关节角加速度（°/s?，暂填0）
 * @return 轨迹状态
 */
TrajStatus_t get_leg_joint_cmd(LegTrajContext_t *ctx, float current_time,
                               LegJointAngles_t *joint_angles, float joint_vel[3], float joint_acc[3])
{
    // 参数合法性校验
    if (ctx == NULL || joint_angles == NULL || joint_vel == NULL || joint_acc == NULL)
        return TRAJ_STATUS_ERROR_INPUT;

    if (isnan(current_time) || current_time < 0)
    {
        *joint_angles = ctx->runtime.last_joint_angles;
        memset(joint_vel, 0, sizeof(float) * 3);
        memset(joint_acc, 0, sizeof(float) * 3);
        return TRAJ_STATUS_ERROR_INPUT;
    }

    // 归一化时间
    float cycle_time = fmaxf(ctx->config.cycle_time, TIME_EPSILON);
    float t_norm = current_time / cycle_time;
    t_norm = fminf(t_norm, 1.0f);

    // 查表获取目标足端位置（四舍五入取最近点）
    uint16_t idx = (uint16_t)(t_norm * (ctx->data.point_num - 1) + 0.5f);
    idx = (idx >= ctx->data.point_num) ? (ctx->data.point_num - 1) : idx;
    FootPosition_t target_pos = ctx->data.point_list[idx];

    IKJacobianError_t ik_err = leg_ik_solve(ctx->leg_id, &target_pos, joint_angles, IK_MODE_GEOMETRY);

    // IK失败则保持上一帧有效角度，防止跳变
    if (joint_angles->valid != 1)
    {
        *joint_angles = ctx->runtime.last_joint_angles;
        memset(joint_vel, 0, sizeof(float) * 3);
        memset(joint_acc, 0, sizeof(float) * 3);
        ctx->runtime.last_joint_angles.valid = 1;
        return TRAJ_STATUS_ERROR_IK;
    }
    ctx->runtime.last_joint_angles.valid = 0;

    // 差分计算关节角速度
    float dt = current_time - ctx->runtime.last_time;
    if (dt > TIME_EPSILON)
    {
        joint_vel[0] = (joint_angles->joint1_deg - ctx->runtime.last_joint_angles.joint1_deg) / dt;
        joint_vel[1] = (joint_angles->joint2_deg - ctx->runtime.last_joint_angles.joint2_deg) / dt;
        joint_vel[2] = (joint_angles->joint3_deg - ctx->runtime.last_joint_angles.joint3_deg) / dt;
    }
    else
    {
        joint_vel[0] = 0.0f;
        joint_vel[1] = 0.0f;
        joint_vel[2] = 0.0f;
    }

    // 加速度暂填0，需要平滑可后续加滤波
    memset(joint_acc, 0, sizeof(float) * 3);

    // 更新运行时状态
    update_runtime(ctx, &target_pos, joint_angles, current_time);
    memcpy(ctx->runtime.last_joint_vel, joint_vel, sizeof(float) * 3);

    // 判断轨迹是否结束
    return (t_norm >= 1.0f - TIME_EPSILON) ? TRAJ_STATUS_END : TRAJ_STATUS_OK;
}

/**
 * @brief  轨迹控制器去初始化
 */
void bezier_traj_controller_deinit(LegTrajContext_t *ctx)
{
    if (ctx == NULL)
        return;
    memset(ctx, 0, sizeof(LegTrajContext_t));
}
