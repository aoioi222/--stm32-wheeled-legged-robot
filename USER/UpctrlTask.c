#include "UpctrlTask.h"
#include "rcs.h"
#include "Core407_RCS12_PinMapping.h"
#include "Kalman_Filter.h"
#include "Wheel_mode_contrl.h"
#include "Foot_mode_control.h"
#include "gait_controller.h"
#include "body_control.h"
#include "gravity_comp.h"
#include "ax_sbus.h"
#include "rc_event_dispatch.h"
#include "jump_controller.h"

#define UPCTRL_PERIOD_TICKS (UPCTRL_TASK_CYCLE_MS * OS_TICKS_PER_SEC / 1000u)
#if (UPCTRL_PERIOD_TICKS == 0) || (UPCTRL_TASK_CYCLE_MS * OS_TICKS_PER_SEC % 1000u != 0)
#error Control period must be a positive whole number of OS ticks
#endif

static OS_STK upctrl_task_stk[NORMAL_TASK_STK_SIZE];
static LegStateMachine_t *leg_sm[4];
static RemoteEventState_t remote_state = {.shutdown = true};
static LegJointAngles_t leg_target_joints_raw[4];

static void wheel_motors_update(void);
static bool robot_mode_execute(float now);
static bool joint_output(void);

void upctrl_task(void *p_arg)
{
    (void)p_arg;
    const float seconds_per_tick = 1.0f / OS_TICKS_PER_SEC;
    const INT32U start_tick = OSTimeGet();
    INT32U previous_tick = start_tick;
    INT32U deadline = start_tick + UPCTRL_PERIOD_TICKS;
    uint64_t elapsed_ticks = 0;

    while (1)
    {
        INT32U tick = OSTimeGet();
        INT32U dt_ticks = tick - previous_tick;
        previous_tick = tick;
        elapsed_ticks += dt_ticks;
        float now = (float)elapsed_ticks * seconds_per_tick;
        g_robot_state.sim_time_s = now;
        g_robot_state.dt_s = (float)(dt_ticks ? dt_ticks : UPCTRL_PERIOD_TICKS) * seconds_per_tick;

        Angles *angles = fusion_algorithm(g_robot_state.dt_s);
        g_robot_state.body_roll = angles->roll;
        g_robot_state.body_pitch = angles->pitch;
        g_robot_state.body_yaw = angles->yaw;

        remote_control_sbus_update(&g_robot_state);
        rc_event_dispatch(&remote_state, remote_control_sbus_get_command(), now);

        bool drive_wheels = false;
        if (remote_state.shutdown)
        {
            Foot_Mode_Stop();
        }
        else
        {
            bool wheel_pose_ready = robot_mode_execute(now);
            bool joints_ready = joint_output();
            drive_wheels = wheel_pose_ready && joints_ready && !remote_state.emergency_stop;
        }

        if (drive_wheels)
            wheel_motors_update();
        else
            Wheel_Mode_Stop();

        /* 以固定节拍唤醒，跳过已经错过的周期边界。 */
        INT32U tick_now = OSTimeGet();
        if ((INT32S)(deadline - tick_now) <= 0)
        {
            INT32U missed = (tick_now - deadline) / UPCTRL_PERIOD_TICKS + 1u;
            deadline += missed * UPCTRL_PERIOD_TICKS;
        }
        OSTimeDly(deadline - tick_now);
        deadline += UPCTRL_PERIOD_TICKS;
    }
}

void UpCtrlTask_Init(void)
{
    SBUS_Init(USART6_MAP);
    IMU_Fusion_Init();
    Wheel_Mode_Init();
    Foot_Mode_Init();
    remote_control_sbus_init();

    robot_state_init();
    g_robot_state.mode = ROBOT_MODE_QUADRUPED;
    g_robot_state.quad_sub_mode = QUAD_MODE;
    g_robot_state.dt_s = (float)UPCTRL_PERIOD_TICKS / OS_TICKS_PER_SEC;
    g_robot_state.body_height = FOOT_OFFSET_Z;
    g_robot_state.cmd_vel_x = 0.0f;
    g_robot_state.cmd_vel_y = 0.0f;
    g_robot_state.cmd_vel_yaw = 0.0f;

    gait_controller_init();
    body_attitude_control_init();
    jump_controller_init(&g_jump_ctrl);

    for (int i = 0; i < 4; i++)
        leg_sm[i] = &leg_fsm[i];

    OSTaskCreate(upctrl_task, (void *)0, &upctrl_task_stk[NORMAL_TASK_STK_SIZE - 1],
                 upctrl_task_PRIO);
}

static void wheel_motors_update(void)
{
    float v_x = g_robot_state.cmd_vel_x;
    float v_yaw = g_robot_state.cmd_vel_yaw;

    float v_left  = v_x - v_yaw * WHEEL_BASE_HALF;
    float v_right = v_x + v_yaw * WHEEL_BASE_HALF;
    float omega_left  = v_left  / WHEEL_RADIUS;
    float omega_right = v_right / WHEEL_RADIUS;

    if (omega_left  >  WHEEL_MAX_SPEED) omega_left  =  WHEEL_MAX_SPEED;
    if (omega_left  < -WHEEL_MAX_SPEED) omega_left  = -WHEEL_MAX_SPEED;
    if (omega_right >  WHEEL_MAX_SPEED) omega_right =  WHEEL_MAX_SPEED;
    if (omega_right < -WHEEL_MAX_SPEED) omega_right = -WHEEL_MAX_SPEED;

    float wheel_speed[WHEEL_COUNT] = {omega_right, omega_left, omega_right, omega_left};
    Wheel_Mode_Control(wheel_speed);
}

static bool robot_mode_execute(float now)
{
    /* 跳跃：优先级最高，期间不走姿态环、不走步态调度，关节角全部由跳跃控制器给 */
    if (jump_controller_is_active(&g_jump_ctrl))
    {
        jump_controller_update(&g_jump_ctrl, now);
        for (int i = 0; i < 4; i++)
        {
            leg_target_joints_raw[i] = g_jump_ctrl.output_joints[i];
        }
        return false;
    }
    if (g_jump_ctrl.completed)
    {
        /* 跳跃收尾：从恢复站立姿态切回支撑相，交还常规步态控制 */
        g_jump_ctrl.completed = 0;
        gait_controller_enter_stance(now);
    }

    /* 折叠：FOLD态关节空间插值，角度由状态机给出 */
    if (remote_state.folded)
    {
        for (int i = 0; i < 4; i++)
        {
            leg_state_machine_update(&leg_fsm[i], now);
            leg_get_target_angles(&leg_fsm[i], &leg_target_joints_raw[i]);
        }
        return false;
    }

    /* 轮式模式 */
    if (remote_state.wheeled_mode)
    {
        for (int i = 0; i < 4; i++)
        {
            leg_state_machine_update(&leg_fsm[i], now);
            leg_get_target_angles(&leg_fsm[i], &leg_target_joints_raw[i]);
        }
        /* 稳态才跑姿态环与轮速；过渡期间WHEEL态正在做关节插值，姿态环不介入 */
        if (leg_fsm[0].current_state != LEG_STATE_WHEEL ||
            leg_fsm[0].phase >= 1.0f)
        {
            body_attitude_control_update(leg_sm);
            return true;
        }
        return false;
    }

    /* 站立：UNFOLD态过渡，由状态机做关节空间插值 */
    if (leg_fsm[0].current_state == LEG_STATE_UNFOLD)
    {
        for (int i = 0; i < 4; i++)
        {
            leg_state_machine_update(&leg_fsm[i], now);
            leg_get_target_angles(&leg_fsm[i], &leg_target_joints_raw[i]);
        }
        return false;
    }

    /* 步态模式 */
    if (remote_state.gait_active)
    {
        gait_gen_update(&gait_gen, &g_robot_state);
        gait_scheduler_update(&gait_scheduler);
        body_attitude_control_update(leg_sm);
        gait_controller_update(now);
        for (int i = 0; i < 4; i++)
        {
            leg_get_target_angles(&leg_fsm[i], &leg_target_joints_raw[i]);
        }
        return false;
    }

    /* 站立静止：状态机维持支撑相 */
    body_attitude_control_update(leg_sm);
    for (int i = 0; i < 4; i++)
    {
        LegStateMachine_t *sm = &leg_fsm[i];
        if (sm->current_state == LEG_STATE_STANCE)
        {
            float elapsed = now - sm->state_start_time_s;
            sm->phase = (sm->state_duration_s > 0.0f) ? (elapsed / sm->state_duration_s) : 1.0f;
            if (sm->phase > 1.0f) sm->phase = 1.0f;
        }
        leg_state_machine_update(sm, now);
        leg_get_target_angles(&leg_fsm[i], &leg_target_joints_raw[i]);
        leg_get_target_pos(sm, &foot_target_pos[i]);
    }
    return false;
}

static bool joint_output(void)
{
    float joint_angle[FOOT_JOINT_COUNT];
    for (int i = 0; i < 4; i++)
    {
        LegJointAngles_t angles = leg_target_joints_raw[i];
#if GRAV_COMP_ENABLE
        float torque[3];
        leg_gravity_torque(angles.joint1_deg, angles.joint2_deg, angles.joint3_deg, torque);
        angles.joint1_deg += GRAV_COMP_GAIN * torque[0];
        angles.joint2_deg += GRAV_COMP_GAIN * torque[1];
        angles.joint3_deg += GRAV_COMP_GAIN * torque[2];
#endif
        joint_angle[i * 3] = angles.joint1_deg;
        joint_angle[i * 3 + 1] = angles.joint2_deg;
        joint_angle[i * 3 + 2] = angles.joint3_deg;
    }
    return Foot_Mode_Control(joint_angle);
}
