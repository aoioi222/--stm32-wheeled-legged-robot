#include "main.h"
#include "cmsis_os2.h"
#include "Kalman_Filter.h"
#include "Wheel_mode_contrl.h"
#include "Foot_mode_control.h"
#include "gait_controller.h"
#include "body_control.h"
#include "gravity_comp.h"
#include "robot_sbus.h"
#include "rc_event_dispatch.h"
#include "jump_controller.h"

#define CONTROL_RATE_HZ 200u

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart6;

static LegStateMachine_t *leg_sm[4];
static RemoteEventState_t remote_state = {.shutdown = true};
static LegJointAngles_t leg_target_joints_raw[4];

static void wheel_motors_update(void);
static bool robot_mode_execute(float now);
static bool joint_output(void);

void UpctrlTaskMain(void *argument)
{
    (void)argument;
    const uint32_t frequency = osKernelGetTickFreq();
    if (frequency < CONTROL_RATE_HZ || frequency % CONTROL_RATE_HZ != 0u)
        osThreadExit();

    bool sbus_ready = RobotSbus_Start(&huart6);
    bool imu_ready = IMU_Fusion_Init(&hi2c1);
    Wheel_Mode_Init(&hcan1);
    Foot_Mode_Init(&hcan2);
    remote_control_sbus_init();

    robot_state_init();
    g_robot_state.mode = ROBOT_MODE_QUADRUPED;
    g_robot_state.quad_sub_mode = QUAD_MODE;
    g_robot_state.dt_s = 1.0f / CONTROL_RATE_HZ;
    gait_controller_init();
    body_attitude_control_init();
    jump_controller_init(&g_jump_ctrl);
    for (int i = 0; i < 4; i++)
        leg_sm[i] = &leg_fsm[i];

    const uint32_t period_ticks = frequency / CONTROL_RATE_HZ;
    const float seconds_per_tick = 1.0f / (float)frequency;
    uint32_t previous_tick = osKernelGetTickCount();
    uint32_t deadline = previous_tick + period_ticks;
    uint64_t elapsed_ticks = 0;

    while (1)
    {
        uint32_t tick = osKernelGetTickCount();
        uint32_t dt_ticks = tick - previous_tick;
        previous_tick = tick;
        elapsed_ticks += dt_ticks;
        float now = (float)elapsed_ticks * seconds_per_tick;
        g_robot_state.sim_time_s = now;
        g_robot_state.dt_s = (float)(dt_ticks ? dt_ticks : period_ticks) * seconds_per_tick;

        if (imu_ready && sbus_ready)
        {
            Angles *angles = fusion_algorithm(g_robot_state.dt_s);
            g_robot_state.body_roll = angles->roll;
            g_robot_state.body_pitch = angles->pitch;
            g_robot_state.body_yaw = angles->yaw;

            remote_control_sbus_update(&g_robot_state);
            rc_event_dispatch(&remote_state, remote_control_sbus_get_command(), now);
        }

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
        uint32_t tick_now = osKernelGetTickCount();
        if ((int32_t)(deadline - tick_now) <= 0)
        {
            uint32_t missed = (tick_now - deadline) / period_ticks + 1u;
            deadline += missed * period_ticks;
        }
        if (osDelayUntil(deadline) != osOK)
            osDelay(1u);
        deadline += period_ticks;
    }
}

void app_main(void *argument)
{
    (void)argument;
    osThreadExit();
}

void DebugTaskMain(void *argument)
{
    (void)argument;
    osThreadExit();
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
