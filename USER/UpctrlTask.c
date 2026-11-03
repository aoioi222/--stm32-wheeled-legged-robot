/* ===========头文件==============================================*/
#include "UpctrlTask.h"
#include "rcs.h"

#include "Tester.h"
#include "RCS_dsp.h"
#include "RCS_Scara.h"

#include "R1_UpCtrl_Motion.h"
#include "R2_CombCtrl.h"
#include "Ch_Ctrl.h"

#include "GpsTask.h"
#include "RCS_Motor_Reposition.h"
#include "RCS_Dribble.h"

#include "Valve_2006.h"
#include "located_position.h"
#include "RCS_DT35.h"
#include "I2C_ICM_42688_P_control.h"
#include "Kalman_Filter.h"

#include "Core407_RCS12_PinMapping.h"
#include "Wheel_mode_contrl.h"
#include "Foot_mode_control.h"

/* 机器人步态控制 */
#include "robot_state.h"
#include "gait_controller.h"
#include "body_control.h"
#include "gravity_comp.h"
#include "robot_config.h"

/* SBUS遥控 */
#include "ax_sbus.h"
#include "remote_control_sbus.h"

/* 跳跃控制器 */
#include "jump_controller.h"

/* ============私有全局变量========================================*/
OS_STK   upctrl_task_stk[NORMAL_TASK_STK_SIZE];

static LegStateMachine_t *leg_sm[4];

/* ============遥控状态变量========================================*/
static int g_shutdown = 0;
static int g_folded = 0;
static int g_gait_active = 0;
static int g_wheeled_mode = 0;

/* 轮式/折叠/站立三种姿态的过渡时长：插值均由腿状态机（WHEEL/FOLD/UNFOLD）执行，
   目标关节角表已移入 leg_state_machine.h，时长复用 robot_config.h 的宏 */
#define WHEEL_TRANS_DUR 0.5f

/* ============私有函数声明========================================*/
static void leg_direction_remap(int leg_idx, LegJointAngles_t *angles);
static void leg_cur_joints(int leg_idx, LegJointAngles_t *out);
static void wheel_motors_update(void);
static void rc_event_dispatch(const RemoteCommand_t *rc, float now);
static void robot_mode_execute(float now);
static void servo_output(void);

/* 状态机给的IK规范角，仅在 servo_output 里映射一次成舵机角 */
static LegJointAngles_t leg_target_joints_raw[4];

/* ============任务函数实现========================================*/

void upctrl_task(void *p_arg)
{
    p_arg = p_arg;
    float print_timer = 0.0f;
    /* 恒定控制周期5s/1000=5ms，步态相位按固定步长累积推进 */
    const float ctrl_dt = (float)UPCTRL_TASK_CYCLE_MS / 1000.0f;
    /* 硬等周期节拍：OS_TICKS_PER_SEC=1000，1 tick = 1ms */
    const INT32U tick_period = UPCTRL_TASK_CYCLE_MS;

    /* 启动步态，各腿进入支撑相 */
    float now = 0.0f;
    g_robot_state.sim_time_s = now;
    g_robot_state.dt_s = ctrl_dt;
    gait_gen_update(&gait_gen, &g_robot_state);
    float stance_time = CYCLE_TIME * (1.0f - SWING_RATIO);
    for (int i = 0; i < 4; i++)
    {
        leg_state_switch(&leg_fsm[i], LEG_STATE_STANCE, stance_time,
                         &gait_gen.leg_output[i].stance_end, now);
    }

    /* 记录下一个唤醒节拍，硬等周期而非固定休眠 */
    INT32U deadline = OSTimeGet() + tick_period;

    while(1)
    {
        /* 相位时间用固定步长累积推进，不读取墙钟，消除步态周期抖动 */
        now += ctrl_dt;
        g_robot_state.sim_time_s = now;

        /* 读取IMU */
        const float dt = (float)UPCTRL_TASK_CYCLE_MS * 0.001f;
        Angles *angles = fusion_algorithm(dt);
        g_robot_state.body_roll  = angles->roll;
        g_robot_state.body_pitch = angles->pitch;
        g_robot_state.body_yaw   = angles->yaw;

        /* 遥控：读SBUS → 写速度指令 + 生成触发事件 */
        remote_control_sbus_update(&g_robot_state);
        rc_event_dispatch(remote_control_sbus_get_command(), now);

        /* 根据当前状态执行对应控制模式 */
        robot_mode_execute(now);

        /* 输出舵机 */
        servo_output();

        /* 周期调试打印 */
        if (now - print_timer > 0.2f)
        {
            print_timer = now;
//            printf("t=%.2f phase=%.2f pitch=%.1f roll=%.1f yaw=%.1f gait=%d wheel=%d\n",
//                   now, gait_gen.global_phase,
//                   g_robot_state.body_pitch, g_robot_state.body_roll, g_robot_state.body_yaw,
//                   g_gait_active, g_wheeled_mode);
        }

        /* 硬等到下一个周期边界，真实周期严格=tick_period，不受代码耗时影响 */
        INT32U tick_now = OSTimeGet();
        if ((INT32S)(deadline - tick_now) > 0)
            OSTimeDly(deadline - tick_now);
        else
            deadline = tick_now;  /* 代码超时，重新对齐，避免追赶 */
        deadline += tick_period;
    }
}


/* =============上层任务初始化======================================*/

void UpCtrlTask_Init()
{
    SBUS_Init(USART6_MAP);

    IMU_Fusion_Init();
    Wheel_Mode_Init();
    Foot_Mode_Init();
    remote_control_sbus_init();

    robot_state_init();
    g_robot_state.mode = ROBOT_MODE_QUADRUPED;
    g_robot_state.quad_sub_mode = QUAD_MODE;
    g_robot_state.dt_s = UPCTRL_TASK_CYCLE_MS / 1000.0f;
    g_robot_state.body_height = FOOT_OFFSET_Z;
    g_robot_state.cmd_vel_x = CMD_V_X;
    g_robot_state.cmd_vel_y = CMD_V_Y;
    g_robot_state.cmd_vel_yaw = CMD_V_YAW;

    gait_controller_init();
    body_attitude_control_init();
    jump_controller_init(&g_jump_ctrl);

    leg_sm[0] = &leg_fsm[LEG_R_F];
    leg_sm[1] = &leg_fsm[LEG_L_F];
    leg_sm[2] = &leg_fsm[LEG_R_B];
    leg_sm[3] = &leg_fsm[LEG_L_B];

    OSTaskCreate(upctrl_task,
               (void *)0,
               &upctrl_task_stk[NORMAL_TASK_STK_SIZE - 1],
               upctrl_task_PRIO);
}

/* =============私有函数实现========================================*/

/**
 * @brief 关节角安装方向映射：IK规范角 <-> 舵机实际转动方向
 * @param leg_idx 腿索引 0~3
 * @param angles 输入输出，待修正的关节角
 * @note  状态机内部只保存规范角，故本函数在 servo_output 里按腿调用一次；
 *        连续调用两次可还原（各腿翻转的关节互不相同）
 */
static void leg_direction_remap(int leg_idx, LegJointAngles_t *angles)
{
    switch (leg_idx)
    {
    case LEG_R_F: angles->joint2_deg = -angles->joint2_deg; break;
    case LEG_L_F: angles->joint3_deg = -angles->joint3_deg; break;
    case LEG_R_B:
        angles->joint1_deg = -angles->joint1_deg;
        angles->joint2_deg = -angles->joint2_deg;
        break;
    case LEG_L_B:
        angles->joint1_deg = -angles->joint1_deg;
        angles->joint3_deg = -angles->joint3_deg;
        break;
    default: break;
    }
}

/**
 * @brief 取出某条腿当前姿态的 IK 规范角
 * @param leg_idx 腿索引 0~3
 * @param out 输出关节角
 * @note  状态机内部保存的一直是规范角，非舵机安装方向修正后的角度；
 *        过渡起点直接用它，避免先在做舵机映射的数组上取角度再反算回来
 */
static void leg_cur_joints(int leg_idx, LegJointAngles_t *out)
{
    if (out == NULL || leg_idx < 0 || leg_idx >= 4)
        return;
    *out = leg_fsm[leg_idx].current_joint_angles;
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

    float wheel_speed[4] = {omega_right, omega_left, omega_right, omega_left};
    Wheel_Mode_Control(wheel_speed);
}


/* =============遥控事件分发========================================*/


static void rc_event_dispatch(const RemoteCommand_t *rc, float now)
{
    /* 关机/开机切换 */
    if (rc->shutdown_trigger)
    {
        g_shutdown = !g_shutdown;
        if (g_shutdown)
        {
            g_robot_state.cmd_vel_x = 0.0f;
            g_robot_state.cmd_vel_y = 0.0f;
            g_robot_state.cmd_vel_yaw = 0.0f;
            Foot_Mode_Stop();  /* 关机卸载舵机扭矩，瘫软收纳 */
        }
//        else
//            printf("[开机] 恢复运行\n");
    }

    /* 急停 */
    if (rc->emergency_stop)
    {
        g_robot_state.cmd_vel_x = 0.0f;
        g_robot_state.cmd_vel_y = 0.0f;
        g_robot_state.cmd_vel_yaw = 0.0f;
        //printf("[急停] 速度归零\n");
    }

    /* 跳跃触发：双摇杆同时到底的上升沿。
       仅在四足步态且非关机/非折叠时接受，避免与折叠、轮式、过渡态冲突 */
    if (rc->jump_trigger)
    {
        if (g_gait_active && !g_shutdown && !g_folded && !g_wheeled_mode &&
            !jump_controller_is_active(&g_jump_ctrl))
        {
            /* 跳跃全程由跳跃控制器直接给关节角，先把速度指令清掉 */
            g_robot_state.cmd_vel_x = 0.0f;
            g_robot_state.cmd_vel_y = 0.0f;
            g_robot_state.cmd_vel_yaw = 0.0f;
            jump_controller_trigger(&g_jump_ctrl, now);
        }
    }

    /* 折叠/展开切换：折叠→FOLD态插值，展开→UNFOLD态插值回站立姿态 */
    if (rc->fold_trigger)
    {
        if (!g_folded)
        {
            LegJointAngles_t fold_angles;
            for (int i = 0; i < 4; i++)
            {
                fold_angles = leg_fold_mode_angles((Leg_ID_t)i);
                leg_state_switch_joint(&leg_fsm[i], LEG_STATE_FOLD,
                                       FOLD_DURATION, &fold_angles, now);
            }
            g_folded = 1;
        }
        else
        {
            g_folded = 0;
            LegJointAngles_t stand_angles;
            for (int i = 0; i < 4; i++)
            {
                stand_angles = leg_stand_mode_angles((Leg_ID_t)i);
                leg_state_switch_joint(&leg_fsm[i], LEG_STATE_UNFOLD,
                                       STAND_TRANS_DUR, &stand_angles, now);
            }
            g_gait_active = 1;
            float st = CYCLE_TIME * (1.0f - SWING_RATIO);
            gait_gen_update(&gait_gen, &g_robot_state);
            for (int i = 0; i < 4; i++)
                leg_state_switch(&leg_fsm[i], LEG_STATE_STANCE, st,
                                 &gait_gen.leg_output[i].stance_end, now);
        }
    }

    /* 站立触发：切到站立姿态，由 UNFOLD 态完成过渡 */
    if (rc->stand_trigger && !g_wheeled_mode)
    {
        g_gait_active = 0;
        g_wheeled_mode = 0;
        g_shutdown = 0;
        g_folded = 0;
        Wheel_Mode_Stop();

        g_robot_state.cmd_vel_x = 0.0f;
        g_robot_state.cmd_vel_y = 0.0f;
        g_robot_state.cmd_vel_yaw = 0.0f;
        g_yaw_target_initialized = 0;
        g_yaw_manual_offset = 0.0f;
        g_pitch_target = 0.0f;

        for (int i = 0; i < 4; i++)
        {
            LegJointAngles_t stand_angles = leg_stand_mode_angles((Leg_ID_t)i);
            leg_state_switch_joint(&leg_fsm[i], LEG_STATE_UNFOLD,
                                   STAND_TRANS_DUR, &stand_angles, now);
        }
    }

    /* 步态启动触发 */
    if (rc->gait_start_trigger && !g_wheeled_mode)
    {
        g_shutdown = 0;
        g_folded = 0;
        g_gait_active = 1;
        g_robot_state.mode = ROBOT_MODE_QUADRUPED;
        g_robot_state.quad_sub_mode = QUAD_MODE;
        g_pitch_target = 0.0f;

        float st = CYCLE_TIME * (1.0f - SWING_RATIO);
        gait_gen_update(&gait_gen, &g_robot_state);
        for (int i = 0; i < 4; i++)
            leg_state_switch(&leg_fsm[i], LEG_STATE_STANCE, st,
                             &gait_gen.leg_output[i].stance_end, now);
        //printf("[步态] 步态周期启动\n");
    }

    /* 模式切换 */
    if (rc->mode_switch == 1)
    {
        g_wheeled_mode = 0;
        g_gait_active = 1;
        g_robot_state.mode = ROBOT_MODE_QUADRUPED;
        g_robot_state.quad_sub_mode = QUAD_MODE;
        g_robot_state.cmd_vel_x = 0.0f;
        g_robot_state.cmd_vel_y = 0.0f;
        g_robot_state.cmd_vel_yaw = 0.0f;
        g_pitch_target = 0.0f;
        Wheel_Mode_Stop();

        float st = CYCLE_TIME * (1.0f - SWING_RATIO);
        gait_gen_update(&gait_gen, &g_robot_state);
        for (int i = 0; i < 4; i++)
            leg_state_switch(&leg_fsm[i], LEG_STATE_STANCE, st,
                             &gait_gen.leg_output[i].stance_end, now);
        //printf("[模式] 切换到四足模式\n");
    }
    if (rc->mode_switch == 2)
    {
        g_shutdown = 0;
        g_folded = 0;
        g_wheeled_mode = 1;
        g_gait_active = 0;
        g_robot_state.mode = ROBOT_MODE_WHEELED;
        g_robot_state.cmd_vel_x = 0.0f;
        g_robot_state.cmd_vel_y = 0.0f;
        g_robot_state.cmd_vel_yaw = 0.0f;
        g_pitch_target = 0.0f;

        for (int i = 0; i < 4; i++)
        {
            LegJointAngles_t wheel_angles = leg_wheel_mode_angles((Leg_ID_t)i);
            leg_state_switch_joint(&leg_fsm[i], LEG_STATE_WHEEL,
                                   WHEEL_TRANS_DUR, &wheel_angles, now);
        }
        //printf("[模式] 切换到轮式模式\n");
    }

    /* 步态切换 */
    if (rc->gait_switch != 0)
    {
        GaitType_t target = (rc->gait_switch == 1) ? GAIT_TROT :
                            (rc->gait_switch == 2) ? GAIT_WALK : GAIT_PRONK;
        gait_controller_switch_gait(target);
    }
}

/* =============模式执行============================================*/

static void robot_mode_execute(float now)
{
    /* 关机：舵机保持当前位置（在upctrl_task中输出） */
    if (g_shutdown)
    {
        return;
    }

    /* 跳跃：优先级最高，期间不走姿态环、不走步态调度，关节角全部由跳跃控制器给 */
    if (jump_controller_is_active(&g_jump_ctrl))
    {
        jump_controller_update(&g_jump_ctrl, now);
        for (int i = 0; i < 4; i++)
        {
            leg_target_joints_raw[i] = g_jump_ctrl.output_joints[i];
        }
        return;
    }
    if (g_jump_ctrl.completed)
    {
        /* 跳跃收尾：从恢复站立姿态切回支撑相，交还常规步态控制 */
        g_jump_ctrl.completed = 0;
        float st = CYCLE_TIME * (1.0f - SWING_RATIO);
        gait_gen_update(&gait_gen, &g_robot_state);
        for (int i = 0; i < 4; i++)
            leg_state_switch(&leg_fsm[i], LEG_STATE_STANCE, st,
                             &gait_gen.leg_output[i].stance_end, now);
    }

    /* 折叠：FOLD态关节空间插值，角度由状态机给出 */
    if (g_folded)
    {
        for (int i = 0; i < 4; i++)
        {
            leg_state_machine_update(&leg_fsm[i], now);
            leg_get_target_angles(&leg_fsm[i], &leg_target_joints_raw[i]);
        }
        return;
    }

    /* 轮式模式 */
    if (g_wheeled_mode)
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
            wheel_motors_update();
        }
        return;
    }

    /* 步态模式 */
    if (g_gait_active)
    {
        gait_gen_update(&gait_gen, &g_robot_state);
        gait_scheduler_update(&gait_scheduler);
        body_attitude_control_update(leg_sm);
        gait_controller_update(now);
        for (int i = 0; i < 4; i++)
        {
            leg_cur_joints(i, &leg_target_joints_raw[i]);
        }
        return;
    }

    /* 站立：UNFOLD态过渡，由状态机做关节空间插值 */
    if (leg_fsm[0].current_state == LEG_STATE_UNFOLD)
    {
        for (int i = 0; i < 4; i++)
        {
            leg_state_machine_update(&leg_fsm[i], now);
            leg_get_target_angles(&leg_fsm[i], &leg_target_joints_raw[i]);
        }
        /* 过渡结束后把足端目标复位到默认站立点，衔接支撑相 */
        if (leg_fsm[0].phase >= 1.0f)
        {
            float st = CYCLE_TIME * (1.0f - SWING_RATIO);
            gait_gen_update(&gait_gen, &g_robot_state);
            for (int i = 0; i < 4; i++)
            {
                float sx = (i == LEG_L_F || i == LEG_R_F) ? FOOT_OFFSET_X : -FOOT_OFFSET_X;
                float sy = (i == LEG_L_F || i == LEG_L_B) ? FOOT_OFFSET_Y : -FOOT_OFFSET_Y;
                FootPosition_t sp = {sx, sy, -FOOT_OFFSET_Z};
                leg_state_switch(&leg_fsm[i], LEG_STATE_STANCE, st, &sp, now);
            }
        }
        return;
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
        leg_cur_joints(i, &leg_target_joints_raw[i]);
        leg_get_target_pos(sm, &foot_target_pos[i]);
    }
}

/* =============关节舵机输出============================================*/

static void servo_output(void)
{
    if (g_shutdown)
        return;  /* 关机卸载后不再下发位置指令，保持瘫软 */

    float joint_angle[12];
    for (int i = 0; i < 4; i++)
    {
        /* 状态机内部保存IK规范角，此处做一次舵机安装方向映射 */
        LegJointAngles_t ang = leg_target_joints_raw[i];
        leg_direction_remap(i, &ang);

        /* 位置空间前馈：把重力矩换算成角度补偿量叠加到目标角 */
#if GRAV_COMP_ENABLE
        float tau_nm[3];
        leg_gravity_torque(ang.joint1_deg, ang.joint2_deg, ang.joint3_deg, tau_nm);
        ang.joint1_deg += GRAV_COMP_GAIN * tau_nm[0];
        ang.joint2_deg += GRAV_COMP_GAIN * tau_nm[1];
        ang.joint3_deg += GRAV_COMP_GAIN * tau_nm[2];
#endif

        joint_angle[i * 3 + 0] = ang.joint1_deg;
        joint_angle[i * 3 + 1] = ang.joint2_deg;
        joint_angle[i * 3 + 2] = ang.joint3_deg;
    }
    Foot_Mode_Control(joint_angle);
}
