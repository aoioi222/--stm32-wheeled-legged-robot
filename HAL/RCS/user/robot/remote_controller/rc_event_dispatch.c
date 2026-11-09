#include "rc_event_dispatch.h"
#include "body_control.h"
#include "gait_controller.h"
#include "jump_controller.h"

#define WHEEL_TRANS_DUR 0.5f

static void clear_velocity(void)
{
    g_robot_state.cmd_vel_x = 0.0f;
    g_robot_state.cmd_vel_y = 0.0f;
    g_robot_state.cmd_vel_yaw = 0.0f;
}

void rc_event_dispatch(RemoteEventState_t *state, const RemoteCommand_t *command, float now)
{
    state->emergency_stop = command->emergency_stop;
    if (command->shutdown_trigger)
    {
        if (!state->shutdown)
            jump_controller_init(&g_jump_ctrl);
        state->shutdown = true;
        state->folded = false;
        state->gait_active = false;
        state->wheeled_mode = false;
        g_robot_state.mode = ROBOT_MODE_QUADRUPED;
        clear_velocity();
        return;
    }

    if (state->emergency_stop ||
        (state->shutdown && !command->stand_trigger && !command->gait_start_trigger))
    {
        clear_velocity();
        return;
    }

    if (jump_controller_is_active(&g_jump_ctrl))
        return;

    if (command->stand_trigger && !state->wheeled_mode)
    {
        state->shutdown = false;
        state->folded = false;
        state->gait_active = false;
        clear_velocity();
        g_yaw_target_initialized = 0;
        g_yaw_manual_offset = 0.0f;
        g_pitch_target = 0.0f;
        for (int i = 0; i < 4; i++)
        {
            LegJointAngles_t angles = leg_stand_mode_angles((Leg_ID_t)i);
            leg_state_switch_joint(&leg_fsm[i], LEG_STATE_UNFOLD, STAND_TRANS_DUR, &angles, now);
        }
    }

    if ((command->gait_start_trigger && !state->wheeled_mode) || command->mode_switch == 1)
    {
        if (command->mode_switch == 1)
            clear_velocity();
        state->shutdown = false;
        state->folded = false;
        state->wheeled_mode = false;
        state->gait_active = true;
        g_robot_state.mode = ROBOT_MODE_QUADRUPED;
        g_robot_state.quad_sub_mode = QUAD_MODE;
        g_pitch_target = 0.0f;
        gait_controller_enter_stance(now);
    }
    if (command->mode_switch == 2)
    {
        state->shutdown = false;
        state->folded = false;
        state->wheeled_mode = true;
        state->gait_active = false;
        g_robot_state.mode = ROBOT_MODE_WHEELED;
        clear_velocity();
        g_pitch_target = 0.0f;
        for (int i = 0; i < 4; i++)
        {
            LegJointAngles_t angles = leg_wheel_mode_angles((Leg_ID_t)i);
            leg_state_switch_joint(&leg_fsm[i], LEG_STATE_WHEEL, WHEEL_TRANS_DUR, &angles, now);
        }
    }

    if (command->fold_trigger)
    {
        state->folded = !state->folded;
        state->wheeled_mode = false;
        state->gait_active = !state->folded;
        g_robot_state.mode = ROBOT_MODE_QUADRUPED;
        g_robot_state.quad_sub_mode = QUAD_MODE;
        clear_velocity();
        for (int i = 0; i < 4; i++)
        {
            LegJointAngles_t angles = state->folded ? leg_fold_mode_angles((Leg_ID_t)i) :
                                                     leg_stand_mode_angles((Leg_ID_t)i);
            leg_state_switch_joint(&leg_fsm[i], state->folded ? LEG_STATE_FOLD : LEG_STATE_UNFOLD,
                                   state->folded ? FOLD_DURATION : STAND_TRANS_DUR, &angles, now);
        }
    }

    if (command->jump_trigger && state->gait_active && !state->folded && !state->wheeled_mode &&
        leg_is_quadruped_mode(&leg_fsm[0]))
    {
        clear_velocity();
        jump_controller_trigger(&g_jump_ctrl, now);
    }

    switch (command->gait_switch)
    {
        case 1: gait_controller_switch_gait(GAIT_TROT); break;
        case 2: gait_controller_switch_gait(GAIT_WALK); break;
        case 3: gait_controller_switch_gait(GAIT_PRONK); break;
        default: break;
    }
}
