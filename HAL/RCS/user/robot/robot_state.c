#include "robot_state.h"

RobotGlobalState_t g_robot_state;



void robot_state_init(void)
{
    memset(&g_robot_state, 0, sizeof(RobotGlobalState_t));

    g_robot_state.mode = ROBOT_MODE_IDLE;
    g_robot_state.target_mode = ROBOT_MODE_IDLE;
    g_robot_state.last_mode = ROBOT_MODE_IDLE;
    g_robot_state.body_height = 0.15f;
    g_robot_state.dt_s = 0.001f;

    g_robot_state.quad_sub_mode = QUAD_MODE_STAND;
    g_robot_state.step_length = 0.05f;
    g_robot_state.swing_height = 0.04f;

    g_robot_state.wheel_sub_mode = WHEEL_MODE_STOP;
    g_robot_state.wheel_max_speed = 10.0f;

    g_robot_state.flight_sub_mode = FLIGHT_MODE_HOVER;
    g_robot_state.max_throttle = 1.0f;
    g_robot_state.min_throttle = 0.2f;
}

void robot_state_reset(void)
{
    robot_state_init();
}

int robot_mode_switch_request(RobotMode_t target_mode)
{
    if (!robot_mode_switch_allowed(g_robot_state.mode, target_mode))
    {
        return -1;
    }

    g_robot_state.target_mode = target_mode;
    g_robot_state.is_transitioning = 1;
    g_robot_state.transition_progress = 0.0f;

    return 0;
}

int robot_mode_switch_allowed(RobotMode_t current_mode, RobotMode_t target_mode)
{
    if (current_mode == ROBOT_MODE_ERROR)
    {
        return (target_mode == ROBOT_MODE_IDLE) ? 1 : 0;
    }

    if (target_mode == ROBOT_MODE_IDLE)
    {
        return 1;
    }

    if (current_mode == ROBOT_MODE_IDLE)
    {
        return 1;
    }

    if (current_mode == ROBOT_MODE_QUADRUPED &&
        (target_mode == ROBOT_MODE_WHEELED || target_mode == ROBOT_MODE_FLIGHT))
    {
        return 1;
    }

    if (current_mode == ROBOT_MODE_WHEELED &&
        (target_mode == ROBOT_MODE_QUADRUPED || target_mode == ROBOT_MODE_FLIGHT))
    {
        return 1;
    }

    if (current_mode == ROBOT_MODE_FLIGHT &&
        (target_mode == ROBOT_MODE_QUADRUPED || target_mode == ROBOT_MODE_WHEELED))
    {
        return 1;
    }

    return 0;
}
