#include "remote_control.h"

static RemoteCommand_t g_rc_command;
static int g_last_key = -1;
static float g_speed_scale = 1.0f; // 速度缩放因子，上键增大，下键减小

void remote_control_init(void)
{
    memset(&g_rc_command, 0, sizeof(RemoteCommand_t));
    wb_keyboard_enable(TIME_STEP);
    g_last_key = -1;
    g_speed_scale = 1.0f;
    printf("[遥控] 键盘遥控已启用\n");
    printf("  1/2: 四足/轮式模式切换 | 3: Pronk | 空格: 跳跃 | R: 急停\n");
    printf("  四足: WASD移动 Q/E转向 | 轮式: WS前后 AD转向\n");
    printf("  Esc: 关机/开机 | F: 折叠/展开 | I: 站立 | O: 启动步态\n");
    printf("  站立时: [ 后仰 | ] 低头\n");
    printf("  ↑/↓: 加速/减速 (当前:%.1fx)\n", g_speed_scale);
}

void remote_control_update(RobotGlobalState_t *state)
{
    int key = wb_keyboard_get_key();

    // 每帧重置边沿触发标志
    g_rc_command.jump_trigger = 0;
    g_rc_command.gait_switch = 0;
    g_rc_command.emergency_stop = 0;
    g_rc_command.shutdown_trigger = 0;
    g_rc_command.fold_trigger = 0;
    g_rc_command.stand_trigger = 0;
    g_rc_command.gait_start_trigger = 0;
    g_rc_command.mode_switch = 0;
    g_rc_command.pitch_adjust = 0;

    // 检测新按键按下（不同于上一帧的按键）
    uint8_t is_new_key = (key != -1 && key != g_last_key);

    switch (key)
    {
    // 速度指令：每个按键只修改对应轴，其他轴保持不??
    case 'W':
    case 'w':
        g_rc_command.cmd_vel_x = RC_MAX_VEL_X;
        break;
    case 'S':
    case 's':
        g_rc_command.cmd_vel_x = -RC_MAX_VEL_X;
        break;
    case 'A':
    case 'a':
        if (state->mode == ROBOT_MODE_WHEELED)
            g_rc_command.cmd_vel_yaw = -RC_MAX_VEL_YAW; // 轮式：左转
        else
            g_rc_command.cmd_vel_y = RC_MAX_VEL_Y;
        break;
    case 'D':
    case 'd':
        if (state->mode == ROBOT_MODE_WHEELED)
            g_rc_command.cmd_vel_yaw = RC_MAX_VEL_YAW; // 轮式：右转
        else
            g_rc_command.cmd_vel_y = -RC_MAX_VEL_Y;
        break;
    case 'Q':
    case 'q':
        g_rc_command.cmd_vel_yaw = -RC_MAX_VEL_YAW;
        break;
    case 'E':
    case 'e':
        g_rc_command.cmd_vel_yaw = RC_MAX_VEL_YAW;
        break;
    case 'R':
    case 'r':
        g_rc_command.cmd_vel_x = 0.0f;
        g_rc_command.cmd_vel_y = 0.0f;
        g_rc_command.cmd_vel_yaw = 0.0f;
        g_speed_scale = 1.0f;
        if (is_new_key)
            printf("[遥控] 急停, 速度缩放重置为 1.0x\n");
        break;

    // 跳跃触发（边沿触发，按下瞬间有效）
    case ' ':
        if (is_new_key)
            g_rc_command.jump_trigger = 1;
        break;

    // 模式切换（边沿触发）
    case '1':
        if (is_new_key)
            g_rc_command.mode_switch = 1; // 四足模式
        break;
    case '2':
        if (is_new_key)
            g_rc_command.mode_switch = 2; // 轮式模式
        break;

    // Pronk 步态切换（边沿触发）
    case '3':
        if (is_new_key)
            g_rc_command.gait_switch = 3; // Pronk
        break;

    // 关机/开机切换（Esc 键，边沿触发）
    case 27: // ESC
        if (is_new_key)
            g_rc_command.shutdown_trigger = 1;
        break;

    // 折叠/展开切换（F 键，边沿触发）
    case 'F':
    case 'f':
        if (is_new_key)
            g_rc_command.fold_trigger = 1;
        break;

    // 加速/减速（方向键上/下，边沿触发）
    case 315: // 方向键上
        if (is_new_key)
        {
            g_speed_scale += RC_SPEED_SCALE_STEP;
            if (g_speed_scale > RC_SPEED_SCALE_MAX)
                g_speed_scale = RC_SPEED_SCALE_MAX;
            printf("[遥控] 加速, 速度缩放: %.1fx\n", g_speed_scale);
        }
        break;
    case 317: // 方向键下
        if (is_new_key)
        {
            g_speed_scale -= RC_SPEED_SCALE_STEP;
            if (g_speed_scale < RC_SPEED_SCALE_MIN)
                g_speed_scale = RC_SPEED_SCALE_MIN;
            printf("[遥控] 减速, 速度缩放: %.1fx\n", g_speed_scale);
        }
        break;

    // 回到初始站立位置（I 键，边沿触发）
    case 'I':
    case 'i':
        if (is_new_key)
            g_rc_command.stand_trigger = 1;
        break;

    // 启动步态周期（O 键，边沿触发）
    case 'O':
    case 'o':
        if (is_new_key)
            g_rc_command.gait_start_trigger = 1;
        break;

    // 站立姿态俯仰调整（[ 键后仰，] 键低头，边沿触发）
    case '[':
        if (is_new_key)
            g_rc_command.pitch_adjust = -1;
        break;
    case ']':
        if (is_new_key)
            g_rc_command.pitch_adjust = 1;
        break;

    case -1:
        // 无按键按下，保持当前速度指令不变
        break;
    default:
        break;
    }

    // 将速度命令乘以缩放因子后写入机器人全局状态
    state->cmd_vel_x = g_rc_command.cmd_vel_x * g_speed_scale;
    state->cmd_vel_y = g_rc_command.cmd_vel_y * g_speed_scale;
    state->cmd_vel_yaw = g_rc_command.cmd_vel_yaw * g_speed_scale;

    g_last_key = key;
}

const RemoteCommand_t *remote_control_get_command(void)
{
    return &g_rc_command;
}
