#include "remote_control_sbus.h"

#include "ax_sbus.h"

static RemoteCommand_t g_rc_command;
static SbusSwitchZone_t g_sw_prev[4];  /* SWA~SWD上一帧档位 */
static SbusSwitchZone_t g_sw_curr[4];  /* SWA~SWD当前帧档位 */
static float g_speed_scale = 1.0f;
static uint8_t g_first_frame = 1;
static uint8_t g_jump_gesture_prev = 0; /* 上一帧是否处于跳跃手势 */

/*
 * 跳跃手势：左摇杆(ch2)与右摇杆(ch1)同时推到最低端，边沿触发一次。
 * 摇杆行程中段到最低端约 SBUS_RAW_MIN~SBUS_RAW_MID，取靠近最低端的窗口判定，
 * 阈值可按实测手感调整（越小越难触发）。
 */
#define SBUS_JUMP_GESTURE_MARGIN 120

static uint8_t sbus_jump_gesture(uint16_t raw_ch1, uint16_t raw_ch2)
{
    return (raw_ch1 <= (uint16_t)(SBUS_RAW_MIN + SBUS_JUMP_GESTURE_MARGIN) &&
            raw_ch2 <= (uint16_t)(SBUS_RAW_MIN + SBUS_JUMP_GESTURE_MARGIN)) ? 1 : 0;
}

/* 将SBUS摇杆值映射为速度指令 */
static float sbus_stick_to_vel(uint16_t raw, float max_vel)
{
    if (raw >= (SBUS_RAW_MID - SBUS_RAW_DEAD) &&
        raw <= (SBUS_RAW_MID + SBUS_RAW_DEAD))
    {
        return 0.0f;
    }
    if (raw > SBUS_RAW_MID)
    {
        float t = (float)(raw - SBUS_RAW_MID - SBUS_RAW_DEAD) /
                  (float)(SBUS_RAW_MAX - SBUS_RAW_MID - SBUS_RAW_DEAD);
        return t * max_vel;
    }
    else
    {
        float t = (float)(SBUS_RAW_MID - SBUS_RAW_DEAD - raw) /
                  (float)(SBUS_RAW_MID - SBUS_RAW_DEAD - SBUS_RAW_MIN);
        return -t * max_vel;
    }
}

/* 判断三段开关档位 */
static SbusSwitchZone_t sbus_get_zone(uint16_t raw)
{
    if (raw < SBUS_ZONE_LOW_MAX)
        return SBUS_SW_LOW;
    else if (raw > SBUS_ZONE_HIGH_MIN)
        return SBUS_SW_HIGH;
    else
        return SBUS_SW_MID;
}

/* 检测档位上升沿（LOW→MID, LOW→HIGH, MID→HIGH） */
static uint8_t sbus_edge_rising(int ch_idx)
{
    return (g_sw_curr[ch_idx] != g_sw_prev[ch_idx] &&
            g_sw_curr[ch_idx] > g_sw_prev[ch_idx]);
}

/* 检测档位变化（任意方向） */
static uint8_t sbus_edge_any(int ch_idx)
{
    return (g_sw_curr[ch_idx] != g_sw_prev[ch_idx]);
}

void remote_control_sbus_init(void)
{
    memset(&g_rc_command, 0, sizeof(RemoteCommand_t));
    memset(g_sw_prev, 0, sizeof(g_sw_prev));
    memset(g_sw_curr, 0, sizeof(g_sw_curr));
    g_speed_scale = 1.0f;
    g_first_frame = 1;
#if ROBOT_DEBUG_PRINT
    printf("[遥控] SBUS遥控已启用 (USART6)\n");
    printf("  通道映射:\n");
    printf("    ch0 右摇杆左右 -> 转向速度\n");
    printf("    ch1 右摇杆上下 -> 前进速度\n");
    printf("    ch2 左摇杆上下 -> 横向速度\n");
    printf("    ch3 左摇杆左右 -> (预留)\n");
    printf("    SWA(ch4) HIGH=启动步态 MID=待机 LOW=关机\n");
    printf("    SWB(ch5) HIGH=Pronk MID=Trot LOW=Walk\n");
    printf("    SWC(ch6) HIGH=轮式模式 MID=四足模式 LOW=飞行模式\n");
    printf("    SWD(ch7) HIGH=折叠 MID=无 LOW=急停\n");
    printf("    旋钮ch8 -> 速度缩放 (当前:%.1fx)\n", g_speed_scale);
#endif
}

void remote_control_sbus_update(RobotGlobalState_t *state)
{
    uint16_t ch[16];

    /* 每帧重置边沿触发标志 */
    g_rc_command.jump_trigger = 0;
    g_rc_command.gait_switch = 0;
    g_rc_command.emergency_stop = 0;
    g_rc_command.shutdown_trigger = 0;
    g_rc_command.fold_trigger = 0;
    g_rc_command.stand_trigger = 0;
    g_rc_command.gait_start_trigger = 0;
    g_rc_command.mode_switch = 0;
    g_rc_command.pitch_adjust = 0;

    if (!AX_SBUS_GetLiveChannels(ch))
    {
        g_first_frame = 1;
        g_rc_command.shutdown_trigger = 1;
        return;
    }

    /* 读取三段开关当前档位 */
    for (int i = 0; i < 4; i++)
        g_sw_curr[i] = sbus_get_zone(ch[4 + i]);

    g_rc_command.shutdown_trigger = (g_sw_curr[0] == SBUS_SW_LOW);
    g_rc_command.emergency_stop = (g_sw_curr[3] == SBUS_SW_LOW);

    /* 首帧初始化：只记录档位，不触发任何边沿事件 */
    if (g_first_frame)
    {
        for (int i = 0; i < 4; i++)
            g_sw_prev[i] = g_sw_curr[i];
        g_jump_gesture_prev = sbus_jump_gesture(ch[1], ch[2]);
        g_first_frame = 0;
        return;
    }

    /* ==== 摇杆 -> 速度指令 ==== */
    /* ch0 右摇杆左右 -> 转向速度 */
    g_rc_command.cmd_vel_yaw = sbus_stick_to_vel(ch[0], RC_MAX_VEL_YAW);
    /* ch1 右摇杆上下 -> 前进速度（取反：推杆向上=前进=正值） */
    g_rc_command.cmd_vel_x = sbus_stick_to_vel(ch[1], RC_MAX_VEL_X);
    /* ch2 左摇杆上下 -> 横向速度 */
    g_rc_command.cmd_vel_y = sbus_stick_to_vel(ch[2], RC_MAX_VEL_Y);
    /* ch3 左摇杆左右 -> 预留 */

    /* ==== 跳跃手势（ch1+ch2 同时到底，上升沿触发一次） ==== */
    uint8_t jump_gesture = sbus_jump_gesture(ch[1], ch[2]);
    if (jump_gesture && !g_jump_gesture_prev)
    {
        g_rc_command.jump_trigger = 1;
    }
    g_jump_gesture_prev = jump_gesture;
    /* ch3 左摇杆左右 -> 预留 */

    /* ==== SWA (ch4): 模式控制 ==== */
    /* HIGH = 启动步态, LOW = 关机 */
    if (g_sw_curr[0] == SBUS_SW_HIGH && sbus_edge_rising(0))
    {
        g_rc_command.gait_start_trigger = 1;
    }
    /* SWA回到MID=待机（站立） */
    if (g_sw_curr[0] == SBUS_SW_MID && sbus_edge_any(0))
    {
        g_rc_command.stand_trigger = 1;
    }

    /* ==== SWB (ch5): 步态选择 ==== */
    /* HIGH=Pronk, MID=Trot, LOW=Walk */
    if (sbus_edge_any(1))
    {
        if (g_sw_curr[1] == SBUS_SW_HIGH)
            g_rc_command.gait_switch = 3; /* Pronk */
        else if (g_sw_curr[1] == SBUS_SW_MID)
            g_rc_command.gait_switch = 1; /* Trot */
        else
            g_rc_command.gait_switch = 2; /* Walk */
    }

    /* ==== SWC (ch6): 模式类型 ==== */
    /* HIGH=轮式模式, MID=四足模式, LOW 档位已弃用，不触发任何模式切换 */
    if (sbus_edge_any(2))
    {
        if (g_sw_curr[2] == SBUS_SW_HIGH)
            g_rc_command.mode_switch = 2; /* 轮式 */
        else if (g_sw_curr[2] == SBUS_SW_MID)
            g_rc_command.mode_switch = 1; /* 四足 */
        else if (g_sw_curr[2] == SBUS_SW_LOW)
            g_rc_command.mode_switch = 3; /* 弃用档位 */
    }

    /* ==== SWD (ch7): 特殊动作 ==== */
    /* HIGH=折叠, LOW=急停 */
    if (g_sw_curr[3] == SBUS_SW_HIGH && sbus_edge_rising(3))
    {
        g_rc_command.fold_trigger = 1;
    }

    /* ==== 旋钮ch8: 速度缩放 ==== */
    float knob_t = (float)(ch[8] - SBUS_RAW_MIN) /
                   (float)(SBUS_RAW_MAX - SBUS_RAW_MIN);
    g_speed_scale = RC_SPEED_SCALE_MIN +
                    knob_t * (RC_SPEED_SCALE_MAX - RC_SPEED_SCALE_MIN);

    /* 速度指令乘以缩放因子后写入全局状态 */
    state->cmd_vel_x   = g_rc_command.cmd_vel_x   * g_speed_scale;
    state->cmd_vel_y   = g_rc_command.cmd_vel_y   * g_speed_scale;
    state->cmd_vel_yaw = g_rc_command.cmd_vel_yaw * g_speed_scale;

    /* 保存当前档位供下一帧边沿检测 */
    for (int i = 0; i < 4; i++)
        g_sw_prev[i] = g_sw_curr[i];
}

const RemoteCommand_t *remote_control_sbus_get_command(void)
{
    return &g_rc_command;
}
