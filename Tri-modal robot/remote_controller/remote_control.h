#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include "include_head.h"
#include "robot_state.h"

// 遥控命令结构体（后续蓝牙通信的直接数据包格式）
typedef struct
{
    float cmd_vel_x;      // 前后速度 m/s（正=前进??
    float cmd_vel_y;      // 左右速度 m/s（正=左移??
    float cmd_vel_yaw;    // 转向速度 rad/s（正=左转??
    uint8_t jump_trigger; // 跳跃触发标志（边沿触发，本帧有效??
    uint8_t gait_switch;  // 步态切换请求 0=??1=Trot 2=Walk 3=Pronk
    uint8_t emergency_stop;  // 急停标志
    uint8_t shutdown_trigger; // 关机/开机切换（边沿触发）
    uint8_t fold_trigger;     // 折叠/展开切换（边沿触发）
    uint8_t stand_trigger;    // 回到初始站立位置（边沿触发，I键）
    uint8_t gait_start_trigger; // 启动步态周期（边沿触发，O键）
    uint8_t mode_switch;        // 模式切换: 1=四足模式 2=轮式模式（边沿触发）
    int8_t pitch_adjust;        // 俯仰调整: -1=后仰([键) +1=低头(]键) 0=无操作（边沿触发）
} RemoteCommand_t;

void remote_control_init(void);
void remote_control_update(RobotGlobalState_t *state);
const RemoteCommand_t *remote_control_get_command(void);

#endif
