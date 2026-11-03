#ifndef REMOTE_CONTROL_SBUS_H
#define REMOTE_CONTROL_SBUS_H

#include "include_head.h"
#include "robot_state.h"

/* SBUS通道原始值范围 */
#define SBUS_RAW_MIN     192
#define SBUS_RAW_MAX     1792
#define SBUS_RAW_MID     992
#define SBUS_RAW_DEAD    40     /* 摇杆死区 */

/* 三段开关档位阈值 */
#define SBUS_ZONE_LOW_MAX    592    /* < 592 = LOW */
#define SBUS_ZONE_HIGH_MIN   1392   /* > 1392 = HIGH，中间 = MID */

/* 三段开关档位枚举 */
typedef enum {
    SBUS_SW_LOW  = 0,
    SBUS_SW_MID  = 1,
    SBUS_SW_HIGH = 2
} SbusSwitchZone_t;

/* 遥控命令结构体（与Webots remote_control.h完全一致） */
typedef struct {
    float cmd_vel_x;
    float cmd_vel_y;
    float cmd_vel_yaw;
    uint8_t jump_trigger;
    uint8_t gait_switch;
    uint8_t emergency_stop;
    uint8_t shutdown_trigger;
    uint8_t fold_trigger;
    uint8_t stand_trigger;
    uint8_t gait_start_trigger;
    uint8_t mode_switch;
    int8_t pitch_adjust;
} RemoteCommand_t;

void remote_control_sbus_init(void);
void remote_control_sbus_update(RobotGlobalState_t *state);
const RemoteCommand_t *remote_control_sbus_get_command(void);

#endif
