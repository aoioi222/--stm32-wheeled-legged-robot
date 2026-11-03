#ifndef GAIT_CONTROLLER_H
#define GAIT_CONTROLLER_H

#include "gait_generator.h"
#include "gait_scheduler.h"
#include "robot_state.h"
#include "include_head.h"

// 步态枚举值映射（依赖 robot_state.h / gait_generator.h 的枚举定义，无法放入 robot_config.h??
#ifdef TEST_TROT
#define QUAD_MODE QUAD_MODE_TROT
#define GaitType GAIT_TROT
#endif

#ifdef TEST_WALK
#define QUAD_MODE QUAD_MODE_WALK
#define GaitType GAIT_WALK
#endif

#ifdef TEST_PRONK
#define QUAD_MODE QUAD_MODE_PRONK
#define GaitType GAIT_PRONK
#endif

extern GaitGenerator_t gait_gen;
extern GaitScheduler_t gait_scheduler;
extern LegStateMachine_t leg_fsm[4];
extern FootPosition_t foot_target_pos[4];

void gait_controller_init(void);
void gait_controller_update(float abs_time);
void gait_controller_switch_gait(GaitType_t new_gait_type);

#endif // GAIT_CONTROLLER_H
