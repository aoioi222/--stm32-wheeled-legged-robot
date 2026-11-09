#ifndef JUMP_CONTROLLER_H
#define JUMP_CONTROLLER_H

#include "include_head.h"
#include "Foot_structure.h"

typedef enum {
    JUMP_IDLE = 0,
    JUMP_CROUCH,   // �����¶�
    JUMP_PUSH_OFF, // �ŵ��ڿ�
    JUMP_AERIAL,   // ���е���
    JUMP_LANDING,  // ��ػ���
    JUMP_RECOVER,  // �ָ�վ��
    JUMP_DONE
} JumpPhase_t;

typedef struct {
    JumpPhase_t phase;
    float phase_start_time;
    uint8_t active;
    uint8_t completed;
    LegJointAngles_t output_joints[4]; // ��ǰ֡�����Ŀ��ؽڽǶ�
} JumpController_t;

// ȫ����Ծ������
extern JumpController_t g_jump_ctrl;

void jump_controller_init(JumpController_t *jc);
void jump_controller_trigger(JumpController_t *jc, float current_time);
void jump_controller_update(JumpController_t *jc, float current_time);
uint8_t jump_controller_is_active(const JumpController_t *jc);

#endif
