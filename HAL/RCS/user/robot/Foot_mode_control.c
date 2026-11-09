#include "Foot_mode_control.h"
#include "robot_math.h"
#include "robot_config.h"
#include "edulite05.h"

typedef struct {
    uint8_t id;
    int8_t direction;
    float zero_rad;
    float min_deg;
    float max_deg;
} JointCalibration_t;

/* 每条腿按髋展、髋转、膝关节排列。零位为 IK 角为零时的电机角度。 */
static const JointCalibration_t joint_calibration[FOOT_JOINT_COUNT] = {
    { 1,  1, 0.0f, JOINT1_MIN,  JOINT1_MAX},
    { 2, -1, 0.0f, -JOINT2_MAX, -JOINT2_MIN},
    { 3,  1, 0.0f, -FOLD_J3,    JOINT3_MIN},
    { 4,  1, 0.0f, JOINT1_MIN,  JOINT1_MAX},
    { 5,  1, 0.0f, JOINT2_MIN,  JOINT2_MAX},
    { 6, -1, 0.0f, JOINT3_MIN,  FOLD_J3},
    { 7, -1, 0.0f, JOINT1_MIN,  JOINT1_MAX},
    { 8, -1, 0.0f, -JOINT2_MAX, -JOINT2_MIN},
    { 9,  1, 0.0f, -FOLD_J3,    JOINT3_MIN},
    {10, -1, 0.0f, JOINT1_MIN,  JOINT1_MAX},
    {11,  1, 0.0f, JOINT2_MIN,  JOINT2_MAX},
    {12, -1, 0.0f, JOINT3_MIN,  FOLD_J3},
};

static const EduLite05Config_t joint_settings = {
    .host_id = 0xFDu,
    .position_kp = 8.0f,
    .position_kd = 0.4f,
    .max_position_step_rad = 0.025f,
};
static bool joints_configured;

void Foot_Mode_Init(CAN_HandleTypeDef *can)
{
    if (!FOOT_JOINTS_CALIBRATED || joints_configured)
        return;

    uint8_t motor_ids[FOOT_JOINT_COUNT];
    for (uint8_t i = 0; i < FOOT_JOINT_COUNT; i++)
    {
        const JointCalibration_t *cal = &joint_calibration[i];
        if ((cal->direction != 1 && cal->direction != -1) || !(cal->min_deg < cal->max_deg))
            return;

        float min_pos = cal->zero_rad + cal->direction * cal->min_deg * DEG2RAD;
        float max_pos = cal->zero_rad + cal->direction * cal->max_deg * DEG2RAD;
        if (!(fabsf(min_pos) <= EDULITE05_POSITION_MAX_RAD &&
              fabsf(max_pos) <= EDULITE05_POSITION_MAX_RAD))
            return;
        motor_ids[i] = cal->id;
    }
    joints_configured = EduLite05_Init(can, motor_ids, FOOT_JOINT_COUNT, &joint_settings);
}

bool Foot_Mode_Control(const float joint_angle_deg[FOOT_JOINT_COUNT])
{
    if (!joints_configured)
        return false;

    float position_rad[FOOT_JOINT_COUNT];
    for (uint8_t i = 0; i < FOOT_JOINT_COUNT; i++)
    {
        const JointCalibration_t *cal = &joint_calibration[i];
        float angle = joint_angle_deg[i];
        if (isfinite(angle))
            angle = fminf(fmaxf(angle, cal->min_deg), cal->max_deg);
        position_rad[i] = cal->zero_rad + cal->direction * angle * DEG2RAD;
    }
    return EduLite05_PositionControl(position_rad);
}

void Foot_Mode_Stop(void)
{
    if (joints_configured)
        EduLite05_Stop();
}
