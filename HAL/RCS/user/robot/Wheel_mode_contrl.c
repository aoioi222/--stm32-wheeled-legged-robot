#include "Wheel_mode_contrl.h"
#include "dm_h6215.h"

/* 四路轮速按右前、左前、右后、左后排列。 */
static const DM_H6215Address_t wheel_addresses[WHEEL_COUNT] = {
    {WHEEL_CAN_ID_RF, WHEEL_FEEDBACK_ID_RF},
    {WHEEL_CAN_ID_LF, WHEEL_FEEDBACK_ID_LF},
    {WHEEL_CAN_ID_RB, WHEEL_FEEDBACK_ID_RB},
    {WHEEL_CAN_ID_LB, WHEEL_FEEDBACK_ID_LB},
};

static const int8_t wheel_direction[WHEEL_COUNT] = {
    WHEEL_DIRECTION_RF, WHEEL_DIRECTION_LF, WHEEL_DIRECTION_RB, WHEEL_DIRECTION_LB,
};

static const DM_H6215Config_t wheel_settings = {
    .service_id_offset = WHEEL_DM_SERVICE_OFFSET,
    .speed_limit_rad_s = WHEEL_MAX_SPEED,
};
static bool wheels_configured;

void Wheel_Mode_Init(CAN_HandleTypeDef *can)
{
    if (!WHEEL_MOTORS_CONFIGURED || wheels_configured)
        return;

    for (uint8_t i = 0; i < WHEEL_COUNT; i++)
        if (wheel_direction[i] != 1 && wheel_direction[i] != -1)
            return;

    wheels_configured = DM_H6215_Init(can, wheel_addresses, WHEEL_COUNT, &wheel_settings);
}

void Wheel_Mode_Control(const float wheel_speed[WHEEL_COUNT])
{
    if (!wheels_configured)
        return;

    float speed_rad_s[WHEEL_COUNT];
    for (uint8_t i = 0; i < WHEEL_COUNT; i++)
        speed_rad_s[i] = wheel_speed[i] * wheel_direction[i];

    DM_H6215_VelocityControl(speed_rad_s);
}

void Wheel_Mode_Stop(void)
{
    if (wheels_configured)
        DM_H6215_Stop();
}
