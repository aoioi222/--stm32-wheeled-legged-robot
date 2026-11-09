#include "edulite05.h"
#include "motor_can_hal.h"
#include <math.h>
#include <string.h>

#define EL05_FEEDBACK_TIMEOUT_MS 50u
#define EL05_CAN_TIMEOUT_VALUE 2000u
#define EL05_PARAMETER_RUN_MODE 0x7005u
#define EL05_PARAMETER_CAN_TIMEOUT 0x7028u
#define EL05_MODE_RUNNING 0x80u

typedef enum {
    EL05_CONTROL = 1,
    EL05_FEEDBACK = 2,
    EL05_ENABLE = 3,
    EL05_STOP = 4,
    EL05_WRITE_PARAMETER = 18
} El05Command_t;

typedef enum {
    EL05_DRIVE_STOPPING,
    EL05_DRIVE_STOPPED,
    EL05_DRIVE_SET_TIMEOUT,
    EL05_DRIVE_ENABLE,
    EL05_DRIVE_WAIT_FEEDBACK,
    EL05_DRIVE_ACTIVE
} El05DriveState_t;

static CAN_HandleTypeDef *motor_can;
static uint8_t motor_ids[EDULITE05_MAX_MOTORS];
static uint8_t motor_count;
static EduLite05Config_t settings;

static uint32_t tx_start_tick;

static volatile uint8_t feedback_status[EDULITE05_MAX_MOTORS];
static volatile uint32_t feedback_tick[EDULITE05_MAX_MOTORS];
static volatile uint16_t feedback_position[EDULITE05_MAX_MOTORS];
static float target_position[EDULITE05_MAX_MOTORS];

static bool stop_sent;
static bool fault_latched;
static El05DriveState_t drive_state = EL05_DRIVE_STOPPING;
static uint32_t enable_tick;

static float clamp(float value, float min, float max)
{
    return fminf(fmaxf(value, min), max);
}

static uint16_t encode(float value, float min, float max)
{
    return (uint16_t)((value - min) * (65535.0f / (max - min)) + 0.5f);
}

static void put_be16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value >> 8);
    data[1] = (uint8_t)value;
}

static void receive_feedback(const CAN_RxHeaderTypeDef *header, const uint8_t data[8])
{
    if (header->IDE != CAN_ID_EXT || header->RTR != CAN_RTR_DATA || header->DLC != 8u ||
        (header->ExtId >> 24) != EL05_FEEDBACK || (header->ExtId & 0xFFu) != settings.host_id)
        return;

    uint8_t id = (uint8_t)(header->ExtId >> 8);
    for (uint8_t i = 0; i < motor_count; i++)
    {
        if (motor_ids[i] != id)
            continue;
        feedback_position[i] = ((uint16_t)data[0] << 8) | data[1];
        feedback_tick[i] = HAL_GetTick();
        feedback_status[i] = (uint8_t)(header->ExtId >> 16);
        break;
    }
}

static void send_batch(El05Command_t command, uint16_t parameter, uint32_t value, uint32_t now)
{
    MotorCanFrame_t frames[EDULITE05_MAX_MOTORS];
    for (uint8_t i = 0; i < motor_count; i++)
    {
        MotorCanFrame_t *frame = &frames[i];
        *frame = (MotorCanFrame_t){
            .id = ((uint32_t)command << 24) | (settings.host_id << 8) | motor_ids[i],
            .extended = true,
            .length = 8u,
        };

        if (command == EL05_WRITE_PARAMETER)
        {
            frame->data[0] = (uint8_t)parameter;
            frame->data[1] = (uint8_t)(parameter >> 8);
            for (uint8_t byte = 0; byte < 4u; byte++)
                frame->data[4u + byte] = (uint8_t)(value >> (byte * 8u));
        }
        else if (command == EL05_CONTROL)
        {
            frame->id = ((uint32_t)command << 24) |
                           ((uint32_t)encode(0.0f, -6.0f, 6.0f) << 8) | motor_ids[i];
            put_be16(&frame->data[0], encode(target_position[i], -EDULITE05_POSITION_MAX_RAD, EDULITE05_POSITION_MAX_RAD));
            put_be16(&frame->data[2], encode(0.0f, -50.0f, 50.0f));
            put_be16(&frame->data[4], encode(settings.position_kp, 0.0f, 500.0f));
            put_be16(&frame->data[6], encode(settings.position_kd, 0.0f, 5.0f));
        }
    }

    tx_start_tick = now;
    MotorCan_SendBatch(motor_can, frames, motor_count, command == EL05_STOP);
}

static void stop_motors(uint32_t now)
{
    if (drive_state != EL05_DRIVE_STOPPING && drive_state != EL05_DRIVE_STOPPED)
    {
        drive_state = EL05_DRIVE_STOPPING;
        stop_sent = false;
        MotorCan_Abort(motor_can);
    }
    if (drive_state == EL05_DRIVE_STOPPED)
        return;
    if (MotorCan_IsBusy(motor_can))
    {
        if (now - tx_start_tick >= EL05_FEEDBACK_TIMEOUT_MS)
            MotorCan_Abort(motor_can);
        return;
    }

    if (stop_sent && !MotorCan_TxFailed(motor_can))
        drive_state = EL05_DRIVE_STOPPED;
    else
    {
        send_batch(EL05_STOP, 0, 0, now);
        stop_sent = true;
    }
}

static bool feedback_ready(bool initialize_targets)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint32_t now = HAL_GetTick();
    for (uint8_t i = 0; i < motor_count; i++)
    {
        if (feedback_status[i] != EL05_MODE_RUNNING ||
            now - feedback_tick[i] >= EL05_FEEDBACK_TIMEOUT_MS)
        {
            __set_PRIMASK(primask);
            return false;
        }
        if (initialize_targets)
            target_position[i] = feedback_position[i] * (2.0f * EDULITE05_POSITION_MAX_RAD / 65535.0f) -
                                 EDULITE05_POSITION_MAX_RAD;
    }
    __set_PRIMASK(primask);
    return true;
}

bool EduLite05_Init(CAN_HandleTypeDef *can, const uint8_t ids[], uint8_t count,
                    const EduLite05Config_t *config)
{
    if (motor_can != NULL || ids == NULL || config == NULL ||
        count == 0u || count > EDULITE05_MAX_MOTORS ||
        can == NULL ||
        config->host_id == 0u ||
        !(config->position_kp >= 0.0f && config->position_kp <= 500.0f) ||
        !(config->position_kd >= 0.0f && config->position_kd <= 5.0f) ||
        !(config->max_position_step_rad > 0.0f &&
          config->max_position_step_rad <= EDULITE05_POSITION_MAX_RAD))
        return false;

    for (uint8_t i = 0; i < count; i++)
    {
        if (ids[i] == 0u || ids[i] == config->host_id || ids[i] == 0xFFu)
            return false;
        for (uint8_t other = 0; other < i; other++)
            if (ids[other] == ids[i])
                return false;
    }

    memcpy(motor_ids, ids, count);
    motor_count = count;
    settings = *config;
    if (!MotorCan_Start(can, receive_feedback))
        return false;
    motor_can = can;
    return true;
}

void EduLite05_Stop(void)
{
    if (motor_can != NULL)
    {
        fault_latched = false;
        stop_motors(HAL_GetTick());
    }
}

bool EduLite05_PositionControl(const float position_rad[])
{
    if (motor_can == NULL)
        return false;

    uint32_t now = HAL_GetTick();
    if (drive_state == EL05_DRIVE_STOPPING || fault_latched)
    {
        stop_motors(now);
        return false;
    }
    if (drive_state == EL05_DRIVE_ACTIVE && !feedback_ready(false))
        goto fault;
    if (MotorCan_IsBusy(motor_can) && now - tx_start_tick < EL05_FEEDBACK_TIMEOUT_MS)
        return drive_state == EL05_DRIVE_ACTIVE;
    if (MotorCan_IsBusy(motor_can) || MotorCan_TxFailed(motor_can))
        goto fault;

    switch (drive_state)
    {
        case EL05_DRIVE_STOPPED:
            send_batch(EL05_WRITE_PARAMETER, EL05_PARAMETER_RUN_MODE, 0u, now);
            drive_state = EL05_DRIVE_SET_TIMEOUT;
            return false;

        case EL05_DRIVE_SET_TIMEOUT:
            send_batch(EL05_WRITE_PARAMETER, EL05_PARAMETER_CAN_TIMEOUT, EL05_CAN_TIMEOUT_VALUE, now);
            drive_state = EL05_DRIVE_ENABLE;
            return false;

        case EL05_DRIVE_ENABLE:
        {
            uint32_t primask = __get_PRIMASK();
            __disable_irq();
            memset((void *)feedback_status, 0, sizeof(feedback_status));
            __set_PRIMASK(primask);
            send_batch(EL05_ENABLE, 0, 0, now);
            enable_tick = now;
            drive_state = EL05_DRIVE_WAIT_FEEDBACK;
            return false;
        }

        case EL05_DRIVE_WAIT_FEEDBACK:
            if (!feedback_ready(true))
            {
                if (now - enable_tick >= EL05_FEEDBACK_TIMEOUT_MS)
                    goto fault;
                return false;
            }
            drive_state = EL05_DRIVE_ACTIVE;
            break;

        case EL05_DRIVE_ACTIVE:
            break;

        default:
            return false;
    }

    for (uint8_t i = 0; i < motor_count; i++)
    {
        if (!isfinite(position_rad[i]))
            goto fault;

        float desired = clamp(position_rad[i], -EDULITE05_POSITION_MAX_RAD,
                              EDULITE05_POSITION_MAX_RAD);
        float delta = desired - target_position[i];
        target_position[i] += clamp(delta, -settings.max_position_step_rad, settings.max_position_step_rad);
        target_position[i] = clamp(target_position[i], -EDULITE05_POSITION_MAX_RAD, EDULITE05_POSITION_MAX_RAD);
    }
    send_batch(EL05_CONTROL, 0, 0, now);
    return true;

fault:
    fault_latched = true;
    stop_motors(now);
    return false;
}
