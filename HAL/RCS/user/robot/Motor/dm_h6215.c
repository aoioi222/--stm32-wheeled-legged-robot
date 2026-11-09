#include "motor_can_hal.h"
#include <math.h>
#include <string.h>
#include "dm_h6215.h"

#define DM_VELOCITY_OFFSET 0x200u
#define DM_PARAM_CAN_ID 0x7FFu
#define DM_ENABLE 0xFCu
#define DM_DISABLE 0xFDu
#define DM_READ_PARAM 0x33u
#define DM_TIMEOUT_REGISTER 9u
#define DM_MODE_REGISTER 10u
#define DM_VELOCITY_MODE 3u
#define DM_STATUS_DISABLED 0u
#define DM_STATUS_ENABLED 1u
#define DM_FEEDBACK_MAX_AGE 50u
#define DM_START_TIMEOUT 100u
#define DM_STOP_RETRY_INTERVAL 20u

typedef struct {
    uint32_t tick;
    uint8_t status;
    bool received;
    bool velocity_mode;
    bool timeout_configured;
} DmFeedback_t;

typedef enum {
    DM_GROUP_OFF,
    DM_GROUP_CHECK_PARAMETERS,
    DM_GROUP_WAIT_ENABLE,
    DM_GROUP_RUNNING,
    DM_GROUP_FAULT
} DmDriveState_t;

static CAN_HandleTypeDef *motor_can;
static DM_H6215Address_t motor_addresses[DM_H6215_MAX_MOTORS];
static uint8_t motor_count;
static DM_H6215Config_t settings;

static volatile DmFeedback_t feedback[DM_H6215_MAX_MOTORS];
static DmDriveState_t drive_state;
static bool stop_sent;
static bool stop_confirmed;
static uint32_t state_tick;
static uint32_t stop_tick;
static uint32_t tx_start_ms;

static void receive_feedback(const CAN_RxHeaderTypeDef *header, const uint8_t data[8])
{
    if (header->IDE != CAN_ID_STD || header->RTR != CAN_RTR_DATA || header->DLC != 8u)
        return;

    for (uint8_t i = 0; i < motor_count; i++)
    {
        const DM_H6215Address_t *motor = &motor_addresses[i];
        if (header->StdId != motor->feedback_id)
            continue;

        uint16_t parameter_id = data[0] | ((uint16_t)data[1] << 8);
        if (parameter_id == motor->motor_id && data[2] == DM_READ_PARAM)
        {
            uint32_t value = (uint32_t)data[4] | ((uint32_t)data[5] << 8) |
                             ((uint32_t)data[6] << 16) | ((uint32_t)data[7] << 24);
            if (data[3] == DM_MODE_REGISTER)
            {
                feedback[i].velocity_mode = value == DM_VELOCITY_MODE;
                return;
            }
            if (data[3] == DM_TIMEOUT_REGISTER)
            {
                feedback[i].timeout_configured = value != 0u;
                return;
            }
        }

        if ((data[0] & 0x0Fu) == (motor->motor_id & 0x0Fu))
        {
            feedback[i].status = data[0] >> 4;
            feedback[i].tick = HAL_GetTick();
            feedback[i].received = true;
        }
        return;
    }
}

static void send_frames(const MotorCanFrame_t frames[], uint8_t count, bool stop)
{
    tx_start_ms = HAL_GetTick();
    MotorCan_SendBatch(motor_can, frames, count, stop);
}

static void send_service(uint8_t command)
{
    MotorCanFrame_t frames[DM_H6215_MAX_MOTORS];
    for (uint8_t i = 0; i < motor_count; i++)
    {
        frames[i] = (MotorCanFrame_t){
            .id = motor_addresses[i].motor_id + settings.service_id_offset,
            .length = 8u,
            .data = {0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, command},
        };
    }
    send_frames(frames, motor_count, command == DM_DISABLE);
}

static MotorCanFrame_t parameter_request(uint16_t motor_id, uint8_t parameter)
{
    return (MotorCanFrame_t){
        .id = DM_PARAM_CAN_ID,
        .length = 8u,
        .data = {(uint8_t)motor_id, (uint8_t)(motor_id >> 8), DM_READ_PARAM, parameter},
    };
}

static void send_speeds(const float speed_rad_s[])
{
    MotorCanFrame_t frames[DM_H6215_MAX_MOTORS];
    for (uint8_t i = 0; i < motor_count; i++)
    {
        float speed = speed_rad_s == NULL ? 0.0f : speed_rad_s[i];
        float target = fminf(fmaxf(speed, -settings.speed_limit_rad_s), settings.speed_limit_rad_s);
        uint32_t bits;
        memcpy(&bits, &target, sizeof(bits));
        frames[i] = (MotorCanFrame_t){
            .id = motor_addresses[i].motor_id + DM_VELOCITY_OFFSET,
            .length = 4u,
            .data = {(uint8_t)bits, (uint8_t)(bits >> 8), (uint8_t)(bits >> 16), (uint8_t)(bits >> 24)},
        };
    }
    send_frames(frames, motor_count, false);
}

static bool all_motors_report(uint8_t status)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint32_t now = HAL_GetTick();
    bool ready = true;
    for (uint8_t i = 0; i < motor_count; i++)
    {
        if (!feedback[i].received || feedback[i].status != status ||
            now - feedback[i].tick >= DM_FEEDBACK_MAX_AGE)
        {
            ready = false;
            break;
        }
    }
    __set_PRIMASK(primask);
    return ready;
}

static void stop_motors(uint32_t now)
{
    if (stop_confirmed)
        return;
    if (MotorCan_IsBusy(motor_can))
    {
        if (now - tx_start_ms >= DM_FEEDBACK_MAX_AGE)
            MotorCan_Abort(motor_can);
        return;
    }

    if (stop_sent && all_motors_report(DM_STATUS_DISABLED))
    {
        stop_confirmed = true;
        return;
    }
    if (!stop_sent || now - stop_tick >= DM_STOP_RETRY_INTERVAL)
    {
        send_service(DM_DISABLE);
        stop_sent = true;
        stop_tick = now;
    }
}

static void begin_parameter_check(uint32_t now)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    for (uint8_t i = 0; i < motor_count; i++)
    {
        feedback[i].velocity_mode = false;
        feedback[i].timeout_configured = false;
    }
    __set_PRIMASK(primask);

    MotorCanFrame_t frames[2u * DM_H6215_MAX_MOTORS];
    for (uint8_t i = 0; i < motor_count; i++)
    {
        frames[2u * i] = parameter_request(motor_addresses[i].motor_id, DM_MODE_REGISTER);
        frames[2u * i + 1u] = parameter_request(motor_addresses[i].motor_id, DM_TIMEOUT_REGISTER);
    }
    send_frames(frames, (uint8_t)(2u * motor_count), false);
    drive_state = DM_GROUP_CHECK_PARAMETERS;
    state_tick = now;
}

static bool parameters_ready(void)
{
    for (uint8_t i = 0; i < motor_count; i++)
    {
        if (!feedback[i].velocity_mode || !feedback[i].timeout_configured)
            return false;
    }
    return true;
}

bool DM_H6215_Init(CAN_HandleTypeDef *can, const DM_H6215Address_t addresses[], uint8_t count,
                   const DM_H6215Config_t *config)
{
    if (motor_can != NULL || addresses == NULL || config == NULL ||
        can == NULL || count == 0u || count > DM_H6215_MAX_MOTORS ||
        (config->service_id_offset != 0u && config->service_id_offset != DM_VELOCITY_OFFSET) ||
        !isfinite(config->speed_limit_rad_s) || config->speed_limit_rad_s <= 0.0f)
        return false;

    for (uint8_t i = 0; i < count; i++)
    {
        const DM_H6215Address_t *motor = &addresses[i];
        if (motor->motor_id == 0u || motor->motor_id > 0xFFu ||
            motor->feedback_id >= DM_PARAM_CAN_ID)
            return false;

        for (uint8_t other = 0; other < count; other++)
        {
            const DM_H6215Address_t *peer = &addresses[other];
            if (motor->feedback_id == peer->motor_id + config->service_id_offset ||
                motor->feedback_id == peer->motor_id + DM_VELOCITY_OFFSET ||
                (i != other && (motor->motor_id == peer->motor_id ||
                                motor->feedback_id == peer->feedback_id)))
                return false;
        }
    }

    memcpy(motor_addresses, addresses, count * sizeof(motor_addresses[0]));
    motor_count = count;
    settings = *config;
    if (!MotorCan_Start(can, receive_feedback))
        return false;
    motor_can = can;
    return true;
}

void DM_H6215_VelocityControl(const float speed_rad_s[])
{
    if (motor_can == NULL)
        return;

    uint32_t now = HAL_GetTick();
    if (drive_state == DM_GROUP_FAULT)
    {
        stop_motors(now);
        return;
    }
    if (MotorCan_IsBusy(motor_can))
    {
        if (now - tx_start_ms >= DM_FEEDBACK_MAX_AGE)
            goto fault;
        return;
    }
    if (drive_state != DM_GROUP_OFF && MotorCan_TxFailed(motor_can))
        goto fault;

    switch (drive_state)
    {
        case DM_GROUP_OFF:
            if (!stop_sent)
                stop_motors(now);
            else
                begin_parameter_check(now);
            return;

        case DM_GROUP_CHECK_PARAMETERS:
            if (parameters_ready())
            {
                uint32_t primask = __get_PRIMASK();
                __disable_irq();
                for (uint8_t i = 0; i < motor_count; i++)
                    feedback[i].received = false;
                __set_PRIMASK(primask);
                send_service(DM_ENABLE);
                stop_sent = false;
                stop_confirmed = false;
                drive_state = DM_GROUP_WAIT_ENABLE;
                state_tick = now;
                return;
            }
            if (now - state_tick >= DM_START_TIMEOUT)
                break;
            return;

        case DM_GROUP_WAIT_ENABLE:
            if (all_motors_report(DM_STATUS_ENABLED))
            {
                drive_state = DM_GROUP_RUNNING;
                /* 所有电机确认使能后才发送非零速度。 */
                return;
            }
            if (now - state_tick >= DM_START_TIMEOUT)
                break;
            send_speeds(NULL);
            return;

        case DM_GROUP_RUNNING:
            if (!all_motors_report(DM_STATUS_ENABLED))
                break;
            for (uint8_t i = 0; i < motor_count; i++)
                if (!isfinite(speed_rad_s[i]))
                    goto fault;
            send_speeds(speed_rad_s);
            return;

        case DM_GROUP_FAULT:
            return;
    }

fault:
    drive_state = DM_GROUP_FAULT;
    MotorCan_Abort(motor_can);
    stop_motors(now);
}

void DM_H6215_Stop(void)
{
    if (motor_can == NULL)
        return;

    if (drive_state != DM_GROUP_OFF && drive_state != DM_GROUP_FAULT)
        MotorCan_Abort(motor_can);
    drive_state = DM_GROUP_OFF;
    stop_motors(HAL_GetTick());
}
