#include "RCS_CAN_Interface.h"
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
#define DM_FEEDBACK_MAX_AGE (OS_TICKS_PER_SEC / 20u)
#define DM_START_TIMEOUT (OS_TICKS_PER_SEC / 10u)
#define DM_STOP_RETRY_INTERVAL (OS_TICKS_PER_SEC / 50u)

typedef struct {
    INT32U tick;
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

static CAN_TypeDef *motor_can;
static DM_H6215Address_t motor_addresses[DM_H6215_MAX_MOTORS];
static uint8_t motor_count;
static DM_H6215Config_t settings;

static volatile DmFeedback_t feedback[DM_H6215_MAX_MOTORS];
static DmDriveState_t drive_state;
static bool stop_sent;
static bool stop_confirmed;
static INT32U state_tick;
static INT32U stop_tick;

static void receive_feedback(CanRxMsg frame)
{
    if (frame.IDE != CAN_Id_Standard || frame.RTR != CAN_RTR_Data || frame.DLC != 8u)
        return;

    for (uint8_t i = 0; i < motor_count; i++)
    {
        const DM_H6215Address_t *motor = &motor_addresses[i];
        if (frame.StdId != motor->feedback_id)
            continue;

        uint16_t parameter_id = frame.Data[0] | ((uint16_t)frame.Data[1] << 8);
        if (parameter_id == motor->motor_id && frame.Data[2] == DM_READ_PARAM)
        {
            uint32_t value = (uint32_t)frame.Data[4] | ((uint32_t)frame.Data[5] << 8) |
                             ((uint32_t)frame.Data[6] << 16) | ((uint32_t)frame.Data[7] << 24);
            if (frame.Data[3] == DM_MODE_REGISTER)
            {
                feedback[i].velocity_mode = value == DM_VELOCITY_MODE;
                return;
            }
            if (frame.Data[3] == DM_TIMEOUT_REGISTER)
            {
                feedback[i].timeout_configured = value != 0u;
                return;
            }
        }

        if ((frame.Data[0] & 0x0Fu) == (motor->motor_id & 0x0Fu))
        {
            feedback[i].status = frame.Data[0] >> 4;
            feedback[i].tick = OSTimeGet();
            feedback[i].received = true;
        }
        return;
    }
}

static void send_service(uint8_t command)
{
    uint8_t data[8] = {0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, command};
    for (uint8_t i = 0; i < motor_count; i++)
        RCS_CANx_Send_STDID(motor_can, motor_addresses[i].motor_id + settings.service_id_offset, 8u, data);
}

static void read_parameter(uint16_t motor_id, uint8_t parameter)
{
    uint8_t data[8] = {(uint8_t)motor_id, (uint8_t)(motor_id >> 8), DM_READ_PARAM, parameter};
    RCS_CANx_Send_STDID(motor_can, DM_PARAM_CAN_ID, 8u, data);
}

static void send_speed(uint8_t index, float speed)
{
    const DM_H6215Address_t *motor = &motor_addresses[index];
    float target = fminf(fmaxf(speed, -settings.speed_limit_rad_s), settings.speed_limit_rad_s);
    uint32_t bits;
    memcpy(&bits, &target, sizeof(bits));
    uint8_t data[4] = {
        (uint8_t)bits, (uint8_t)(bits >> 8), (uint8_t)(bits >> 16), (uint8_t)(bits >> 24)
    };
    RCS_CANx_Send_STDID(motor_can, motor->motor_id + DM_VELOCITY_OFFSET, sizeof(data), data);
}

static bool all_motors_report(uint8_t status)
{
    OS_CPU_SR cpu_sr;
    OS_ENTER_CRITICAL();
    INT32U now = OSTimeGet();
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
    OS_EXIT_CRITICAL();
    return ready;
}

static void stop_motors(INT32U now)
{
    if (stop_confirmed)
        return;

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

static void begin_parameter_check(INT32U now)
{
    OS_CPU_SR cpu_sr;
    OS_ENTER_CRITICAL();
    for (uint8_t i = 0; i < motor_count; i++)
    {
        feedback[i].velocity_mode = false;
        feedback[i].timeout_configured = false;
    }
    OS_EXIT_CRITICAL();

    for (uint8_t i = 0; i < motor_count; i++)
    {
        read_parameter(motor_addresses[i].motor_id, DM_MODE_REGISTER);
        read_parameter(motor_addresses[i].motor_id, DM_TIMEOUT_REGISTER);
    }
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

bool DM_H6215_Init(CAN_TypeDef *can, const DM_H6215Address_t addresses[], uint8_t count,
                   const DM_H6215Config_t *config)
{
    if (motor_can != NULL || addresses == NULL || config == NULL ||
        (can != CAN1 && can != CAN2) || count == 0u || count > DM_H6215_MAX_MOTORS ||
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

    if (RCS_Get_CAN_Service_Len(can) >= CAN_HANDLER_MAX_LEN - 1u)
        return false;

    memcpy(motor_addresses, addresses, count * sizeof(motor_addresses[0]));
    motor_count = count;
    settings = *config;
    motor_can = can;
    RCS_CANx_Add_Handler(motor_can, receive_feedback);
    return true;
}

void DM_H6215_VelocityControl(const float speed_rad_s[])
{
    if (motor_can == NULL)
        return;

    INT32U now = OSTimeGet();
    switch (drive_state)
    {
        case DM_GROUP_OFF:
            stop_motors(now);
            begin_parameter_check(now);
            return;

        case DM_GROUP_CHECK_PARAMETERS:
            if (parameters_ready())
            {
                OS_CPU_SR cpu_sr;
                OS_ENTER_CRITICAL();
                for (uint8_t i = 0; i < motor_count; i++)
                    feedback[i].received = false;
                OS_EXIT_CRITICAL();
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
            for (uint8_t i = 0; i < motor_count; i++)
                send_speed(i, 0.0f);
            return;

        case DM_GROUP_RUNNING:
            if (!all_motors_report(DM_STATUS_ENABLED))
                break;
            for (uint8_t i = 0; i < motor_count; i++)
                if (!isfinite(speed_rad_s[i]))
                    goto fault;
            for (uint8_t i = 0; i < motor_count; i++)
                send_speed(i, speed_rad_s[i]);
            return;

        case DM_GROUP_FAULT:
            stop_motors(now);
            return;
    }

fault:
    drive_state = DM_GROUP_FAULT;
    stop_motors(now);
}

void DM_H6215_Stop(void)
{
    if (motor_can == NULL)
        return;

    drive_state = DM_GROUP_OFF;
    stop_motors(OSTimeGet());
}
