#include "edulite05.h"

#define EL05_FEEDBACK_TIMEOUT_TICKS (OS_TICKS_PER_SEC / 20u)
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

static CAN_TypeDef *motor_can;
static uint8_t motor_ids[EDULITE05_MAX_MOTORS];
static uint8_t motor_count;
static EduLite05Config_t settings;

static CanTxMsg tx_batch[EDULITE05_MAX_MOTORS];
static volatile bool tx_active;
static volatile bool tx_failed;
static volatile uint8_t tx_next;
static volatile uint8_t tx_mailbox;
static INT32U tx_start_tick;

static volatile uint8_t feedback_status[EDULITE05_MAX_MOTORS];
static volatile INT32U feedback_tick[EDULITE05_MAX_MOTORS];
static volatile uint16_t feedback_position[EDULITE05_MAX_MOTORS];
static float target_position[EDULITE05_MAX_MOTORS];

static bool stop_sent;
static bool fault_latched;
static El05DriveState_t drive_state = EL05_DRIVE_STOPPING;
static INT32U enable_tick;

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

static void receive_isr(void)
{
    if (CAN_MessagePending(motor_can, CAN_FIFO0) == 0u)
        return;

    CanRxMsg msg;
    CAN_Receive(motor_can, CAN_FIFO0, &msg);
    if (msg.IDE != CAN_Id_Extended || msg.RTR != CAN_RTR_Data || msg.DLC != 8u ||
        (msg.ExtId >> 24) != EL05_FEEDBACK || (msg.ExtId & 0xFFu) != settings.host_id)
        return;

    uint8_t id = (uint8_t)(msg.ExtId >> 8);
    for (uint8_t i = 0; i < motor_count; i++)
    {
        if (motor_ids[i] != id)
            continue;
        feedback_position[i] = ((uint16_t)msg.Data[0] << 8) | msg.Data[1];
        feedback_tick[i] = OSTimeGet();
        feedback_status[i] = (uint8_t)(msg.ExtId >> 16);
        break;
    }
}

static void send_next_frame(void)
{
    tx_mailbox = CAN_Transmit(motor_can, &tx_batch[tx_next++]);
    if (tx_mailbox == CAN_TxStatus_NoMailBox)
    {
        tx_failed = true;
        tx_active = false;
    }
}

static void transmit_isr(void)
{
    bool sent = CAN_TransmitStatus(motor_can, tx_mailbox) == CAN_TxStatus_Ok;
    CAN_ClearITPendingBit(motor_can, CAN_IT_TME);
    if (!tx_active)
        return;

    if (!sent)
        tx_failed = true;

    /* 停止帧即使有发送失败，也继续尝试其余电机。 */
    if (tx_next < motor_count && (sent || (tx_batch[0].ExtId >> 24) == EL05_STOP))
        send_next_frame();
    else
        tx_active = false;
}

static void send_batch(El05Command_t command, uint16_t parameter, uint32_t value, INT32U now)
{
    for (uint8_t i = 0; i < motor_count; i++)
    {
        CanTxMsg *frame = &tx_batch[i];
        *frame = (CanTxMsg){
            .ExtId = ((uint32_t)command << 24) | (settings.host_id << 8) | motor_ids[i],
            .IDE = CAN_Id_Extended,
            .RTR = CAN_RTR_Data,
            .DLC = 8u,
        };

        if (command == EL05_WRITE_PARAMETER)
        {
            frame->Data[0] = (uint8_t)parameter;
            frame->Data[1] = (uint8_t)(parameter >> 8);
            for (uint8_t byte = 0; byte < 4u; byte++)
                frame->Data[4u + byte] = (uint8_t)(value >> (byte * 8u));
        }
        else if (command == EL05_CONTROL)
        {
            frame->ExtId = ((uint32_t)command << 24) |
                           ((uint32_t)encode(0.0f, -6.0f, 6.0f) << 8) | motor_ids[i];
            put_be16(&frame->Data[0], encode(target_position[i], -EDULITE05_POSITION_MAX_RAD, EDULITE05_POSITION_MAX_RAD));
            put_be16(&frame->Data[2], encode(0.0f, -50.0f, 50.0f));
            put_be16(&frame->Data[4], encode(settings.position_kp, 0.0f, 500.0f));
            put_be16(&frame->Data[6], encode(settings.position_kd, 0.0f, 5.0f));
        }
    }

    OS_CPU_SR cpu_sr;
    OS_ENTER_CRITICAL();
    tx_next = 0;
    tx_failed = false;
    tx_active = true;
    tx_start_tick = now;
    send_next_frame();
    OS_EXIT_CRITICAL();
}

static void cancel_batch(void)
{
    OS_CPU_SR cpu_sr;
    OS_ENTER_CRITICAL();
    if (tx_active)
    {
        tx_next = motor_count;
        CAN_CancelTransmit(motor_can, tx_mailbox);
    }
    OS_EXIT_CRITICAL();
}

static void stop_motors(INT32U now)
{
    if (drive_state != EL05_DRIVE_STOPPING && drive_state != EL05_DRIVE_STOPPED)
    {
        drive_state = EL05_DRIVE_STOPPING;
        stop_sent = false;
        cancel_batch();
    }
    if (drive_state == EL05_DRIVE_STOPPED)
        return;
    if (tx_active)
    {
        if (now - tx_start_tick >= EL05_FEEDBACK_TIMEOUT_TICKS)
            cancel_batch();
        return;
    }

    if (stop_sent && !tx_failed)
        drive_state = EL05_DRIVE_STOPPED;
    else
    {
        send_batch(EL05_STOP, 0, 0, now);
        stop_sent = true;
    }
}

static bool feedback_ready(bool initialize_targets)
{
    OS_CPU_SR cpu_sr;
    OS_ENTER_CRITICAL();
    INT32U now = OSTimeGet();
    for (uint8_t i = 0; i < motor_count; i++)
    {
        if (feedback_status[i] != EL05_MODE_RUNNING ||
            now - feedback_tick[i] >= EL05_FEEDBACK_TIMEOUT_TICKS)
        {
            OS_EXIT_CRITICAL();
            return false;
        }
        if (initialize_targets)
            target_position[i] = feedback_position[i] * (2.0f * EDULITE05_POSITION_MAX_RAD / 65535.0f) -
                                 EDULITE05_POSITION_MAX_RAD;
    }
    OS_EXIT_CRITICAL();
    return true;
}

bool EduLite05_Init(RCS_PIN_CAN can_map, const uint8_t ids[], uint8_t count,
                    const EduLite05Config_t *config)
{
    if (motor_can != NULL || ids == NULL || config == NULL ||
        count == 0u || count > EDULITE05_MAX_MOTORS ||
        (can_map.CANx != CAN1 && can_map.CANx != CAN2) ||
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
    motor_can = can_map.CANx;

    IRQn_Type tx_irq;
    IRQn_Type rx_irq;
    if (motor_can == CAN1)
    {
        RCS_CAN1_Config(can_map, receive_isr, CAN1_BAUD);
        BSP_IntVectSet(BSP_INT_ID_CAN1_TX, transmit_isr);
        tx_irq = CAN1_TX_IRQn;
        rx_irq = CAN1_RX0_IRQn;
    }
    else
    {
        /* CAN2 的过滤器依赖 CAN1 时钟。 */
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
        RCS_CAN2_Config(can_map, receive_isr, CAN2_BAUD);
        BSP_IntVectSet(BSP_INT_ID_CAN2_TX, transmit_isr);
        tx_irq = CAN2_TX_IRQn;
        rx_irq = CAN2_RX0_IRQn;
    }
    NVIC_SetPriority(tx_irq, NVIC_GetPriority(rx_irq));
    NVIC_EnableIRQ(tx_irq);
    CAN_ITConfig(motor_can, CAN_IT_TME, ENABLE);
    return true;
}

void EduLite05_Stop(void)
{
    if (motor_can != NULL)
    {
        fault_latched = false;
        stop_motors(OSTimeGet());
    }
}

bool EduLite05_PositionControl(const float position_rad[])
{
    if (motor_can == NULL)
        return false;

    INT32U now = OSTimeGet();
    if (drive_state == EL05_DRIVE_STOPPING || fault_latched)
    {
        stop_motors(now);
        return false;
    }
    if (drive_state == EL05_DRIVE_ACTIVE && !feedback_ready(false))
        goto fault;
    if (tx_active && now - tx_start_tick < EL05_FEEDBACK_TIMEOUT_TICKS)
        return drive_state == EL05_DRIVE_ACTIVE;
    if (tx_active || tx_failed)
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
            OS_CPU_SR cpu_sr;
            OS_ENTER_CRITICAL();
            memset((void *)feedback_status, 0, sizeof(feedback_status));
            OS_EXIT_CRITICAL();
            send_batch(EL05_ENABLE, 0, 0, now);
            enable_tick = now;
            drive_state = EL05_DRIVE_WAIT_FEEDBACK;
            return false;
        }

        case EL05_DRIVE_WAIT_FEEDBACK:
            if (!feedback_ready(true))
            {
                if (now - enable_tick >= EL05_FEEDBACK_TIMEOUT_TICKS)
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
