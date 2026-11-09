#include "motor_can_hal.h"
#include <string.h>

typedef struct {
    CAN_HandleTypeDef *can;
    MotorCanReceive_t receive;
    MotorCanFrame_t batch[MOTOR_CAN_MAX_BATCH];
    uint32_t mailbox;
    uint8_t next;
    uint8_t count;
    volatile bool active;
    volatile bool failed;
    bool keep_sending_on_error;
} MotorCanBus_t;

static MotorCanBus_t buses[2];

static MotorCanBus_t *bus_for(CAN_HandleTypeDef *can)
{
    if (can == NULL)
        return NULL;
    if (can->Instance == CAN1)
        return &buses[0];
    if (can->Instance == CAN2)
        return &buses[1];
    return NULL;
}

static void send_next(MotorCanBus_t *bus)
{
    while (bus->next < bus->count)
    {
        MotorCanFrame_t *frame = &bus->batch[bus->next++];
        CAN_TxHeaderTypeDef header = {
            .StdId = frame->extended ? 0u : frame->id,
            .ExtId = frame->extended ? frame->id : 0u,
            .IDE = frame->extended ? CAN_ID_EXT : CAN_ID_STD,
            .RTR = CAN_RTR_DATA,
            .DLC = frame->length,
            .TransmitGlobalTime = DISABLE,
        };
        if (HAL_CAN_AddTxMessage(bus->can, &header, frame->data, &bus->mailbox) == HAL_OK)
            return;

        bus->failed = true;
        if (!bus->keep_sending_on_error)
            break;
    }
    bus->active = false;
}

static void complete(CAN_HandleTypeDef *can)
{
    MotorCanBus_t *bus = bus_for(can);
    if (bus != NULL && bus->active)
        send_next(bus);
}

static void transfer_failed(CAN_HandleTypeDef *can)
{
    MotorCanBus_t *bus = bus_for(can);
    if (bus == NULL || !bus->active)
        return;

    bus->failed = true;
    if (!bus->keep_sending_on_error)
        bus->next = bus->count;
    send_next(bus);
}

static void can_error(CAN_HandleTypeDef *can)
{
    uint32_t error = HAL_CAN_GetError(can);
    HAL_CAN_ResetError(can);
    const uint32_t tx_errors = HAL_CAN_ERROR_TX_ALST0 | HAL_CAN_ERROR_TX_TERR0 |
                               HAL_CAN_ERROR_TX_ALST1 | HAL_CAN_ERROR_TX_TERR1 |
                               HAL_CAN_ERROR_TX_ALST2 | HAL_CAN_ERROR_TX_TERR2;
    if ((error & tx_errors) != 0u)
        transfer_failed(can);
}

static void receive(CAN_HandleTypeDef *can)
{
    MotorCanBus_t *bus = bus_for(can);
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
    while (HAL_CAN_GetRxFifoFillLevel(can, CAN_RX_FIFO0) != 0u &&
           HAL_CAN_GetRxMessage(can, CAN_RX_FIFO0, &header, data) == HAL_OK)
    {
        if (bus != NULL)
            bus->receive(&header, data);
    }
}

bool MotorCan_Start(CAN_HandleTypeDef *can, MotorCanReceive_t receiver)
{
    MotorCanBus_t *bus = bus_for(can);
    if (bus == NULL || bus->can != NULL || receiver == NULL)
        return false;

    CAN_FilterTypeDef filter = {
        .FilterBank = can->Instance == CAN1 ? 0u : 14u,
        .FilterMode = CAN_FILTERMODE_IDMASK,
        .FilterScale = CAN_FILTERSCALE_32BIT,
        .FilterFIFOAssignment = CAN_FILTER_FIFO0,
        .FilterActivation = ENABLE,
        .SlaveStartFilterBank = 14u,
    };
    if (HAL_CAN_ConfigFilter(can, &filter) != HAL_OK)
        return false;

    static const HAL_CAN_CallbackIDTypeDef tx_complete[] = {
        HAL_CAN_TX_MAILBOX0_COMPLETE_CB_ID, HAL_CAN_TX_MAILBOX1_COMPLETE_CB_ID,
        HAL_CAN_TX_MAILBOX2_COMPLETE_CB_ID,
    };
    static const HAL_CAN_CallbackIDTypeDef tx_abort[] = {
        HAL_CAN_TX_MAILBOX0_ABORT_CB_ID, HAL_CAN_TX_MAILBOX1_ABORT_CB_ID,
        HAL_CAN_TX_MAILBOX2_ABORT_CB_ID,
    };
    for (uint8_t i = 0; i < 3u; i++)
    {
        if (HAL_CAN_RegisterCallback(can, tx_complete[i], complete) != HAL_OK ||
            HAL_CAN_RegisterCallback(can, tx_abort[i], transfer_failed) != HAL_OK)
            return false;
    }
    if (HAL_CAN_RegisterCallback(can, HAL_CAN_ERROR_CB_ID, can_error) != HAL_OK ||
        HAL_CAN_RegisterCallback(can, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, receive) != HAL_OK ||
        HAL_CAN_Start(can) != HAL_OK)
        return false;

    bus->can = can;
    bus->receive = receiver;
    uint32_t events = CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING;
    if (HAL_CAN_ActivateNotification(can, events) != HAL_OK)
    {
        HAL_CAN_Stop(can);
        bus->can = NULL;
        return false;
    }
    return true;
}

bool MotorCan_SendBatch(CAN_HandleTypeDef *can, const MotorCanFrame_t frames[],
                        uint8_t count, bool keep_sending_on_error)
{
    MotorCanBus_t *bus = bus_for(can);
    if (bus == NULL || bus->can != can || bus->active ||
        count == 0u || count > MOTOR_CAN_MAX_BATCH)
        return false;

    memcpy(bus->batch, frames, count * sizeof(frames[0]));
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    bus->next = 0u;
    bus->count = count;
    bus->failed = false;
    bus->active = true;
    bus->keep_sending_on_error = keep_sending_on_error;
    send_next(bus);
    __set_PRIMASK(primask);
    return true;
}

bool MotorCan_IsBusy(CAN_HandleTypeDef *can)
{
    return bus_for(can)->active;
}

bool MotorCan_TxFailed(CAN_HandleTypeDef *can)
{
    return bus_for(can)->failed;
}

void MotorCan_Abort(CAN_HandleTypeDef *can)
{
    MotorCanBus_t *bus = bus_for(can);
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (bus->active)
    {
        bus->next = bus->count;
        HAL_CAN_AbortTxRequest(can, bus->mailbox);
    }
    __set_PRIMASK(primask);
}
