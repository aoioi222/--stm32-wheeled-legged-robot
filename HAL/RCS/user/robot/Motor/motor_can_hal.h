#ifndef MOTOR_CAN_HAL_H
#define MOTOR_CAN_HAL_H

#include <stdbool.h>
#include "stm32f4xx_hal.h"

#define MOTOR_CAN_MAX_BATCH 24u

typedef struct {
    uint32_t id;
    bool extended;
    uint8_t length;
    uint8_t data[8];
} MotorCanFrame_t;

typedef void (*MotorCanReceive_t)(const CAN_RxHeaderTypeDef *header, const uint8_t data[8]);

/* CAN 句柄需由 CubeMX 初始化。每路总线只注册一个电机组。 */
bool MotorCan_Start(CAN_HandleTypeDef *can, MotorCanReceive_t receiver);
bool MotorCan_SendBatch(CAN_HandleTypeDef *can, const MotorCanFrame_t frames[],
                        uint8_t count, bool keep_sending_on_error);
bool MotorCan_IsBusy(CAN_HandleTypeDef *can);
bool MotorCan_TxFailed(CAN_HandleTypeDef *can);
void MotorCan_Abort(CAN_HandleTypeDef *can);

#endif
