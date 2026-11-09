#ifndef __AX_SBUS_H
#define __AX_SBUS_H

#include "stm32f4xx.h"

#include "stdint.h"
#include "rcs.h"

#include "RCSLIB_inc.h"        //外设硬件抽象

extern  uint16_t uart_sbus_data[16];

void SBUS_Init(RCS_PIN_USART USARTx_MAP );

uint8_t AX_SBUS_GetRxData(uint16_t *pbuf);

bool AX_SBUS_GetLiveChannels(uint16_t channels[16]);

#endif
