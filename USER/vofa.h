#ifndef __VOFA_H
#define __VOFA_H

#include "stm32f4xx_usart.h"

//==================== 发送通道数，根据实际需要修改 ====================
#define VOFA_CH_NUM 3U // 需要发送的通道数，填写 1 2 3 4
//====================================================================

// 接收帧长度: 包头(1) + 参数ID(1) + float数据(4) + 包尾(1)
#define VOFA_RX_FRAME_LEN 7U

// 接收句柄
typedef struct {
    USART_TypeDef *USARTx;
    uint8_t header;
    uint8_t tail;
    uint8_t rx_buf[VOFA_RX_FRAME_LEN];
    uint8_t rx_index;
    uint8_t rx_ok;
} VOFA_RxHandler_t;

// 发送
void float_turn_u8(float f, uint8_t *out_buf);
void JustFloat_Send(float *fdata, uint16_t fdata_num, USART_TypeDef *Usartx);

// 接收
void VOFA_Rx_Init(VOFA_RxHandler_t *h, USART_TypeDef *USARTx,
                  uint8_t header, uint8_t tail, uint8_t pri);
void VOFA_IRQHandler(void);
uint8_t VOFA_Rx_GetParam(VOFA_RxHandler_t *h, uint8_t *param_id, float *value);

#endif
