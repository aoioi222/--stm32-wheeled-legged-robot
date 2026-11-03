#include "vofa.h"
#include "RCS_Usart.h"

static VOFA_RxHandler_t *p_vofa_rx;

/**
 * @brief 将float拆分为4字节（小端序）
 * @param f 待拆分float
 * @param out_buf 输出4字节uint8数组
 */
void float_turn_u8(float f, uint8_t *out_buf)
{
    union {
        float fdata;
        uint8_t ldata[4];
    } un;

    un.fdata = f;
    // STM32小端序，直接按内存顺序发送
    out_buf[0] = un.ldata[0];
    out_buf[1] = un.ldata[1];
    out_buf[2] = un.ldata[2];
    out_buf[3] = un.ldata[3];
}

/**
 * @brief 发送JustFloat协议数据帧
 * @param fdata      float数据数组
 * @param fdata_num  数据通道数
 * @param Usartx     使用的串口，如 USART1
 * @note 帧尾为 0x00 0x00 0x80 0x7f
 */
void JustFloat_Send(float *fdata, uint16_t fdata_num, USART_TypeDef *Usartx)
{
    uint8_t temp_buf[4];
    for (uint16_t i = 0; i < fdata_num; i++) {
        float_turn_u8(fdata[i], temp_buf);
        RCS_USART_Send_Data(Usartx, temp_buf, 4);
    }
    // JustFloat协议帧尾
    uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};
    RCS_USART_Send_Data(Usartx, tail, 4);
}

// ======================== 接收部分 ========================

/**
 * @brief 初始化VOFA接收
 * @param h       接收句柄指针
 * @param USARTx  使用的串口，如 USART1
 * @param header  帧头字节，如 0xAA
 * @param tail    帧尾字节，如 0x55
 * @param pri     中断优先级，高4位抢占、低4位响应
 * @note 会注册中断回调并使能NVIC和RXNE中断
 */
void VOFA_Rx_Init(VOFA_RxHandler_t *h, USART_TypeDef *USARTx,
                  uint8_t header, uint8_t tail, uint8_t pri)
{
    p_vofa_rx   = h;
    h->USARTx   = USARTx;
    h->header   = header;
    h->tail     = tail;
    h->rx_index = 0;
    h->rx_ok    = 0;

    // 注册中断回调
    BSP_IntVectSet(GetBSP_INT_ID_USART(USARTx), VOFA_IRQHandler);

    // 配置NVIC
    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannel                   = GetRCS_USART_IRQn(USARTx);
    nvic.NVIC_IRQChannelPreemptionPriority = (pri >> 4) & 0x0f;
    nvic.NVIC_IRQChannelSubPriority        = pri & 0x0f;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    // 使能接收中断
    USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);
}

/**
 * @brief VOFA接收中断服务函数，由USARTx_IRQHandler间接调用
 * @note 状态机: 等包头 → 收数据(参数ID + 4字节float) → 验包尾 → 置rx_ok
 */
void VOFA_IRQHandler(void)
{
    if (p_vofa_rx == NULL) return;

    USART_TypeDef *USARTx = p_vofa_rx->USARTx;

    // 处理溢出错误，清ORE标志后复位接收状态
    if (USART_GetFlagStatus(USARTx, USART_FLAG_ORE) != RESET) {
        USART_ReceiveData(USARTx);
        p_vofa_rx->rx_index = 0;
        return;
    }

    if (USART_GetITStatus(USARTx, USART_IT_RXNE) == RESET) return;
    USART_ClearITPendingBit(USARTx, USART_IT_RXNE);

    uint8_t ch = (uint8_t)USART_ReceiveData(USARTx);

    if (p_vofa_rx->rx_index == 0) {
        // 等待帧头
        if (ch == p_vofa_rx->header) {
            p_vofa_rx->rx_buf[0] = ch;
            p_vofa_rx->rx_index  = 1;
        }
    } else if (p_vofa_rx->rx_index < VOFA_RX_FRAME_LEN - 1) {
        // 收数据段: buf[1]=参数ID, buf[2..5]=float(小端)
        p_vofa_rx->rx_buf[p_vofa_rx->rx_index] = ch;
        p_vofa_rx->rx_index++;
    } else {
        // 最后一字节，验证帧尾
        p_vofa_rx->rx_buf[VOFA_RX_FRAME_LEN - 1] = ch;
        p_vofa_rx->rx_index                      = 0;
        if (ch == p_vofa_rx->tail) {
            p_vofa_rx->rx_ok = 1;
        }
    }
}

/**
 * @brief 查询是否有新数据并提取参数
 * @param h        接收句柄
 * @param param_id 输出参数ID（0=Kp, 1=Ki, 2=Kd 等）
 * @param value    输出float值
 * @retval 1 有新数据，0 无
 * @note 关中断保护下读取缓冲区，避免读过程中被新帧覆盖
 */
uint8_t VOFA_Rx_GetParam(VOFA_RxHandler_t *h, uint8_t *param_id, float *value)
{
    if (h->rx_ok == 0) return 0;

    USART_TypeDef *USARTx = h->USARTx;
    USART_ITConfig(USARTx, USART_IT_RXNE, DISABLE);

    *param_id = h->rx_buf[1];

    union {
        float f;
        uint8_t b[4];
    } u;
    u.b[0] = h->rx_buf[2];
    u.b[1] = h->rx_buf[3];
    u.b[2] = h->rx_buf[4];
    u.b[3] = h->rx_buf[5];
    *value = u.f;

    h->rx_ok = 0;

    USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);
    return 1;
}
