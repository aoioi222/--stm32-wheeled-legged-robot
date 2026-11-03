#include "uart_comm.h"
#include <string.h>

/************************ 静态全局变量 ************************/
static USART_TypeDef *s_usart = NULL;   // 当前绑定的串口外设

// 接收状态机
typedef enum
{
    WAIT_HEAD1,
    WAIT_HEAD2,
    WAIT_ID,
    WAIT_LEN,
    WAIT_PAYLOAD,
    WAIT_CRC,
    WAIT_TAIL
} recv_state_t;

static recv_state_t s_state = WAIT_HEAD1;
static uint8_t s_rbuf[128];
static uint8_t s_r_idx = 0;
static uint8_t s_payload_len = 0;

/************************ 全局变量定义 ************************/
ControlData_t g_ctrl_data;
uint8_t g_ctrl_updated = 0;

/************************ 中断服务函数 ************************/
// 传给 RCS_USART_Config 的中断回调入口
static void USART_Rx_Isr_Handler(void)
{
    // 判断接收中断标志
    if (RCS_USART_Judge_RcvCplt_ITPendingBit(s_usart) == 1)
    {
        // 读取接收到的字节
        uint8_t byte = (uint8_t)RCS_USART_Accept_Char(s_usart);
        // 送入状态机解析
        uart_recv_byte_isr(byte);
        // 清除中断标志
        RCS_USART_Clear_RcvCplt_ITPendingBit(s_usart);
    }
}

/************************ 公共函数实现 ************************/
void uart_comm_init(const RCS_PIN_USART *pin_map, uint32_t baudrate, uint8_t pri)
{
    // 保存串口外设指针，供发送使用
    s_usart = pin_map->USARTx;

    // 调用 RCS 标准串口初始化接口：GPIO + 串口 + 接收中断
    RCS_USART_Config(
        pin_map->USARTx,
        pin_map->GPIOx,
        pin_map->GPIO_Pin_Tx,
        pin_map->GPIO_Pin_Rx,
        USART_Rx_Isr_Handler,  // 绑定接收中断处理函数
        baudrate,
        pri
    );

    // 状态机复位
    s_state = WAIT_HEAD1;
    s_r_idx = 0;
    g_ctrl_updated = 0;
    memset(&g_ctrl_data, 0, sizeof(ControlData_t));
}

void uart_recv_byte_isr(uint8_t byte)
{
    switch (s_state)
    {
        case WAIT_HEAD1:
            if (byte == FRAME_HEAD1)
                s_state = WAIT_HEAD2;
            break;

        case WAIT_HEAD2:
            if (byte == FRAME_HEAD2)
            {
                s_r_idx = 0;
                s_state = WAIT_ID;
            }
            else
                s_state = WAIT_HEAD1;
            break;

        case WAIT_ID:
            if (byte == FRAME_ID_CTRL)
            {
                s_rbuf[s_r_idx++] = byte;
                s_state = WAIT_LEN;
            }
            else
                s_state = WAIT_HEAD1;
            break;

        case WAIT_LEN:
            s_rbuf[s_r_idx++] = byte;
            s_payload_len = byte;
            s_state = WAIT_PAYLOAD;
            break;

        case WAIT_PAYLOAD:
            s_rbuf[s_r_idx++] = byte;
            // rbuf[0]=帧ID, rbuf[1]=长度，从rbuf[2]开始是payload
            if (s_r_idx - 2 >= s_payload_len)
                s_state = WAIT_CRC;
            break;

        case WAIT_CRC:
        {
            uint8_t calc_crc = crc8_calc(s_rbuf, s_r_idx);
            if (calc_crc != byte)
            {
                s_state = WAIT_HEAD1;
                break;
            }
            s_state = WAIT_TAIL;
            break;
        }

        case WAIT_TAIL:
            s_state = WAIT_HEAD1;
            if (byte == FRAME_TAIL)
            {
                // 校验通过，拷贝数据到全局结构体
                memcpy(&g_ctrl_data, &s_rbuf[2], s_payload_len);
                g_ctrl_updated = 1;
            }
            break;
    }
}

void uart_send_feedback(const FeedbackData_t *fb)
{
    uint8_t tx_buf[256];
    uint16_t idx = 0;
    uint8_t payload_len = sizeof(FeedbackData_t);

    // 1. 组装帧头
    tx_buf[idx++] = FRAME_HEAD1;
    tx_buf[idx++] = FRAME_HEAD2;
    tx_buf[idx++] = FRAME_ID_FB;
    tx_buf[idx++] = payload_len;

    // 2. 拷贝载荷数据
    memcpy(&tx_buf[idx], fb, payload_len);
    idx += payload_len;

    // 3. 计算 CRC8（范围：帧ID + 长度 + 载荷）
    tx_buf[idx++] = crc8_calc(&tx_buf[2], 2 + payload_len);

    // 4. 帧尾
    tx_buf[idx++] = FRAME_TAIL;

    // 5. 调用 RCS 接口阻塞发送
    RCS_USART_Send_Data(s_usart, tx_buf, idx);
}