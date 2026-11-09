#include "robot_sbus.h"
#include <string.h>

#define SBUS_START_BYTE 0x0Fu
#define SBUS_FRAME_BYTES 25u
#define SBUS_TIMEOUT_MS 100u

static uint8_t receive_byte;
static uint8_t frame[SBUS_FRAME_BYTES];
static uint8_t frame_size;
static uint32_t last_byte_ms;
static volatile uint32_t frame_ms;
static volatile bool frame_valid;
static uint16_t decoded[16];

static void receive_complete(UART_HandleTypeDef *uart)
{
    uint32_t now = HAL_GetTick();
    if (now - last_byte_ms > 3u)
        frame_size = 0u;
    last_byte_ms = now;

    if (frame_size != 0u || receive_byte == SBUS_START_BYTE)
        frame[frame_size++] = receive_byte;

    if (frame_size == SBUS_FRAME_BYTES)
    {
        frame_size = 0u;
        if (frame[24] == 0u)
        {
            frame_valid = (frame[23] & 0x0Cu) == 0u;
            if (frame_valid)
            {
                for (uint8_t i = 0; i < 16u; i++)
                {
                    unsigned bit = 11u * i;
                    unsigned byte = 1u + bit / 8u;
                    uint32_t packed = frame[byte] | ((uint32_t)frame[byte + 1u] << 8) |
                                      ((uint32_t)frame[byte + 2u] << 16);
                    decoded[i] = (uint16_t)((packed >> (bit % 8u)) & 0x7FFu);
                }
                frame_ms = now;
            }
        }
        else if (receive_byte == SBUS_START_BYTE)
        {
            frame[0] = receive_byte;
            frame_size = 1u;
        }
    }
    if (HAL_UART_Receive_IT(uart, &receive_byte, 1u) != HAL_OK)
        frame_valid = false;
}

static void receive_error(UART_HandleTypeDef *uart)
{
    frame_size = 0u;
    frame_valid = false;
    HAL_UART_Receive_IT(uart, &receive_byte, 1u);
}

bool RobotSbus_Start(UART_HandleTypeDef *uart)
{
    return HAL_UART_RegisterCallback(uart, HAL_UART_RX_COMPLETE_CB_ID, receive_complete) == HAL_OK &&
           HAL_UART_RegisterCallback(uart, HAL_UART_ERROR_CB_ID, receive_error) == HAL_OK &&
           HAL_UART_Receive_IT(uart, &receive_byte, 1u) == HAL_OK;
}

bool RobotSbus_GetLiveChannels(uint16_t channels[16])
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (HAL_GetTick() - frame_ms >= SBUS_TIMEOUT_MS)
        frame_valid = false;
    bool valid = frame_valid;
    if (valid)
        memcpy(channels, decoded, sizeof(decoded));
    __set_PRIMASK(primask);
    return valid;
}
