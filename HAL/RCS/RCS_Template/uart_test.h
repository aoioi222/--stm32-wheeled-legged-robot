#pragma once

#include "rcs_uart.h"

#ifdef __cplusplus      
extern "C" {
#endif

rcs_err_t uart_test_init(UART_HandleTypeDef *huartx,rcs_uart_t RCS_UART_NUM_x);

#ifdef __cplusplus
}
#endif
