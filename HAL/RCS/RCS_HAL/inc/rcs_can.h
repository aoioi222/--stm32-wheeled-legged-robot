/**
 * @file rcs_can.h
 * @brief RCS CAN接口层，对HAL CAN进行二次封装，支持发送缓冲和多个订阅回调
 */
#pragma once

#include "rcs_private_config.h"
#include "rcs_types.h"

#include <FreeRTOS.h>
#include <task.h>
#include <timers.h>
#include <semphr.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 配置宏 --------------------------------------------------------------------------------------------*/

// CAN外设数量
#define RCS_CAN_COUNT 2

// 内存分配宏
#define RCS_CAN_MALLOC pvPortMalloc
#define RCS_CAN_FREE   vPortFree

// 发送缓冲区数量
#define RCS_CAN_MAX_TX_BUFFER_COUNT 16
// 订阅者数量
#define RCS_CAN_MAX_SUBSCRIBER_COUNT 16
// 错误队列长度
#define RCS_CAN_MAX_ERR_QUEUE_LENGTH 32


/* 导出宏 --------------------------------------------------------------------------------------------*/

#define RCS_CAN_ERR_NONE    0
#define RCS_CAN_ERR_BUSY    1
#define RCS_CAN_ERR_TIMEOUT 2

#define RCS_CAN_ERR_TXFUNC_MUTEX_TAKE_FAILED 3
#define RCS_CAN_ERR_TXFUNC_BUFFER_FULL 4

#define RCS_CAN_ERR_TXTIMER_MUTEX_TAKE_FAILED 5
#define RCS_CAN_ERR_TXTIMER_PERI_BUSY 6

#define RCS_CAN_ERR_TXCALLBACK_MUTEX_TAKE_FAILED 7
#define RCS_CAN_ERR_TXCALLBACK_BUFFER_FULL 8
#define RCS_CAN_ERR_TXCALLBACK_PERI_FAILED 9

/* 导出类型 -------------------------------------------------------------------------------------------*/

typedef enum 
{
    RCS_CAN_NUM_1 = 0,
    RCS_CAN_NUM_2 = 1,
    RCS_CAN_NUM_3 = 2,
    RCS_CAN_NUM_4 = 3,
    RCS_CAN_NUM_5 = 4,
    RCS_CAN_NUM_MAX
}rcs_can_t;


/**
 * @brief CAN接收回调函数类型
 * @param handle     CAN操作对象
 * @param header    接收头部信息
 * @param data      数据结构
 */
typedef void (*rcs_can_callback_ptr)(rcs_can_t RCS_CAN_NUM_x,const CAN_RxHeaderTypeDef *header, uint8_t *data);

/**
 * @brief RCS_CAN对象的配置
 * @note 不同于RCS_UART，所有的RCS_CAN配置都以“配置宏”的形式提供，详见上面第21行，因为这些配置不推荐更改。
 *       和RCS_UART类似，CAN外设相关的配置，应在CubeMX中完成，由CubeMX负责把配置写入HAL_CAN对象。
 */
typedef struct {
    CAN_HandleTypeDef *hcan;
} rcs_can_conf_t;


rcs_err_t rcs_can_driver_install(CAN_HandleTypeDef *hcan,rcs_can_t RCS_CAN_NUM_x);

rcs_err_t rcs_can_subscribe(rcs_can_t RCS_CAN_NUM_x, rcs_can_callback_ptr callback);
rcs_err_t rcs_can_is_subscribe(rcs_can_t RCS_CAN_NUM_x, rcs_can_callback_ptr callback);

rcs_err_t rcs_can_send(rcs_can_t RCS_CAN_NUM_x, CAN_TxHeaderTypeDef *header,const uint8_t *data, TickType_t timeout);
rcs_err_t rcs_can_send_from_isr(rcs_can_t RCS_CAN_NUM_x, CAN_TxHeaderTypeDef *header,const uint8_t *data, BaseType_t *pxHigherPriorityTaskWoken);

int rcs_can_fetch_error(char *output_buffer,size_t buffer_size);

#ifdef __cplusplus
}
#endif
