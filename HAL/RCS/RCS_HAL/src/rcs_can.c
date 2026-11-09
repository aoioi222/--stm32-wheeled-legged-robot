/**
 *@filename rcs_can.c   
 *@brief RCS对CAN外设操作进行的二次封装，用于通用的CAN设备，例如大疆电机、VESC电调、关节电机等。
 *---------------------------------------------------------------------------------------------------
 *@version v1.0
 *@author  CYK-Dot@2025-05-21
 *@changelog 标准库时期的CAN接口不太合理，因此在HAL之上进行了重新设计，提供非阻塞并且有缓冲机制的CAN发送
 *           应用场景：数据发送存在周期性，一段时间内会冒出很多待发送数据，而一段时间内完全没有数据
 * 
 *           不适合的场景：时时刻刻都有数据需要发送，没有周期性，从不停歇。
 *           由于CAN总线情况不一(取决于谁挂到了总线上)、回调事件多，上述情况难以解决。就算解决了，代码也会变得非常难以阅读和维护。
 *           因此，请在为新的CAN总线设备写代码时，集中在一段时间内发送CAN报文。没有尽可能，这是强制要求。
 * 
 *@note CAN2.0B中，当CAN总线波特率拉到最大(1Mbps)，1ms仍然顶多发送7个标准帧，或者5个拓展帧，顶破天了也差强人意。
 *      除此之外，CAN总线还是半双工的，因此对于STM32而言，1ms不可能真的发送这么多报文，因为其他设备也会占用CAN总线。
 *      所以不要想着什么都往同一条CAN总线上怼，否则报文发不出去，或者读不到。
 * 
 *@note HAL_CAN允许存在针对同一路CAN的多个实例，但多个实例却无法同时运行，因此容易写错代码。
 *      因此，RCS_CAN句柄做成了单例的，一路CAN只能有一个实例。
 *      但一个RCS_CAN实例可以有多个句柄，句柄通过rcs_can_open获取。
 * 
 *---------------------------------------------------------------------------------------------------
 *@version v1.1
 *@author  CYK-Dot@2025-05-25
 *@changelog 排查了一个bug:当CAN总线未初始化时，CAN总线回调函数不应具有功能，否则会crash。
 *           bug原因：只要开了CAN中断，不管你有没有打算收数据，都得调用HAL_CAN_GetRxMessage收数据，否则HAL库会卡死
 *---------------------------------------------------------------------------------------------------
**/
#include "rcs_can.h"

#include <string.h>
#include "rcs_os.h"

/* 私有结构体 ------------------------------------------------------------------------------------------*/

typedef struct {
    CAN_HandleTypeDef *hcan;
    uint8_t errorFlag;
    
    rcs_can_callback_ptr subscribers[RCS_CAN_MAX_SUBSCRIBER_COUNT];
    size_t subscriberIndex;

    CAN_TxHeaderTypeDef txHeaderBuffer[RCS_CAN_MAX_TX_BUFFER_COUNT];
    uint8_t txDataBuffer[RCS_CAN_MAX_TX_BUFFER_COUNT][8];
    uint8_t txBufferIndex;
    SemaphoreHandle_t txMutex;
    TimerHandle_t txTimeoutTimer;
    uint8_t txState;
} rcs_can_internal_t;

typedef struct {
    CAN_HandleTypeDef *hcan;
    uint8_t errorFlag;
    TickType_t time;
} rcs_can_err_t;

/*全局变量 -------------------------------------------------------------------------------------------- */

rcs_can_internal_t *g_canList[RCS_CAN_COUNT] = {NULL};

rcs_can_err_t g_canErrQueue[RCS_CAN_MAX_ERR_QUEUE_LENGTH];
static size_t g_canErrQueueIndex = 0;

/* 私有宏定义 ------------------------------------------------------------------------------------------*/

#define RCS_CAN_TX_STATE_IDLE 0
#define RCS_CAN_TX_STATE_SENDING 1

/* 静态函数声明 ------------------------------------------------------------------------------------------*/

static void rcs_can_tx_timeout_callback(TimerHandle_t xTimer);
static void rcs_can_err_add(rcs_can_internal_t *can,uint8_t errorFlag);
static void rcs_can_err_add_from_isr(rcs_can_internal_t *can,uint8_t errorFlag);

/* 导出函数实现 ------------------------------------------------------------------------------------------*/

/**
 * @brief 为HAL_CAN安装RCS_CAN工作所需的驱动，以初始化CAN
 * 
 * @param hcan CubeMX生成的CAN句柄，例如hcan1
 * @param RCS_CAN_NUM_x CAN的编号，RCS_CAN_NUM_1代表CAN1
 * 
 * @warning 不要把RCS_CAN_NUM_x和hcan的对应关系弄错！
 *          比如hcan1就要对应RCS_CAN_NUM_1，hcan2就要对应RCS_CAN_NUM_2，否则会出问题！
 */
rcs_err_t rcs_can_driver_install(CAN_HandleTypeDef *hcan,rcs_can_t RCS_CAN_NUM_x)
{
    rcs_kprintf("rcs_can:installing CAN%d...\n",portMAX_DELAY,RCS_CAN_NUM_x+1);
    // 参数检查
    if (RCS_CAN_NUM_x >= RCS_CAN_COUNT) {
        rcs_kprintf("rcs_can:not exist in this STM32.\n",portMAX_DELAY);
    }
    // 先检查该实例此前是否创建过，如果是，直接返回，无需二次安装
    if (g_canList[RCS_CAN_NUM_x]!=NULL) {
        rcs_kprintf("rcs_can:has been installed before,OK.\n",portMAX_DELAY);
        return RCS_OK;
    }
    // 程序走到这里，代表该实例没有创建过，因此创建一个
    g_canList[RCS_CAN_NUM_x] = RCS_CAN_MALLOC(sizeof(rcs_can_internal_t));
    if (!g_canList[RCS_CAN_NUM_x]) {
		rcs_kprintf("rcs_can:object malloc failed.\n",portMAX_DELAY);
		return NULL;
	}
    memset(g_canList[RCS_CAN_NUM_x], 0, sizeof(rcs_can_internal_t));
    // 初始化CAN对象的索引句柄
    g_canList[RCS_CAN_NUM_x]->hcan = hcan;
    // 初始化CAN对象的订阅者队列
    g_canList[RCS_CAN_NUM_x]->subscriberIndex = 0;
    // 初始化CAN对象的发送缓冲区(缓冲区长度在.h中直接给定，不允许手动调整) 
    g_canList[RCS_CAN_NUM_x]->txState = RCS_CAN_TX_STATE_IDLE;
    g_canList[RCS_CAN_NUM_x]->txBufferIndex = 0;
    g_canList[RCS_CAN_NUM_x]->txState = RCS_CAN_TX_STATE_IDLE;
    // 初始化CAN对象的发送互斥锁
    g_canList[RCS_CAN_NUM_x]->txMutex = xSemaphoreCreateBinary();
    if (!g_canList[RCS_CAN_NUM_x]->txMutex) {
		rcs_kprintf("rcs_can:mutex inited failed.\n",portMAX_DELAY);
        RCS_CAN_FREE(g_canList[RCS_CAN_NUM_x]);
        return RCS_ERR_NO_MEM;
    }
	xSemaphoreGive(g_canList[RCS_CAN_NUM_x]->txMutex);
    // 初始化CAN对象的发送超时定时器
    g_canList[RCS_CAN_NUM_x]->txTimeoutTimer = xTimerCreate(
        "canTx", 
        pdMS_TO_TICKS(1), 
        pdFALSE, 
        (void*)g_canList[RCS_CAN_NUM_x], // 把RCS_CAN对象作为参数传入定时器，以区分不同CAN的定时回调
        rcs_can_tx_timeout_callback
    );
    if (!g_canList[RCS_CAN_NUM_x]->txTimeoutTimer) {
		rcs_kprintf("rcs_can:timer inited failed.\n",portMAX_DELAY);
        vSemaphoreDelete(g_canList[RCS_CAN_NUM_x]->txMutex);
        RCS_CAN_FREE(g_canList[RCS_CAN_NUM_x]);
        return RCS_ERR_NO_MEM;
    }
    // 设置Filter：接收所有报文
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(g_canList[RCS_CAN_NUM_x]->hcan, &can_filter_st);
    HAL_CAN_Start(g_canList[RCS_CAN_NUM_x]->hcan);
    HAL_CAN_ActivateNotification(g_canList[RCS_CAN_NUM_x]->hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(g_canList[RCS_CAN_NUM_x]->hcan,CAN_IT_TX_MAILBOX_EMPTY);
    // 成功
	rcs_kprintf("rcs_can:install OK.\n",portMAX_DELAY);
    return RCS_OK;
}

/**
 * @brief 订阅CAN总线收到的报文
 * @param handle CAN对象
 * @param callback 回调函数
 * @return 返回是否订阅成功
 */
rcs_err_t rcs_can_subscribe(rcs_can_t RCS_CAN_NUM_x, rcs_can_callback_ptr callback)
{
    // 参数检查
    if (!g_canList[RCS_CAN_NUM_x] || !callback) {
        rcs_kprintf("rcs_can:subscribe failed,invalid parameter.\n",portMAX_DELAY);
        return RCS_ERR_NOT_ALLOWED;
    }
    // 边界检查
    if (g_canList[RCS_CAN_NUM_x]->subscriberIndex >= RCS_CAN_MAX_SUBSCRIBER_COUNT){
        rcs_kprintf("rcs_can:subscribe failed,subscriber queue is full.\n",portMAX_DELAY);
        return RCS_ERR_NOT_ALLOWED;
    }
    // 此前是否注册过
    for(int i=0;i<g_canList[RCS_CAN_NUM_x]->subscriberIndex;i++) {
        if(g_canList[RCS_CAN_NUM_x]->subscribers[i] == callback) {
            rcs_kprintf("rcs_can:subscribe finished,has been subscribed before.\n",portMAX_DELAY);
            return RCS_OK;
        }
    }
    // 注册
    g_canList[RCS_CAN_NUM_x]->subscribers[g_canList[RCS_CAN_NUM_x]->subscriberIndex] = callback;
    g_canList[RCS_CAN_NUM_x]->subscriberIndex++;
    rcs_kprintf("rcs_can:subscribe finished.now %hd subscriber on CAN%d\n",portMAX_DELAY,g_canList[RCS_CAN_NUM_x]->subscriberIndex,RCS_CAN_NUM_x+1);
    return RCS_OK;
}

/**
 * @brief 检查此前是否订阅过
 * 
 * @param handle RCS_CAN对象
 * @param callback 订阅者回调函数
 * @retval bool
 */
rcs_err_t rcs_can_is_subscribe(rcs_can_t RCS_CAN_NUM_x, rcs_can_callback_ptr callback)
{
    // 参数检查
    if (!g_canList[RCS_CAN_NUM_x] || !callback) {
        rcs_kprintf("rcs_can:subscribe check failed,invalid parameter.\n",portMAX_DELAY);
        return RCS_ERR_NOT_ALLOWED;
    }
    // 此前是否注册过
    for(int i=0;i<g_canList[RCS_CAN_NUM_x]->subscriberIndex;i++) {
        if(g_canList[RCS_CAN_NUM_x]->subscribers[i] == callback) {
            return RCS_OK;
        }
    }
    return RCS_FAIL;
}

/**
 * @brief 发送CAN报文
 * @param handle CAN对象
 * @param header 报文头部
 * @param data 报文数据
 * @param timeout 超时时间
 * @return 返回是否发送成功
 * 
 * @note 发送逻辑：
 *      1. 等待发送缓冲区可用
 *      2. 占有缓冲区并向其中写入数据
 *      3. 释放缓冲区，并复位定时器
 * 
 * @note CAN的缓冲区没有实现成FIFO(先进先出)，而是先进后出。
 *       因此如果出现堵塞现象，会优先让后发送的人发。
 * 
 * @warning 如果超出了总线承载力上限，堵塞将会成为必然，这不是缓冲区能解决的。
 */ 
rcs_err_t rcs_can_send(rcs_can_t RCS_CAN_NUM_x, CAN_TxHeaderTypeDef *header,const uint8_t *data, TickType_t timeout)
{
    // 参数检查
    if (!g_canList[RCS_CAN_NUM_x] || !header || !data) {
        rcs_kprintf("rcs_can:send failed,invalid parameter.\n",portMAX_DELAY);
        return RCS_ERR_NOT_ALLOWED;
    }

    // 等待发送缓冲区可用
    if (xSemaphoreTake(g_canList[RCS_CAN_NUM_x]->txMutex, timeout) != pdPASS) {
        rcs_can_err_add(g_canList[RCS_CAN_NUM_x],RCS_CAN_ERR_TXFUNC_MUTEX_TAKE_FAILED);
        return RCS_ERR_BUSY;
    }
    // 检查缓冲区是否满
    if (g_canList[RCS_CAN_NUM_x]->txBufferIndex >= RCS_CAN_MAX_TX_BUFFER_COUNT) {
        rcs_can_err_add(g_canList[RCS_CAN_NUM_x],RCS_CAN_ERR_TXFUNC_BUFFER_FULL);
        xSemaphoreGive(g_canList[RCS_CAN_NUM_x]->txMutex);
        return RCS_ERR_OVERFLOW;
    }   
    // 占有缓冲区并向其中写入数据
    memcpy(&g_canList[RCS_CAN_NUM_x]->txHeaderBuffer[g_canList[RCS_CAN_NUM_x]->txBufferIndex], header, sizeof(CAN_TxHeaderTypeDef));
    memcpy(g_canList[RCS_CAN_NUM_x]->txDataBuffer[g_canList[RCS_CAN_NUM_x]->txBufferIndex], data, header->DLC);
    g_canList[RCS_CAN_NUM_x]->txBufferIndex++;   

    // 释放缓冲区，并复位定时器
    xSemaphoreGive(g_canList[RCS_CAN_NUM_x]->txMutex);
    xTimerReset(g_canList[RCS_CAN_NUM_x]->txTimeoutTimer, timeout);
    xTimerStart(g_canList[RCS_CAN_NUM_x]->txTimeoutTimer, timeout);
    return RCS_OK;
}

/* 静态函数实现 ------------------------------------------------------------------------------------------*/


/**
 * @brief 添加错误信息到错误队列(非ISR版本)
 * @param can CAN对象
 * @param errorFlag 错误标志
 * @warning 请勿在ISR中调用此函数
 */
static void rcs_can_err_add(rcs_can_internal_t *can,uint8_t errorFlag)
{
    if (g_canErrQueueIndex >= RCS_CAN_MAX_ERR_QUEUE_LENGTH) g_canErrQueueIndex = 0;
    g_canErrQueue[g_canErrQueueIndex].hcan = can->hcan;
    g_canErrQueue[g_canErrQueueIndex].errorFlag = errorFlag;
    g_canErrQueue[g_canErrQueueIndex].time = xTaskGetTickCount();
    g_canErrQueueIndex++;
}

/**
 * @brief 添加错误信息到错误队列(ISR版本)
 * @param can CAN对象
 * @param errorFlag 错误标志
 * @warning 请勿在任务中调用此函数
 */
static void rcs_can_err_add_from_isr(rcs_can_internal_t *can,uint8_t errorFlag)
{
    if (g_canErrQueueIndex >= RCS_CAN_MAX_ERR_QUEUE_LENGTH) g_canErrQueueIndex = 0;
    g_canErrQueue[g_canErrQueueIndex].hcan = can->hcan;
    g_canErrQueue[g_canErrQueueIndex].errorFlag = errorFlag;
    g_canErrQueue[g_canErrQueueIndex].time = xTaskGetTickCountFromISR();
    g_canErrQueueIndex++;
}

/**
 * @brief 根据CAN句柄查找CAN对象
 * @param hcan CAN句柄
 * @return 返回CAN对象
 */
static rcs_can_internal_t *rcs_can_find_by_handle(CAN_HandleTypeDef *hcan)
{
    for (size_t i = 0; i < RCS_CAN_COUNT; i++)
        if (g_canList[i]->hcan == hcan)
            return g_canList[i];
    return NULL;
}

/**
 * @brief 根据CAN句柄查找CAN对象下标
 * @param hcan CAN句柄
 * @return 返回CAN对象
 */
static rcs_can_t rcs_canidx_find_by_handle(CAN_HandleTypeDef *hcan)
{
    for (rcs_can_t i = 0; i < RCS_CAN_COUNT; i++)
        if (g_canList[i]->hcan == hcan)
            return i;
    return RCS_CAN_NUM_MAX;
}

/* 回调函数实现 ------------------------------------------------------------------------------------------*/

/**
 * @brief 发送超时定时器回调函数
 * @param xTimer 定时器句柄
 * 
 * @note 回调逻辑：
 *      1. 等待最后一个写入者完成写入
 *      2. 检查发送状态机是否为空闲，否则意味着CAN总线过载了，赋值errorFlag并退出
 *      3. 将发送状态机置为发送中
 *      4. 发送第一条报文，剩下的过程全部交给HAL_CAN_TxMailbox0CompleteCallback()处理，包括锁解除
 */
static void rcs_can_tx_timeout_callback(TimerHandle_t xTimer)
{
    // 获取是哪一个RCS_CAN触发的触发的定时器
    rcs_can_internal_t *can = (rcs_can_internal_t *)pvTimerGetTimerID(xTimer);
	uint32_t txMailBoxId;
    if (!can) return;
    // 检查发送状态机是否为空闲
    if (can->txState == RCS_CAN_TX_STATE_IDLE) {
        // 不等待最后一个写入者完成写入
        if (xSemaphoreTake(can->txMutex, 0) == pdPASS) {
            // 将发送状态机置为发送中，并发送第一条报文
            can->txState = RCS_CAN_TX_STATE_SENDING;
            HAL_CAN_AddTxMessage(can->hcan, &can->txHeaderBuffer[can->txBufferIndex], can->txDataBuffer[can->txBufferIndex], &txMailBoxId);
        }   
        // 等不上就下一个定时器抵达后再来，反正定时器会一直触发
        else {
            rcs_can_err_add(can,RCS_CAN_ERR_TXTIMER_MUTEX_TAKE_FAILED);
        }
    }
    // 否则意味着CAN总线过载了，赋值errorFlag并退出
    else {
        rcs_can_err_add(can,RCS_CAN_ERR_TXTIMER_PERI_BUSY);
    }
}

/**
 * @brief CAN发送完成回调函数
 * @param hcan CAN句柄
 * 
 * @note 回调逻辑：
 *      1. 向邮箱中写入下一条报文，无论是否成功，均将发送的报文从缓冲区移除
 *      2. 如果向邮箱写入失败，则赋值errorFlag
 *      3. 如果缓冲区中还有报文，则继续发送下一条，否则将发送状态机置为空闲，并解锁互斥锁
 */
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
    HAL_StatusTypeDef status;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t txMailBoxId;
    // 获取是哪一个RCS_CAN触发的HAL_CAN回调
    rcs_can_internal_t *can = rcs_can_find_by_handle(hcan);
    if (!can) return;
    // 如果缓冲区中还有报文，则继续发送下一条
    if (can->txBufferIndex > 0) {
        can->txBufferIndex--;
        status = HAL_CAN_AddTxMessage(can->hcan, &can->txHeaderBuffer[can->txBufferIndex], can->txDataBuffer[can->txBufferIndex], &txMailBoxId);
        if (status != HAL_OK) {
            can->errorFlag = RCS_CAN_ERR_BUSY;
            can->txBufferIndex = 0;
            rcs_can_err_add_from_isr(can,RCS_CAN_ERR_TXCALLBACK_PERI_FAILED);
            xSemaphoreGiveFromISR(can->txMutex, &xHigherPriorityTaskWoken);
        }
    }
    // 否则将发送状态机置为空闲，并解锁互斥锁
    else {
        can->txState = RCS_CAN_TX_STATE_IDLE;
        can->errorFlag = RCS_CAN_ERR_NONE;
        xSemaphoreGiveFromISR(can->txMutex, &xHigherPriorityTaskWoken);
    }
    // 退出中断后唤醒等待缓冲区锁的任务
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
    // 获取是哪一个RCS_CAN触发的HAL_CAN回调
    rcs_can_internal_t *can = rcs_can_find_by_handle(hcan);
    // 如果这一路RCS_CAN尚未初始化，就要读一个空帧，避免卡死
    if (!can) {
		// 只要开了CAN中断，不管你有没有打算收数据，都得调用HAL_CAN_GetRxMessage，否则代码会卡死
		CAN_RxHeaderTypeDef rx_header;
		uint8_t rx_data[8];
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
		return;
	}    
    // 反之这一路RCS_CAN初始化过，直接触发回调函数即可
	else if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) == HAL_OK) {
		for (size_t i = 0; i < can->subscriberIndex; ++i) {
            rcs_can_t can_idx = rcs_canidx_find_by_handle(hcan);
            if (can_idx >= RCS_CAN_COUNT) return;
			if (can->subscribers[i])
				can->subscribers[i](can_idx, &header, data);
        }
	}
}
