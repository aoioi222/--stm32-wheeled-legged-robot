/**
 * @addtogroup RCSLIB-fifo
 * @{
 */


/* .h头文件开始------------------------------------------*/
#pragma once
#ifdef __cplusplus
extern "C" {
#endif

/* 引入头文件--------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

/* 配置宏定义 --------------------------------------------*/

#define LOCKFREE_FIFO_MALLOC           pvPortMalloc
#define LOCKFREE_FIFO_FREE             vPortFree
#define LOCKFREE_FIFO_ENTER_CRITICAL   __disable_irq()
#define LOCKFREE_FIFO_EXIT_CRITICAL    __enable_irq()
#define LOCKFREE_FIFO_ASSERT(x) ;


	
#if LOG_MODE == 0
	//完全不记录任何日志
	#define LOGD(fmt, ...)
	#define LOGI(fmt, ...)
	#define LOGW(fmt, ...)
	#define LOGE(fmt, ...)
#elif LOG_MODE == 1
	//通过带有颜色的字符记录日志
	#include <stdio.h>
	const char* TAG = LOG_TAG;
	#define LOGD(fmt, ...) printf("\033[37m%s: " fmt "\033[0m\n", TAG, ##__VA_ARGS__) 
	#define LOGI(fmt, ...) printf("\033[32m%s: " fmt "\033[0m\n", TAG, ##__VA_ARGS__)
	#define LOGW(fmt, ...) printf("\033[33m%s: " fmt "\033[0m\n", TAG, ##__VA_ARGS__)
	#define LOGE(fmt, ...) printf("\033[31m%s: " fmt "\033[0m\n", TAG, ##__VA_ARGS__)
#elif LOG_MODE == 2
	//通过不带颜色的字符记录日志
	#include <stdio.h>
	#define LOGD(fmt, ...) printf("[DEBUG] %s: " fmt "\n", LOG_TAG __VA_OPT__(,) __VA_ARGS__)
	#define LOGI(fmt, ...) printf("[INFO]  %s: " fmt "\n", LOG_TAG __VA_OPT__(,) __VA_ARGS__)
	#define LOGW(fmt, ...) printf("[WARN]  %s: " fmt "\n", LOG_TAG __VA_OPT__(,) __VA_ARGS__)
	#define LOGE(fmt, ...) printf("[ERROR] %s: " fmt "\n", LOG_TAG __VA_OPT__(,) __VA_ARGS__)
#endif

#if LOG_LEVEL == 1
	//隐藏debug日志
	#undef LOGD
	#define LOGD(fmt, ...)
#elif LOG_LEVEL == 2
	//隐藏debug和info日志
	#undef LOGD
	#undef LOGI
	#define LOGD(fmt, ...)
	#define LOGI(fmt, ...)
#elif LOG_LEVEL == 3
	//隐藏debug、info、warn日志
	#undef LOGD
	#undef LOGI
	#undef LOGW
	#define LOGD(fmt, ...)
	#define LOGI(fmt, ...)
	#define LOGW(fmt, ...)
#endif




/* 导出宏定义--------------------------------------------*/
/**
 *@brief 获取移动若干步后的的索引
**/
#define DumbRingFifoGetNextIndex(size,index,step) ( ((index) + (step)) % (size) )

/**
 *@brief 获取向前若干步后的索引
**/
#define DumbRingFifoGetPrevIndex(size, index, step)  (((index) >= (step)) ? ((index) - (step)) : ((size) - ((step) - (index))))

/**
 *@brief 给定head哑元和数据长度，获取tail哑元的位置
 *@note  不确保数据可以使用，因此在get之前需要自行判断len是否合理
**/
#define DumbRingFifoGetTailIndex(size,head,len) DumbRingFifoGetNextIndex(size,head,len)

/**
 *@brief 获取剩余的空间
 *
 *@note head<tail代表head快要追上tail了，剩下的空间即为追上的距离
**/
#define DumbRingFifoGetFreeSize(size,head,tail)  (((tail) >= (head)) ? ((size) - (tail) + (head)) : ((head) - (tail)))

/**
 *@brief 获取剩余的不可分割空间
 *
 *@note head<tail代表head快要追上tail了，剩下的空间即为追上的距离
**/
#define DumbRingFifoGetNoSplitFreeSize(size,head,tail) (((tail) >= (head)) ? ((size) - (tail)) : ((head) - (tail)))

/**
 *@brief 获取使用掉的空间
 *
 *@note tail>head代表head并未绕回，直接相减即可得到结果
**/
#define DumbRingFifoGetUsedSize(size,head,tail) (((head) >= (tail)) ? ((size) - (head) + (tail)) : ((tail) - (head)))

/**
 *@brief 获取实际有内容的队尾的索引
 *@note  不确保数据可以使用，因此在get之前需要自行获取UsedSize，当UsedSize==0时Head无效
**/
#define DumbRingFifoGetRealHead(size,head,tail) (head)

/**
 *@brief 获取实际有内容的队头的索引
 *@note  不确保数据可以使用，因此在get之前需要自行获取UsedSize，当UsedSize==0时RealTail无效
**/
#define DumbRingFifoGetRealTail(size,head,tail) DumbRingFifoGetPrevIndex(size,tail,1)

/**
 *@brief 获取环形队列的绕回长度
**/
#define DumbRingFifoGetLoopLength(size,head,tail) ((tail) > (head)) ? 0 : tail


/// 向前移动n步
#define RingFifoGetNextIndex(capacity,index,step) (((index) + (step)) % (capacity))
/// 向后移动n步
#define RingFifoGetPrevIndex(capacity,index,step) (((index) >= (step)) ? ((index) - (step)) : ((capacity) - ((step) - (index))))
/// 给定队头和数据长度，获取实际队尾位置
#define RingFifoGetRealTail(capacity,head,step) ((step)==0)? (head):RingFifoGetNextIndex((capacity),(head),(step)-1)
/// 给定队头和数据长度，获取下一个队尾位置
#define RingFifoGetDumbTail(capacity, head, step) (((head) + (step)) % (capacity))
/// 给定数据长度，获取可用空间大小
#define RingFifoGetFreeSize(capacity,step) ((capacity) - (step))
/// 给定队头和数据长度，获取连续可用剩余长度
#define RingFifoGetNoSplitFreeSize(capacity,head,step) ((((head) + (step)) < (capacity)) ? ((capacity) - ((head) + (step))) : RingFifoGetFreeSize(capacity, step))
/// 给定队头和数据长度，获取队列中不环回的长度
#define RingFifoGetNoSplitUsedSize(capacity,head,step) ((((head) + (step)) <= (capacity)) ? (step) : ((capacity) - (head)))
/// 给定队头和队尾，获取数据长度
#define RingFifoGetStep(capacity,real_head,real_tail) (((real_head) <= (real_tail)) ? ((real_tail) - (real_head) + 1) : ((capacity) - (real_head) + (real_tail) + 1))

/* 导出类型定义------------------------------------------*/
	
/**
 *@brief 用于表示生产者/消费者占有情况
**/
typedef struct
{
    uint16_t usHeadIndex;
    uint16_t usDataSize;
    uint8_t  ucFinishFlag;
}LockFreeActorNode;
typedef LockFreeActorNode* FifoProducer_t; ///< 生产者环形队列
typedef LockFreeActorNode* FifoConsumer_t; ///< 消费者环形队列

/**
 *@brief 无锁流缓冲区的静态句柄
**/
typedef struct
{
    void*      pvBufferMemory;            ///< 缓冲区物理起点
    uint16_t   usBlockSize;               ///< 块大小
    uint16_t   usBlockCount;              ///< 块个数

    FifoProducer_t xProducerList;         ///< 生产者环形队列句柄
    FifoConsumer_t xConsumerList;         ///< 消费者环形队列句柄
    uint8_t        ucProducerMax;       ///< 生产者队列长度
	uint8_t        ucConsumerMax;       ///< 消费者队列长度
	
	uint8_t        ucProducerHead;
	uint8_t        ucProducerSize; 
	uint8_t        ucConsumerHead;
	uint8_t        ucConsumerSize; 
	
	uint16_t bufProducerUsedHead;
	uint16_t bufProducerUsedSize;
	
	uint16_t bufProducerFreeHead;
	uint16_t bufProducerFreeSize;
	
	uint16_t bufConsumerUsedHead;
	uint16_t bufConsumerUsedSize;
	
	uint16_t bufConsumerFreeHead;
	uint16_t bufConsumerFreeSize;
	
}StaticLockFreeFifoHandle;

/**
 *@brief 无锁流缓冲区句柄
**/
typedef StaticLockFreeFifoHandle* LockFreeFifo_t;


/* 导出函数----------------------------------------------*/

//创建与删除
LockFreeFifo_t xLockFreeFifoCreate(uint16_t usBlockSize,uint16_t usBlockCount,uint8_t ucProducerSize,uint8_t ucConsumerSize);
void vLockFreeFifoDelete(LockFreeFifo_t xHandle);

//获取空闲空间
int xLockFreeFifoGetProducerFreeSize(LockFreeFifo_t xHandle);
int xLockFreeFifoGetProducerNoSplitFreeSize(LockFreeFifo_t xHandle);
int xLockFreeFifoGetConsumerFreeSize(LockFreeFifo_t xHandle);
int xLockFreeFifoGetConsumerNoSplitFreeSize(LockFreeFifo_t xHandle);

//间接写入
int xLockFreeFifoSendAcquire(LockFreeFifo_t xHandle,void* ppvItem[2],int xItemSize);
int xLockFreeFifoSendAcquireNoSplit(LockFreeFifo_t xHandle,void* ppvItem[2],int xItemSize);
int xLockFreeFifoSendComplete(LockFreeFifo_t xHandle, void* ppvItem[2]);

//间接读出
int xLockFreeFifoRecieveAcquire(LockFreeFifo_t xHandle,void* ppvItem[2],int xItemSize);
int xLockFreeFifoRecieveAcquireNoSplit(LockFreeFifo_t xHandle,void* ppvItem[2],int xItemSize);
int xLockFreeFifoRecieveComplete(LockFreeFifo_t xHandle, void* ppvItem[2]);

//单元测试
void vLockFreeFifoUnitTest_All(void);

//直接写入(todo)
int xLockFreeFifoSend(LockFreeFifo_t xHandle,const void* pvData,int xSize);//无法发送xSize则直接退出，完全不发送
int xLockFreeFifoSendUpto(LockFreeFifo_t xHandle,const void* pvData,int xSize);//无法发送xSize则尽可能多地发送

//直接读取(todo)
int xLockFreeFifoRecieve(LockFreeFifo_t xHandle,void* pvData,int xSize);//无法接收xSize则直接退出，完全不接收
int xLockFreeFifoRecieveUpto(LockFreeFifo_t xHandle,void* pvData,int xSize);//无法接收xSize则尽可能多地接收

/**
 *@}
**/

	
#ifdef __cplusplus
}
#endif