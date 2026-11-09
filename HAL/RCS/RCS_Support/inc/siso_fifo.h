/**
 * @file siso_fifo.h
 * @brief 实现FIFO的创建、销毁、发送、接收、声明完成等功能
 * @author CYK-Dot
 * @date 2025-05-13
 * @version 1.0
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* 头文件 -----------------------------------------------------*/

#include <stdlib.h>
#include <FreeRTOS.h>
#include <task.h>
#include "rcs_private_config.h"
#include "rcs_types.h"
	
/* 系统调用 ---------------------------------------------------*/

#define FifoPortMalloc pvPortMalloc
#define FifoPortFree   vPortFree
#define FifoPortEnterCriticalFromAll() do { } while (0)
#define FifoPortExitCriticalFromAll() do { } while (0)


/* 错误码 -----------------------------------------------------*/

#define RCS_FIFO_OK 0
#define RCS_FIFO_ERROR -1
#define RCS_FIFO_INVALID_PARAM -2
#define RCS_FIFO_NO_SPACE -3
#define RCS_FIFO_NO_DATA -4 
#define RCS_FIFO_NOT_ALLOWED -5


/* 导出类型 ---------------------------------------------------*/

/**
 * @brief 缓冲区对象
 */
typedef void* RcsFifo_t;

/**
 * @brief 缓冲区实例
 */
typedef struct 
{
    uint8_t *mem;
    size_t   memSize;
    size_t   indexWriteHead;
    size_t   indexWriteTail;
    size_t   indexReadHead;
    size_t   indexReadTail;
}RcsFifoHandle_t;

/* 导出函数 ---------------------------------------------------*/

RcsFifo_t RcsFifoCreateStatic(size_t fifoSize,RcsFifoHandle_t *staticHandle,uint8_t *fifoMemory);
RcsFifo_t RcsFifoCreate(size_t fifosize);
void RcsFifoDestroy(RcsFifo_t fifo);

int RcsFifoGetSendFreeSpace(RcsFifo_t fifo);
int RcsFifoGetSendFreeSpaceNoSplit(RcsFifo_t fifo);
int RcsFifoSendAcquire(RcsFifo_t fifo, size_t size, void *memAcquired[2]);
int RcsFifoSendAcquireNoSplit(RcsFifo_t fifo, size_t size, void *memAcquired[2]);
int RcsFifoSendComplete(RcsFifo_t fifo, const void *memAcquired[2]);

int RcsFifoGetRecvFreeSpace(RcsFifo_t fifo);
int RcsFifoGetRecvFreeSpaceNoSplit(RcsFifo_t fifo);
int RcsFifoRecvAcquire(RcsFifo_t fifo, size_t size, void *memAcquired[2]);
int RcsFifoRecvAcquireNoSplit(RcsFifo_t fifo, size_t size, void *memAcquired[2]);
int RcsFifoRecvComplete(RcsFifo_t fifo,const void *memAcquired[2]);

#ifdef __cplusplus
}
#endif