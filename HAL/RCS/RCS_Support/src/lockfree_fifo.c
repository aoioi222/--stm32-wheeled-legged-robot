/**
 *@defgroup RCSLIB-fifo
 *@addtogroup RCSLIB-fifo
 *
 * @author 陈煜楷@2025/3/19
 * 
 * @brief 以字节为单位的先进先出队列。
          概念上等于FreeRTOS的StreamBuffer，但使用了无锁设计，适用于裸机到RTOS的任意场景
 *
 * @note  在keil下打开rcs_lockfreefifo.c，下方可切换到Configuration Wizard，在wizard内可以配置缓冲区的行为
**/

/* 引入头文件-----------------------------------------------------*/
#include <string.h>
#include "lockfree_fifo.h"

#define LOG_TAG "[RCS_FIFO]"



/* 私有宏定义-----------------------------------------------------*/


/* 导出函数定义-----------------------------------------------------*/

/**
 *@brief     通过动态的方式创建一个无锁缓冲区
 *@param[IN] usBlockSize  单次写入的最小长度
 *@param[IN] usBlockCount 有多少个块
 *@param[IN] ucListSize   最大同时有多少个消费者/生产者
 *
 *@note      实际可用的字节为usBlockSize*(usBlockCount-1)
**/
LockFreeFifo_t xLockFreeFifoCreate(uint16_t usBlockSize,uint16_t usBlockCount,uint8_t ucProducerSize,uint8_t ucConsumerSize)
{
	//1个长度的缓冲区没有任何意义，还会导致代码出错
	//如果想要为单个数据提供读写缓冲，请使用shared_memory
	ASSERT(usBlockCount > 1);
	
    //申请句柄空间并将其归零
    LockFreeFifo_t xHandle  = (LockFreeFifo_t)pvCpuMalloc(sizeof(StaticLockFreeFifoHandle));
	ASSERT(xHandle);
	#ifdef ASSERT_FAULT_CODE
    if (xHandle == NULL)
    {
		LOGE("create:Malloc failed.Could not create fifo.\n");
        return NULL;
    }
	#endif
	
    memset(xHandle,0,sizeof(StaticLockFreeFifoHandle));

    //申请三个队列空间并将其归零
    xHandle->pvBufferMemory = pvCpuMalloc(usBlockSize * usBlockCount);
    xHandle->xProducerList  = (FifoProducer_t)pvCpuMalloc(sizeof(LockFreeActorNode) * (ucProducerSize));
    xHandle->xConsumerList  = (FifoConsumer_t)pvCpuMalloc(sizeof(LockFreeActorNode) * (ucConsumerSize));
	
	ASSERT(xHandle->pvBufferMemory);
	ASSERT(xHandle->xProducerList);
	ASSERT(xHandle->xConsumerList);
	#ifdef ASSERT_FAULT_CODE
    if (xHandle->pvBufferMemory == NULL || xHandle->xProducerList == NULL || xHandle->xConsumerList == NULL)
    {
        vCpuFree(xHandle->pvBufferMemory);
        vCpuFree(xHandle->xProducerList);
        vCpuFree(xHandle->xConsumerList);
        vCpuFree(xHandle);
		ASSERT(NULL);
        return NULL;
    }
	#endif
	
    memset(xHandle->pvBufferMemory, 0, usBlockSize * usBlockCount);
    memset(xHandle->xProducerList, 0, sizeof(LockFreeActorNode) * (ucProducerSize));
    memset(xHandle->xConsumerList, 0, sizeof(LockFreeActorNode) * (ucConsumerSize));
	//逆天，memset参数填错，直接导致FreeRTOS malloc失败，这个bug太难找了

    //设置队列参数
    xHandle->usBlockCount   = usBlockCount;
    xHandle->usBlockSize    = usBlockSize;
    xHandle->ucProducerMax = ucProducerSize;
	xHandle->ucConsumerMax = ucConsumerSize;
	xHandle->bufProducerFreeSize = usBlockCount;
	LOGI("create$ fifo at 0x%X created.Total size:%d\n",(uint32_t*)xHandle,(usBlockCount*usBlockSize));

    //成功
    return xHandle;
}

/**
 *@brief     将无锁缓冲区句柄的内存归还给堆区
 *@param[IN] xHandle 无锁缓冲区句柄
**/
void vLockFreeFifoDelete(LockFreeFifo_t xHandle)
{
	ASSERT(xHandle);
    if (xHandle == NULL)
    {
		LOGE("delete$ failed.\n");
        return;
    }
	
    // 释放 FIFO 缓冲区
    vCpuFree(xHandle->pvBufferMemory);

    // 释放生产者和消费者队列
    vCpuFree(xHandle->xProducerList);
    vCpuFree(xHandle->xConsumerList);

    // 释放主句柄
    vCpuFree(xHandle);
	LOGI("delete$ fifo at %X deleted.\n",(uint32_t*)xHandle);
}


/**
 *@brief 获取缓冲区中当前最大允许写入的空间
 *@param[IN] xHandle 无锁缓冲区句柄
 *@retval    int     最大可写入的字节数量
 *@note  如果生产者队列满了，无论缓冲区是否有空间，本函数都会返回0
**/
int xLockFreeFifoGetProducerFreeSize(LockFreeFifo_t xHandle)
{
    ASSERT(xHandle);
    #ifdef ASSERT_FAULT_CODE
    if (xHandle == NULL){
		LOGE("get_producer_free$ failed.fifo not exist.\n");
		return -EINVAL;
	}
    #endif
	
    return xHandle->bufProducerFreeSize * xHandle->usBlockSize;
}

/**
*@brief 获取缓冲区中当前最大允许写入的，不可分割的空间
 *@param[IN] xHandle 无锁缓冲区句柄
 *@retval    int     最大可写入的字节数量
 *@note  如果生产者队列满了，无论缓冲区是否有空间，本函数都会返回0
**/
int xLockFreeFifoGetProducerNoSplitFreeSize(LockFreeFifo_t xHandle)
{
    ASSERT(xHandle);
    #ifdef ASSERT_FAULT_CODE
    if (xHandle == NULL){
		LOGE("get_producer_nsplit_free$ failed.fifo not exist.\n");
		return -EINVAL;
	}
    #endif
	
	vCpuEnterCritical();
	uint16_t freeSizeNosplit = RingFifoGetNoSplitFreeSize(xHandle->usBlockCount,xHandle->bufProducerFreeHead,xHandle->bufProducerFreeSize);
    vCpuExitCritical();
	return freeSizeNosplit * xHandle->usBlockSize;
}


/**
 *@brief 获取缓冲区中当前最大允许读取的空间
 *@param[IN] xHandle 无锁缓冲区句柄
 *@retval    int     最大可读取的字节数量
 *@note  如果消费者队列满了，无论缓冲区是否有空间，本函数都会返回0
**/
int xLockFreeFifoGetConsumerFreeSize(LockFreeFifo_t xHandle)
{
	ASSERT(xHandle);
	#ifdef ASSERT_FAULT_CODE
	if (xHandle == NULL){
		LOGE("get_consumer_free$ failed.fifo not exist.\n");
		return -EINVAL;
	}
	#endif

	return xHandle->bufConsumerFreeSize * xHandle->usBlockSize;
}

/**
 *@brief 获取缓冲区中当前最大允许读取的，不可分割的空间
 *@param[IN] xHandle 无锁缓冲区句柄
 *@retval    int     最大可读取的字节数量
 *@note  如果消费者队列满了，无论缓冲区是否有空间，本函数都会返回0
**/
int xLockFreeFifoGetConsumerNoSplitFreeSize(LockFreeFifo_t xHandle)
{
	ASSERT(xHandle);
	#ifdef ASSERT_FAULT_CODE
	if (xHandle == NULL){
		LOGE("get_consumer_nsplit_free$ failed.fifo not exist.\n");
		return -EINVAL;
	}
	#endif
	
	vCpuEnterCritical();
	uint16_t freeSizeNosplit = RingFifoGetNoSplitUsedSize(xHandle->usBlockCount,xHandle->bufConsumerFreeHead,xHandle->bufConsumerFreeSize);
	vCpuExitCritical();
	return freeSizeNosplit * xHandle->usBlockSize;
}

/**
 *@brief      通过可分割的方式，从无锁缓冲区中申请写入
 *@param[IN]  xHandle    无锁缓冲区句柄
 *@param[OUT] ppvItem    获取的写入地址，是一个void*类型，长度为2的数组。[0]代表第一段，[1]代表第二段。如果没有第二段，那么[1]会变成NULL
 *@param[IN]  xItemSize  需要写入的数据长度，单位字节
 *@retval    int         负数代表失败，正数代表第一段写入地址的长度，注意取整。
 *
 *@warning   为了确保线程安全，本函数的内容大部分位于临界区。如果本函数执行期间在同一个中断通道上产生了>1次中断，会产生数据丢失。
**/
int xLockFreeFifoSendAcquire(LockFreeFifo_t xHandle,void* ppvItem[2],int xItemSize)
{
	LOGI("send_acquire$ acquire = %d bytes,handler = 0x%X",xItemSize,xHandle);
	LOGD("send_acquire$ P-FIFO       :head = %d block,length = %d block.",xHandle->ucProducerHead,xHandle->ucProducerSize);
	LOGD("send_acquire$ D-FIFO-P-FREE:head = %d block,length = %d block.",xHandle->bufProducerFreeHead,xHandle->bufProducerFreeSize);
	LOGD("send_acquire$ D-FIFO-P-USED:head = %d block,length = %d block.",xHandle->bufProducerUsedHead,xHandle->bufProducerUsedSize);
	ASSERT(xHandle);
	ASSERT(ppvItem);
	ASSERT(xItemSize>0);
	#ifdef ASSERT_FAULT_CODE
	if (xHandle == NULL || ppvItem==NULL || xItemSize<=0){
		LOGE("send_acquire$ failed.param not valid.\n");
		return -EINVAL;
	}
	#endif
	
	vCpuEnterCritical();
    //检查生产者队列是否已满
	if (xHandle->ucProducerSize == xHandle->ucProducerMax){
		LOGW("send_acquire$ P-FIFO full,failed.\n");
		vCpuExitCritical();
		return -ENOMEM;
	}
	//检查数据队列是否已满
	ASSERT(xHandle->bufProducerFreeSize*xHandle->usBlockSize >= xItemSize)
	#ifdef ASSERT_FAULT_CODE
    if (xHandle->bufProducerFreeSize*xHandle->usBlockSize < xItemSize)
    {
		LOGW("send_acquire$ Not enough space,failed.\n");
		vCpuExitCritical();
        return -ENOMEM;
    }
	#endif
    //计算占用的块个数和块起点
    uint16_t occupyCount = (xItemSize + xHandle->usBlockSize - 1) / xHandle->usBlockSize;
	uint16_t occupyHead  = xHandle->bufProducerFreeHead;
	uint8_t  occupyProducer = RingFifoGetDumbTail(xHandle->ucProducerMax,xHandle->ucProducerHead,xHandle->ucProducerSize);
    //增大P-FIFO-USED
    xHandle->xProducerList[occupyProducer].ucFinishFlag = 0;
    xHandle->xProducerList[occupyProducer].usHeadIndex  = occupyHead;
    xHandle->xProducerList[occupyProducer].usDataSize   = occupyCount;
	xHandle->ucProducerSize ++;
    //缩小D-FIFO-P-FREE
    xHandle->bufProducerFreeSize -= occupyCount;
	xHandle->bufProducerFreeHead  = RingFifoGetDumbTail(xHandle->usBlockCount,xHandle->bufProducerFreeHead,occupyCount);
	//扩大D-FIFO-P-USED
	xHandle->bufProducerUsedSize += occupyCount;
	LOGD("send_acquire$ Acquire done.");
	LOGD("send_acquire$ P-FIFO       :head = %d block,length = %d block.",xHandle->ucProducerHead,xHandle->ucProducerSize);
	LOGD("send_acquire$ D-FIFO-P-FREE:head = %d block,length = %d block.",xHandle->bufProducerFreeHead,xHandle->bufProducerFreeSize);
	LOGD("send_acquire$ D-FIFO-P-USED:head = %d block,length = %d block.",xHandle->bufProducerUsedHead,xHandle->bufProducerUsedSize);
	
	vCpuExitCritical();

    // 计算实际的写入地址
	uint16_t noSplitSize = RingFifoGetNoSplitUsedSize(xHandle->usBlockCount,occupyHead,occupyCount);
    ppvItem[0] = (uint8_t*)xHandle->pvBufferMemory + occupyHead * xHandle->usBlockSize;

    //环绕
    if (noSplitSize == occupyCount){
        ppvItem[1] = NULL;
    }
    else{
        ppvItem[1] = xHandle->pvBufferMemory;
    }
	
	LOGI("send_acquire$ acquired = %d+%d blocks,acquired = %d bytes\n",noSplitSize,occupyCount-noSplitSize,occupyCount*xHandle->usBlockSize);
    return noSplitSize * xHandle->usBlockSize;
}

/**
 *@brief      通过不可分割的方式，从无锁缓冲区中申请写入
 *@param[IN]  xHandle    无锁缓冲区句柄
 *@param[OUT] ppvItem    获取的写入地址，是一个void*类型，长度为2的数组。对不可分割而言，只有ppvItem[0]是有效的
 *@param[IN]  xItemSize  需要写入的数据长度，单位字节
 *@retval    int         负数代表失败，正数代表第二段写入地址的长度
 *
 *@warning   为了确保线程安全，本函数的内容大部分位于临界区。如果本函数执行期间在同一个中断通道上产生了>1次中断，会产生数据丢失。
**/
int xLockFreeFifoSendAcquireNoSplit(LockFreeFifo_t xHandle,void* ppvItem[2],int xItemSize)
{
	LOGI("send_acnsplt$ acquire = %d bytes,handler = 0x%X",xItemSize,xHandle);
	LOGD("send_acnsplt$ P-FIFO       :head = %d block,length = %d block.",xHandle->ucProducerHead,xHandle->ucProducerSize);
	LOGD("send_acnsplt$ D-FIFO-P-FREE:head = %d block,length = %d block.",xHandle->bufProducerFreeHead,xHandle->bufProducerFreeSize);
	LOGD("send_acnsplt$ D-FIFO-P-USED:head = %d block,length = %d block.",xHandle->bufProducerUsedHead,xHandle->bufProducerUsedSize);
	ASSERT(xHandle);
	ASSERT(ppvItem);
	ASSERT(xItemSize>0);
	#ifdef ASSERT_FAULT_CODE
	if (xHandle == NULL || ppvItem==NULL || xItemSize<=0){
		LOGE("send_acnsplt$ failed.param not valid.\n");
		return -EINVAL;
	}
	#endif
	
	vCpuEnterCritical();
    //检查生产者队列是否已满
	if (xHandle->ucProducerSize == xHandle->ucProducerMax){
		LOGW("send_acnsplt$ P-FIFO full,failed.\n");
		vCpuExitCritical();
		return -ENOMEM;
	}
	//检查数据队列是否已满
	//2025/4/16:RingFifoGetNoSplitFreeSize更改为RingFifoGetNoSplitUsedSize
	uint16_t freeSizeNosplit = RingFifoGetNoSplitUsedSize(xHandle->usBlockCount,xHandle->bufProducerFreeHead,xHandle->bufProducerFreeSize);
	LOGD("send_acnsplt$ D-FIFO-P-FREE:head = %d block,no-split-length = %d block,acquire = %d bytes",xHandle->bufProducerFreeHead,freeSizeNosplit,xItemSize);
	ASSERT(freeSizeNosplit*xHandle->usBlockSize >= xItemSize)
	#ifdef ASSERT_FAULT_CODE
    if (freeSizeNosplit*xHandle->usBlockSize < xItemSize)
    {
		LOGW("send_acnsplt$ Not enough space,failed.\n");
		vCpuExitCritical();
        return -ENOMEM;
    }
	#endif
    //计算占用的块个数和块起点
    uint16_t occupyCount = (xItemSize + xHandle->usBlockSize - 1) / xHandle->usBlockSize;
	uint16_t occupyHead  = xHandle->bufProducerFreeHead;
	uint8_t  occupyProducer = RingFifoGetDumbTail(xHandle->ucProducerMax,xHandle->ucProducerHead,xHandle->ucProducerSize);
    //增大P-FIFO-USED
    xHandle->xProducerList[occupyProducer].ucFinishFlag = 0;
    xHandle->xProducerList[occupyProducer].usHeadIndex  = occupyHead;
    xHandle->xProducerList[occupyProducer].usDataSize   = occupyCount;
	xHandle->ucProducerSize ++;
    //缩小D-FIFO-P-FREE
    xHandle->bufProducerFreeSize -= occupyCount;
	xHandle->bufProducerFreeHead  = RingFifoGetDumbTail(xHandle->usBlockCount,xHandle->bufProducerFreeHead,occupyCount);
	//扩大D-FIFO-P-USED
	xHandle->bufProducerUsedSize += occupyCount;
	LOGD("send_acnsplt$ Acquire done.");
	LOGD("send_acnsplt$ P-FIFO       :head = %d block,length = %d block.",xHandle->ucProducerHead,xHandle->ucProducerSize);
	LOGD("send_acnsplt$ D-FIFO-P-FREE:head = %d block,length = %d block.",xHandle->bufProducerFreeHead,xHandle->bufProducerFreeSize);
	LOGD("send_acnsplt$ D-FIFO-P-USED:head = %d block,length = %d block.",xHandle->bufProducerUsedHead,xHandle->bufProducerUsedSize);
	
	vCpuExitCritical();

    // 计算实际的写入地址
    ppvItem[0] = (uint8_t*)xHandle->pvBufferMemory + occupyHead * xHandle->usBlockSize;
	ppvItem[1] = NULL;
	LOGI("send_acnsplt$ acquired = %d+0 blocks,acquired = %d bytes\n",occupyCount,occupyCount*xHandle->usBlockSize);
    return occupyCount * xHandle->usBlockSize;
}
/**
 *@brief     告知缓冲区，该段数据已经发送完成，可以交由消费者接收
 *@param[IN]  xHandle 无锁缓冲区句柄
 *@param[IN]  ppvItem 完成写入的数据的地址
 *
 *@retval    int     负数代表失败，非负数代表成功
**/
int xLockFreeFifoSendComplete(LockFreeFifo_t xHandle, void* ppvItem[2])
{
	//入参校验
	ASSERT(xHandle);
	ASSERT(ppvItem);
	ASSERT(ppvItem[0]);
	#ifdef ASSERT_FAULT_CODE
	if (xHandle==NULL || ppvItem==NULL || ppvItem[0]==NULL){
		LOGE("send_complet$ failed.param not valid.\n");
		return -EINVAL;
	}
	#endif

	vCpuEnterCritical();
	//操作校验
	ASSERT(xHandle->ucProducerSize != 0);
	#ifdef ASSERT_FAULT_CODE
	if (xHandle->ucProducerSize == 0){
		LOGE("send_complet$ P-FIFO has no valid members,failed.\n");
		vCpuExitCritical();
        return -EINVAL;
    }
	#endif
    //寄存器拷贝
    uint16_t dataHeadInFifo   = ((uint8_t*)ppvItem[0] - (uint8_t*)xHandle->pvBufferMemory) / xHandle->usBlockSize;
	LOGI("send_complet$ return-block = 0x%X(index=%d block),handler = 0x%X",ppvItem[0],dataHeadInFifo,xHandle);
	LOGD("send_complet$ P-FIFO       :head = %hhd block,size = %hhd block.",xHandle->ucProducerHead,xHandle->ucProducerSize);
	LOGD("send_complet$ D-FIFO-P-USED:head = %hd block,size = %hd block.",xHandle->bufProducerUsedHead,xHandle->bufProducerUsedSize);
	LOGD("send_complet$ D-FIFO-C-FREE:head = %hd block,size = %hd block.",xHandle->bufConsumerFreeHead,xHandle->bufConsumerFreeSize);
	
	//只有一个元素，直接检验即可
	if (xHandle->ucProducerMax ==1){
		ASSERT(xHandle->xProducerList[xHandle->ucProducerHead].usHeadIndex == dataHeadInFifo);
		if (xHandle->xProducerList[xHandle->ucProducerHead].usHeadIndex != dataHeadInFifo){
			LOGE("send_complet$ block not found,failed.\n");
			vCpuExitCritical();
			return -EEXIST;
		}
		//D-FIFO-C-FREE扩大
		xHandle->bufConsumerFreeSize += xHandle->bufProducerUsedSize;
		//D-FIFO-P-USED缩小
		xHandle->bufProducerUsedHead = RingFifoGetDumbTail(xHandle->usBlockCount,xHandle->bufProducerUsedHead,xHandle->bufProducerUsedSize);
		xHandle->bufProducerUsedSize = 0;
		//P-FIFO缩小
		xHandle->ucProducerSize = 0;
	}
	//有多个元素，需要遍历
	else{
		//检查待complete的元素是否在D-FIFO-P-USED中
		uint16_t step =  RingFifoGetStep(xHandle->usBlockCount,xHandle->bufProducerUsedHead,dataHeadInFifo);
		ASSERT(step <= xHandle->bufProducerUsedSize);
		if (step > xHandle->bufProducerUsedSize){
			LOGE("send_complet$ block not found,failed.\n");
			vCpuExitCritical();
			return -EEXIST;
		}
		//为该元素标记完成，并检查是否可以移动队头
		uint8_t index = xHandle->ucProducerHead;
		uint8_t indexFinishDumb = index;
		uint8_t indexTail   = RingFifoGetDumbTail(xHandle->ucProducerMax,xHandle->ucProducerHead,xHandle->ucProducerSize);
		LOGD("send_complet$ find %hd(D-FIFO-P-USED) from %hhd(P-FIFO,real) to %hhd(P-FIFO,dumb)",dataHeadInFifo,index,indexTail);
		while(index != indexTail){
			//当前位置是否为目标位置
			if (xHandle->xProducerList[index].usHeadIndex == dataHeadInFifo){
				LOGD("send_complet$ %hhd(P-FIFO): set finish flag.",index);
				xHandle->xProducerList[index].ucFinishFlag = 1;
			}
			//当indexFinishDumb跟着index移动时，证明前面的位置都完成了，只要有一个没有完成，就会跟不上
			if (indexFinishDumb == index){
				if (xHandle->xProducerList[index].ucFinishFlag == 1){
					LOGD("send_complet$ %hhd(P-FIFO): move finish index.",index);
					indexFinishDumb = RingFifoGetNextIndex(xHandle->ucProducerMax,indexFinishDumb,1);
				}
			}
			index = RingFifoGetNextIndex(xHandle->ucProducerMax,index,1);
		}
		LOGD("send_complet$ finish index = %hhd(P-FIFO,dumb)",indexFinishDumb);
		//没有任何人完成
		if ((indexFinishDumb == xHandle->ucProducerHead)){
			LOGE("send_complet$ Send Complete Done.Not move D-FIFO-P-USED.\n");
			vCpuExitCritical();
			return 0;
		}
		//有人完成
		else{
			//哑元所在的位置不一定有数据，不能访问
			uint8_t indexFinishReal = RingFifoGetPrevIndex(xHandle->ucProducerMax,indexFinishDumb,1);
			//D-FIFO-P-USED缩小
			uint16_t shrinkDumbHead = RingFifoGetDumbTail(xHandle->usBlockCount,
			                                          xHandle->xProducerList[indexFinishReal].usHeadIndex,
			                                          xHandle->xProducerList[indexFinishReal].usDataSize);
			uint16_t shrinkRealHead = RingFifoGetPrevIndex(xHandle->usBlockCount,shrinkDumbHead,1);
			uint16_t shrinkSize = RingFifoGetStep(xHandle->usBlockCount,xHandle->bufProducerUsedHead,shrinkRealHead);
			xHandle->bufProducerUsedHead = shrinkDumbHead;
			xHandle->bufProducerUsedSize -= shrinkSize;
			//D-FIFO-C-FREE扩大
			xHandle->bufConsumerFreeSize += shrinkSize;
			//P-FIFO缩小
			uint8_t shrinkProducer = RingFifoGetStep(xHandle->ucProducerMax,xHandle->ucProducerHead,indexFinishReal);
			xHandle->ucProducerHead = indexFinishDumb;
			xHandle->ucProducerSize -= shrinkProducer;
		}
	}
	//碎片整理
	if ((xHandle->bufConsumerUsedSize == 0) &&
		(xHandle->bufConsumerFreeSize == 0) &&
		(xHandle->bufProducerUsedSize == 0)) {
			xHandle->bufConsumerUsedHead = 0;
			xHandle->bufConsumerFreeHead = 0;
			xHandle->bufProducerUsedHead = 0;
			xHandle->bufProducerFreeHead = 0;
		LOGD("send_complet$ defragment done.");
	}
	
	LOGD("send_complet$ send somplete report.");
	LOGD("send_complet$ P-FIFO       :head = %hhd block,size = %hhd block.",xHandle->ucProducerHead,xHandle->ucProducerSize);
	LOGD("send_complet$ D-FIFO-P-USED:head = %hd block,size = %hd block.",xHandle->bufProducerUsedHead,xHandle->bufProducerUsedSize);
	LOGD("send_complet$ D-FIFO-C-FREE:head = %hd block,size = %hd block.",xHandle->bufConsumerFreeHead,xHandle->bufConsumerFreeSize);
	LOGI("send_complet$ Send Complete Done.\n");
	vCpuExitCritical();
    return 0;
}

/**
 *@brief      通过可分割的方式，从无锁缓冲区中申请读出
 *@param[IN]  xHandle    无锁缓冲区句柄
 *@param[OUT] ppvItem    获取的读取地址，是一个void*类型，长度为2的数组。[0]代表第一段，[1]代表第二段。如果没有第二段，那么[1]会变成NULL
 *@param[IN]  xItemSize  需要读取的数据长度，单位字节
 *@retval    int         负数代表失败，正数代表第二段写入地址的长度
 *
 *@warning   为了确保线程安全，本函数的内容大部分位于临界区。如果本函数执行期间在同一个中断通道上产生了>1次中断，会产生数据丢失。
**/
int xLockFreeFifoRecieveAcquire(LockFreeFifo_t xHandle,void* ppvItem[2],int xItemSize)
{
	LOGI("recv_acquire$ acquire = %d bytes,handler = 0x%X",xItemSize,xHandle);
	LOGD("recv_acquire$ C-FIFO       :head = %d block,length = %d block.",xHandle->ucConsumerHead,xHandle->ucConsumerSize);
	LOGD("recv_acquire$ D-FIFO-C-FREE:head = %d block,length = %d block.",xHandle->bufConsumerFreeHead,xHandle->bufConsumerFreeSize);
	LOGD("recv_acquire$ D-FIFO-C-USED:head = %d block,length = %d block.",xHandle->bufConsumerUsedHead,xHandle->bufConsumerUsedSize);
	ASSERT(xHandle);
	ASSERT(ppvItem);
	ASSERT(xItemSize >0);
	#ifdef ASSERT_FAULT_CODE
	if (xHandle==NULL || ppvItem==NULL || xItemSize<=0){
		LOGE("recv_acquire$ failed.param not valid.\n");
		return -EINVAL;
	}
	#endif
	
    vCpuEnterCritical();
	//检查消费者者队列是否已满
	if (xHandle->ucConsumerSize == xHandle->ucConsumerMax){
		LOGW("recv_acquire$ C-FIFO full,failed.\n");
		vCpuExitCritical();
		return -ENOMEM;
	}
	//求解剩余空间
    ASSERT(xItemSize <= (xHandle->bufConsumerFreeSize * xHandle->usBlockSize))
	#ifdef ASSERT_FAULT_CODE
    if (xItemSize > (xHandle->bufConsumerFreeSize * xHandle->usBlockSize)){
		LOGW("recv_acquire$ Not enough space,failed.\n");
		vCpuExitCritical();
        return -ENOMEM;
    }
	#endif
    //计算占用的块个数和块起点
    uint16_t occupyCount = (xItemSize + xHandle->usBlockSize - 1) / xHandle->usBlockSize;
	uint16_t occupyHead  = xHandle->bufConsumerFreeHead;
	uint8_t  occupyConsumer = RingFifoGetDumbTail(xHandle->ucConsumerMax,xHandle->ucConsumerHead,xHandle->ucConsumerSize);
    //增大C-FIFO
    xHandle->xConsumerList[occupyConsumer].ucFinishFlag = 0;
    xHandle->xConsumerList[occupyConsumer].usHeadIndex  = occupyHead;
    xHandle->xConsumerList[occupyConsumer].usDataSize   = occupyCount;
	xHandle->ucConsumerSize ++;
    //缩小D-FIFO-C-FREE
    xHandle->bufConsumerFreeSize -= occupyCount;
	xHandle->bufConsumerFreeHead  = RingFifoGetDumbTail(xHandle->usBlockCount,xHandle->bufConsumerFreeHead,occupyCount);
	//扩大D-FIFO-C-USED
	xHandle->bufConsumerUsedSize += occupyCount;
	LOGD("recv_acquire$ Acquire done.");
	LOGD("recv_acquire$ C-FIFO       :head = %d block,length = %d block.",xHandle->ucConsumerHead,xHandle->ucConsumerSize);
	LOGD("recv_acquire$ D-FIFO-C-FREE:head = %d block,length = %d block.",xHandle->bufConsumerFreeHead,xHandle->bufConsumerFreeSize);
	LOGD("recv_acquire$ D-FIFO-C-USED:head = %d block,length = %d block.",xHandle->bufConsumerUsedHead,xHandle->bufConsumerUsedSize);
	vCpuExitCritical();
    // 计算实际的写入地址
	uint16_t noSplitSize = RingFifoGetNoSplitUsedSize(xHandle->usBlockCount,occupyHead,occupyCount);
    ppvItem[0] = (uint8_t*)xHandle->pvBufferMemory + occupyHead * xHandle->usBlockSize;
    //环绕
    if (noSplitSize == occupyCount){
        ppvItem[1] = NULL;
    }
    else{
        ppvItem[1] = xHandle->pvBufferMemory;
    }
	LOGI("recv_acquire$ acquired = %d+%d blocks,acquired = %d bytes\n",noSplitSize,occupyCount-noSplitSize,occupyCount*xHandle->usBlockSize);
    return noSplitSize * xHandle->usBlockSize;
}

/**
 *@brief      通过不可分割的方式，从无锁缓冲区中申请读出
 *@param[IN]  xHandle    无锁缓冲区句柄
 *@param[OUT] ppvItem    获取的读取地址，是一个void*类型，长度为2的数组。对不可分割而言，只有ppvItem[0]是有效的
 *@param[IN]  xItemSize  需要读取的数据长度，单位字节
 *@retval    int         负数代表失败，正数代表第二段写入地址的长度
 *
 *@warning   为了确保线程安全，本函数的内容大部分位于临界区。如果本函数执行期间在同一个中断通道上产生了>1次中断，会产生数据丢失。
**/
int xLockFreeFifoRecieveAcquireNoSplit(LockFreeFifo_t xHandle,void* ppvItem[2],int xItemSize)
{
	LOGI("recv_acnsplt$ acquire = %d bytes,handler = 0x%X",xItemSize,xHandle);
	LOGD("recv_acnsplt$ C-FIFO       :head = %d block,length = %d block.",xHandle->ucConsumerHead,xHandle->ucConsumerSize);
	LOGD("recv_acnsplt$ D-FIFO-C-FREE:head = %d block,length = %d block.",xHandle->bufConsumerFreeHead,xHandle->bufConsumerFreeSize);
	LOGD("recv_acnsplt$ D-FIFO-C-USED:head = %d block,length = %d block.",xHandle->bufConsumerUsedHead,xHandle->bufConsumerUsedSize);
	ASSERT(xHandle);
	ASSERT(ppvItem);
	ASSERT(xItemSize>0);
	#ifdef ASSERT_FAULT_CODE
	if (xHandle == NULL || ppvItem==NULL || xItemSize<=0){
		LOGE("recv_acnsplt$ failed.param not valid.\n");
		return -EINVAL;
	}
	#endif
	
	vCpuEnterCritical();
    //检查生产者队列是否已满
	if (xHandle->ucConsumerSize == xHandle->ucConsumerMax){
		LOGW("recv_acnsplt$ P-FIFO full,failed.\n");
		vCpuExitCritical();
		return -ENOMEM;
	}
	//检查数据队列是否已满
	uint16_t freeSizeNosplit = RingFifoGetNoSplitUsedSize(xHandle->usBlockCount,xHandle->bufConsumerFreeHead,xHandle->bufConsumerFreeSize);
	ASSERT(freeSizeNosplit*xHandle->usBlockSize >= xItemSize)
	#ifdef ASSERT_FAULT_CODE
    if (freeSizeNosplit*xHandle->usBlockSize < xItemSize)
    {
		LOGW("recv_acnsplt$ Not enough space,failed.\n");
		vCpuExitCritical();
        return -ENOMEM;
    }
	#endif
    //计算占用的块个数和块起点
    uint16_t occupyCount = (xItemSize + xHandle->usBlockSize - 1) / xHandle->usBlockSize;
	uint16_t occupyHead  = xHandle->bufConsumerFreeHead;
	uint8_t  occupyConsumer = RingFifoGetDumbTail(xHandle->ucConsumerMax,xHandle->ucConsumerHead,xHandle->ucConsumerSize);
    //增大C-FIFO
    xHandle->xConsumerList[occupyConsumer].ucFinishFlag = 0;
    xHandle->xConsumerList[occupyConsumer].usHeadIndex  = occupyHead;
    xHandle->xConsumerList[occupyConsumer].usDataSize   = occupyCount;
	xHandle->ucConsumerSize ++;
    //缩小D-FIFO-C-FREE
    xHandle->bufConsumerFreeSize -= occupyCount;
	xHandle->bufConsumerFreeHead  = RingFifoGetDumbTail(xHandle->usBlockCount,xHandle->bufConsumerFreeHead,occupyCount);
	//扩大D-FIFO-C-USED
	xHandle->bufConsumerUsedSize += occupyCount;
	LOGD("recv_acnsplt$ Acquire done.");
	LOGD("recv_acnsplt$ C-FIFO       :head = %d block,length = %d block.",xHandle->ucConsumerHead,xHandle->ucConsumerSize);
	LOGD("recv_acnsplt$ D-FIFO-C-FREE:head = %d block,length = %d block.",xHandle->bufConsumerFreeHead,xHandle->bufConsumerFreeSize);
	LOGD("recv_acnsplt$ D-FIFO-C-USED:head = %d block,length = %d block.",xHandle->bufConsumerUsedHead,xHandle->bufConsumerUsedSize);
	
	vCpuExitCritical();

    // 计算实际的写入地址
    ppvItem[0] = (uint8_t*)xHandle->pvBufferMemory + occupyHead * xHandle->usBlockSize;
	ppvItem[1] = NULL;
	LOGI("recv_acnsplt$ acquired = %d+0 blocks,acquired = %d bytes\n",occupyCount,occupyCount*xHandle->usBlockSize);
    return occupyCount * xHandle->usBlockSize;
}

/**
 *@brief     告知缓冲区，该段数据已经接收完成，内存空间可以交由消费者发送
 *@param[IN]  xHandle 无锁缓冲区句柄
 *@param[IN]  ppvItem 完成读取的数据的地址
 *
 *@retval    int     负数代表失败，非负数代表成功
**/
int xLockFreeFifoRecieveComplete(LockFreeFifo_t xHandle, void* ppvItem[2])
{
	//入参校验
	ASSERT(xHandle);
	ASSERT(ppvItem);
	ASSERT(ppvItem[0]);
	#ifdef ASSERT_FAULT_CODE
	if (xHandle==NULL || ppvItem==NULL || ppvItem[0]==NULL){
		LOGE("recv_complet$ failed.param not valid.\n");
		return -EINVAL;
	}
	#endif

	vCpuEnterCritical();
	//操作校验
	ASSERT(xHandle->ucConsumerSize != 0);
	#ifdef ASSERT_FAULT_CODE
	if (xHandle->ucConsumerSize == 0){
		LOGE("recv_complet$ C-FIFO has no valid members,failed.\n");
		vCpuExitCritical();
        return -EINVAL;
    }
	#endif
    //寄存器拷贝
    uint16_t dataHeadInFifo   = ((uint8_t*)ppvItem[0] - (uint8_t*)xHandle->pvBufferMemory) / xHandle->usBlockSize;
	LOGI("recv_complet$ return-block = 0x%X(index=%d block),handler = 0x%X",ppvItem[0],dataHeadInFifo,xHandle);
	LOGD("recv_complet$ C-FIFO       :head = %d block,size = %d block.",xHandle->ucConsumerHead,xHandle->ucConsumerSize);
	LOGD("recv_complet$ D-FIFO-C-USED:head = %d block,size = %d block.",xHandle->bufConsumerUsedHead,xHandle->bufConsumerUsedSize);
	LOGD("recv_complet$ D-FIFO-P-FREE:head = %d block,size = %d block.",xHandle->bufProducerFreeHead,xHandle->bufProducerFreeSize);
	
	//只有一个元素，直接检验即可
	if (xHandle->ucConsumerMax ==1){
		ASSERT(xHandle->xConsumerList[xHandle->ucConsumerHead].usHeadIndex == dataHeadInFifo);
		if (xHandle->xConsumerList[xHandle->ucConsumerHead].usHeadIndex != dataHeadInFifo){
			LOGE("recv_complet$ block not found,failed.\n");
			vCpuExitCritical();
			return -EEXIST;
		}
		//D-FIFO-P-FREE扩大
		xHandle->bufProducerFreeSize += xHandle->bufConsumerUsedSize;
		//D-FIFO-C-USED缩小
		xHandle->bufConsumerUsedHead = RingFifoGetDumbTail(xHandle->usBlockCount,xHandle->bufConsumerUsedHead,xHandle->bufConsumerUsedSize);
		xHandle->bufConsumerUsedSize = 0;
		//C-FIFO缩小
		xHandle->ucConsumerSize = 0;
	}
	//有多个元素，需要遍历
	else{
		//检查待complete的元素是否在D-FIFO-P-USED中
		uint16_t step =  RingFifoGetStep(xHandle->usBlockCount,xHandle->bufConsumerUsedHead,dataHeadInFifo);
		ASSERT(step <= xHandle->bufConsumerUsedSize);
		if (step > xHandle->bufConsumerUsedSize){
			LOGE("recv_complet$ block not found,failed.\n");
			vCpuExitCritical();
			return -EEXIST;
		}
		//为该元素标记完成，并检查是否可以移动队头
		uint8_t index = xHandle->ucConsumerHead;
		uint8_t indexFinishDumb = index;
		uint8_t indexTail   = RingFifoGetDumbTail(xHandle->ucConsumerMax,xHandle->ucConsumerHead,xHandle->ucConsumerSize);
		while(index != indexTail){
			//当前位置是否为目标位置
			if (xHandle->xConsumerList[index].usHeadIndex == dataHeadInFifo){
				xHandle->xConsumerList[index].ucFinishFlag = 1;
			}
			//当indexFinishDumb跟着index移动时，证明前面的位置都完成了，只要有一个没有完成，就会跟不上
			if (indexFinishDumb == index){
				if (xHandle->xConsumerList[index].ucFinishFlag == 1){
					indexFinishDumb = RingFifoGetNextIndex(xHandle->ucConsumerMax,indexFinishDumb,1);
				}
			}
			index = RingFifoGetNextIndex(xHandle->ucProducerMax,index,1);
		}
		//没有任何人完成
		if ((indexFinishDumb == xHandle->ucConsumerHead)){
			vCpuExitCritical();
			return 0;
		}
		//有人完成
		else{
			//哑元所在的位置不一定有数据，不能访问
			uint8_t indexFinishReal = RingFifoGetPrevIndex(xHandle->ucConsumerMax,indexFinishDumb,1);
			//D-FIFO-C-USED缩小
			uint16_t shrinkDumbHead = RingFifoGetDumbTail(xHandle->usBlockCount,
			                                          xHandle->xConsumerList[indexFinishReal].usHeadIndex,
			                                          xHandle->xConsumerList[indexFinishReal].usDataSize);
			uint16_t shrinkRealHead = RingFifoGetPrevIndex(xHandle->usBlockCount,shrinkDumbHead,1);
			uint16_t shrinkSize = RingFifoGetStep(xHandle->usBlockCount,xHandle->bufConsumerUsedHead,shrinkRealHead);
			xHandle->bufConsumerUsedHead = shrinkDumbHead;
			xHandle->bufConsumerUsedSize -= shrinkSize;
			//D-FIFO-P-FREE扩大
			xHandle->bufProducerFreeSize += shrinkSize;
			//C-FIFO缩小
			uint8_t shrinkConsumer = RingFifoGetStep(xHandle->ucConsumerMax,xHandle->ucConsumerHead,indexFinishReal);
			xHandle->ucConsumerHead = indexFinishDumb;
			xHandle->ucConsumerSize -= shrinkConsumer;
		}
	}
	
	//碎片整理
	if ((xHandle->bufConsumerUsedSize == 0) &&
		(xHandle->bufConsumerFreeSize == 0) &&
		(xHandle->bufProducerUsedSize == 0)) {
			xHandle->bufConsumerUsedHead = 0;
			xHandle->bufConsumerFreeHead = 0;
			xHandle->bufProducerUsedHead = 0;
			xHandle->bufProducerFreeHead = 0;
		LOGD("recv_complet$ defragment done.");
	}
		
	LOGD("recv_complet$ Recv complete report.");
	LOGD("recv_complet$ C-FIFO       :head = %hhd block,size = %hhd block.",xHandle->ucConsumerHead,xHandle->ucConsumerSize);
	LOGD("recv_complet$ D-FIFO-C-USED:head = %hd block,size = %hd block.",xHandle->bufConsumerUsedHead,xHandle->bufConsumerUsedSize);
	LOGD("recv_complet$ D-FIFO-P-FREE:head = %hd block,size = %hd block.\n",xHandle->bufProducerFreeHead,xHandle->bufProducerFreeSize);
	LOGI("recv_complet$ Done.\n");
	vCpuExitCritical();
    return 0;
}


/**
 *@breif 单元测试1：一人发一人收，无绕回
**/
void vLockFreeFifoUnitTest_SISO_NotCirlcle(void)
{
	static LockFreeFifo_t g_utestFifo;
	LOGI("UTest1$ block 1*40,producer 1,consumer 1");
	g_utestFifo = xLockFreeFifoCreate(1,20,1,1);
	void* item[2];
	int count;
	int err;
	
	//不完全占用写入，理应成功，5数据
	LOGI("-------------------------------------------------");
	count = xLockFreeFifoSendAcquire(g_utestFifo,item,5);
	memcpy(item[0],"Hello",5);
	LOGI(" ");
	err = xLockFreeFifoSendComplete(g_utestFifo,item);
	LOGI(" ");
	LOGI("UTest1$ get %d+%d block,complete err code %d",count,5-count,err);
	//完全占用写入，理应成功，15数据
	LOGI("-------------------------------------------------");
	count = xLockFreeFifoSendAcquire(g_utestFifo,item,15);
	memcpy(item[0],"!!!@@@###$$$%%%",15);
	LOGI(" ");
	err = xLockFreeFifoSendComplete(g_utestFifo,item);
	LOGI(" ");
	LOGI("UTest1$ get %d+%d block,complete err code %d",count,15-count,err);
	//不完全占用读取，理应成功，10数据
	LOGI("-------------------------------------------------");
	count = xLockFreeFifoRecieveAcquire(g_utestFifo,item,10);
	LOGI(" ");
	err = xLockFreeFifoRecieveComplete(g_utestFifo,item);
	LOGI(" ");
	LOGI("UTest1$ get %d+%d block,complete err code %d",count,10-count,err);
	LOGI("UTest1$ recieved %s",item[0]);
	//完全占用读取，理应成功，0数据
	LOGI("-------------------------------------------------");
	count = xLockFreeFifoRecieveAcquire(g_utestFifo,item,10);
	LOGI(" ");
	err = xLockFreeFifoRecieveComplete(g_utestFifo,item);
	LOGI(" ");
	LOGI("UTest1$ get %d+%d block,complete err code %d",count,10-count,err);
	LOGI("UTest1$ recieved %s",item[0]);
	//完全占用写入，理应成功，20数据
	LOGI("-------------------------------------------------");
	count = xLockFreeFifoSendAcquire(g_utestFifo,item,20);
	memcpy(item[0],"AAAABBBBCCCCDDDDEEE",20);
	LOGI(" ");
	err = xLockFreeFifoSendComplete(g_utestFifo,item);
	LOGI(" ");
	LOGI("UTest1$ get %d+%d block,complete err code %d",count,20-count,err);
	//完全占用读取，理应成功，0数据
	LOGI("-------------------------------------------------");
	count = xLockFreeFifoRecieveAcquire(g_utestFifo,item,20);
	LOGI(" ");
	err = xLockFreeFifoRecieveComplete(g_utestFifo,item);
	LOGI(" ");
	LOGI("UTest1$ get %d+%d block,complete err code %d",count,20-count,err);
	LOGI("UTest1$ recieved %s",item[0]);
}

/**
 *@breif 单元测试2：一人发一人收，绕回
**/
void vLockFreeFifoUnitTest_SISO_Cirlcle(void)
{
	static LockFreeFifo_t g_utestFifo;
	LOGI("UTest$ block 1*40,producer 1,consumer 1");
	g_utestFifo = xLockFreeFifoCreate(1,25,1,1);
	void* item[2];
	int count;
	int err;
	
	//条件准备
	LOGI("-------------------------------------------------");
	count = xLockFreeFifoSendAcquire(g_utestFifo,item,10);
	memcpy(item[0],"HelloWorld",10);
	LOGI(" ");
	err = xLockFreeFifoSendComplete(g_utestFifo,item);
	LOGI(" ");
	LOGI("UTest$ get %d+%d block,complete err code %d",count,10-count,err);
	//条件准备
	LOGI("-------------------------------------------------");
	count = xLockFreeFifoRecieveAcquire(g_utestFifo,item,10);
	LOGI(" ");
	err = xLockFreeFifoRecieveComplete(g_utestFifo,item);
	LOGI(" ");
	LOGI("UTest$ get %d+%d block,complete err code %d",count,10-count,err);
	LOGI("UTest$ recieved %s",item[0]);
	
	//绕回不完全占用写入，理应成功，20数据
	LOGI("-------------------------------------------------");
	count = xLockFreeFifoSendAcquire(g_utestFifo,item,20);
	memcpy(item[0],"AAAABBBBCCCCDDDDEEE",20);
	LOGI(" ");
	err = xLockFreeFifoSendComplete(g_utestFifo,item);
	LOGI(" ");
	LOGI("UTest$ get %d+%d block,complete err code %d",count,20-count,err);
	//绕回完全占用读取，理应成功，0数据
	LOGI("-------------------------------------------------");
	count = xLockFreeFifoRecieveAcquire(g_utestFifo,item,20);
	LOGI(" ");
	err = xLockFreeFifoRecieveComplete(g_utestFifo,item);
	LOGI(" ");
	LOGI("UTest$ get %d+%d block,complete err code %d",count,20-count,err);
	LOGI("UTest$ recieved %s",item[0]);
	//绕回完全占用写入，理应成功，25数据
	LOGI("-------------------------------------------------");
	count = xLockFreeFifoSendAcquire(g_utestFifo,item,25);
	memcpy(item[0],"aaaaabbbbbcccccdddddeeee",25);
	LOGI(" ");
	err = xLockFreeFifoSendComplete(g_utestFifo,item);
	LOGI(" ");
	LOGI("UTest$ get %d+%d block,complete err code %d",count,25-count,err);
	//绕回完全占用读取，理应成功，0数据
	LOGI("-------------------------------------------------");
	count = xLockFreeFifoRecieveAcquire(g_utestFifo,item,25);
	LOGI(" ");
	err = xLockFreeFifoRecieveComplete(g_utestFifo,item);
	LOGI(" ");
	LOGI("UTest$ get %d+%d block,complete err code %d",count,25-count,err);
	LOGI("UTest$ recieved %s",item[0]);
}

/**
 *@breif 单元测试3：多人发多人收，无绕回
**/
void vLockFreeFifoUnitTest_MISO_NotCirlcle(void)
{
	static LockFreeFifo_t g_utestFifo;
	LOGI("UTest1$ block 1*40,producer 1,consumer 1");
	g_utestFifo = xLockFreeFifoCreate(1,20,3,3);
	void* item[2];
	void* pitem1[2];
	void* pitem2[2];
	void* pitem3[2];
	void* citem1[2];
	void* citem2[2];
	void* citem3[2];
	int count;
	int err;
	
	//不完全P+不完全D占用写入，理应成功，5数据
	LOGI("-------------------------------------------------");
	count = xLockFreeFifoSendAcquire(g_utestFifo,pitem1,5);
	memcpy(pitem1[0],"Hello",5);
	LOGI(" ");
	LOGI("UTest$ get %d+%d block",count,5-count);
	//不完全P+完全占用D写入，理应成功，15数据
	LOGI("-------------------------------------------------");
	count = xLockFreeFifoSendAcquire(g_utestFifo,pitem2,15);
	memcpy(pitem2[0],"!!!@@@###$$$%%%",15);
	LOGI(" ");
	LOGI("UTest$ get %d+%d block",count,15-count);
	//写复原
	LOGI("-------------------------------------------------");
	err = xLockFreeFifoSendComplete(g_utestFifo,pitem1);
	LOGI(" ");
	LOGI("UTest$ Complete,err code %d",err);
	LOGI(" ");
	err = xLockFreeFifoSendComplete(g_utestFifo,pitem2);
	LOGI(" ");
	LOGI("UTest$ Complete,err code %d",err);
	LOGI(" ");
	//不完全C+不完全D占用读取，理应成功，10数据
	LOGI("-------------------------------------------------");
	count = xLockFreeFifoRecieveAcquire(g_utestFifo,citem1,10);
	LOGI(" ");
	LOGI("UTest$ get %d+%d block",count,10-count);
	LOGI("UTest$ recieved %s",citem1[0]);
	//不完全C+完全D占用读取，理应成功，0数据
	LOGI("-------------------------------------------------");
	count = xLockFreeFifoRecieveAcquire(g_utestFifo,citem2,10);
	LOGI(" ");
	LOGI("UTest$ get %d+%d block",count,10-count);
	LOGI("UTest$ recieved %s",citem2[0]);
	//读复原
	LOGI("-------------------------------------------------");
	err = xLockFreeFifoRecieveComplete(g_utestFifo,citem1);
	LOGI(" ");
	LOGI("UTest$ Complete,err code %d",err);
	LOGI(" ");
	err = xLockFreeFifoRecieveComplete(g_utestFifo,citem2);
	LOGI(" ");
	LOGI("UTest$ Complete,err code %d",err);
	LOGI(" ");
}

volatile uint32_t my_data;
void vLockFreeFifoUnitTest_All(void)
{
	static uint8_t test[] = {0x0,0x1,0x2,0x3,0x4,0x5,0x6,0x7,0x8};
	uint32_t* ptr = &test[1];
	my_data = *ptr;
	ptr = &test[2];
	my_data = *ptr;
	ptr = &test[3];
	my_data = *ptr;
	ptr = &test[4];
	my_data = *ptr;
	ptr = &test[5];
	my_data = *ptr;
}

/**
 *@}
**/