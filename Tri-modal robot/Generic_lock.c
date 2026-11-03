#include "generic_lock.h"

// ====================== 底层指令封装（Cortex-M通用） ======================
// 关全局可屏蔽中断
static inline void disable_irq(void)
{
    __asm volatile ("cpsid i");  // 直接内嵌汇编，无库依赖
}

// 开全局可屏蔽中断
static inline void enable_irq(void)
{
    __asm volatile ("cpsie i");
}


// ====================== 通用加锁函数实现 ======================
LockStatus_t generic_lock(GenericLock_t *lock)
{
    // 1. 空指针校验
    if (lock == NULL) {
        return LOCK_ERROR_NULL_PTR;
    }

    // 2. 关中断，保证原子操作
    disable_irq();

    // 3. 检查并设置锁标志
    if (lock->lock_flag == 0) {
        lock->lock_flag = 1;
        enable_irq();
        return LOCK_SUCCESS;
    }

    // 4. 锁已被占用
    enable_irq();
    return LOCK_ERROR_BUSY;
}

// ====================== 通用加锁（带超时） ======================
LockStatus_t generic_lock_timeout(GenericLock_t *lock, uint32_t timeout_us)
{
    if (lock == NULL) {
        return LOCK_ERROR_NULL_PTR;
    }

    uint32_t start_us = 0;
    uint32_t elapsed_us = 0;

    // 超时时间>0时，记录起始时间（通过循环计数模拟）
    if (timeout_us > 0) {
        start_us = 0; // 裸机无系统时钟，直接以循环次数计时
    }

    while (1) {
        // 尝试加锁
        if (generic_lock(lock) == LOCK_SUCCESS) {
            return LOCK_SUCCESS;
        }

        // 超时判断（timeout_us=0时无限等待）
        if (timeout_us > 0) {
            elapsed_us++;
            delay_us(1);  // 每次循环延时1us
            if (elapsed_us >= timeout_us) {
                return LOCK_ERROR_TIMEOUT;
            }
        } else {
            delay_us(1);  // 无限等待时，短延时降低CPU占用
        }
    }
}

// ====================== 通用解锁函数实现 ======================
LockStatus_t generic_unlock(GenericLock_t *lock)
{
    if (lock == NULL) {
        return LOCK_ERROR_NULL_PTR;
    }

    // 关中断，保证解锁+版本号更新原子执行
    disable_irq();
    lock->lock_flag = 0;
    lock->version++;  // 版本号+1，标记数据更新
    enable_irq();

    return LOCK_SUCCESS;
}