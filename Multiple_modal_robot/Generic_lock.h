#ifndef GENERIC_LOCK_H
#define GENERIC_LOCK_H

/* 避免C++编译冲突 */
#ifdef __cplusplus
extern "C" {
#endif

/* ====================== 头文件依赖（仅标准库，无HAL/RTOS） ====================== */
#include <stdint.h>   // 标准整数类型
#include <stddef.h>   // NULL、size_t等
#include <stdbool.h>  // bool类型（可选，根据你的编译器是否支持）
#include "rcs.h"
/* ====================== 通用锁状态枚举（返回值） ====================== */
/**
 * @brief 通用锁操作状态码
 */
typedef enum {
    LOCK_SUCCESS      = 0,    /* 加锁/解锁成功 */
    LOCK_ERROR_NULL_PTR = 1, /* 传入NULL指针错误 */
    LOCK_ERROR_BUSY     = 2, /* 锁被占用（忙），加锁失败 */
    LOCK_ERROR_TIMEOUT  = 3  /* 加锁超时 */
} LockStatus_t;

/* ====================== 通用锁核心结构体（适配任意共享资源） ====================== */
/**
 * @brief 通用锁结构体（每个共享资源对应一个实例）
 * @note  volatile关键字防止编译器优化，保证中断中读写的实时性
 */
typedef struct {
    volatile uint8_t  lock_flag; /* 锁标志：0=未锁定，1=已锁定 */
    volatile uint16_t version;   /* 版本号：用于校验数据完整性（防脏读） */
} GenericLock_t;

/* ====================== 核心函数声明（无HAL依赖，纯裸机） ====================== */
/**
 * @brief  通用加锁函数（关中断，原子操作）
 * @param  lock: 锁实例指针（不能为空）
 * @retval LockStatus_t - 操作状态码
 * @note   加锁失败时立即返回，不等待
 */
LockStatus_t generic_lock(GenericLock_t *lock);

/**
 * @brief  通用加锁函数（带超时，无HAL依赖）
 * @param  lock: 锁实例指针
 * @param  timeout_us: 超时时间（微秒，0=无限等待）
 * @retval LockStatus_t - 操作状态码
 * @note   超时时间根据CPU主频适配，默认适配168MHz，需自行修改delay_us内的主频
 */
LockStatus_t generic_lock_timeout(GenericLock_t *lock, uint32_t timeout_us);

/**
 * @brief  通用解锁函数（关中断，版本号自增）
 * @param  lock: 锁实例指针（不能为空）
 * @retval LockStatus_t - 操作状态码
 * @note   解锁时自动将版本号+1，标记数据已更新
 */
LockStatus_t generic_unlock(GenericLock_t *lock);

/**
 * @brief  通用数据完整性校验（版本号对比）
 * @param  lock: 锁实例指针
 * @param  ver_before: 读取数据前记录的版本号
 * @retval bool - true=数据完整（无脏读），false=数据被修改（脏读）
 * @note   内联函数，无函数调用开销
 */
static inline bool generic_check_version(GenericLock_t *lock, uint16_t ver_before)
{
    if (lock == NULL) {
        return false;
    }
    return (lock->version == ver_before) ? true : false;
}

/* ====================== 可选：快捷宏定义（简化调用） ====================== */
/**
 * @brief  快捷加锁宏（无限等待）
 * @param  LOCK_INST: 锁实例变量名
 * @retval 无（加锁失败会一直等）
 */
#define GENERIC_LOCK_BLOCK(LOCK_INST)            \
    do {                                         \
        while (generic_lock(&LOCK_INST) != LOCK_SUCCESS) { \
            __asm volatile ("nop");              \
        }                                        \
    } while (0)

/**
 * @brief  快捷解锁宏
 * @param  LOCK_INST: 锁实例变量名
 * @retval 无
 */
#define GENERIC_UNLOCK(LOCK_INST)                \
    do {                                         \
        generic_unlock(&LOCK_INST);              \
    } while (0)

/* C++编译兼容 */
#ifdef __cplusplus
}
#endif

#endif /* GENERIC_LOCK_H */