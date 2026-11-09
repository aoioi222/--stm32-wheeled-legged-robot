/**
 * @file rcs_os.h
 * @brief RCS在RTOS之上实现的私有内核功能、以及一些常用的辅助函数
 * @author CYK-Dot
 * -------------------------------------------------------------------------------------------
 * @date 2025-05-27
 * @version 1.0.0
 * @changelog 
 * 1. 实现了线程安全的kprintf，通过keil::EventRecorder发送到调试窗口，无需额外占有硬件资源
 * 2. 对老代码中的非阻塞延时进行了面向对象封装
 * -------------------------------------------------------------------------------------------
 * @date 2025-6-1
 * @version 1.0.1
 * @changelog
 * 1. 增加了rcs_bit_set、rcs_bit_reset、rcs_bit_toggle、rcs_bit_read等位操作函数
 * -------------------------------------------------------------------------------------------
 */
#pragma once

#include <stdarg.h>
#include "rcs_private_config.h"
#include "rcs_types.h"



// 配置宏
#define RCS_OS_MALLOC   pvPortMalloc
#define RCS_OS_FREE     vPortFree
#define RCS_COUNTER_MAX 200

#ifdef __cplusplus
extern "C" {
#endif

// 内核打印函数
BaseType_t rcs_kprintf_init(void);
BaseType_t rcs_kprintf(const char *format,uint32_t timeout, ...);
BaseType_t rcs_kprintf_from_isr(const char *format,BaseType_t *pxHigherPriorityTaskWoken, ...);

// 无阻塞延时函数
BaseType_t rcs_count_delay(uint16_t counter_id,size_t count_period);
BaseType_t rcs_count_reset(uint16_t counter_id);

/**
 *@brief                  设置某个数值的某个二进制位为1
 *@param[IN][OUT]  pvData 需要设置的数值，类型任意
 *@param[IN]       ucPos  需要更改的二进制位的位置
 *
 *@note    请自行确保bit_pos不会超过数据的范围。比如对于uint8_t类型的数据，不应该填入7以上的数字。
 *@warning 请自行确保pvData内的数据有效。本函数为了提高性能，没有对pvData指针做assert检查。
**/
__STATIC_FORCEINLINE void rcs_bit_set(void* pvData,uint8_t ucPos)
{
	*(uint32_t*)pvData |= (1U << ucPos);
}

/**
 *@brief                  设置某个数值的某个二进制位为0
 *@param[IN][OUT]  pvData 需要设置的数值，类型任意
 *@param[IN]       ucPos  需要更改的二进制位的位置
 *
 *@note  请自行确保bit_pos不会超过数据的范围。比如对于uint8_t类型的数据，不应该填入7以上的数字。
 *@warning 请自行确保pvData内的数据有效。本函数为了提高性能，没有对pvData指针做assert检查。
**/
__STATIC_FORCEINLINE void rcs_bit_reset(void* pvData, uint8_t ucPos) 
{
    *(uint32_t*)pvData &= ~(1U << ucPos);
}

/**
 *@brief                   翻转某个数值的某个二进制位
 *@param[IN][OUT]  data    需要设置的数值，类型任意
 *@param[IN]       bit_pos 需要更改的二进制位的位置
 *
 *@note  请自行确保bit_pos不会超过数据的范围。比如对于uint8_t类型的数据，不应该填入7以上的数字。
 *@warning 请自行确保pvData内的数据有效。本函数为了提高性能，没有对pvData指针做assert检查。
**/
__STATIC_FORCEINLINE void rcs_bit_toggle(void* pvData, uint8_t ucPos) 
{
    *(uint32_t*)pvData ^= (1U << ucPos);
}

/**
 * @brief      读取某个数值的某个二进制位
 * @param[IN]  pvData    需要读取的变量的地址，类型任意
 * @param[IN]  ucPos     需要读取的二进制位的位置，单位为byte
 * @retval     uint8_t 0或1
 *
 * @note  请自行确保bit_pos不会超过数据的范围。比如对于uint8_t类型的数据，不应该填入>7的数字。
**/
__STATIC_FORCEINLINE uint8_t rcs_bit_read(const void* pvData, uint8_t ucPos) 
{
    return (*(const uint32_t*)pvData >> ucPos) & 1U;
}

/**
 * @brief      读取某个数值的某个字节
 * @param[IN]  pvData    需要读取的变量的地址，类型任意
 * @param[IN]  ucPos     需要读取的字节的位置，单位为byte(8bit)
 * @retval     uint8_t   获取到的字节
 *
 * @note  请自行确保ucPos不会超过数据的范围。比如对于uint16_t类型的数据，不应该填入>1的数字。
**/
__STATIC_FORCEINLINE uint8_t rcs_byte_read(const void* pvData, uint8_t ucPos)
{
	return *((const uint8_t*)pvData + ucPos);
}

/**
 * @brief      将某个长度为16字节的数据分割为两个uint8_t，并放入指定数组
 * @param[IN]  value   需要分割的数值
 * @param[OUT] bytes_2 目标数组，长度至少为2
 *
 * @note  请自行确保bit_pos不会超过或者小于数据的范围。比如不应该填入uint8_t类型的数据
**/
__STATIC_FORCEINLINE void rcs_byte_split_16(const void* pvDataSource, uint8_t* pucDataDestination) 
{
    uint16_t val = *(const uint16_t*)pvDataSource;
    pucDataDestination[0] = (val >> 8) & 0xFF;
    pucDataDestination[1] = val & 0xFF;
}

/**
 * @brief      将某个长度为32字节的数据分割为四个uint8_t，并放入指定数组
 * @param[IN]  value   需要分割的数值
 * @param[OUT] bytes_4 目标数组，长度至少为4
 *
 * @note  请自行确保bit_pos不会超过或者小于数据的范围。比如不应该填入uint16_t类型的数据
**/
__STATIC_FORCEINLINE void rcs_byte_split_32(const void* pvDataSource, uint8_t* pucDataDestination) 
{
    uint32_t val = *(const uint32_t*)pvDataSource;
    pucDataDestination[0] = (val >> 24) & 0xFF;
    pucDataDestination[1] = (val >> 16) & 0xFF;
    pucDataDestination[2] = (val >> 8) & 0xFF;
    pucDataDestination[3] = val & 0xFF;
}

/**
 * @brief      将某个长度为64字节的数据分割为八个uint8_t，并放入指定数组
 * @param[IN]  value   需要分割的数值
 * @param[OUT] bytes_8 目标数组，长度至少为8
 *
 * @note  请自行确保bit_pos不会超过数据的范围。比如可以填入double类型的数据，但不能填入float
**/
__STATIC_FORCEINLINE void rcs_byte_split_64(const void* pvDataSource, uint8_t* pucDataDestination) 
{
	uint64_t val = *(const uint64_t*)pvDataSource;
    pucDataDestination[0] = (val >> 56) & 0xFF;
    pucDataDestination[1] = (val >> 48) & 0xFF;
    pucDataDestination[2] = (val >> 40) & 0xFF;
    pucDataDestination[3] = (val >> 32) & 0xFF;
    pucDataDestination[4] = (val >> 24) & 0xFF;
    pucDataDestination[5] = (val >> 16) & 0xFF;
    pucDataDestination[6] = (val >> 8) & 0xFF;
    pucDataDestination[7] = val & 0xFF;
}

/**
 * @brief      将两个uint8_t拼接成一个16位的数据
 * @param[IN]  bytes_2 需要拼接的数组，长度至少2，arr[0]是最高位。
 * @param[OUT] value   目标数据，长度应为16，比如uint16_t、int16_t等。
 *
 * @note  请自行确保数组和指针所指位置的长度不会超过指定。
**/
__STATIC_FORCEINLINE void vRcsByteEncap16(uint8_t* pucDataSource,void* pvDataDestination)
{
	*(uint16_t*)pvDataDestination = ((uint16_t)pucDataSource[0] << 8) | pucDataSource[1];
}

/**
 * @brief      将四个uint8_t拼接成一个32位的数据
 * @param[IN]  bytes_4 需要拼接的数组，长度至少4，arr[0]是最高位。
 * @param[OUT] value   目标数据，长度应为32，比如uint32_t、float等。
 *
 * @note  请自行确保数组和指针所指位置的长度不会超过指定。
**/
__STATIC_FORCEINLINE void vRcsByteEncap32(uint8_t* pucDataSource,void* pvDataDestination)
{
	*(uint32_t*)pvDataDestination = ((uint32_t)pucDataSource[0] << 24) 
	                  | ((uint32_t)pucDataSource[1] << 16) 
	                  | ((uint32_t)pucDataSource[2] << 8) 
	                  |   pucDataSource[3];
}

/**
 * @brief      将八个uint8_t拼接成一个64位的数据
 * @param[IN]  bytes_8 需要拼接的数组，长度至少8，arr[0]是最高位。
 * @param[OUT] value   目标数据，长度应为64，比如int64_t、double等。
 *
 * @note  请自行确保数组和指针所指位置的长度不会超过指定。
**/
__STATIC_FORCEINLINE void vRcsByteEncap64(uint8_t* pucDataSource,void* pvDataDestination)
{
	*(uint64_t*)pvDataDestination = ((uint64_t)pucDataSource[0] << 56) 
	                  | ((uint64_t)pucDataSource[1] << 48) 
	                  | ((uint64_t)pucDataSource[2] << 40) 
	                  | ((uint64_t)pucDataSource[3] << 32) 
	                  | ((uint64_t)pucDataSource[4] << 24) 
	                  | ((uint64_t)pucDataSource[5] << 16) 
	                  | ((uint64_t)pucDataSource[6] << 8) 
	                  | pucDataSource[7];
}

#ifdef __cplusplus
}
#endif



