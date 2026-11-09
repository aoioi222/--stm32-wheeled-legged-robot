/**
  ******************************************************************************
  * @file    wit_c_sdk.h
  * @author  Wit
  * @version V1.0
  * @date    05-May-2022
  * @brief   This file provides all Configure sensor function.
  ******************************************************************************
  * @attention
  *
  *        http://wit-motion.cn/
  *
  ******************************************************************************
  *
  * @author CYK-Dot
  * @version V1.1.0
  * @date    2025-05-23
  * @brief
  * 1. 将维特智能提供的SDK进行面向对象化，允许同时存在和使用多个IMU实例
  * 2. 保留维特智能提供的系统调用接口，并在rcs_wit_gyro.cpp中部署到RCSLIB中
  * 3. 重新组织维特智能提供的.h文件，符合RCS阅读习惯
  * 
  ******************************************************************************
  */

#ifndef __WIT_C_SDK_H
#define __WIT_C_SDK_H
#ifdef __cplusplus
extern "C" {
#endif
/* 头文件引入 -------------------------------------------------------------------------- */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "REG.h"
#include "rcs_types.h"

/* 导出宏定义 -------------------------------------------------------------------------- */

#define WIT_HAL_OK      (0)     /**< There is no error */
#define WIT_HAL_BUSY    (-1)    /**< Busy */
#define WIT_HAL_TIMEOUT (-2)    /**< Timed out */
#define WIT_HAL_ERROR   (-3)    /**< A generic error happens */
#define WIT_HAL_NOMEM   (-4)    /**< No memory */
#define WIT_HAL_EMPTY   (-5)    /**< The resource is empty */
#define WIT_HAL_INVAL   (-6)    /**< Invalid argument */

#define WIT_DATA_BUFF_SIZE  256

#define WIT_PROTOCOL_NORMAL        0
#define WIT_PROTOCOL_MODBUS        1
#define WIT_PROTOCOL_CAN           2
#define WIT_PROTOCOL_I2C           3
#define WIT_PROTOCOL_JY61          4
#define WIT_PROTOCOL_905x_MODBUS   5
#define WIT_PROTOCOL_905x_CAN      6

/* 导出类型定义 ------------------------------------------------------------------------ */

/**
 * @brief 维特智能IMU类
 */
typedef void *wit_gyro_t;

/**
 * @brief 串口写入函数指针
 * @param handle 句柄
 * @param p_ucData 数据
 * @param uiLen 长度
 */
typedef void (*SerialWrite)(wit_gyro_t handle, uint8_t *p_ucData, uint32_t uiLen);

/**
 * @brief I2C读写函数指针
 * @param handle 句柄
 * @param ucAddr 地址
 * @param ucReg 寄存器
 * @param p_ucVal 数据
 * @param uiLen 长度
 */
typedef int32_t (*WitI2cWrite)(wit_gyro_t handle, uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen);
typedef int32_t (*WitI2cRead)(wit_gyro_t handle, uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen);

/**
 * @brief CAN写入函数指针
 * @param handle 句柄
 * @param ucStdId 标准ID
 * @param p_ucData 数据
 * @param uiLen 长度
 */
typedef void (*CanWrite)(wit_gyro_t handle, uint8_t ucStdId, uint8_t *p_ucData, uint32_t uiLen);

/**
 * @brief 延时函数指针
 * @param ucMs 毫秒
 */
typedef void (*DelaymsCb)(uint16_t ucMs);

/**
 * @brief 寄存器更新回调函数指针
 * @param handle 句柄
 * @param uiReg 寄存器
 * @param uiRegNum 寄存器数量
 */
typedef void (*RegUpdateCb)(wit_gyro_t handle,uint32_t uiReg, uint32_t uiRegNum);

/* 导出结构体 -------------------------------------------------------------------------- */

/**
 * @brief 维特智能IMU对象
 */
typedef struct
{
    // 硬件接口
    SerialWrite p_WitSerialWriteFunc;
    WitI2cWrite p_WitI2cWriteFunc;
    WitI2cRead  p_WitI2cReadFunc;
    CanWrite    p_WitCanWriteFunc;
    RegUpdateCb p_WitRegUpdateCbFunc;
    DelaymsCb   p_WitDelaymsFunc;
    
    // 数据缓存
    uint8_t ucAddr;
    uint8_t ucWitDataBuff[WIT_DATA_BUFF_SIZE];
    uint32_t uiWitDataCnt;

    // 协议
    uint32_t uiProtoclo;

    // 寄存器
    uint32_t uiReadRegIndex;
    int16_t sReg[REGSIZE];

    // 继承者私有数据
    void *pvChildData;
}wit_gyro_internal_t;

/* 导出函数 ---------------------------------------------------------------------------- */

// 构造方法与析构方法
int32_t WitInit(wit_gyro_t handle,uint32_t uiProtocol, uint8_t ucAddr, void *pvChildData);
int32_t WitSerialWriteRegister(wit_gyro_t handle,SerialWrite write_func);
int32_t WitI2cFuncRegister(wit_gyro_t handle,WitI2cWrite write_func, WitI2cRead read_func);
int32_t WitCanWriteRegister(wit_gyro_t handle,CanWrite write_func);
int32_t WitDelayMsRegister(wit_gyro_t handle,DelaymsCb delayms_func);
int32_t WitRegisterCallBack(wit_gyro_t handle,RegUpdateCb update_func);
void WitDeInit(wit_gyro_t handle);

// 成员函数--寄存器操作(可以读取数据)
int32_t WitWriteReg(wit_gyro_t handle,uint32_t uiReg, uint16_t usData);
int32_t WitReadReg(wit_gyro_t handle,uint32_t uiReg, uint32_t uiReadNum);

// 成员函数--适配器
void WitSerialDataIn(wit_gyro_t handle,uint8_t ucData);
void WitCanDataIn(wit_gyro_t handle,uint8_t ucData[8], uint8_t ucLen);

// 成员函数--校准
int32_t WitStartAccCali(wit_gyro_t handle);
int32_t WitStopAccCali(wit_gyro_t handle);
int32_t WitStartMagCali(wit_gyro_t handle);
int32_t WitStopMagCali(wit_gyro_t handle);

// 成员函数--配置
int32_t WitSetUartBaud(wit_gyro_t handle,int32_t uiBaudIndex);
int32_t WitSetBandwidth(wit_gyro_t handle,int32_t uiBaudWidth);
int32_t WitSetOutputRate(wit_gyro_t handle,int32_t uiRate);
int32_t WitSetContent(wit_gyro_t handle,int32_t uiRsw);
int32_t WitSetCanBaud(wit_gyro_t handle,int32_t uiBaudIndex);
int32_t WitSaveParameter(wit_gyro_t handle);
int32_t WitSetForReset(wit_gyro_t handle);
int32_t WitCaliRefAngle(wit_gyro_t handle);

// 辅助函数
char CheckRange(short sTemp,short sMin,short sMax);

#ifdef __cplusplus
}
#endif
#endif /* __WIT_C_SDK_H */
