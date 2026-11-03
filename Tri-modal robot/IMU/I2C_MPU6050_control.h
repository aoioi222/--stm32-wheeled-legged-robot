#ifndef __MPU6050_CONTROL_H
#define __MPU6050_CONTROL_H

#include "stdint.h"
#include "stm32f4xx.h"          // 根据MCU型号调整（如stm32f1xx.h）
#include "Hardware_I2C.h"       // I2C底层驱动头文件
#include "rcs.h"

/*************************** 宏定义 ***************************/
// MPU6050设备地址（AD0接地=0x68，接VCC=0x69）
#define MPU6050_DEVICE_ADDR        0x68U    // 8位写地址为0xD0
// WHO_AM_I寄存器默认值
#define MPU6050_WHOAMI_VAL         0x68U
// 读取失败标记
#define MPU6050_READ_FAIL_FLAG     0x7FFFU

// 寄存器地址定义
#define MPU6050_REG_WHO_AM_I       0x75U    // 设备ID寄存器
#define MPU6050_REG_PWR_MGMT_1     0x6BU    // 电源管理1寄存器
#define MPU6050_REG_PWR_MGMT_2     0x6CU    // 电源管理2寄存器
#define MPU6050_REG_SMPLRT_DIV     0x19U    // 采样率分频寄存器
#define MPU6050_REG_CONFIG         0x1AU    // 低通滤波配置寄存器
#define MPU6050_REG_GYRO_CONFIG    0x1BU    // 陀螺仪配置寄存器
#define MPU6050_REG_ACCEL_CONFIG   0x1CU    // 加速度计配置寄存器

// 寄存器默认配置值
#define MPU6050_PWR_MGMT_1_VAL     0x01U    // 退出休眠，X轴陀螺仪为时钟源
#define MPU6050_PWR_MGMT_2_VAL     0x00U    // 开启加速度计+陀螺仪所有轴
#define MPU6050_SMPLRT_DIV_VAL     0x07U    // 采样率125Hz（1kHz/(1+7)）
#define MPU6050_CONFIG_VAL         0x06U    // 低通滤波截止频率5Hz
#define MPU6050_GYRO_CONFIG_VAL    0x10U    // 陀螺仪量程±1000°/s
#define MPU6050_ACCEL_CONFIG_VAL   0x10U    // 加速度计量程±8g

/*
量程配置说明：
- 加速度计（ACCEL_CONFIG bit3-bit4）：0x00=±2g     | 0x08=±4g     | 0x10=±8g      | 0x18=±16g
- 陀螺仪（GYRO_CONFIG bit3-bit4）   ：0x00=±250°/s | 0x08=±500°/s | 0x10=±1000°/s | 0x18=±2000°/s
*/

// 数据寄存器地址
#define MPU6050_REG_ACCEL_XOUT_H   0x3BU    // 加速度X轴高字节
#define MPU6050_REG_ACCEL_YOUT_H   0x3DU    // 加速度Y轴高字节
#define MPU6050_REG_ACCEL_ZOUT_H   0x3FU    // 加速度Z轴高字节

#define MPU6050_REG_GYRO_XOUT_H    0x43U    // 陀螺仪X轴高字节
#define MPU6050_REG_GYRO_YOUT_H    0x45U    // 陀螺仪Y轴高字节
#define MPU6050_REG_GYRO_ZOUT_H    0x47U    // 陀螺仪Z轴高字节

/*************************** 结构体定义 ***************************/
// MPU6050初始化配置结构体
typedef struct
{
    RCS_I2C_Type I2C_TYPE;              // I2C外设类型（I2C1/I2C2/I2C3）
    RCS_I2C_Hardware_TypeDef* Hi2c;     // I2C硬件配置句柄
    uint8_t MPU6050_Addr;               // MPU6050设备地址
} MPU6050_InitConfigTypeDef;

typedef struct 
{
    int16_t acc[3];  // acc [0]=X轴 | acc [1]=Y轴 | acc [2]=Z轴
    int16_t gyro[3]; // gyro[0]=X轴 | gyro[1]=Y轴 | gyro[2]=Z轴
} MPU6050_Raw_t;

/*************************** 全局变量声明 ***************************/
// 外部可访问的原始数据结构体
extern MPU6050_Raw_t mpu6050_raw;

/*************************** 函数原型声明 ***************************/
// 获取设备ID函数
uint8_t MPU6050_GetID(I2C_TypeDef* I2Cx, uint8_t DevAddr);
// 初始化函数
uint8_t MPU6050_Init(MPU6050_InitConfigTypeDef* MPU6050_Config);
// 读取原始数据函数
uint8_t MPU6050_GetAccRawData(I2C_TypeDef* I2Cx, uint8_t DevAddr);
uint8_t MPU6050_GetGyroRawData(I2C_TypeDef* I2Cx, uint8_t DevAddr);
// 数据转换函数
float MPU6050_ConvGyro(int16_t raw_gyro);
float MPU6050_ConvAcc(int16_t raw_acc);
//获取数据的函数
float MPU6050_GetGyroXData(void);
float MPU6050_GetGyroYData(void);
float MPU6050_GetGyroZData(void);
float MPU6050_GetAccXData(void);
float MPU6050_GetAccYData(void);
float MPU6050_GetAccZData(void);

#endif /* __MPU6050_CONTROL_H */