#ifndef __I2C_MPU9250_CONTROL_H
#define __I2C_MPU9250_CONTROL_H

#include "stm32f4xx.h"
#include "Hardware_I2C.h"
#include "rcs.h"

/*************************** 设备地址定义 ***************************/
#define MPU9250_DEVICE_ADDR        0x68U    // MPU9250 I2C地址（AD0接地）
#define AK8963_DEVICE_ADDR         0x0CU    // AK8963磁力计I2C地址

/*************************** 寄存器地址定义 ***************************/
// MPU9250核心寄存器
#define MPU9250_REG_WHO_AM_I       0x75U
#define MPU9250_REG_PWR_MGMT_1     0x6BU
#define MPU9250_REG_PWR_MGMT_2     0x6CU
#define MPU9250_REG_SMPLRT_DIV     0x19U
#define MPU9250_REG_CONFIG         0x1AU
#define MPU9250_REG_GYRO_CONFIG    0x1BU
#define MPU9250_REG_ACCEL_CONFIG   0x1CU
#define MPU9250_REG_INT_PIN_CFG    0x37U
#define MPU9250_REG_USER_CTRL      0x6AU
#define MPU9250_REG_INT_ENABLE     0x38U    // 中断使能寄存器

// MPU9250数据寄存器（Burst读取起始地址）
#define MPU9250_REG_BURST_START    0x3BU    // 加速度X高字节开始，共14字节
#define MPU9250_REG_ACCEL_XOUT_H   0x3BU
#define MPU9250_REG_ACCEL_YOUT_H   0x3DU
#define MPU9250_REG_ACCEL_ZOUT_H   0x3FU
#define MPU9250_REG_TEMP_OUT_H     0x41U
#define MPU9250_REG_GYRO_XOUT_H    0x43U
#define MPU9250_REG_GYRO_YOUT_H    0x45U
#define MPU9250_REG_GYRO_ZOUT_H    0x47U

// AK8963磁力计寄存器
#define AK8963_REG_WHO_AM_I        0x00U
#define AK8963_REG_CNTL1           0x0AU
#define AK8963_REG_CNTL2           0x0BU
#define AK8963_REG_ST1             0x02U
#define AK8963_REG_ST2             0x09U
#define AK8963_REG_HXL             0x03U
#define AK8963_REG_ASAX            0x10U
#define AK8963_REG_ASAY            0x11U
#define AK8963_REG_ASAZ            0x12U
#define AK8963_REG_BURST_START     0x02U    // ST1+6字节数据，共7字节

/*************************** 寄存器配置值 ***************************/
// MPU9250配置值
#define MPU9250_WHOAMI_VAL         0x71U
#define MPU9250_PWR_MGMT_1_VAL     0x01U    // 唤醒，X轴陀螺仪为时钟源
#define MPU9250_PWR_MGMT_2_VAL     0x00U    // 开启加速度计+陀螺仪所有轴
#define MPU9250_SMPLRT_DIV_VAL     0x07U    // 采样率分频值
#define MPU9250_CONFIG_VAL         0x06U    // 低通滤波配置
#define MPU9250_INT_PIN_CFG_VAL    0x02U    // 开启I2C旁路模式
#define MPU9250_USER_CTRL_VAL      0x00U    // 禁用I2C主模式
#define MPU9250_READ_FAIL_FLAG     0x7FFFU  // 读取失败标记（无合法数据冲突）

// AK8963配置值
#define AK8963_WHOAMI_VAL          0x48U
#define AK8963_CNTL2_RESET_VAL     0x01U    // 复位磁力计
#define AK8963_CNTL1_FUSE_MODE     0x0FU    // FUSE ROM模式（读校准系数）
#define AK8963_CNTL1_MEAS_MODE     0x16U    // 连续测量模式2（100Hz，16位）
#define MPU9250_AK8963_FUSE_EXIT_VAL 0x00U // 退出FUSE模式
#define AK8963_ST1_DATA_READY_BIT  0x01U    // 数据就绪位
#define AK8963_ST2_DATA_OVERFLOW_BIT 0x08U // 数据溢出位

/*************************** 校准相关定义 ***************************/
#define MPU9250_CALIB_BASE_VAL     128U
#define MPU9250_CALIB_DIV_VAL      256.0f
#define MPU9250_CALIB_SAMPLES      200U     // 校准采样次数（200次≈200ms）
#define MPU9250_CALIB_DELAY_MS     10U      // 采样间隔

/*************************** 延时相关定义 ***************************/
#define MPU9250_DELAY_5MS          5U
#define MPU9250_DELAY_10MS         10U

/*************************** 量程定义 ***************************/
// 加速度计量程枚举
typedef enum
{
    MPU9250_ACC_RANGE_2G    = 0U,
    MPU9250_ACC_RANGE_4G    = 1U,
    MPU9250_ACC_RANGE_8G    = 2U,
    MPU9250_ACC_RANGE_16G   = 3U
} MPU9250_AccRange_t;

// 陀螺仪量程枚举
typedef enum
{
    MPU9250_GYRO_RANGE_250DPS  = 0U,
    MPU9250_GYRO_RANGE_500DPS  = 1U,
    MPU9250_GYRO_RANGE_1000DPS = 2U,
    MPU9250_GYRO_RANGE_2000DPS = 3U
} MPU9250_GyroRange_t;

/*************************** 结构体定义 ***************************/


// MPU9250初始化配置结构体
typedef struct
{
    RCS_I2C_Type I2C_TYPE;                      // I2C外设类型（I2C1_t/I2C2_t/I2C3_t）
    RCS_I2C_Hardware_TypeDef* Hi2c;             // I2C硬件配置指针
    uint8_t MPU9250_Addr;                       // MPU9250设备地址
    uint8_t AK8963_Addr;                        // AK8963设备地址
    MPU9250_AccRange_t AccRange;                // 加速度计量程
    MPU9250_GyroRange_t GyroRange;              // 陀螺仪量程
} MPU9250_InitConfigTypeDef;

// MPU9250原始数据结构体
typedef struct 
{
    int16_t acc[3];  // acc[0]=X轴 | acc[1]=Y轴 | acc[2]=Z轴
    int16_t gyro[3]; // gyro[0]=X轴 | gyro[1]=Y轴 | gyro[2]=Z轴
    int16_t mag[3];  // mag[0]=X轴 | mag[1]=Y轴 | mag[2]=Z轴
    int16_t temp;    // 温度原始数据
} MPU9250_Raw_t;

// MPU9250零偏校准结构体
typedef struct 
{
    float acc_offset[3];  // 加速度计零偏（g）
    float gyro_offset[3]; // 陀螺仪零偏（°/s）
} MPU9250_Offset_t;

/*************************** 全局变量声明 ***************************/
extern MPU9250_Raw_t mpu9250_raw;            // 9轴原始数据
extern MPU9250_Offset_t mpu9250_offset;      // 零偏校准数据
extern float MagCalibX, MagCalibY, MagCalibZ; // 磁力计校准系数
extern uint8_t AK8963_Enabled;                // 磁力计使能状态
// 全局量程变量（支持运行时修改）
extern MPU9250_AccRange_t mpu9250_acc_range;
extern MPU9250_GyroRange_t mpu9250_gyro_range;

/*************************** 函数声明 ***************************/
// 核心初始化函数
uint8_t MPU9250_Init(MPU9250_InitConfigTypeDef* MPU9250_Config, uint8_t AK_Use_or_not);

// ID读取函数
uint8_t MPU9250_GetID(I2C_TypeDef* I2Cx, uint8_t DevAddr);
uint8_t AK8963_GetID(I2C_TypeDef* I2Cx, uint8_t DevAddr);

// 原始数据读取函数（普通版）
uint8_t MPU9250_GetAccRawData(I2C_TypeDef* I2Cx, uint8_t DevAddr);
uint8_t MPU9250_GetGyroRawData(I2C_TypeDef* I2Cx, uint8_t DevAddr);
uint8_t MPU9250_GetMagRawData(I2C_TypeDef* I2Cx, uint8_t DevAddr);

// 原始数据读取函数（Burst版，高效）
uint8_t MPU9250_GetAccGyroRawBurst(I2C_TypeDef *I2Cx, uint8_t DevAddr);
uint8_t AK8963_GetMagRawBurst(I2C_TypeDef *I2Cx, uint8_t DevAddr);

// 校准函数
uint8_t MPU9250_Calibrate(I2C_TypeDef* I2Cx, uint8_t DevAddr, uint16_t samples);

// 量程设置函数
uint8_t MPU9250_SetAccRange(I2C_TypeDef* I2Cx, uint8_t DevAddr, MPU9250_AccRange_t range);
uint8_t MPU9250_SetGyroRange(I2C_TypeDef* I2Cx, uint8_t DevAddr, MPU9250_GyroRange_t range);

// 数据转换函数
float MPU9250_ConvGyro(int16_t raw_gyro);
float MPU9250_ConvAcc(int16_t raw_acc);
float MPU9250_ConvMag(int16_t raw_mag);

// 转换后数据获取函数
float MPU9250_GetGyroXData(void);
float MPU9250_GetGyroYData(void);
float MPU9250_GetGyroZData(void);
float MPU9250_GetAccXData(void);
float MPU9250_GetAccYData(void);
float MPU9250_GetAccZData(void);
float MPU9250_GetMagXData(void);
float MPU9250_GetMagYData(void);
float MPU9250_GetMagZData(void);

// 内部恢复函数（外部无需调用）
static void AK8963_SoftReset(I2C_TypeDef* I2Cx, uint8_t DevAddr);

#endif /* __I2C_MPU9250_CONTROL_H */