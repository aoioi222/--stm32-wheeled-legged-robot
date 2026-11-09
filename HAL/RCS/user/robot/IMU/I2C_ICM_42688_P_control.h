#ifndef I2C_ICM_42688_P_CONTROL_H_
#define I2C_ICM_42688_P_CONTROL_H_

#include <stdint.h>
#include <math.h>
#include "robot_math.h"
#include "stm32f4xx_hal.h"
#include "ICM_42688_P_RegAddress.h"

/*************************** 寄存器配置值 ***************************/
#define ICM42688P_WHOAMI_VAL         0x47U    // WHO_AM_I寄存器默认值（设备ID）
#define ICM42688P_READ_FAIL_FLAG     0x7FFFU  // 读取失败标记

/*************************** 量程宏定义 ***************************/
#define ICM42688P_DEFAULT_ACC_RANGE  ICM42688P_ACC_RANGE_8G
#define ICM42688P_DEFAULT_GYRO_RANGE ICM42688P_GYRO_RANGE_1000DPS

// 传感器参数配置（量程+输出速率ODR）
#define ICM42688P_ACC_CONFIG   ICM42688P_ACCEL_ODR_8000_LN | ICM42688P_ACC_RANGE_8G
#define ICM42688P_GYRO_CONFIG  ICM42688P_GYRO_ODR_8000    | ICM42688P_GYRO_RANGE_1000DPS

// 加速度计量程配置(ACCEL_CONFIG0 Bit[11:10])
#define ICM42688P_ACC_RANGE_2G     (0x03U << 5)
#define ICM42688P_ACC_RANGE_4G     (0x02U << 5)
#define ICM42688P_ACC_RANGE_8G     (0x01U << 5)
#define ICM42688P_ACC_RANGE_16G    (0x00U << 5)

// 陀螺仪量程配置(GYRO_CONFIG0 Bit[7:5])
#define ICM42688P_GYRO_RANGE_15_625DPS  (0x07U << 5)
#define ICM42688P_GYRO_RANGE_31_25DPS   (0x06U << 5)
#define ICM42688P_GYRO_RANGE_62_5DPS    (0x05U << 5)
#define ICM42688P_GYRO_RANGE_125DPS     (0x04U << 5)
#define ICM42688P_GYRO_RANGE_250DPS     (0x03U << 5)
#define ICM42688P_GYRO_RANGE_500DPS     (0x02U << 5)
#define ICM42688P_GYRO_RANGE_1000DPS    (0x01U << 5)
#define ICM42688P_GYRO_RANGE_2000DPS    (0x00U << 5)

// 加速度计ODR宏定义（单位：Hz）
#define ICM42688P_ACCEL_ODR_32000_LN    0x01
#define ICM42688P_ACCEL_ODR_16000_LN    0x02
#define ICM42688P_ACCEL_ODR_8000_LN     0x03
#define ICM42688P_ACCEL_ODR_4000_LN     0x04
#define ICM42688P_ACCEL_ODR_2000_LN     0x05
#define ICM42688P_ACCEL_ODR_1000_LN     0x06
#define ICM42688P_ACCEL_ODR_200_LP_LN   0x07
#define ICM42688P_ACCEL_ODR_100_LP_LN   0x08
#define ICM42688P_ACCEL_ODR_50_LP_LN    0x09
#define ICM42688P_ACCEL_ODR_25_LP_LN    0x0A
#define ICM42688P_ACCEL_ODR_12_5_LP_LN  0x0B
#define ICM42688P_ACCEL_ODR_6_25_LP     0x0C
#define ICM42688P_ACCEL_ODR_3_125_LP    0x0D
#define ICM42688P_ACCEL_ODR_1_5625_LP   0x0E
#define ICM42688P_ACCEL_ODR_500_LP_LN   0x0F

// 陀螺仪ODR宏定义（单位：Hz）
#define ICM42688P_GYRO_ODR_32000        0x01
#define ICM42688P_GYRO_ODR_16000        0x02
#define ICM42688P_GYRO_ODR_8000         0x03
#define ICM42688P_GYRO_ODR_4000         0x04
#define ICM42688P_GYRO_ODR_2000         0x05
#define ICM42688P_GYRO_ODR_1000         0x06
#define ICM42688P_GYRO_ODR_200          0x07
#define ICM42688P_GYRO_ODR_100          0x08
#define ICM42688P_GYRO_ODR_50           0x09
#define ICM42688P_GYRO_ODR_25           0x0A
#define ICM42688P_GYRO_ODR_12_5         0x0B
#define ICM42688P_GYRO_ODR_500          0x0F

// 加速度计灵敏度 LSB/g
#define ICM42688P_ACC_SCALE_2G       16384.0f
#define ICM42688P_ACC_SCALE_4G       8192.0f
#define ICM42688P_ACC_SCALE_8G       4096.0f
#define ICM42688P_ACC_SCALE_16G      2048.0f

// 陀螺仪灵敏度 LSB/(°/s)
#define ICM42688P_GYRO_SCALE_15_625DPS   2097.2f
#define ICM42688P_GYRO_SCALE_31_25DPS    1048.6f
#define ICM42688P_GYRO_SCALE_62_5DPS     524.3f
#define ICM42688P_GYRO_SCALE_125DPS      262.0f
#define ICM42688P_GYRO_SCALE_250DPS      131.0f
#define ICM42688P_GYRO_SCALE_500DPS      65.5f
#define ICM42688P_GYRO_SCALE_1000DPS     32.8f
#define ICM42688P_GYRO_SCALE_2000DPS     16.4f

// 校准参数
#define ICM42688P_CALIB_SAMPLES     200U
#define ICM42688P_CALIB_DELAY_MS    1U

// 欧拉角互补滤波参数
#define ICM42688P_SAMPLING_FREQUENCY        1000.0f
#define ICM42688P_COMPLEMENTARY_FILTER_ALPHA 0.98f

/*************************** 结构体定义 ***************************/
/**
 * @brief ICM42688P初始化配置结构体
 */
typedef struct
{
    I2C_HandleTypeDef       *Hi2c;
    uint8_t                  ICM42688P_Addr;
} ICM42688P_InitConfigTypeDef;

/**
 * @brief 传感器原始ADC数据
 */
typedef struct
{
    int16_t acc[3];    // 加速度原始值 X/Y/Z
    int16_t gyro[3];   // 陀螺仪原始值 X/Y/Z
    int16_t temp;      // 片内温度原始值
} ICM42688P_Raw_t;

/**
 * @brief IMU零偏补偿参数
 */
typedef struct
{
    float acc_offset[3];
    float gyro_offset[3];
} ICM42688P_Offset_t;

/**
 * @brief 转换后物理量数据
 */
typedef struct
{
    float acc[3];      // 加速度 g
    float gyro[3];     // 角速度 °/s
    float temp;        // 温度 ℃
} ICM42688P_Data_t;

/**
 * @brief 欧拉角（滚转、俯仰、偏航，单位弧度）
 * @note 互补滤波解算；Yaw无法通过加速度计修正，存在漂移
 */
typedef struct
{
    float yaw;
    float pitch;
    float roll;
} ICM42688P_Euler_angles;

/*************************** 全局变量声明 ***************************/
extern ICM42688P_Raw_t        icm42688p_raw;
extern ICM42688P_Offset_t     icm42688p_offset;
extern ICM42688P_Euler_angles icm42688p_euler_angles;
extern uint8_t                icm42688p_acc_range;
extern uint8_t                icm42688p_gyro_range;
extern ICM42688P_Data_t       icm42688p_data;

/*************************** 函数声明 ***************************/
// 传感器初始化
uint8_t ICM42688P_Init(ICM42688P_InitConfigTypeDef *ICM42688P_Config, uint16_t samples);

// 设备ID读取
uint8_t ICM42688P_GetID(I2C_HandleTypeDef *p_i2c, uint8_t DevAddr);

// IMU静态零偏校准
uint8_t ICM42688P_Calibrate(I2C_HandleTypeDef *p_i2c, uint8_t DevAddr, uint16_t samples);

// 原始数据读取
uint8_t ICM42688P_GetAccRawData(I2C_HandleTypeDef *p_i2c, uint8_t DevAddr);
uint8_t ICM42688P_GetGyroRawData(I2C_HandleTypeDef *p_i2c, uint8_t DevAddr);
uint8_t ICM42688P_GetAccGyroRawBurst(I2C_HandleTypeDef *p_i2c, uint8_t DevAddr);
uint8_t ICM42688P_GetFifoData(I2C_HandleTypeDef *p_i2c, uint8_t DevAddr);

// 无参全局刷新接口（依赖Init保存I2C上下文）
uint8_t ICM42688P_UpdateData(void);

// 温度原始值转换
float ICM42688P_ConvTemp(int16_t raw_temp);
float ICM42688P_ConvFifoTemp(int16_t raw_temp);

// 获取补偿后的物理量
float ICM42688P_GetGyroXData(void);
float ICM42688P_GetGyroYData(void);
float ICM42688P_GetGyroZData(void);
float ICM42688P_GetAccXData(void);
float ICM42688P_GetAccYData(void);
float ICM42688P_GetAccZData(void);
float ICM42688P_GetTempData(void);

// 参数指针获取
uint8_t ICM42688P_GetAccRange(void);
uint8_t ICM42688P_GetGyroRange(void);
ICM42688P_Data_t *ICM42688P_GetData(void);
ICM42688P_Raw_t *ICM42688P_GetRawData(void);
ICM42688P_Offset_t *ICM42688P_GetOffset(void);

// 欧拉角互补滤波解算
ICM42688P_Euler_angles *ICM42688P_Calculate_Euler_Angles(void);
ICM42688P_Euler_angles *ICM42688P_Get_Euler_Angles(void);

#endif /* I2C_ICM_42688_P_CONTROL_H_ */
