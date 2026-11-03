#include "I2C_MPU6050_CONTROL.h"

/*************************** 全局变量定义 ***************************/
// 定义原始数据结构体
MPU6050_Raw_t mpu6050_raw = {0}; 

/*************************** 内部函数声明 ***************************/
static int32_t MPU6050_ReadRawData(I2C_TypeDef* I2Cx, uint8_t DevAddr, uint8_t RegAddr);

/*************************** 函数实现 ***************************/
uint8_t MPU6050_GetID(I2C_TypeDef* I2Cx, uint8_t DevAddr)
{
    return I2C_ReadReg(I2Cx, DevAddr, MPU6050_REG_WHO_AM_I);
}

uint8_t MPU6050_Init(MPU6050_InitConfigTypeDef* MPU6050_Config)
{
    uint8_t ret = 0U;
    uint8_t mpu_id = 0xFFU;
    
    // 参数校验：空指针防护
    if(MPU6050_Config == NULL || MPU6050_Config->Hi2c == NULL)
    {
        return 1U;
    }
    
    // 初始化I2C硬件
    RCS_I2C_Hardware_Init(MPU6050_Config->I2C_TYPE, MPU6050_Config->Hi2c);
    
    // 检查通信是否正常
    mpu_id = MPU6050_GetID(MPU6050_Config->Hi2c->I2Cx, MPU6050_Config->MPU6050_Addr);
    if(mpu_id != MPU6050_WHOAMI_VAL)
    {
        return 1U; // 通信失败
    }
    
    // 配置核心寄存器
    ret = I2C_WriteReg(MPU6050_Config->Hi2c->I2Cx, MPU6050_Config->MPU6050_Addr,
                       MPU6050_REG_PWR_MGMT_1, MPU6050_PWR_MGMT_1_VAL);
    if(ret != 0U) return 1U;
    delay_ms(10); // 等待配置生效
    
    ret = I2C_WriteReg(MPU6050_Config->Hi2c->I2Cx, MPU6050_Config->MPU6050_Addr,
                       MPU6050_REG_PWR_MGMT_2, MPU6050_PWR_MGMT_2_VAL);
    if(ret != 0U) return 1U;
    
    ret = I2C_WriteReg(MPU6050_Config->Hi2c->I2Cx, MPU6050_Config->MPU6050_Addr,
                       MPU6050_REG_SMPLRT_DIV, MPU6050_SMPLRT_DIV_VAL);
    if(ret != 0U) return 1U;
    
    ret = I2C_WriteReg(MPU6050_Config->Hi2c->I2Cx, MPU6050_Config->MPU6050_Addr,
                       MPU6050_REG_CONFIG, MPU6050_CONFIG_VAL);
    if(ret != 0U) return 1U;
    
    ret = I2C_WriteReg(MPU6050_Config->Hi2c->I2Cx, MPU6050_Config->MPU6050_Addr,
                       MPU6050_REG_GYRO_CONFIG, MPU6050_GYRO_CONFIG_VAL);
    if(ret != 0U) return 1U;
    
    ret = I2C_WriteReg(MPU6050_Config->Hi2c->I2Cx, MPU6050_Config->MPU6050_Addr,
                       MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCEL_CONFIG_VAL);
    if(ret != 0U) return 1U;
    
    return 0U; // 初始化成功
}

/**
 * @brief  内部函数：读取单个轴的原始数据
 * @param  I2Cx: I2C外设
 * @param  DevAddr: 设备地址
 * @param  RegAddr: 数据高字节寄存器地址
 * @retval 成功：16位原始数据（int32_t）；失败：MPU6050_READ_FAIL_FLAG
 */
static int32_t MPU6050_ReadRawData(I2C_TypeDef* I2Cx, uint8_t DevAddr, uint8_t RegAddr)
{
    uint8_t data_h, data_l;
    
    // 读取高字节
    data_h = I2C_ReadReg(I2Cx, DevAddr, RegAddr);
    if(data_h == 0xFFU) return MPU6050_READ_FAIL_FLAG;
    
    // 读取低字节
    data_l = I2C_ReadReg(I2Cx, DevAddr, RegAddr + 1U);
    if(data_l == 0xFFU) return MPU6050_READ_FAIL_FLAG;
    
    // 合并为16位有符号数据
    return (int32_t)((int16_t)((data_h << 8U) | data_l));
}

uint8_t MPU6050_GetAccRawData(I2C_TypeDef* I2Cx, uint8_t DevAddr)
{
    int32_t raw_acc_x, raw_acc_y, raw_acc_z;
    
    // 读取三轴加速度数据
    raw_acc_x = MPU6050_ReadRawData(I2Cx, DevAddr, MPU6050_REG_ACCEL_XOUT_H);
    raw_acc_y = MPU6050_ReadRawData(I2Cx, DevAddr, MPU6050_REG_ACCEL_YOUT_H);
    raw_acc_z = MPU6050_ReadRawData(I2Cx, DevAddr, MPU6050_REG_ACCEL_ZOUT_H);
    
    // 检查读取失败
    if(raw_acc_x == MPU6050_READ_FAIL_FLAG || raw_acc_y == MPU6050_READ_FAIL_FLAG || raw_acc_z == MPU6050_READ_FAIL_FLAG)
    {
        return 1U;
    }
    
    // 直接赋值给结构体（无指针寻址，效率更高）
    mpu6050_raw.acc[0] = (int16_t)raw_acc_x;
    mpu6050_raw.acc[1] = (int16_t)raw_acc_y;
    mpu6050_raw.acc[2] = (int16_t)raw_acc_z;
    
    return 0U;
}

uint8_t MPU6050_GetGyroRawData(I2C_TypeDef* I2Cx, uint8_t DevAddr)
{
    int32_t raw_gyro_x, raw_gyro_y, raw_gyro_z;
    
    // 读取三轴陀螺仪数据
    raw_gyro_x = MPU6050_ReadRawData(I2Cx, DevAddr, MPU6050_REG_GYRO_XOUT_H);
    raw_gyro_y = MPU6050_ReadRawData(I2Cx, DevAddr, MPU6050_REG_GYRO_YOUT_H);
    raw_gyro_z = MPU6050_ReadRawData(I2Cx, DevAddr, MPU6050_REG_GYRO_ZOUT_H);
    
    // 检查读取失败
    if(raw_gyro_x == MPU6050_READ_FAIL_FLAG || raw_gyro_y == MPU6050_READ_FAIL_FLAG || raw_gyro_z == MPU6050_READ_FAIL_FLAG)
    {
        return 1U;
    }
    
    // 直接赋值给结构体
    mpu6050_raw.gyro[0] = (int16_t)raw_gyro_x;
    mpu6050_raw.gyro[1] = (int16_t)raw_gyro_y;
    mpu6050_raw.gyro[2] = (int16_t)raw_gyro_z;
    
    return 0U;
}

float MPU6050_ConvGyro(int16_t raw_gyro)
{
    float scale;
    switch(MPU6050_GYRO_CONFIG_VAL & 0x18)
    {
        case 0x00: scale = 131.0f;    break; // ±250°/s
        case 0x08: scale = 65.5f;     break; // ±500°/s
        case 0x10: scale = 32.8f;     break; // ±1000°/s
        case 0x18: scale = 16.4f;     break; // ±2000°/s
        default:   scale = 131.0f;    break;
    }
    return (float)raw_gyro / scale;
}

float MPU6050_ConvAcc(int16_t raw_acc)
{
    float scale;
    switch(MPU6050_ACCEL_CONFIG_VAL & 0x18)
    {
        case 0x00: scale = 16384.0f;  break; // ±2g
        case 0x08: scale = 8192.0f;   break; // ±4g
        case 0x10: scale = 4096.0f;   break; // ±8g
        case 0x18: scale = 2048.0f;   break; // ±16g
        default:   scale = 16384.0f;  break;
    }
    return (float)raw_acc / scale;
}


float MPU6050_GetGyroXData(void)  { return MPU6050_ConvGyro(mpu6050_raw.gyro[0]); }

float MPU6050_GetGyroYData(void)  { return MPU6050_ConvGyro(mpu6050_raw.gyro[1]); }

float MPU6050_GetGyroZData(void)  { return MPU6050_ConvGyro(mpu6050_raw.gyro[2]); }

float MPU6050_GetAccXData(void)   { return MPU6050_ConvAcc(mpu6050_raw.acc[0]); }

float MPU6050_GetAccYData(void)   { return MPU6050_ConvAcc(mpu6050_raw.acc[1]); }

float MPU6050_GetAccZData(void)   { return MPU6050_ConvAcc(mpu6050_raw.acc[2]); }