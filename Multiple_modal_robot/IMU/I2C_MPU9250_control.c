#include "I2C_MPU9250_control.h"


/*************************** 全局变量定义 ***************************/
MPU9250_Raw_t mpu9250_raw = {0};
MPU9250_Offset_t mpu9250_offset = {0}; 
float MagCalibX = 1.0f, MagCalibY = 1.0f, MagCalibZ = 1.0f;
uint8_t AK8963_Enabled = 0U;
// 全局量程变量
MPU9250_AccRange_t mpu9250_acc_range = MPU9250_ACC_RANGE_8G;
MPU9250_GyroRange_t mpu9250_gyro_range = MPU9250_GYRO_RANGE_1000DPS;

/*************************** 内部函数声明 ***************************/
static uint8_t AK8963_Init(I2C_TypeDef* I2Cx, uint8_t DevAddr);
static int32_t MPU9250_ReadRawData(I2C_TypeDef* I2Cx, uint8_t DevAddr, uint8_t RegAddr);
static void AK8963_SoftReset(I2C_TypeDef* I2Cx, uint8_t DevAddr);
// 非阻塞延时函数
static void MPU9250_DelayNonBlock(uint32_t ms);

/*************************** MPU9250核心函数 ***************************/
uint8_t MPU9250_Init(MPU9250_InitConfigTypeDef* MPU9250_Config, uint8_t AK_Use_or_not)
{
    uint8_t ret = 0U;
    uint8_t mpu_id = 0xFFU;
    
    // 参数校验：空指针防护
    if(MPU9250_Config == NULL || MPU9250_Config->Hi2c == NULL)
    {
        return 1U;
    }
    
    // 初始化I2C硬件
    RCS_I2C_Hardware_Init(MPU9250_Config->I2C_TYPE, MPU9250_Config->Hi2c);
    
    // 检查MPU9250通信是否正常
    mpu_id = MPU9250_GetID(MPU9250_Config->Hi2c->I2Cx, MPU9250_Config->MPU9250_Addr);
    if(mpu_id != MPU9250_WHOAMI_VAL)
    {
        return 1U; // MPU9250通信失败
    }
    
    // 配置MPU9250核心寄存器
    ret = I2C_WriteReg(MPU9250_Config->Hi2c->I2Cx, MPU9250_Config->MPU9250_Addr,
                       MPU9250_REG_PWR_MGMT_1, MPU9250_PWR_MGMT_1_VAL);
    if(ret != 0U) return 1U;
    
    // 非阻塞延时
    MPU9250_DelayNonBlock(MPU9250_DELAY_10MS);
    
    ret = I2C_WriteReg(MPU9250_Config->Hi2c->I2Cx, MPU9250_Config->MPU9250_Addr,
                       MPU9250_REG_PWR_MGMT_2, MPU9250_PWR_MGMT_2_VAL);
    if(ret != 0U) return 1U;
    
    ret = I2C_WriteReg(MPU9250_Config->Hi2c->I2Cx, MPU9250_Config->MPU9250_Addr,
                       MPU9250_REG_SMPLRT_DIV, MPU9250_SMPLRT_DIV_VAL);
    if(ret != 0U) return 1U;
    
    ret = I2C_WriteReg(MPU9250_Config->Hi2c->I2Cx, MPU9250_Config->MPU9250_Addr,
                       MPU9250_REG_CONFIG, MPU9250_CONFIG_VAL);
    if(ret != 0U) return 1U;
    
    // 设置初始量程
    ret = MPU9250_SetGyroRange(MPU9250_Config->Hi2c->I2Cx, MPU9250_Config->MPU9250_Addr,
                               MPU9250_Config->GyroRange);
    if(ret != 0U) return 1U;
    
    ret = MPU9250_SetAccRange(MPU9250_Config->Hi2c->I2Cx, MPU9250_Config->MPU9250_Addr,
                              MPU9250_Config->AccRange);
    if(ret != 0U) return 1U;
    
    // 启用磁力计
    if (AK_Use_or_not == 1U)
    {
        ret = I2C_WriteReg(MPU9250_Config->Hi2c->I2Cx, MPU9250_Config->MPU9250_Addr,
                           MPU9250_REG_INT_PIN_CFG, MPU9250_INT_PIN_CFG_VAL);
        if(ret != 0U) return 1U;
        
        ret = I2C_WriteReg(MPU9250_Config->Hi2c->I2Cx, MPU9250_Config->MPU9250_Addr,
                           MPU9250_REG_USER_CTRL, MPU9250_USER_CTRL_VAL);
        if(ret != 0U) return 1U;
        
        // 初始化AK8963磁力计
        ret = AK8963_Init(MPU9250_Config->Hi2c->I2Cx, MPU9250_Config->AK8963_Addr);
        if(ret != 0U)
        {
            return 2U; // 磁力计初始化失败
        }
        AK8963_Enabled = 1U;
    }
    else
    {
        AK8963_Enabled = 0U;
    }
    return 0U; // 初始化成功
}

uint8_t MPU9250_GetID(I2C_TypeDef* I2Cx, uint8_t DevAddr)
{
    return I2C_ReadReg(I2Cx, DevAddr, MPU9250_REG_WHO_AM_I);
}

uint8_t AK8963_GetID(I2C_TypeDef* I2Cx, uint8_t DevAddr)
{
    return I2C_ReadReg(I2Cx, DevAddr, AK8963_REG_WHO_AM_I);
}

/*************************** 原始数据读取函数 ***************************/
static int32_t MPU9250_ReadRawData(I2C_TypeDef* I2Cx, uint8_t DevAddr, uint8_t RegAddr)
{
    uint8_t data_h, data_l;
    
    data_h = I2C_ReadReg(I2Cx, DevAddr, RegAddr);
    if(data_h == 0xFFU) return MPU9250_READ_FAIL_FLAG;
    
    data_l = I2C_ReadReg(I2Cx, DevAddr, RegAddr + 1U);
    if(data_l == 0xFFU) return MPU9250_READ_FAIL_FLAG;
    
    return (int32_t)((int16_t)((data_h << 8U) | data_l));
}

// 加速度读取
uint8_t MPU9250_GetAccRawData(I2C_TypeDef* I2Cx, uint8_t DevAddr)
{
    int32_t raw_acc_x, raw_acc_y, raw_acc_z;
    
    raw_acc_x = MPU9250_ReadRawData(I2Cx, DevAddr, MPU9250_REG_ACCEL_XOUT_H);
    raw_acc_y = MPU9250_ReadRawData(I2Cx, DevAddr, MPU9250_REG_ACCEL_YOUT_H);
    raw_acc_z = MPU9250_ReadRawData(I2Cx, DevAddr, MPU9250_REG_ACCEL_ZOUT_H);
    
    if(raw_acc_x == MPU9250_READ_FAIL_FLAG || raw_acc_y == MPU9250_READ_FAIL_FLAG || raw_acc_z == MPU9250_READ_FAIL_FLAG)
    {
        return 1U;
    }
    
    mpu9250_raw.acc[0] = (int16_t)raw_acc_x;
    mpu9250_raw.acc[1] = (int16_t)raw_acc_y;
    mpu9250_raw.acc[2] = (int16_t)raw_acc_z;
    
    return 0U;
}

// 陀螺仪读取
uint8_t MPU9250_GetGyroRawData(I2C_TypeDef* I2Cx, uint8_t DevAddr)
{
    int32_t raw_gyro_x, raw_gyro_y, raw_gyro_z;
    
    raw_gyro_x = MPU9250_ReadRawData(I2Cx, DevAddr, MPU9250_REG_GYRO_XOUT_H);
    raw_gyro_y = MPU9250_ReadRawData(I2Cx, DevAddr, MPU9250_REG_GYRO_YOUT_H);
    raw_gyro_z = MPU9250_ReadRawData(I2Cx, DevAddr, MPU9250_REG_GYRO_ZOUT_H);
    
    if(raw_gyro_x == MPU9250_READ_FAIL_FLAG || raw_gyro_y == MPU9250_READ_FAIL_FLAG || raw_gyro_z == MPU9250_READ_FAIL_FLAG)
    {
        return 1U;
    }
    
    mpu9250_raw.gyro[0] = (int16_t)raw_gyro_x;
    mpu9250_raw.gyro[1] = (int16_t)raw_gyro_y;
    mpu9250_raw.gyro[2] = (int16_t)raw_gyro_z;
    
    return 0U;
}

// 磁力计读取
uint8_t MPU9250_GetMagRawData(I2C_TypeDef* I2Cx, uint8_t DevAddr)
{
    if(AK8963_Enabled == 0U)
    {
        return 1U; // 磁力计未使能
    }
    
    uint8_t ret = 0U;
    uint8_t mag_data[6] = {0U};
    uint8_t st1 = 0U;
    uint8_t st2 = 0U;
    int16_t raw_mag_x, raw_mag_y, raw_mag_z;
    
    // 检查数据就绪
    st1 = I2C_ReadReg(I2Cx, DevAddr, AK8963_REG_ST1);
    if((st1 & AK8963_ST1_DATA_READY_BIT) == 0U)
    {
        return 1U;
    }
    
    // 批量读取磁力计数据
    ret = I2C_ReadRegs(I2Cx, DevAddr, AK8963_REG_HXL, 6U, mag_data);
    if(ret != 0U)
    {
        AK8963_SoftReset(I2Cx, DevAddr); // 读取失败，复位磁力计
        return 1U;
    }
    
    // 检查数据溢出
    st2 = I2C_ReadReg(I2Cx, DevAddr, AK8963_REG_ST2);
    if((st2 & AK8963_ST2_DATA_OVERFLOW_BIT) != 0U)
    {
        AK8963_SoftReset(I2Cx, DevAddr); // 溢出，复位磁力计
        return 1U;
    }
    
    // 合并磁力计数据
    raw_mag_x = (int16_t)((mag_data[1U] << 8U) | mag_data[0U]);
    raw_mag_y = (int16_t)((mag_data[3U] << 8U) | mag_data[2U]);
    raw_mag_z = (int16_t)((mag_data[5U] << 8U) | mag_data[4U]);
    
    // 应用校准系数
    mpu9250_raw.mag[0] = (int16_t)(raw_mag_x * MagCalibX);
    mpu9250_raw.mag[1] = (int16_t)(raw_mag_y * MagCalibY);
    mpu9250_raw.mag[2] = (int16_t)(raw_mag_z * MagCalibZ);
    
    return 0U;
}

// Burst版：一次读取加速度+陀螺仪+温度（14字节）
uint8_t MPU9250_GetAccGyroRawBurst(I2C_TypeDef *I2Cx, uint8_t DevAddr)
{
    uint8_t ret = 0U;
    uint8_t burst_data[14] = {0U};
    
    // 一次读取14字节
    ret = I2C_ReadRegs(I2Cx, DevAddr, MPU9250_REG_BURST_START, 14U, burst_data);
    if(ret != 0U)
    {
        return 1U;
    }
    
    // 拆包数据
    mpu9250_raw.acc[0] = (int16_t)((burst_data[0] << 8U) | burst_data[1]);
    mpu9250_raw.acc[1] = (int16_t)((burst_data[2] << 8U) | burst_data[3]);
    mpu9250_raw.acc[2] = (int16_t)((burst_data[4] << 8U) | burst_data[5]);
    mpu9250_raw.temp   = (int16_t)((burst_data[6] << 8U) | burst_data[7]);
    mpu9250_raw.gyro[0] = (int16_t)((burst_data[8] << 8U) | burst_data[9]);
    mpu9250_raw.gyro[1] = (int16_t)((burst_data[10] << 8U) | burst_data[11]);
    mpu9250_raw.gyro[2] = (int16_t)((burst_data[12] << 8U) | burst_data[13]);
    
    return 0U;
}

// Burst版：一次读取磁力计7字节（ST1+6数据）
uint8_t AK8963_GetMagRawBurst(I2C_TypeDef *I2Cx, uint8_t DevAddr)
{
    if(AK8963_Enabled == 0U) return 1U;
    
    uint8_t ret = 0U;
    uint8_t burst_data[7] = {0U};
    int16_t raw_mag_x, raw_mag_y, raw_mag_z;
    
    // 一次读取7字节
    ret = I2C_ReadRegs(I2Cx, DevAddr, AK8963_REG_BURST_START, 7U, burst_data);
    if(ret != 0U)
    {
        AK8963_SoftReset(I2Cx, DevAddr);
        return 1U;
    }
    
    // 检查数据就绪和溢出
    if((burst_data[0] & AK8963_ST1_DATA_READY_BIT) == 0U || 
       (burst_data[6] & AK8963_ST2_DATA_OVERFLOW_BIT) != 0U)
    {
        AK8963_SoftReset(I2Cx, DevAddr);
        return 1U;
    }
    
    // 拆包数据
    raw_mag_x = (int16_t)((burst_data[2] << 8U) | burst_data[1]);
    raw_mag_y = (int16_t)((burst_data[4] << 8U) | burst_data[3]);
    raw_mag_z = (int16_t)((burst_data[6] << 8U) | burst_data[5]);
    
    // 应用校准系数
    mpu9250_raw.mag[0] = (int16_t)(raw_mag_x * MagCalibX);
    mpu9250_raw.mag[1] = (int16_t)(raw_mag_y * MagCalibY);
    mpu9250_raw.mag[2] = (int16_t)(raw_mag_z * MagCalibZ);
    
    return 0U;
}

/*************************** 校准函数 ***************************/
uint8_t MPU9250_Calibrate(I2C_TypeDef* I2Cx, uint8_t DevAddr, uint16_t samples)
{
    if(samples == 0U) samples = MPU9250_CALIB_SAMPLES;
    
    int32_t acc_sum[3] = {0}, gyro_sum[3] = {0};
    uint16_t i;
    uint8_t ret;
    
    // 清零之前的偏移
    mpu9250_offset.acc_offset[0] = 0.0f;
    mpu9250_offset.acc_offset[1] = 0.0f;
    mpu9250_offset.acc_offset[2] = 0.0f;
    mpu9250_offset.gyro_offset[0] = 0.0f;
    mpu9250_offset.gyro_offset[1] = 0.0f;
    mpu9250_offset.gyro_offset[2] = 0.0f;
    
    // 采样并累加
    for(i = 0U; i < samples; i++)
    {
        ret = MPU9250_GetAccGyroRawBurst(I2Cx, DevAddr);
        if(ret != 0U) return 1U;
        
        acc_sum[0] += mpu9250_raw.acc[0];
        acc_sum[1] += mpu9250_raw.acc[1];
        acc_sum[2] += mpu9250_raw.acc[2];
        gyro_sum[0] += mpu9250_raw.gyro[0];
        gyro_sum[1] += mpu9250_raw.gyro[1];
        gyro_sum[2] += mpu9250_raw.gyro[2];
        
        MPU9250_DelayNonBlock(MPU9250_CALIB_DELAY_MS);
    }
    
    // 计算平均值（原始数据）
    float acc_avg[3], gyro_avg[3];
    acc_avg[0] = (float)acc_sum[0] / samples;
    acc_avg[1] = (float)acc_sum[1] / samples;
    acc_avg[2] = (float)acc_sum[2] / samples;
    gyro_avg[0] = (float)gyro_sum[0] / samples;
    gyro_avg[1] = (float)gyro_sum[1] / samples;
    gyro_avg[2] = (float)gyro_sum[2] / samples;
    
    // 转换为物理值并计算偏移
    // 加速度计：水平静止时，Z轴应为1g，X/Y轴为0g
    mpu9250_offset.acc_offset[0] = -MPU9250_ConvAcc((int16_t)acc_avg[0]);
    mpu9250_offset.acc_offset[1] = -MPU9250_ConvAcc((int16_t)acc_avg[1]);
    mpu9250_offset.acc_offset[2] = 1.0f - MPU9250_ConvAcc((int16_t)acc_avg[2]);
    
    // 陀螺仪：静止时应为0°/s
    mpu9250_offset.gyro_offset[0] = -MPU9250_ConvGyro((int16_t)gyro_avg[0]);
    mpu9250_offset.gyro_offset[1] = -MPU9250_ConvGyro((int16_t)gyro_avg[1]);
    mpu9250_offset.gyro_offset[2] = -MPU9250_ConvGyro((int16_t)gyro_avg[2]);
    
    return 0U;
}

/*************************** 量程设置函数 ***************************/
uint8_t MPU9250_SetAccRange(I2C_TypeDef* I2Cx, uint8_t DevAddr, MPU9250_AccRange_t range)
{
    uint8_t reg_val = 0U;
    uint8_t ret = 0U;
    
    // 读取当前配置
    reg_val = I2C_ReadReg(I2Cx, DevAddr, MPU9250_REG_ACCEL_CONFIG);
    if(reg_val == 0xFFU) return 1U;
    
    // 清除原有量程位
    reg_val &= ~0x18U;
    
    // 设置新量程
    switch(range)
    {
        case MPU9250_ACC_RANGE_2G:
            reg_val |= 0x00U;
            break;
        case MPU9250_ACC_RANGE_4G:
            reg_val |= 0x08U;
            break;
        case MPU9250_ACC_RANGE_8G:
            reg_val |= 0x10U;
            break;
        case MPU9250_ACC_RANGE_16G:
            reg_val |= 0x18U;
            break;
        default:
            return 1U;
    }
    
    // 写入寄存器
    ret = I2C_WriteReg(I2Cx, DevAddr, MPU9250_REG_ACCEL_CONFIG, reg_val);
    if(ret == 0U)
    {
        mpu9250_acc_range = range; // 更新全局量程变量
    }
    
    return ret;
}

uint8_t MPU9250_SetGyroRange(I2C_TypeDef* I2Cx, uint8_t DevAddr, MPU9250_GyroRange_t range)
{
    uint8_t reg_val = 0U;
    uint8_t ret = 0U;
    
    reg_val = I2C_ReadReg(I2Cx, DevAddr, MPU9250_REG_GYRO_CONFIG);
    if(reg_val == 0xFFU) return 1U;
    
    reg_val &= ~0x18U;
    
    switch(range)
    {
        case MPU9250_GYRO_RANGE_250DPS:
            reg_val |= 0x00U;
            break;
        case MPU9250_GYRO_RANGE_500DPS:
            reg_val |= 0x08U;
            break;
        case MPU9250_GYRO_RANGE_1000DPS:
            reg_val |= 0x10U;
            break;
        case MPU9250_GYRO_RANGE_2000DPS:
            reg_val |= 0x18U;
            break;
        default:
            return 1U;
    }
    
    ret = I2C_WriteReg(I2Cx, DevAddr, MPU9250_REG_GYRO_CONFIG, reg_val);
    if(ret == 0U)
    {
        mpu9250_gyro_range = range; // 更新全局量程变量
    }
    
    return ret;
}

/*************************** 磁力计复位函数 ***************************/
static void AK8963_SoftReset(I2C_TypeDef* I2Cx, uint8_t DevAddr)
{
    // 复位磁力计
    I2C_WriteReg(I2Cx, DevAddr, AK8963_REG_CNTL2, AK8963_CNTL2_RESET_VAL);
    MPU9250_DelayNonBlock(MPU9250_DELAY_10MS);
    
    // 重新设置为连续测量模式2
    I2C_WriteReg(I2Cx, DevAddr, AK8963_REG_CNTL1, AK8963_CNTL1_MEAS_MODE);
    MPU9250_DelayNonBlock(MPU9250_DELAY_5MS);
}

/*************************** 数据转换函数 ***************************/
float MPU9250_ConvGyro(int16_t raw_gyro)
{
    float scale;
    switch(mpu9250_gyro_range)
    {
        case MPU9250_GYRO_RANGE_250DPS:
            scale = 131.0f;
            break;
        case MPU9250_GYRO_RANGE_500DPS:
            scale = 65.5f;
            break;
        case MPU9250_GYRO_RANGE_1000DPS:
            scale = 32.8f;
            break;
        case MPU9250_GYRO_RANGE_2000DPS:
            scale = 16.4f;
            break;
        default:
            scale = 131.0f;
            break;
    }
    // 先减偏移，再换算
    return ((float)raw_gyro / scale) + mpu9250_offset.gyro_offset[0];
}

float MPU9250_ConvAcc(int16_t raw_acc)
{
    float scale;
    switch(mpu9250_acc_range)
    {
        case MPU9250_ACC_RANGE_2G:
            scale = 16384.0f;
            break;
        case MPU9250_ACC_RANGE_4G:
            scale = 8192.0f;
            break;
        case MPU9250_ACC_RANGE_8G:
            scale = 4096.0f;
            break;
        case MPU9250_ACC_RANGE_16G:
            scale = 2048.0f;
            break;
        default:
            scale = 16384.0f;
            break;
    }
    // 先减偏移，再换算
    return ((float)raw_acc / scale) + mpu9250_offset.acc_offset[0];
}

float MPU9250_ConvMag(int16_t raw_mag)
{
    return (float)raw_mag * 0.15f; // 1LSB = 0.15μT
}

/*************************** 获取转换后数据 ***************************/
float MPU9250_GetGyroXData(void)  
{ 
    return ((float)mpu9250_raw.gyro[0] / MPU9250_ConvGyro(1) ) + mpu9250_offset.gyro_offset[0]; 
}
float MPU9250_GetGyroYData(void)  
{ 
    return ((float)mpu9250_raw.gyro[1] / MPU9250_ConvGyro(1) ) + mpu9250_offset.gyro_offset[1]; 
}
float MPU9250_GetGyroZData(void)  
{ 
    return ((float)mpu9250_raw.gyro[2] / MPU9250_ConvGyro(1) ) + mpu9250_offset.gyro_offset[2]; 
}
float MPU9250_GetAccXData(void)   
{ 
    return ((float)mpu9250_raw.acc[0] / MPU9250_ConvAcc(1) ) + mpu9250_offset.acc_offset[0]; 
}
float MPU9250_GetAccYData(void)   
{ 
    return ((float)mpu9250_raw.acc[1] / MPU9250_ConvAcc(1) ) + mpu9250_offset.acc_offset[1]; 
}
float MPU9250_GetAccZData(void)   
{ 
    return ((float)mpu9250_raw.acc[2] / MPU9250_ConvAcc(1) ) + mpu9250_offset.acc_offset[2]; 
}
float MPU9250_GetMagXData(void)   { return MPU9250_ConvMag(mpu9250_raw.mag[0]); }
float MPU9250_GetMagYData(void)   { return MPU9250_ConvMag(mpu9250_raw.mag[1]); }
float MPU9250_GetMagZData(void)   { return MPU9250_ConvMag(mpu9250_raw.mag[2]); }

/*************************** 辅助函数 ***************************/
// 非阻塞延时（基于HAL_GetTick）
static void MPU9250_DelayNonBlock(uint32_t ms)
{
    // 转换ms到μs：ms * 1000
    uint64_t start_us = Hard_Get_RealTime_Stamp();  // 用Stamp（整数）比Us（浮点）更高效
    uint64_t target_us = start_us + (uint64_t)ms * 1000U;
    
    // 循环等待，直到达到目标时间（非阻塞，可加其他逻辑）
    while(Hard_Get_RealTime_Stamp() < target_us);
}


// AK8963初始化函数
static uint8_t AK8963_Init(I2C_TypeDef* I2Cx, uint8_t DevAddr)
{
    uint8_t ret = 0U;
    uint8_t mag_id = 0xFFU;
    uint8_t ASAX, ASAY, ASAZ;

    mag_id = AK8963_GetID(I2Cx, DevAddr);
    if(mag_id != AK8963_WHOAMI_VAL)
    {
        return 1U;
    }
    
    // 复位磁力计（非阻塞延时）
    ret = I2C_WriteReg(I2Cx, DevAddr, AK8963_REG_CNTL2, AK8963_CNTL2_RESET_VAL);
    if(ret != 0U) return 1U;
    MPU9250_DelayNonBlock(MPU9250_DELAY_10MS);
    
    // 进入FUSE ROM模式
    ret = I2C_WriteReg(I2Cx, DevAddr, AK8963_REG_CNTL1, AK8963_CNTL1_FUSE_MODE);
    if(ret != 0U) return 1U;
    MPU9250_DelayNonBlock(MPU9250_DELAY_10MS);
    
    // 读取出厂校准系数
    ASAX = I2C_ReadReg(I2Cx, DevAddr, AK8963_REG_ASAX);
    ASAY = I2C_ReadReg(I2Cx, DevAddr, AK8963_REG_ASAY);
    ASAZ = I2C_ReadReg(I2Cx, DevAddr, AK8963_REG_ASAZ);
    
    // 计算校准系数
    if((ASAX == 0xFFU) || (ASAY == 0xFFU) || (ASAZ == 0xFFU))
    {
        MagCalibX = 1.0f;
        MagCalibY = 1.0f;
        MagCalibZ = 1.0f;
    }
    else
    {
        MagCalibX = (float)(ASAX - MPU9250_CALIB_BASE_VAL) / MPU9250_CALIB_DIV_VAL + 1.0f;
        MagCalibY = (float)(ASAY - MPU9250_CALIB_BASE_VAL) / MPU9250_CALIB_DIV_VAL + 1.0f;
        MagCalibZ = (float)(ASAZ - MPU9250_CALIB_BASE_VAL) / MPU9250_CALIB_DIV_VAL + 1.0f;
    }
    
    // 退出FUSE模式
    ret = I2C_WriteReg(I2Cx, DevAddr, AK8963_REG_CNTL1, MPU9250_AK8963_FUSE_EXIT_VAL);
    if(ret != 0U) return 1U;
    MPU9250_DelayNonBlock(MPU9250_DELAY_5MS);
    
    // 设置连续测量模式2
    ret = I2C_WriteReg(I2Cx, DevAddr, AK8963_REG_CNTL1, AK8963_CNTL1_MEAS_MODE);
    if(ret != 0U) return 1U;
    
    return 0U;
}