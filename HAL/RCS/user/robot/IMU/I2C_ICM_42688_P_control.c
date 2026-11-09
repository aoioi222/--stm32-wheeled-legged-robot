#include "I2C_ICM_42688_P_control.h"
#include "cmsis_os2.h"

#define IMU_I2C_TIMEOUT_MS 2u

static uint8_t I2C_ReadRegs(I2C_HandleTypeDef *i2c, uint8_t address, uint8_t reg,
                            uint8_t count, uint8_t *data)
{
    return HAL_I2C_Mem_Read(i2c, (uint16_t)address << 1, reg, I2C_MEMADD_SIZE_8BIT,
                            data, count, IMU_I2C_TIMEOUT_MS) != HAL_OK;
}

static uint8_t I2C_ReadReg(I2C_HandleTypeDef *i2c, uint8_t address, uint8_t reg)
{
    uint8_t value;
    return I2C_ReadRegs(i2c, address, reg, 1u, &value) ? 0xFFu : value;
}

static uint8_t I2C_WriteReg(I2C_HandleTypeDef *i2c, uint8_t address, uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(i2c, (uint16_t)address << 1, reg, I2C_MEMADD_SIZE_8BIT,
                             &value, 1u, IMU_I2C_TIMEOUT_MS) != HAL_OK;
}
/**
 ******************************************************************************
 * @file    I2C_ICM_42688_P_control.c
 * @brief   ICM-42688-P I2C驱动，支持单寄存器读取、Burst连续读、FIFO读取、零偏校准、互补滤波欧拉角解算
 *
 * @note
 * 1. 适配STM32F4xx硬件I2C，支持I2C1/I2C2/I2C3
 * 2. 支持传感器硬件零偏写入（User Offset寄存器）
 * 3. Burst模式连续读取温感+加速度+陀螺仪，效率高于单寄存器循环读取
 * 4. 支持FIFO读取；FIFO数据包格式为Packet3
 * 5. I2C底层依赖 Hardware_I2C.h 提供读写接口
 * 6. 校准过程要求传感器水平静置，不可晃动
 * @version V2.3
 * @date    2026-01-25
 ******************************************************************************
 * @attention
 *
 * 1. ICM42688P I2C地址：ADDR引脚拉低=0x68，ADDR引脚接VDD=0x69
 * 2. 所有寄存器读写均需要等待设备上电稳定
 * 3. 校准前保证传感器静止，否则零偏参数无效
 * 4. Burst读取起始地址固定0x1D，不可随意修改
 * 5. FIFO配置：FIFO_CONFIG1/2/3，本驱动使用Packet3格式，一包16字节
 ******************************************************************************
 */

/*************************** 全局变量 ***************************/
ICM42688P_Raw_t icm42688p_raw = {0};       // ICM42688P原始传感器数据
ICM42688P_Offset_t icm42688p_offset = {0}; // IMU零偏补偿参数
ICM42688P_Euler_angles icm42688p_euler_angles = {0}; // 欧拉角解算结果

uint8_t icm42688p_acc_range = ICM42688P_DEFAULT_ACC_RANGE;
uint8_t icm42688p_gyro_range = ICM42688P_DEFAULT_GYRO_RANGE;

ICM42688P_Data_t icm42688p_data = {0};
uint32_t g_interrupt_status;

/* I2C上下文静态保存，供无参刷新函数 ICM42688P_UpdateData 使用 */
static I2C_HandleTypeDef *g_icm_i2c = NULL;
static uint8_t       g_icm_addr = 0x00;

/*************************** 静态函数声明 ***************************/
static int32_t ICM42688P_ReadRawData(I2C_HandleTypeDef *p_i2c, uint8_t DevAddr, uint8_t RegAddr);
static float get_gyro_scale(void);
static float get_acc_scale(void);

/**
 * @brief  获取当前陀螺仪量程对应的LSB/(°/s)换算系数
 * @retval scale 灵敏度系数
 */
static float get_gyro_scale(void)
{
    uint8_t gyro_fs_sel = (icm42688p_gyro_range >> 5) & 0x07U;
    switch (gyro_fs_sel)
    {
    case 0x07U: return ICM42688P_GYRO_SCALE_15_625DPS;
    case 0x06U: return ICM42688P_GYRO_SCALE_31_25DPS;
    case 0x05U: return ICM42688P_GYRO_SCALE_62_5DPS;
    case 0x04U: return ICM42688P_GYRO_SCALE_125DPS;
    case 0x03U: return ICM42688P_GYRO_SCALE_250DPS;
    case 0x02U: return ICM42688P_GYRO_SCALE_500DPS;
    case 0x01U: return ICM42688P_GYRO_SCALE_1000DPS;
    case 0x00U: return ICM42688P_GYRO_SCALE_2000DPS;
    default:     return ICM42688P_GYRO_SCALE_2000DPS;
    }
}

/**
 * @brief  获取当前加速度计量程对应的LSB/g换算系数
 * @retval scale 灵敏度系数
 */
static float get_acc_scale(void)
{
    uint8_t acc_fs_sel = (icm42688p_acc_range >> 5) & 0x03U;
    switch (acc_fs_sel)
    {
    case 0x03U: return ICM42688P_ACC_SCALE_2G;
    case 0x02U: return ICM42688P_ACC_SCALE_4G;
    case 0x01U: return ICM42688P_ACC_SCALE_8G;
    case 0x00U: return ICM42688P_ACC_SCALE_16G;
    default:     return ICM42688P_ACC_SCALE_2G;
    }
}

/*************************** ICM42688P初始化 ***************************/
/**
 * @brief  ICM42688-P传感器初始化
 * @param  ICM42688P_Config: 初始化配置指针
 * @param  samples: 校准采样点数，0则使用默认值
 * @retval 0-成功 1-失败（空指针/通讯失败/ID不匹配）
 * @note   默认配置：陀螺仪1000dps@8kHz，加速度8g@8kHz；开启FIFO Packet3
 */
uint8_t ICM42688P_Init(ICM42688P_InitConfigTypeDef *ICM42688P_Config, uint16_t samples)
{
    uint8_t ret = 0U;
    uint8_t icm_id = 0xFFU;

    if (ICM42688P_Config == NULL || ICM42688P_Config->Hi2c == NULL)
    {
        return 1U;
    }
    g_icm_i2c = ICM42688P_Config->Hi2c;
    g_icm_addr = ICM42688P_Config->ICM42688P_Addr;


    // 读取设备ID校验
    icm_id = ICM42688P_GetID(ICM42688P_Config->Hi2c, ICM42688P_Config->ICM42688P_Addr);
    if (icm_id != ICM42688P_WHOAMI_VAL)
    {
        return 1U;
    }

    // 软件复位
    ret = I2C_WriteReg(ICM42688P_Config->Hi2c, ICM42688P_Config->ICM42688P_Addr,
                       ICM42688P_REG_DEVICE_CONFIG, 0x01U);
    osDelay(5);
    if (ret != 0U)
        return 1U;

    // PWR_MGMT0：使能加速度计+陀螺仪，工作模式开启
    ret = I2C_WriteReg(ICM42688P_Config->Hi2c, ICM42688P_Config->ICM42688P_Addr,
                       ICM42688P_REG_PWR_MGMT0, 0x37);
    if (ret != 0U)
        return 1U;

    // 加速度计量程与ODR配置
    ret = I2C_WriteReg(ICM42688P_Config->Hi2c, ICM42688P_Config->ICM42688P_Addr,
                       ICM42688P_REG_ACCEL_CONFIG0, ICM42688P_ACC_CONFIG);
    if (ret != 0U)
        return 1U;

    // 陀螺仪量程与ODR配置
    ret = I2C_WriteReg(ICM42688P_Config->Hi2c, ICM42688P_Config->ICM42688P_Addr,
                       ICM42688P_REG_GYRO_CONFIG0, ICM42688P_GYRO_CONFIG);
    if (ret != 0U)
        return 1U;

    // INT_CONFIG1：异步复位关闭
    ret = I2C_WriteReg(ICM42688P_Config->Hi2c, ICM42688P_Config->ICM42688P_Addr, 0x64, 0x00);
    if (ret != 0U) return 1U;

    // INTF_CONFIG0 I2C接口配置
    ret = I2C_WriteReg(ICM42688P_Config->Hi2c, ICM42688P_Config->ICM42688P_Addr,
                       ICM42688P_REG_INTF_CONFIG0, 0x30U);
    if (ret != 0U)
        return 1U;

    // GYRO_ACCEL_CONFIG0 滤波配置
    ret = I2C_WriteReg(ICM42688P_Config->Hi2c, ICM42688P_Config->ICM42688P_Addr,
                       ICM42688P_REG_GYRO_ACCEL_CONFIG0, 0x11);
    if (ret != 0U)
        return 1U;

    // DRIVE_CONFIG I2C驱动配置
    ret = I2C_WriteReg(ICM42688P_Config->Hi2c, ICM42688P_Config->ICM42688P_Addr,
                       ICM42688P_REG_DRIVE_CONFIG, 0x51);
    if (ret != 0U)
        return 1U;

    // 陀螺仪滤波配置
    ret = I2C_WriteReg(ICM42688P_Config->Hi2c, ICM42688P_Config->ICM42688P_Addr,
                       ICM42688P_REG_GYRO_CONFIG1, 0x03U);
    if (ret != 0U)
        return 1U;

    // 加速度计滤波配置
    ret = I2C_WriteReg(ICM42688P_Config->Hi2c, ICM42688P_Config->ICM42688P_Addr,
                       ICM42688P_REG_ACCEL_CONFIG1, 0x03U);
    if (ret != 0U)
        return 1U;

    // INT_CONFIG1 中断配置
    ret = I2C_WriteReg(ICM42688P_Config->Hi2c, ICM42688P_Config->ICM42688P_Addr,
               ICM42688P_REG_INT_CONFIG1, 0x40);
    if (ret != 0U)
        return 1U;

    // FIFO配置：开启加速度、陀螺仪、温度、时间戳
    ret = I2C_WriteReg(ICM42688P_Config->Hi2c, ICM42688P_Config->ICM42688P_Addr,
                   ICM42688P_REG_FIFO_CONFIG1, 0x17);
    if (ret != 0U)
        return 1U;

    // FIFO Packet3模式，一包16字节
    ret = I2C_WriteReg(ICM42688P_Config->Hi2c, ICM42688P_Config->ICM42688P_Addr,
                   ICM42688P_REG_FIFO_CONFIG2, 0x20);
    if (ret != 0U)
        return 1U;
    ret = I2C_WriteReg(ICM42688P_Config->Hi2c, ICM42688P_Config->ICM42688P_Addr,
                   ICM42688P_REG_FIFO_CONFIG3, 0x00);
    if (ret != 0U)
        return 1U;

    osDelay(30); // 等待传感器稳定
    icm42688p_acc_range = ICM42688P_ACC_RANGE_8G;
    icm42688p_gyro_range = ICM42688P_GYRO_RANGE_1000DPS;

    // 静态零偏校准
    return ICM42688P_Calibrate(ICM42688P_Config->Hi2c, ICM42688P_Config->ICM42688P_Addr, samples);
}

/**
 * @brief  读取ICM42688P设备ID
 * @param  p_i2c: I2C外设指针
 * @param  DevAddr: I2C设备地址
 * @retval 设备ID；通讯失败返回0xFF
 */
uint8_t ICM42688P_GetID(I2C_HandleTypeDef *p_i2c, uint8_t DevAddr)
{
    return I2C_ReadReg(p_i2c, DevAddr, ICM42688P_REG_WHO_AM_I);
}

/*************************** 原始数据读取底层 ***************************/
/**
 * @brief  读取16bit寄存器（高字节+低字节）
 * @param  p_i2c: I2C外设
 * @param  DevAddr: 设备地址
 * @param  RegAddr: 高字节寄存器地址
 * @retval 有符号16位原始值；读取失败返回ICM42688P_READ_FAIL_FLAG
 */
static int32_t ICM42688P_ReadRawData(I2C_HandleTypeDef *p_i2c, uint8_t DevAddr, uint8_t RegAddr)
{
    uint8_t data_h, data_l;

    data_h = I2C_ReadReg(p_i2c, DevAddr, RegAddr);
    if (data_h == 0xFFU)
        return ICM42688P_READ_FAIL_FLAG;

    data_l = I2C_ReadReg(p_i2c, DevAddr, RegAddr + 1U);
    if (data_l == 0xFFU)
        return ICM42688P_READ_FAIL_FLAG;

    return (int32_t)((int16_t)((data_h << 8U) | data_l));
}

/**
 * @brief  单独读取加速度三轴原始数据
 * @param  p_i2c: I2C外设
 * @param  DevAddr: 设备地址
 * @retval 0成功 1失败
 * @note 结果存入全局 icm42688p_raw.acc[]
 */
uint8_t ICM42688P_GetAccRawData(I2C_HandleTypeDef *p_i2c, uint8_t DevAddr)
{
    int32_t raw_acc_x, raw_acc_y, raw_acc_z;

    raw_acc_x = ICM42688P_ReadRawData(p_i2c, DevAddr, ICM42688P_REG_ACCEL_XOUT_H);
    raw_acc_y = ICM42688P_ReadRawData(p_i2c, DevAddr, ICM42688P_REG_ACCEL_YOUT_H);
    raw_acc_z = ICM42688P_ReadRawData(p_i2c, DevAddr, ICM42688P_REG_ACCEL_ZOUT_H);

    if (raw_acc_x == ICM42688P_READ_FAIL_FLAG || raw_acc_y == ICM42688P_READ_FAIL_FLAG || raw_acc_z == ICM42688P_READ_FAIL_FLAG)
    {
        return 1U;
    }
    g_interrupt_status = __get_PRIMASK();
    __disable_irq();
    icm42688p_raw.acc[0] = (int16_t)raw_acc_x;
    icm42688p_raw.acc[1] = (int16_t)raw_acc_y;
    icm42688p_raw.acc[2] = (int16_t)raw_acc_z;
    __set_PRIMASK(g_interrupt_status);

    return 0U;
}

/**
 * @brief  单独读取陀螺仪三轴原始数据
 * @param  p_i2c: I2C外设
 * @param  DevAddr: 设备地址
 * @retval 0成功 1失败
 */
uint8_t ICM42688P_GetGyroRawData(I2C_HandleTypeDef *p_i2c, uint8_t DevAddr)
{
    int32_t raw_gyro_x, raw_gyro_y, raw_gyro_z;

    raw_gyro_x = ICM42688P_ReadRawData(p_i2c, DevAddr, ICM42688P_REG_GYRO_XOUT_H);
    raw_gyro_y = ICM42688P_ReadRawData(p_i2c, DevAddr, ICM42688P_REG_GYRO_YOUT_H);
    raw_gyro_z = ICM42688P_ReadRawData(p_i2c, DevAddr, ICM42688P_REG_GYRO_ZOUT_H);

    if (raw_gyro_x == ICM42688P_READ_FAIL_FLAG || raw_gyro_y == ICM42688P_READ_FAIL_FLAG || raw_gyro_z == ICM42688P_READ_FAIL_FLAG)
    {
        return 1U;
    }
    g_interrupt_status = __get_PRIMASK();
    __disable_irq();
    icm42688p_raw.gyro[0] = (int16_t)raw_gyro_x;
    icm42688p_raw.gyro[1] = (int16_t)raw_gyro_y;
    icm42688p_raw.gyro[2] = (int16_t)raw_gyro_z;
    __set_PRIMASK(g_interrupt_status);

    return 0U;
}

/**
 * @brief  Burst连续读取：温度+加速度+陀螺仪（14字节）
 * @param  p_i2c: I2C外设
 * @param  DevAddr: 设备地址
 * @retval 0成功 1失败
 * @note Burst起始地址0x1D，顺序：温度(2B)→ACCX/Y/Z(6B)→GYROX/Y/Z(6B)
 */
uint8_t ICM42688P_GetAccGyroRawBurst(I2C_HandleTypeDef *p_i2c, uint8_t DevAddr)
{
    uint8_t ret = 0U;
    uint8_t burst_data[14] = {0U};

    ret = I2C_ReadRegs(p_i2c, DevAddr, ICM42688P_REG_BURST_START, 14U, burst_data);
    if (ret != 0U)
    {
        return 1U;
    }

    g_interrupt_status = __get_PRIMASK();
    __disable_irq();
    icm42688p_raw.temp  = (int16_t)((burst_data[0] << 8U) | burst_data[1]);
    icm42688p_raw.acc[0] = (int16_t)((burst_data[2] << 8U) | burst_data[3]);
    icm42688p_raw.acc[1] = (int16_t)((burst_data[4] << 8U) | burst_data[5]);
    icm42688p_raw.acc[2] = (int16_t)((burst_data[6] << 8U) | burst_data[7]);
    icm42688p_raw.gyro[0] = (int16_t)((burst_data[8] << 8U) | burst_data[9]);
    icm42688p_raw.gyro[1] = (int16_t)((burst_data[10] << 8U) | burst_data[11]);
    icm42688p_raw.gyro[2] = (int16_t)((burst_data[12] << 8U) | burst_data[13]);
    __set_PRIMASK(g_interrupt_status);

    return 0U;
}

/**
 * @brief  FIFO读取Packet3数据包（16字节）
 * @param  p_i2c: I2C外设
 * @param  DevAddr: 设备地址
 * @retval 0成功 1失败
 */
uint8_t ICM42688P_GetFifoData(I2C_HandleTypeDef *p_i2c, uint8_t DevAddr)
{
    uint8_t ret = 0U;
    uint8_t fifo_cnt_h, fifo_cnt_l;
    uint16_t fifo_cnt;
    uint8_t fifo_data[32] = {0};

    fifo_cnt_h = I2C_ReadReg(p_i2c, DevAddr, 0x2E);
    fifo_cnt_l = I2C_ReadReg(p_i2c, DevAddr, 0x2F);
    fifo_cnt = (uint16_t)((fifo_cnt_h << 8U) | fifo_cnt_l);

    // 不足一包(16字节)丢弃数据
    if (fifo_cnt < 16U)
    {
        if (fifo_cnt > 0)
        {
            uint8_t dummy_data[32] = {0};
            I2C_ReadRegs(p_i2c, DevAddr, 0x30, fifo_cnt, dummy_data);
        }
        return 0U;
    }

    ret = I2C_ReadRegs(p_i2c, DevAddr, 0x30, 16U, fifo_data);
    if (ret != 0U) return 1U;

    g_interrupt_status = __get_PRIMASK();
    __disable_irq();
    icm42688p_raw.acc[0] = (int16_t)((fifo_data[1] << 8U) | fifo_data[2]);
    icm42688p_raw.acc[1] = (int16_t)((fifo_data[3] << 8U) | fifo_data[4]);
    icm42688p_raw.acc[2] = (int16_t)((fifo_data[5] << 8U) | fifo_data[6]);
    icm42688p_raw.gyro[0] = (int16_t)((fifo_data[7] << 8U) | fifo_data[8]);
    icm42688p_raw.gyro[1] = (int16_t)((fifo_data[9] << 8U) | fifo_data[10]);
    icm42688p_raw.gyro[2] = (int16_t)((fifo_data[11] << 8U) | fifo_data[12]);
    icm42688p_raw.temp = (int16_t)(int8_t)fifo_data[13];
    __set_PRIMASK(g_interrupt_status);

    return 0U;
}

/*************************** IMU静态零偏校准 ***************************/
/**
 * @brief  IMU静态校准，采集多组数据计算零偏写入传感器User Offset寄存器
 * @param  p_i2c: I2C外设
 * @param  DevAddr: 设备地址
 * @param  samples: 采样点数
 * @retval 0成功 1失败
 * @note 传感器必须水平静置；Z轴加速度理论值=1g
 */
uint8_t ICM42688P_Calibrate(I2C_HandleTypeDef *p_i2c, uint8_t DevAddr, uint16_t samples)
{
    if (p_i2c == NULL)
        return 1U;
    if (samples == 0U)
        samples = ICM42688P_CALIB_SAMPLES;

    int32_t acc_sum[3] = {0}, gyro_sum[3] = {0};
    uint16_t i;
    uint8_t ret;

    for (i = 0U; i < samples; i++)
    {
        ret = ICM42688P_GetAccGyroRawBurst(p_i2c, DevAddr);
        if (ret != 0U)
            return 1U;

        acc_sum[0] += icm42688p_raw.acc[0];
        acc_sum[1] += icm42688p_raw.acc[1];
        acc_sum[2] += icm42688p_raw.acc[2];
        gyro_sum[0] += icm42688p_raw.gyro[0];
        gyro_sum[1] += icm42688p_raw.gyro[1];
        gyro_sum[2] += icm42688p_raw.gyro[2];

        osDelay(ICM42688P_CALIB_DELAY_MS);
    }

    float acc_scale = get_acc_scale();
    int16_t acc_avg[3] = {
        (int16_t)(acc_sum[0] / samples),
        (int16_t)(acc_sum[1] / samples),
        (int16_t)(acc_sum[2] / samples) - (int16_t)acc_scale
    };
    int16_t gyro_avg[3] = {
        (int16_t)(gyro_sum[0] / samples),
        (int16_t)(gyro_sum[1] / samples),
        (int16_t)(gyro_sum[2] / samples)
    };

    int16_t acc_comp[3] = {-acc_avg[0], -acc_avg[1], -acc_avg[2]};
    int16_t gyro_comp[3] = {-gyro_avg[0], -gyro_avg[1], -gyro_avg[2]};

    // 限幅，符合寄存器范围
    for (i = 0; i < 3; i++)
    {
        if (gyro_comp[i] > 2047) gyro_comp[i] = 2047;
        if (gyro_comp[i] < -2048) gyro_comp[i] = -2048;
        if (acc_comp[i] > (int16_t)acc_scale) acc_comp[i] = (int16_t)acc_scale;
        if (acc_comp[i] < -(int16_t)acc_scale) acc_comp[i] = -(int16_t)acc_scale;
    }

    // 切换到BANK4（Offset寄存器所在页）
    ret = I2C_WriteReg(p_i2c, DevAddr, ICM42688P_REG_BANK_SEL, 0x04);
    if (ret != 0U) return 1U;

    uint8_t reg_val;
    // Gyro X OFFSET_USER0 + OFFSET_USER1
    ret = I2C_WriteReg(p_i2c, DevAddr, ICM42688P_REG_OFFSET_USER0, (uint8_t)(gyro_comp[0] & 0xFF));
    if (ret != 0U) return 1U;
    reg_val = (uint8_t)((gyro_comp[0] >> 8) & 0x0F);
    ret = I2C_WriteReg(p_i2c, DevAddr, ICM42688P_REG_OFFSET_USER1, reg_val);
    if (ret != 0U) return 1U;

    // Gyro Y OFFSET_USER1(高4bit) + OFFSET_USER2
    reg_val = (uint8_t)(I2C_ReadReg(p_i2c, DevAddr, ICM42688P_REG_OFFSET_USER1) & 0x0F);
    reg_val |= (uint8_t)((gyro_comp[1] >> 8) & 0xF0);
    ret = I2C_WriteReg(p_i2c, DevAddr, ICM42688P_REG_OFFSET_USER1, reg_val);
    if (ret != 0U) return 1U;
    ret = I2C_WriteReg(p_i2c, DevAddr, ICM42688P_REG_OFFSET_USER2, (uint8_t)(gyro_comp[1] & 0xFF));
    if (ret != 0U) return 1U;

    // Gyro Z OFFSET_USER3 + OFFSET_USER4低4bit
    ret = I2C_WriteReg(p_i2c, DevAddr, ICM42688P_REG_OFFSET_USER3, (uint8_t)(gyro_comp[2] & 0xFF));
    if (ret != 0U) return 1U;
    reg_val = (uint8_t)((gyro_comp[2] >> 8) & 0x0F);
    ret = I2C_WriteReg(p_i2c, DevAddr, ICM42688P_REG_OFFSET_USER4, reg_val);
    if (ret != 0U) return 1U;

    // Acc X OFFSET_USER4高4bit + OFFSET_USER5
    reg_val = (uint8_t)(I2C_ReadReg(p_i2c, DevAddr, ICM42688P_REG_OFFSET_USER4) & 0x0F);
    reg_val |= (uint8_t)((acc_comp[0] >> 8) & 0xF0);
    ret = I2C_WriteReg(p_i2c, DevAddr, ICM42688P_REG_OFFSET_USER4, reg_val);
    if (ret != 0U) return 1U;
    ret = I2C_WriteReg(p_i2c, DevAddr, ICM42688P_REG_OFFSET_USER5, (uint8_t)(acc_comp[0] & 0xFF));
    if (ret != 0U) return 1U;

    // Acc Y OFFSET_USER6 + OFFSET_USER7低4bit
    ret = I2C_WriteReg(p_i2c, DevAddr, ICM42688P_REG_OFFSET_USER6, (uint8_t)(acc_comp[1] & 0xFF));
    if (ret != 0U) return 1U;
    reg_val = (uint8_t)((acc_comp[1] >> 8) & 0x0F);
    ret = I2C_WriteReg(p_i2c, DevAddr, ICM42688P_REG_OFFSET_USER7, reg_val);
    if (ret != 0U) return 1U;

    // Acc Z OFFSET_USER7高4bit + OFFSET_USER8
    reg_val = (uint8_t)(I2C_ReadReg(p_i2c, DevAddr, ICM42688P_REG_OFFSET_USER7) & 0x0F);
    reg_val |= (uint8_t)((acc_comp[2] >> 8) & 0xF0);
    ret = I2C_WriteReg(p_i2c, DevAddr, ICM42688P_REG_OFFSET_USER7, reg_val);
    if (ret != 0U) return 1U;
    ret = I2C_WriteReg(p_i2c, DevAddr, ICM42688P_REG_OFFSET_USER8, (uint8_t)(acc_comp[2] & 0xFF));
    if (ret != 0U) return 1U;

    // 切回Bank0
    ret = I2C_WriteReg(p_i2c, DevAddr, ICM42688P_REG_BANK_SEL, 0x00);
    if (ret != 0U) return 1U;

    // 缓存零偏数值
    for (i = 0; i < 3; i++)
    {
        icm42688p_offset.acc_offset[i] = (float)acc_comp[i] / acc_scale;
        icm42688p_offset.gyro_offset[i] = (float)gyro_comp[i] / get_gyro_scale();
    }

    return 0U;
}

/**
 * @brief 标准温度原始值转换（Burst读取16bit温度）
 * @param raw_temp 16位原始温度
 * @retval 温度 ℃
 */
float ICM42688P_ConvTemp(int16_t raw_temp)
{
    return 25.0f + ((float)raw_temp ) / 132.48f;
}

/**
 * @brief FIFO数据包内8bit温度转换
 * @param raw_temp FIFO读出的8位截断温度
 * @retval 温度 ℃
 */
float ICM42688P_ConvFifoTemp(int16_t raw_temp)
{
    uint8_t fifo_temp = (uint8_t)raw_temp;
    return 25.0f + ((float)(int8_t)fifo_temp) / 2.07f;
}

/*************************** 获取补偿后物理量 ***************************/
float ICM42688P_GetGyroXData(void)
{
    return ((float)icm42688p_raw.gyro[0] / get_gyro_scale()) - icm42688p_offset.gyro_offset[0];
}
float ICM42688P_GetGyroYData(void)
{
    return ((float)icm42688p_raw.gyro[1] / get_gyro_scale()) - icm42688p_offset.gyro_offset[1];
}
float ICM42688P_GetGyroZData(void)
{
    return ((float)icm42688p_raw.gyro[2] / get_gyro_scale()) - icm42688p_offset.gyro_offset[2];
}

float ICM42688P_GetAccXData(void)
{
    return ((float)icm42688p_raw.acc[0] / get_acc_scale()) - icm42688p_offset.acc_offset[0];
}
float ICM42688P_GetAccYData(void)
{
    return ((float)icm42688p_raw.acc[1] / get_acc_scale()) - icm42688p_offset.acc_offset[1];
}
float ICM42688P_GetAccZData(void)
{
    return ((float)icm42688p_raw.acc[2] / get_acc_scale()) - icm42688p_offset.acc_offset[2];
}

float ICM42688P_GetTempData(void)
{
    return ICM42688P_ConvTemp(icm42688p_raw.temp);
}

/*************************** 参数读取接口 ***************************/
uint8_t ICM42688P_GetAccRange(void)
{
    return icm42688p_acc_range;
}
uint8_t ICM42688P_GetGyroRange(void)
{
    return icm42688p_gyro_range;
}

/*************************** Parameterless refresh interface ***************************/
/**
 * @brief  Refresh the latest IMU sample with one register burst
 * @retval 0-success, 1-failure
 * @note   After calling, ICM42688P_GetData() returns latest values
 */
uint8_t ICM42688P_UpdateData(void)
{
    if (g_icm_i2c == NULL)
        return 1U;
    return ICM42688P_GetAccGyroRawBurst(g_icm_i2c, g_icm_addr);
}

/*************************** Aggregation getter interfaces ***************************/
/**
 * @brief  Get all calibrated physical quantities at once
 * @retval Pointer to icm42688p_data, units: acc(g), gyro(deg/s), temp(C)
 */
ICM42688P_Data_t *ICM42688P_GetData(void)
{
    float scale_acc  = get_acc_scale();
    float scale_gyro = get_gyro_scale();

    icm42688p_data.acc[0]  = ((float)icm42688p_raw.acc[0] / scale_acc) - icm42688p_offset.acc_offset[0];
    icm42688p_data.acc[1]  = ((float)icm42688p_raw.acc[1] / scale_acc) - icm42688p_offset.acc_offset[1];
    icm42688p_data.acc[2]  = ((float)icm42688p_raw.acc[2] / scale_acc) - icm42688p_offset.acc_offset[2];
    icm42688p_data.gyro[0] = ((float)icm42688p_raw.gyro[0] / scale_gyro) - icm42688p_offset.gyro_offset[0];
    icm42688p_data.gyro[1] = ((float)icm42688p_raw.gyro[1] / scale_gyro) - icm42688p_offset.gyro_offset[1];
    icm42688p_data.gyro[2] = ((float)icm42688p_raw.gyro[2] / scale_gyro) - icm42688p_offset.gyro_offset[2];
    icm42688p_data.temp     = ICM42688P_ConvTemp(icm42688p_raw.temp);

    return &icm42688p_data;
}

/**
 * @brief  Get pointer to raw ADC data buffer
 * @retval Pointer to icm42688p_raw
 */
ICM42688P_Raw_t *ICM42688P_GetRawData(void)
{
    return &icm42688p_raw;
}

/**
 * @brief  Get pointer to calibration offset buffer
 * @retval Pointer to icm42688p_offset
 */
ICM42688P_Offset_t *ICM42688P_GetOffset(void)
{
    return &icm42688p_offset;
}

/*************************** Complementary filter Euler angles ***************************/
/**
 * @brief  Calculate Euler angles using complementary filter
 * @retval Pointer to Euler angles structure (yaw/pitch/roll, in degrees)
 * @note   Yaw is gyro-only integration and will drift over time
 */
ICM42688P_Euler_angles *ICM42688P_Calculate_Euler_Angles(void)
{
    float dt = 1.0f / ICM42688P_SAMPLING_FREQUENCY;
    float acc_roll, acc_pitch;
    float acc_norm;
    float gyro_x, gyro_y, gyro_z;

    /* Get calibrated physical quantities */
    ICM42688P_Data_t *imu = ICM42688P_GetData();

    /* Accelerometer-derived roll and pitch, in degrees */
    acc_roll  = atan2f(-imu->acc[1], imu->acc[2]) * 57.29577951308232f;
    acc_norm  = sqrtf(imu->acc[1] * imu->acc[1] + imu->acc[2] * imu->acc[2]);
    acc_pitch = atan2f(imu->acc[0], acc_norm) * 57.29577951308232f;

    /* Gyro in deg/s */
    gyro_x = imu->gyro[0];
    gyro_y = imu->gyro[1];
    gyro_z = imu->gyro[2];

    /* Complementary filter: angle = alpha*(angle + gyro*dt) + (1-alpha)*acc_angle */
    icm42688p_euler_angles.roll  = ICM42688P_COMPLEMENTARY_FILTER_ALPHA
                                 * (icm42688p_euler_angles.roll + gyro_x * dt)
                                 + (1.0f - ICM42688P_COMPLEMENTARY_FILTER_ALPHA) * acc_roll;
    icm42688p_euler_angles.pitch = ICM42688P_COMPLEMENTARY_FILTER_ALPHA
                                 * (icm42688p_euler_angles.pitch + gyro_y * dt)
                                 + (1.0f - ICM42688P_COMPLEMENTARY_FILTER_ALPHA) * acc_pitch;
    /* Yaw: gyro integration only */
    icm42688p_euler_angles.yaw   = icm42688p_euler_angles.yaw + gyro_z * dt;

    return &icm42688p_euler_angles;
}

/**
 * @brief  Get latest complementary filter Euler angles
 * @retval Pointer to icm42688p_euler_angles (yaw/pitch/roll, in degrees)
 * @note   Must call ICM42688P_Calculate_Euler_Angles() periodically before use
 */
ICM42688P_Euler_angles *ICM42688P_Get_Euler_Angles(void)
{
    return &icm42688p_euler_angles;
}
