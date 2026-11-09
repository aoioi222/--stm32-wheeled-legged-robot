#ifndef ICM_42688_P_REGADDRESS_H_
#define ICM_42688_P_REGADDRESS_H_

/*************************** 设备地址定义 ***************************/
#define ICM42688P_DEVICE_ADDR 0x68U             // I2C地址（ADDR接GND）
#define ICM42688P_DEVICE_ADDR_ALT 0x69U         // 备用地址（ADDR接VDD）

/*************************** 核心配置寄存器地址 ***************************/
#define ICM42688P_REG_DEVICE_CONFIG 0x11U     // 设备配置寄存器（
#define ICM42688P_REG_DRIVE_CONFIG  0x13U     // 驱动配置寄存器
#define ICM42688P_REG_INTF_CONFIG0  0x4CU     // 接口配置0寄存器
#define ICM42688P_REG_PWR_MGMT0     0x4EU      // 电源管理寄存器（替代原PWR_MGMT_1）
#define ICM42688P_REG_GYRO_CONFIG0  0x4FU     // 陀螺仪配置0（量程+ODR）
#define ICM42688P_REG_ACCEL_CONFIG0 0x50U     // 加速度计配置0（量程+ODR）
#define ICM42688P_REG_GYRO_CONFIG1  0x51U     // 陀螺仪配置1（滤波）
#define ICM42688P_REG_GYRO_ACCEL_CONFIG0 0x52U // 陀螺仪+加速度计配置寄存器
#define ICM42688P_REG_ACCEL_CONFIG1 0x53U     // 加速度计配置1（滤波）
#define ICM42688P_REG_FIFO_CONFIG1   0x5FU     // FIFO配置1寄存器
#define ICM42688P_REG_FIFO_CONFIG2   0x60U     // FIFO配置2寄存器 
#define ICM42688P_REG_FIFO_CONFIG3    0x61U     // FIFO配置3寄存器
#define ICM42688P_REG_INT_CONFIG0    0x63U     // 中断配置0寄存器
#define ICM42688P_REG_INT_CONFIG1   0x64U     // 中断配置1寄存器
#define ICM42688P_REG_WHO_AM_I      0x75U     // 设备ID寄存器
#define ICM42688P_REG_BANK_SEL      0x76U          // 寄存器银行选择寄存器（默认值0x00）
/*************************** 数据寄存器地址 ***************************/
#define ICM42688P_REG_BURST_START  0x1DU      // Burst读取起始地址（温度+加速度+陀螺仪）
#define ICM42688P_REG_TEMP_OUT_H   0x1DU      // 温度高字节
#define ICM42688P_REG_TEMP_OUT_L   0x1EU      // 温度低字节
#define ICM42688P_REG_ACCEL_XOUT_H 0x1FU      // 加速度X高字节
#define ICM42688P_REG_ACCEL_XOUT_L 0x20U      // 加速度X低字节
#define ICM42688P_REG_ACCEL_YOUT_H 0x21U      // 加速度Y高字节
#define ICM42688P_REG_ACCEL_YOUT_L 0x22U      // 加速度Y低字节
#define ICM42688P_REG_ACCEL_ZOUT_H 0x23U      // 加速度Z高字节
#define ICM42688P_REG_ACCEL_ZOUT_L 0x24U      // 加速度Z低字节
#define ICM42688P_REG_GYRO_XOUT_H  0x25U      // 陀螺仪X高字节
#define ICM42688P_REG_GYRO_XOUT_L  0x26U      // 陀螺仪X低字节
#define ICM42688P_REG_GYRO_YOUT_H  0x27U      // 陀螺仪Y高字节
#define ICM42688P_REG_GYRO_YOUT_L  0x28U      // 陀螺仪Y低字节
#define ICM42688P_REG_GYRO_ZOUT_H  0x29U      // 陀螺仪Z高字节
#define ICM42688P_REG_GYRO_ZOUT_L  0x2AU      // 陀螺仪Z低字节

/*************************** 零偏校准寄存器地址 ***************************/
#define ICM42688P_REG_OFFSET_USER0 0x77U // 零偏校准寄存器0
#define ICM42688P_REG_OFFSET_USER1 0x78U // 零偏校准寄存器1
#define ICM42688P_REG_OFFSET_USER2 0x79U // 零偏校准寄存器2
#define ICM42688P_REG_OFFSET_USER3 0x7AU // 零偏校准寄存器3
#define ICM42688P_REG_OFFSET_USER4 0x7BU // 零偏校准寄存器4
#define ICM42688P_REG_OFFSET_USER5 0x7CU // 零偏校准寄存器5
#define ICM42688P_REG_OFFSET_USER6 0x7DU // 零偏校准寄存器6
#define ICM42688P_REG_OFFSET_USER7 0x7EU // 零偏校准寄存器7
#define ICM42688P_REG_OFFSET_USER8 0x7FU // 零偏校准寄存器8

#endif
