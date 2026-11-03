#ifndef __HARDWARE_I2C_H
#define __HARDWARE_I2C_H

#include "stm32f4xx.h"
#include "rcs.h"
#include "stm32f4xx_i2c.h"
/************************** 宏定义 **************************/
#define I2C_DEFAULT_TIMEOUT_MS         10U
#define I2C_DEFAULT_TIMEOUT_US         (I2C_DEFAULT_TIMEOUT_MS * 1000U)
#define I2C_WAIT_POLL_US               5U
#define I2C_DELAY_BASE_MS              1U  /* 已不再使用，保留兼容 */
// 复位时序的时钟半周期延时，总线异步时序只需微秒级，用毫秒会把一次复位拖到几十毫秒
#define I2C_DELAY_BASE_US              5U
#define I2C_RESET_BUS_CLOCK_NUM        9U

#define I2C_OP_SUCCESS                 0U
#define I2C_OP_FAIL                    1U
#define I2C_READ_FAIL_VAL              0xFFU

#define I2C_WRITE                      1U
#define I2C_READ                       0U
#define I2C_ACK_ENABLE                 1U
#define I2C_ACK_DISABLE                0U

#define I2C_STANDARD_SPEED             100000U
#define I2C_FAST_SPEED                 400000U

/************************** 枚举定义 **************************/
typedef enum
{
    I2C1_t = 0,
    I2C2_t,
    I2C3_t
} RCS_I2C_Type;

/************************** 结构体定义 **************************/
typedef struct
{
    I2C_TypeDef*  I2Cx;
    GPIO_TypeDef* GPIOx_SCL;
    GPIO_TypeDef* GPIOx_SDA;
    uint16_t      SCL_Pin;
    uint16_t      SDA_Pin;
    uint32_t      ClockSpeed;
} RCS_I2C_Hardware_TypeDef;

/************************** 函数声明 **************************/
void RCS_I2C_Hardware_Init(RCS_I2C_Type I2C_TYPE, RCS_I2C_Hardware_TypeDef *Hi2c);

uint8_t I2C_WriteReg(I2C_TypeDef* I2Cx, uint8_t DevAddr, uint8_t RegAddr, uint8_t Data);

uint8_t I2C_WriteRegs(I2C_TypeDef* I2Cx, uint8_t DevAddr, uint8_t RegAddr, uint8_t Len, uint8_t* DataBuf);

uint8_t I2C_ReadRegs(I2C_TypeDef* I2Cx, uint8_t DevAddr, uint8_t RegAddr, uint8_t Len, uint8_t* DataBuf);

uint8_t I2C_ReadReg(I2C_TypeDef* I2Cx, uint8_t DevAddr, uint8_t RegAddr);

uint8_t I2C_Start(I2C_TypeDef* I2Cx);

uint8_t I2C_SendAddr(I2C_TypeDef* I2Cx, uint8_t DevAddr, uint8_t Write);

uint8_t I2C_SendByte(I2C_TypeDef* I2Cx, uint8_t Data);

uint8_t I2C_RecvByte(I2C_TypeDef* I2Cx, uint8_t Ack);

void I2C_Stop(I2C_TypeDef* I2Cx);

void I2C_ResetBus(I2C_TypeDef* I2Cx, GPIO_TypeDef* GPIOx_SCL, uint16_t SCL_Pin, GPIO_TypeDef* GPIOx_SDA, uint16_t SDA_Pin);

uint8_t I2C_CheckDevice(I2C_TypeDef* I2Cx, uint8_t DevAddr);

#endif /* __HARDWARE_I2C_H */
