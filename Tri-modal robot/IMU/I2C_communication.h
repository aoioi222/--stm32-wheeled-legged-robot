#ifndef  I2C_COMMUNICATE
#define  I2C_COMMUNICATE

#include "stm32f4xx.h"
#include "rcs.h"
#include "Delay.h"	
#include "stm32f4xx_i2c.h"

#define RCS_I2C_DELAY_US  10 
#define RCS_I2C_TIMEOUT_MAX  1000

#define I2C_MAX_RETRY     2 
#define I2C_RESET_DELAY   1
#define I2C_RETRY_DELAY   5    // 重试间隔(ms)


typedef struct {
    GPIO_TypeDef* GPIOx;        // SCL/SDA所属GPIO端口	
    uint16_t      SCL_Pin;      // SCL引脚	
    uint16_t      SDA_Pin;      // SDA引脚		
    uint8_t       SlaveAddr;    // 从机地址	
} RCS_I2C_HandleTypeDef;
typedef enum {
    RCS_I2C_OK = 0,          // 成功
    RCS_I2C_ERR_ACK = 1,     // 无应答 （从机没回应）
    RCS_I2C_ERR_TIMEOUT = 2, // 超时   （总线卡死）
    RCS_I2C_ERR_NULL = 3,    // 空指针 （参数错误）
    RCS_I2C_ERR_BUSY = 4,    // 总线忙 （初始化失败）
    RCS_I2C_ERR_LEN = 5      // 长度为0（无效参数）
} RCS_I2C_ErrCode;

void I2C_General_SafetyHandler(RCS_I2C_HandleTypeDef* hi2c); 


RCS_I2C_ErrCode RCS_I2C_ResetBus(RCS_I2C_HandleTypeDef* hi2c);
RCS_I2C_ErrCode RCS_I2C_CheckBusIdle(RCS_I2C_HandleTypeDef* hi2c);


void RCS_I2C_Init(RCS_I2C_HandleTypeDef* hi2c, GPIO_TypeDef* GPIOx, uint16_t SCL_Pin, uint16_t SDA_Pin, uint8_t SlaveAddr);//初始化


void RCS_I2C_Start(RCS_I2C_HandleTypeDef* hi2c);
void RCS_I2C_Stop(RCS_I2C_HandleTypeDef* hi2c);
void RCS_I2C_Restart(RCS_I2C_HandleTypeDef* hi2c);//底层代码

void    RCS_I2C_SendByte(RCS_I2C_HandleTypeDef* hi2c, uint8_t Byte);//发送一个字节
uint8_t RCS_I2C_ReceiveByte(RCS_I2C_HandleTypeDef* hi2c);//接收一个字节

void RCS_I2C_SendAck(RCS_I2C_HandleTypeDef* hi2c, uint8_t AckBit);//发送应答
RCS_I2C_ErrCode RCS_I2C_ReceiveAck_Timeout(RCS_I2C_HandleTypeDef* hi2c);//接收应答

RCS_I2C_ErrCode RCS_I2C_WriteTo(RCS_I2C_HandleTypeDef* hi2c, uint8_t RegAddr, uint8_t Data);   //单字节指定地址写入
RCS_I2C_ErrCode RCS_I2C_ReadFrom(RCS_I2C_HandleTypeDef* hi2c, uint8_t RegAddr, uint8_t* pData);//单字节指定地址读取
RCS_I2C_ErrCode RCS_I2C_ReadNow(RCS_I2C_HandleTypeDef* hi2c, uint8_t* pData);                  //单字节当前地址读取
RCS_I2C_ErrCode RCS_I2C_WriteMultiTo(RCS_I2C_HandleTypeDef* hi2c, uint8_t RegAddr, uint8_t* pData, uint8_t Len);   //多字节指定地址写入
RCS_I2C_ErrCode RCS_I2C_ReadMultiFrom(RCS_I2C_HandleTypeDef* hi2c, uint8_t RegAddr, uint8_t* pData, uint8_t Len);





RCS_I2C_ErrCode I2C_General_ErrorHandler(RCS_I2C_HandleTypeDef* hi2c, RCS_I2C_ErrCode err_code,
                                    uint8_t op_type,uint8_t reg_addr,uint8_t* data_buf,uint8_t len); 



RCS_I2C_ErrCode I2C_General_Read(RCS_I2C_HandleTypeDef* hi2c,uint8_t reg_addr,uint8_t* data_buf,uint8_t len); 
RCS_I2C_ErrCode I2C_General_Write(RCS_I2C_HandleTypeDef* hi2c,uint8_t reg_addr,uint8_t* data_buf,uint8_t len); 


#endif
