#include "I2C_communication.h"
#include "robot_config.h"

/* I2C 错误诊断打印：错误分支触发，异常时会在控制周期内连续输出，默认关闭 */
#if ROBOT_DEBUG_PRINT
#define LOG_I2C(...) printf(__VA_ARGS__)
#else
#define LOG_I2C(...) ((void)0)
#endif

	
	
	
/*-------------------------------------软件I2C通信---------------------------------------*/
// 通用安全兜底函数（根据实际机器人修改）
void I2C_General_SafetyHandler(RCS_I2C_HandleTypeDef* hi2c) 
{

}
/**
 * @brief  指定地址写一个字节（核心调用函数）
 * @param  hi2c: 软件I2C句柄指针
 * @param  RegAddr: 寄存器地址
 * @param  Data: 要写入的数据
 * @retval 0=成功，1=失败（无应答）
 */
RCS_I2C_ErrCode RCS_I2C_WriteTo(RCS_I2C_HandleTypeDef* hi2c, uint8_t RegAddr, uint8_t Data)
{
    RCS_I2C_ErrCode  ack;

    RCS_I2C_Start(hi2c);                                 // 起始信号
    RCS_I2C_SendByte(hi2c, hi2c->SlaveAddr);             // 发送从机写地址（最低位=0）
    ack = RCS_I2C_ReceiveAck_Timeout(hi2c);
    if(ack != RCS_I2C_OK) { RCS_I2C_Stop(hi2c); return ack; }

    RCS_I2C_SendByte(hi2c, RegAddr);                     // 发送寄存器地址
    ack = RCS_I2C_ReceiveAck_Timeout(hi2c);
    if(ack != RCS_I2C_OK) { RCS_I2C_Stop(hi2c); return ack; }

    RCS_I2C_SendByte(hi2c, Data);                        // 发送数据
    ack = RCS_I2C_ReceiveAck_Timeout(hi2c);
    if(ack != RCS_I2C_OK) { RCS_I2C_Stop(hi2c); return ack; }

    RCS_I2C_Stop(hi2c);                                  // 停止信号
    return RCS_I2C_OK;
}

/**
 * @brief  指定地址读一个字节（核心调用函数）
 * @param  hi2c: 软件I2C句柄指针
 * @param  RegAddr: 寄存器地址
 * @param  pData: 数据缓冲区指针，存放读取到的数据
 * @retval RCS_I2C_ErrCode：0=成功，其他=对应错误
 */
RCS_I2C_ErrCode RCS_I2C_ReadFrom(RCS_I2C_HandleTypeDef* hi2c, uint8_t RegAddr, uint8_t* pData)
{
    RCS_I2C_ErrCode ack;

    // 空指针校验
    if(hi2c == NULL || pData == NULL) return RCS_I2C_ERR_NULL;

    // 发送寄存器地址（伪写操作）
    RCS_I2C_Start(hi2c);                                 // 起始信号
    RCS_I2C_SendByte(hi2c, hi2c->SlaveAddr);             // 发送从机写地址
    ack = RCS_I2C_ReceiveAck_Timeout(hi2c);
    if(ack != RCS_I2C_OK) { RCS_I2C_Stop(hi2c); return ack; }

    RCS_I2C_SendByte(hi2c, RegAddr);                     // 发送寄存器地址
    ack = RCS_I2C_ReceiveAck_Timeout(hi2c);
    if(ack != RCS_I2C_OK) { RCS_I2C_Stop(hi2c); return ack; }

    // 重复起始，读取数据
    RCS_I2C_Restart(hi2c);                               // 重复起始信号（替换原Start，符合标准协议）
    RCS_I2C_SendByte(hi2c, hi2c->SlaveAddr | 0x01);      // 发送从机读地址（最低位=1）
    ack = RCS_I2C_ReceiveAck_Timeout(hi2c);
    if(ack != RCS_I2C_OK) { RCS_I2C_Stop(hi2c); return ack; }

    *pData = RCS_I2C_ReceiveByte(hi2c);                  // 接收数据
    RCS_I2C_SendAck(hi2c, 1);                            // 发送非应答（读单个字节）

    RCS_I2C_Stop(hi2c);                                  // 停止信号
    return RCS_I2C_OK;
}
/**
 * @brief  当前地址读一个字节（核心调用函数）
 * @retval 读取到的数据（0xFF=失败）
 */
RCS_I2C_ErrCode RCS_I2C_ReadNow(RCS_I2C_HandleTypeDef* hi2c, uint8_t* pData)
{
    RCS_I2C_ErrCode ack;

    if(pData == NULL) return RCS_I2C_ERR_NULL; // 原返回1 → 改为RCS_I2C_ERR_NULL

    RCS_I2C_Start(hi2c);
    RCS_I2C_SendByte(hi2c, hi2c->SlaveAddr | 0x01);
    ack = RCS_I2C_ReceiveAck_Timeout(hi2c);
    if(ack != RCS_I2C_OK) { RCS_I2C_Stop(hi2c); return ack; }

    *pData = RCS_I2C_ReceiveByte(hi2c);
    RCS_I2C_SendAck(hi2c, 1);
    RCS_I2C_Stop(hi2c);

    return RCS_I2C_OK;
}

/**
 * @brief  指定地址开始写多个字节
 * @param  RegAddr: 要写入的起始寄存器地址
 * @param  pData：数据缓冲区指针，存放要写入的多字节数据（需提前分配内存）
 * @param  Len：要写入的字节数（需与pData缓冲区长度匹配，不能为0）
 * @retval RCS_I2C_ErrCode
 */
RCS_I2C_ErrCode RCS_I2C_WriteMultiTo(RCS_I2C_HandleTypeDef* hi2c, uint8_t RegAddr, uint8_t* pData, uint8_t Len)
{
    RCS_I2C_ErrCode ack; // 应答状态（枚举错误码）
    uint8_t i;

    if(hi2c == NULL || pData == NULL) {
        return RCS_I2C_ERR_NULL; // 返回空指针错误码
    }

    if(Len == 0) {
        return RCS_I2C_ERR_LEN; // 返回长度错误码
    }

    RCS_I2C_Start(hi2c);
    RCS_I2C_SendByte(hi2c, hi2c->SlaveAddr);
    // 接收应答
    ack = RCS_I2C_ReceiveAck_Timeout(hi2c);
    if(ack != RCS_I2C_OK) { // 应答失败/超时
        RCS_I2C_Stop(hi2c); // 发送停止信号，避免总线挂起
        return ack;         
    }

    // 发送起始寄存器地址
    RCS_I2C_SendByte(hi2c, RegAddr);
    ack = RCS_I2C_ReceiveAck_Timeout(hi2c);
    if(ack != RCS_I2C_OK) {
        RCS_I2C_Stop(hi2c);
        return ack;
    }

    // 循环发送多字节数据
    for(i = 0; i < Len; i++) {
        // 发送当前字节
        RCS_I2C_SendByte(hi2c, pData[i]);
        // 每个字节发送后都要检测应答（关键：避免单字节失败导致后续数据错误）
        ack = RCS_I2C_ReceiveAck_Timeout(hi2c);
        if(ack != RCS_I2C_OK) {
            RCS_I2C_Stop(hi2c); // 立即停止，终止异常通信
            return ack;
        }
    }

    //I2C停止时序（结束通信）
    RCS_I2C_Stop(hi2c);

    // 所有字节写入成功
    return RCS_I2C_OK;
}
/**
 * @brief  指定地址开始读多个字节（核心调用函数）
 * @param  pData：数据缓冲区指针，用于存放读取到的多字节数据（需提前分配内存）
 * @param  Len：要读取的字节数（需与 pData 缓冲区长度匹配，不能为 0）
 * @retval 读取到的数据（0xFF=失败）
 */
RCS_I2C_ErrCode RCS_I2C_ReadMultiFrom(RCS_I2C_HandleTypeDef* hi2c, uint8_t RegAddr, uint8_t* pData, uint8_t Len)
{
    RCS_I2C_ErrCode ack;
    uint8_t i;

    // 原返回1 → 映射为枚举
    if(Len == 0) return RCS_I2C_ERR_LEN;
    if(pData == NULL) return RCS_I2C_ERR_NULL;

    RCS_I2C_Start(hi2c);
    RCS_I2C_SendByte(hi2c, hi2c->SlaveAddr);
    ack = RCS_I2C_ReceiveAck_Timeout(hi2c);
    if(ack != RCS_I2C_OK) { RCS_I2C_Stop(hi2c); return ack; }

    RCS_I2C_SendByte(hi2c, RegAddr);
    ack = RCS_I2C_ReceiveAck_Timeout(hi2c);
    if(ack != RCS_I2C_OK) { RCS_I2C_Stop(hi2c); return ack; }

    RCS_I2C_Restart(hi2c);
    RCS_I2C_SendByte(hi2c, hi2c->SlaveAddr | 0x01);
    ack = RCS_I2C_ReceiveAck_Timeout(hi2c);
    if(ack != RCS_I2C_OK) { RCS_I2C_Stop(hi2c); return ack; }

    for(i = 0; i < Len; i++)
    {
        pData[i] = RCS_I2C_ReceiveByte(hi2c);
        if(i == Len - 1)
            RCS_I2C_SendAck(hi2c, 1);
        else
            RCS_I2C_SendAck(hi2c, 0);
    }

    RCS_I2C_Stop(hi2c);
    return RCS_I2C_OK;
}
/**
 * @brief  I2C总线复位（解决总线卡死问题）
 * @param  hi2c: 软件I2C句柄指针
 * @retval RCS_I2C_ErrCode：0=复位成功，其他=对应错误
 */
RCS_I2C_ErrCode RCS_I2C_ResetBus(RCS_I2C_HandleTypeDef* hi2c)
{
    if(hi2c == NULL) return RCS_I2C_ERR_NULL;

    GPIO_InitTypeDef GPIO_InitStruct;
    // 临时切换为推挽输出，强制拉高SCL/SDA
    GPIO_InitStruct.GPIO_Pin = hi2c->SCL_Pin | hi2c->SDA_Pin;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; // 推挽输出
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(hi2c->GPIOx, &GPIO_InitStruct);

    // 强制拉高SCL/SDA，释放总线
    GPIO_SetBits(hi2c->GPIOx, hi2c->SCL_Pin);
    GPIO_SetBits(hi2c->GPIOx, hi2c->SDA_Pin);
    delay_ms(1);

    // 恢复为开漏输出
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
    GPIO_Init(hi2c->GPIOx, &GPIO_InitStruct);
	
    GPIO_SetBits(hi2c->GPIOx, hi2c->SCL_Pin);
    GPIO_SetBits(hi2c->GPIOx, hi2c->SDA_Pin);
    delay_us(RCS_I2C_DELAY_US); // 短延时稳定电平
    // 检测总线是否恢复空闲
    return RCS_I2C_CheckBusIdle(hi2c);
}
/*-----------------------------底层控制函数------------------------------*/
// SCL线电平控制
static void RCS_I2C_W_SCL(RCS_I2C_HandleTypeDef* hi2c, uint8_t BitValue)
{
    if(BitValue == 1)
        GPIO_SetBits(hi2c->GPIOx, hi2c->SCL_Pin);
    else
        GPIO_ResetBits(hi2c->GPIOx, hi2c->SCL_Pin);
    delay_us(RCS_I2C_DELAY_US); // 固定延时
}

// SDA线电平控制
static void RCS_I2C_W_SDA(RCS_I2C_HandleTypeDef* hi2c, uint8_t BitValue)
{
    if(BitValue == 1)
        GPIO_SetBits(hi2c->GPIOx, hi2c->SDA_Pin);
    else
        GPIO_ResetBits(hi2c->GPIOx, hi2c->SDA_Pin);
    delay_us(RCS_I2C_DELAY_US);
}

// SDA线电平读取
static uint8_t RCS_I2C_R_SDA(RCS_I2C_HandleTypeDef* hi2c)
{
    uint8_t BitValue;
    GPIO_InitTypeDef GPIO_InitStruct;

    // 临时切换SDA为输入模式
    GPIO_InitStruct.GPIO_Pin = hi2c->SDA_Pin;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(hi2c->GPIOx, &GPIO_InitStruct);

    // 读取SDA电平
    BitValue = GPIO_ReadInputDataBit(hi2c->GPIOx, hi2c->SDA_Pin);
    delay_us(RCS_I2C_DELAY_US);

    // 恢复SDA为开漏输出
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(hi2c->GPIOx, &GPIO_InitStruct);

    return BitValue;
}

/*--------------------------- 公开接口函数 ------------------------------*/

/**
 * @brief  软件I2C初始化前空闲状态检查
 * @retval 无
 */

RCS_I2C_ErrCode RCS_I2C_CheckBusIdle(RCS_I2C_HandleTypeDef* hi2c)
{
    // 临时切换为输入模式检测
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = hi2c->SCL_Pin | hi2c->SDA_Pin;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(hi2c->GPIOx, &GPIO_InitStruct);

    // SCL和SDA都为高才是空闲
    if((GPIO_ReadInputDataBit(hi2c->GPIOx, hi2c->SCL_Pin) == 1) && 
       (GPIO_ReadInputDataBit(hi2c->GPIOx, hi2c->SDA_Pin) == 1))
        return RCS_I2C_OK; // 空闲
    else
        return RCS_I2C_ERR_BUSY; // 忙
}
/**
 * @brief  软件I2C初始化
 * @param  GPIOx: SCL/SDA所属GPIO端口
 * @param  SCL_Pin: SCL引脚
 * @param  SDA_Pin: SDA引脚
 * @param  SlaveAddr: 从机地址（如MPU6050=0x68，24C02=0xA0）
 * @retval 无
 */
void RCS_I2C_Init(RCS_I2C_HandleTypeDef* hi2c, GPIO_TypeDef* GPIOx, uint16_t SCL_Pin, uint16_t SDA_Pin, uint8_t SlaveAddr)
{
    hi2c->GPIOx = GPIOx;
    hi2c->SCL_Pin = SCL_Pin;
    hi2c->SDA_Pin = SDA_Pin;
    hi2c->SlaveAddr = SlaveAddr;

    // 开启对应GPIO时钟
    uint32_t gpio_offset = (uint32_t)GPIOx - (uint32_t)GPIOA;
    if(gpio_offset < 0x1C00)
        RCC->AHB1ENR |= (1 << (gpio_offset / 0x400));

    // 配置SCL/SDA为开漏输出+上拉
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = SCL_Pin | SDA_Pin;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;      // 默认输出（发送数据）
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;     // 开漏输出
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;       // 上拉电阻
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;  
    GPIO_Init(GPIOx, &GPIO_InitStruct);

    // 初始化为I2C空闲状态
    GPIO_SetBits(GPIOx, SCL_Pin | SDA_Pin);
}

/**
 * @brief  I2C起始时序
 */
void RCS_I2C_Start(RCS_I2C_HandleTypeDef* hi2c)
{
    RCS_I2C_W_SDA(hi2c, 1); // SDA先置高
    RCS_I2C_W_SCL(hi2c, 1); // SCL置高
    RCS_I2C_W_SDA(hi2c, 0); // SDA拉低
    RCS_I2C_W_SCL(hi2c, 0); // SCL拉低，准备发送数据
}


/**
 * @brief  I2C结束时序
 */
void RCS_I2C_Stop(RCS_I2C_HandleTypeDef* hi2c)
{
    RCS_I2C_W_SDA(hi2c, 0); // SDA先置低
    RCS_I2C_W_SCL(hi2c, 1); // SCL置高
    RCS_I2C_W_SDA(hi2c, 1); // SDA拉高（结束信号）
}
/**
 * @brief  I2C多字节restart时序
 */
void RCS_I2C_Restart(RCS_I2C_HandleTypeDef* hi2c)
{
    RCS_I2C_W_SDA(hi2c, 1);
    RCS_I2C_W_SCL(hi2c, 1);
    RCS_I2C_W_SDA(hi2c, 0);
    RCS_I2C_W_SCL(hi2c, 0);
}
/**
 * @brief  发送一个字节
 * @param  Byte: 要发送的字节
 */
void RCS_I2C_SendByte(RCS_I2C_HandleTypeDef* hi2c, uint8_t Byte)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        // 高位先行，发送当前位
        RCS_I2C_W_SDA(hi2c, (Byte & (0x80 >> i)) ? 1 : 0);
        RCS_I2C_W_SCL(hi2c, 1); // SCL置高，从机采样
        RCS_I2C_W_SCL(hi2c, 0);
    }
}

/**
 * @brief  接收一个字节
 * @retval 接收到的字节
 */
uint8_t RCS_I2C_ReceiveByte(RCS_I2C_HandleTypeDef* hi2c)
{
    uint8_t i, Byte = 0x00;
    RCS_I2C_W_SDA(hi2c, 1); // 释放SDA，由从机控制电平
    for (i = 0; i < 8; i++)
    {
        RCS_I2C_W_SCL(hi2c, 1); // SCL置高，读取当前位
        if (RCS_I2C_R_SDA(hi2c))
            Byte |= (0x80 >> i); // 高位先行，置位对应位
        RCS_I2C_W_SCL(hi2c, 0); // SCL拉低，准备下一位
    }
    return Byte;
}

/**
 * @brief  发送应答位（0=应答，1=非应答）
 * @param  AckBit: 应答位（0/1）
 */
void RCS_I2C_SendAck(RCS_I2C_HandleTypeDef* hi2c, uint8_t AckBit)
{
    RCS_I2C_W_SDA(hi2c, AckBit);
    RCS_I2C_W_SCL(hi2c, 1); // SCL置高，从机采样
    RCS_I2C_W_SCL(hi2c, 0); // SCL拉低
}

/**
 * @brief  接收应答位
 * @retval 0=应答（成功），1=非应答（失败）
 */
uint8_t RCS_I2C_ReceiveAck_Timeout(RCS_I2C_HandleTypeDef* hi2c)
{
    uint8_t AckBit;
    uint32_t timeout = RCS_I2C_TIMEOUT_MAX;

    RCS_I2C_W_SDA(hi2c, 1);
    RCS_I2C_W_SCL(hi2c, 1);

    // 等待从机拉低SDA，或超时退出
    while((GPIO_ReadInputDataBit(hi2c->GPIOx, hi2c->SDA_Pin) == 1) && (timeout > 0))
    {
        timeout--;
        delay_us(1);
    }

    AckBit = RCS_I2C_R_SDA(hi2c);
    RCS_I2C_W_SCL(hi2c, 0);

    // 超时
    if(timeout == 0) return RCS_I2C_ERR_TIMEOUT;
    return AckBit ? RCS_I2C_ERR_ACK : RCS_I2C_OK;
}

/**
 * @brief  通用I2C错误处理函数
 * @param  err_code:   错误码（如超时、无应答、总线忙等）
 * @param  op_type:    0=读操作，1=写操作
 * @param  reg_addr:   寄存器地址
 * @param  data_buf:   数据缓冲区
 * @param  len:        数据长度
 * @retval 处理后的错误码：OK=重试成功，其他=重试失败
 * @brief  功能：自动处理I2C错误（复位总线/发停止信号），重试2次；失败则触发安全保护；printf配合串口调试
 */
RCS_I2C_ErrCode I2C_General_ErrorHandler(RCS_I2C_HandleTypeDef* hi2c, RCS_I2C_ErrCode err_code,
                                    uint8_t op_type,uint8_t reg_addr,uint8_t* data_buf,uint8_t len) 
{
    RCS_I2C_ErrCode handle_err = err_code;
    uint8_t retry = 0;

    // 空指针防护（通用）
    if(hi2c == NULL || data_buf == NULL) {
        LOG_I2C("I2C通用错误：空指针（句柄/数据缓冲区）\r\n");
        return RCS_I2C_ERR_NULL;
    }

    // 按错误码执行【通用I2C总线控制代码】
    while(retry < I2C_MAX_RETRY && handle_err != RCS_I2C_OK) {
        switch(handle_err) {
            // 1. 超时错误：复位总线 + 重试
            case RCS_I2C_ERR_TIMEOUT:
                LOG_I2C("I2C设备[%02X]：超时，执行总线复位...\r\n", hi2c->SlaveAddr);
                RCS_I2C_ResetBus(hi2c);          // 核心控制：复位总线
                delay_ms(I2C_RESET_DELAY);       // 延时稳定
                break;

            // 2. 总线忙：复位总线 + 重新检测
            case RCS_I2C_ERR_BUSY:
                LOG_I2C("I2C设备[%02X]：总线忙，执行总线复位...\r\n", hi2c->SlaveAddr);
                RCS_I2C_ResetBus(hi2c);          // 核心控制：复位总线
                if(RCS_I2C_CheckBusIdle(hi2c) == RCS_I2C_OK) {
                    LOG_I2C("I2C设备[%02X]：总线已恢复空闲\r\n", hi2c->SlaveAddr);
                }
                delay_ms(I2C_RESET_DELAY);
                break;

            // 3. 无应答：发送停止信号 + 重试
            case RCS_I2C_ERR_ACK:
                LOG_I2C("I2C设备[%02X]：无应答，发送停止信号...\r\n", hi2c->SlaveAddr);
                RCS_I2C_Stop(hi2c);              // 核心控制：终止异常通信
                delay_ms(I2C_RETRY_DELAY);
                break;

            // 4. 长度错误：修正长度（通用默认最小1字节） + 重试
            case RCS_I2C_ERR_LEN:
                LOG_I2C("I2C设备[%02X]：长度错误，修正为1字节重试...\r\n", hi2c->SlaveAddr);
                len = (len == 0) ? 1 : len;      // 核心控制：修正无效长度
                break;

            // 5. 空指针：直接终止（无总线控制，防崩溃）
            case RCS_I2C_ERR_NULL:
                LOG_I2C("I2C设备[%02X]：空指针，终止操作\r\n", hi2c->SlaveAddr);
                return RCS_I2C_ERR_NULL;

            // 未知错误：发送停止信号
            default:
                LOG_I2C("I2C设备[%02X]：未知错误[%d]，发送停止信号\r\n", hi2c->SlaveAddr, handle_err);
                RCS_I2C_Stop(hi2c);
                return handle_err;
        }

        // 通用重试逻辑：根据操作类型（读/写）重新调用函数
        if(op_type == 0) { // 读操作
            handle_err = RCS_I2C_ReadMultiFrom(hi2c, reg_addr, data_buf, len);
        } else { // 写操作
            handle_err = RCS_I2C_WriteMultiTo(hi2c, reg_addr, data_buf, len);
        }
        retry++;
    }

    // 最终兜底：仍失败则触发通用安全保护
    if(handle_err != RCS_I2C_OK) {
        LOG_I2C("I2C设备[%02X]：重试%d次仍失败，触发安全保护\r\n", hi2c->SlaveAddr, I2C_MAX_RETRY);
        I2C_General_SafetyHandler(hi2c); // 调用用户自定义的安全逻辑
    }

    return handle_err;
}
// ------------- 通用I2C读写封装（调用上面的错误处理） -------------
/**
 * @brief  通用I2C读n字节（封装错误处理）
 * @param  reg_addr: 寄存器地址
 * @param  data_buf: 接收缓冲区
 * @param  len: 读取长度
 * @retval RCS_I2C_ErrCode：最终错误码
 */
RCS_I2C_ErrCode I2C_General_Read(RCS_I2C_HandleTypeDef* hi2c,uint8_t reg_addr,uint8_t* data_buf,uint8_t len) 
{
    // 原始读操作
    RCS_I2C_ErrCode err = RCS_I2C_ReadMultiFrom(hi2c, reg_addr, data_buf, len);
    // 通用错误处理（含总线控制）
    if(err != RCS_I2C_OK) {
        err = I2C_General_ErrorHandler(hi2c, err, 0, reg_addr, data_buf, len);
    }
    return err;
}

/**
 * @brief  通用I2C写n字节（封装错误处理）
 * @param  reg_addr: 寄存器地址
 * @param  data_buf: 发送缓冲区
 * @param  len: 写入长度
 * @retval RCS_I2C_ErrCode：最终错误码
 */
RCS_I2C_ErrCode I2C_General_Write(RCS_I2C_HandleTypeDef* hi2c,uint8_t reg_addr,uint8_t* data_buf,uint8_t len) 
{
    // 原始写操作
    RCS_I2C_ErrCode err = RCS_I2C_WriteMultiTo(hi2c, reg_addr, data_buf, len);
    // 通用错误处理（含总线控制）
    if(err != RCS_I2C_OK) {
        err = I2C_General_ErrorHandler(hi2c, err, 1, reg_addr, data_buf, len);
    }
    return err;
}
