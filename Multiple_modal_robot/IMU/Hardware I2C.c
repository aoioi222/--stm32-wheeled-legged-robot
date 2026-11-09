#include "Hardware_I2C.h"

/**
 * @brief  I2C写设备单个寄存器
 * @param  I2Cx: I2C外设
 * @param  DevAddr: 7位设备地址（如MPU6050=0x68）
 * @param  RegAddr: 寄存器地址
 * @param  Data: 要写入的数值
 * @retval 0=成功，1=失败（地址/发送超时）
 */
uint8_t I2C_WriteReg(I2C_TypeDef* I2Cx, uint8_t DevAddr, uint8_t RegAddr, uint8_t Data)
{
    // 1. 发送起始信号
    if(I2C_Start(I2Cx))                goto err;
    // 2. 发送设备地址（写）
    if(I2C_SendAddr(I2Cx, DevAddr, 1)) goto err;
    // 3. 发送寄存器地址
    if(I2C_SendByte(I2Cx, RegAddr))    goto err;
    // 4. 发送写入数据
    if(I2C_SendByte(I2Cx, Data))       goto err;
    // 5. 发送停止信号
    I2C_Stop(I2Cx);
    return 0;

err: // 出错处理：发送停止信号
    I2C_Stop(I2Cx);
    return 1;
}
/**
 * @brief  I2C写多个寄存器（批量写）
 * @param  I2Cx: I2C外设
 * @param  DevAddr: 设备地址
 * @param  RegAddr: 起始寄存器地址
 * @param  Len: 写入长度
 * @param  DataBuf: 写入数据缓冲区
 * @retval 0=成功，1=失败
 */
uint8_t I2C_WriteRegs(I2C_TypeDef* I2Cx, uint8_t DevAddr, uint8_t RegAddr, uint8_t Len, uint8_t* DataBuf)
{
    if(Len == 0 || DataBuf == NULL) return 1;
    // 起始+写设备地址+写寄存器地址
    if(I2C_Start(I2Cx))                    goto err;
    if(I2C_SendAddr(I2Cx, DevAddr, 1))     goto err;
    if(I2C_SendByte(I2Cx, RegAddr))        goto err;
    // 批量写数据
    for(uint8_t i=0; i<Len; i++)
    {
        if(I2C_SendByte(I2Cx, DataBuf[i])) goto err;
    }
    I2C_Stop(I2Cx);
    return 0;

err:
    I2C_Stop(I2Cx);
    return 1;
}

/**
 * @brief  I2C读多个寄存器（批量读）
 * @param  I2Cx: I2C外设
 * @param  DevAddr: 设备地址
 * @param  RegAddr: 起始寄存器地址
 * @param  Len: 读取长度
 * @param  DataBuf: 接收数据缓冲区
 * @retval 0=成功，1=失败
 */
uint8_t I2C_ReadRegs(I2C_TypeDef *I2Cx, uint8_t DevAddr, uint8_t RegAddr, uint8_t Len, uint8_t *DataBuf)
{
    // 1. 参数合法性校验（基础防护）
    if (Len == 0 || DataBuf == NULL || I2Cx == NULL)
    {
        return 1; // 参数错误，直接返回失败
    }

    // 2. 第一步：发送寄存器地址（写操作）
    if (I2C_Start(I2Cx)) // 启动I2C总线
    {
        goto err; // 启动失败，跳转到异常处理
    }
    if (I2C_SendAddr(I2Cx, DevAddr, 1)) // 发送设备地址+写标志（1=写）
    {
        goto err;
    }
    if (I2C_SendByte(I2Cx, RegAddr)) // 发送要读取的起始寄存器地址
    {
        goto err;
    }

    //  第二步：重复起始+读取数据（读操作）
    if (I2C_Start(I2Cx)) // 重复起始（切换为读操作）
    {
        goto err;
    }
    if (I2C_SendAddr(I2Cx, DevAddr, 0)) // 发送设备地址+读标志（0=读）
    {
        goto err;
    }

    // 4. 批量读取数据（核心流程）
    for (uint8_t i = 0; i < Len; i++)
    {
        uint8_t ack_flag = (i == Len - 1) ? 0 : 1;
        uint8_t recv_data = I2C_RecvByte(I2Cx, ack_flag);

        DataBuf[i] = recv_data; // 直接存储数据
    }

    // 读取成功，停止I2C总线
    I2C_Stop(I2Cx);
    return 0;

// 异常处理：通信失败，停止总线并返回失败
err:
    I2C_Stop(I2Cx);
    return 1;
} 
/**
   * @brief  I2C读设备单个寄存器
   * @param  I2Cx: I2C外设
   * @param  DevAddr: 7位设备地址
   * @param  RegAddr: 寄存器地址
   * @retval 寄存器数值（失败返回0xFF）
   */
uint8_t I2C_ReadReg(I2C_TypeDef* I2Cx, uint8_t DevAddr, uint8_t RegAddr)
{
    uint8_t data = 0xFF;
    // 第一步：写寄存器地址（告诉设备要读哪个寄存器）
    if(I2C_Start(I2Cx))                 goto err;
    if(I2C_SendAddr(I2Cx, DevAddr, 1))  goto err;
    if(I2C_SendByte(I2Cx, RegAddr))     goto err;

    // 第二步：重新起始+读数据
    if(I2C_Start(I2Cx))                 goto err; // 重复起始
    if(I2C_SendAddr(I2Cx, DevAddr, 0))  goto err; // 发送读地址
    data = I2C_RecvByte(I2Cx, 0);     // 接收数据（非应答，最后一个字节）
    
    // 发送停止信号
    I2C_Stop(I2Cx);
    return data;

err:
    I2C_Stop(I2Cx);
    return 0xFF;
}

static uint8_t My_GetPinSource(uint16_t GPIO_Pin)
{
    uint8_t PinSource = 0;
    // 逐个匹配GPIO引脚宏，返回对应的PinSource编号（0~15）
    if(GPIO_Pin == GPIO_Pin_0)  PinSource = 0;
    else if(GPIO_Pin == GPIO_Pin_1)  PinSource = 1;
    else if(GPIO_Pin == GPIO_Pin_2)  PinSource = 2;
    else if(GPIO_Pin == GPIO_Pin_3)  PinSource = 3;
    else if(GPIO_Pin == GPIO_Pin_4)  PinSource = 4;
    else if(GPIO_Pin == GPIO_Pin_5)  PinSource = 5;
    else if(GPIO_Pin == GPIO_Pin_6)  PinSource = 6;
    else if(GPIO_Pin == GPIO_Pin_7)  PinSource = 7;
    else if(GPIO_Pin == GPIO_Pin_8)  PinSource = 8;
    else if(GPIO_Pin == GPIO_Pin_9)  PinSource = 9;
    else if(GPIO_Pin == GPIO_Pin_10) PinSource = 10;
    else if(GPIO_Pin == GPIO_Pin_11) PinSource = 11;
    else if(GPIO_Pin == GPIO_Pin_12) PinSource = 12;
    else if(GPIO_Pin == GPIO_Pin_13) PinSource = 13;
    else if(GPIO_Pin == GPIO_Pin_14) PinSource = 14;
    else if(GPIO_Pin == GPIO_Pin_15) PinSource = 15;
    
    return PinSource;
}
/**
 * @brief  初始化指定I2C外设的硬件配置
 * @note   1. 该函数为硬件I2C的核心初始化入口，支持I2C1/I2C2/I2C3，无固定引脚限制；
 *         2. 引脚需满足STM32硬件复用要求，需查阅芯片数据手册确认；
 *         3. 支持两种通信速率：100000Hz（标准模式）、400000Hz（快速模式），速率由Hi2c->ClockSpeed指定；
 *         4. 传入的Hi2c结构体需提前初始化（外设、引脚、速率），且不可为空指针。
 * @param  I2C_TYPE: 枚举类型，指定待初始化的I2C外设（I2C1_t/I2C2_t/I2C3_t）
 * @param  Hi2c: 指向RCS_I2C_Hardware_TypeDef结构体的指针
 */
void RCS_I2C_Hardware_Init(RCS_I2C_Type I2C_TYPE, RCS_I2C_Hardware_TypeDef *Hi2c)
{   
    if(Hi2c == NULL) return; 
    if(I2C_TYPE < I2C1_t || I2C_TYPE > I2C3_t) return; 
          //输入参数校验
    switch(I2C_TYPE) {
        case I2C1_t:
            Hi2c->I2Cx = I2C1; 
            RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
            break;
        case I2C2_t:
            Hi2c->I2Cx = I2C2;  
            RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
            break;
        case I2C3_t:
            Hi2c->I2Cx = I2C3; 
            RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C3, ENABLE);
            break;
        default: return;
    }//开启时钟

    //开启SCL/SDA对应的GPIO时钟
    if(Hi2c->GPIOx_SCL == GPIOA)      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    else if(Hi2c->GPIOx_SCL == GPIOB) RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    else if(Hi2c->GPIOx_SCL == GPIOC) RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    else if(Hi2c->GPIOx_SCL == GPIOD) RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    else if(Hi2c->GPIOx_SCL == GPIOE) RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    if(Hi2c->GPIOx_SDA == GPIOA)      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    else if(Hi2c->GPIOx_SDA == GPIOB) RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    else if(Hi2c->GPIOx_SDA == GPIOC) RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    else if(Hi2c->GPIOx_SDA == GPIOD) RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    else if(Hi2c->GPIOx_SDA == GPIOE) RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;        // 复用模式
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;      // 开漏输出
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;        // 上拉电阻
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

    // 配置SCL引脚
    GPIO_InitStruct.GPIO_Pin = Hi2c->SCL_Pin;
    GPIO_Init(Hi2c->GPIOx_SCL, &GPIO_InitStruct);
    // 配置SDA引脚
    GPIO_InitStruct.GPIO_Pin = Hi2c->SDA_Pin;
    GPIO_Init(Hi2c->GPIOx_SDA, &GPIO_InitStruct);

    // 配置GPIO复用映射
 switch(I2C_TYPE) {
    case I2C1_t:
        // 把GPIO_GetPinSource换成My_GetPinSource
        GPIO_PinAFConfig(Hi2c->GPIOx_SCL, My_GetPinSource(Hi2c->SCL_Pin), GPIO_AF_I2C1);
        GPIO_PinAFConfig(Hi2c->GPIOx_SDA, My_GetPinSource(Hi2c->SDA_Pin), GPIO_AF_I2C1);
        break;
    case I2C2_t:
        GPIO_PinAFConfig(Hi2c->GPIOx_SCL, My_GetPinSource(Hi2c->SCL_Pin), GPIO_AF_I2C2);
        GPIO_PinAFConfig(Hi2c->GPIOx_SDA, My_GetPinSource(Hi2c->SDA_Pin), GPIO_AF_I2C2);
        break;
    case I2C3_t:
        GPIO_PinAFConfig(Hi2c->GPIOx_SCL, My_GetPinSource(Hi2c->SCL_Pin), GPIO_AF_I2C3);
        GPIO_PinAFConfig(Hi2c->GPIOx_SDA, My_GetPinSource(Hi2c->SDA_Pin), GPIO_AF_I2C3);
        break;
}
      I2C_SoftwareResetCmd(Hi2c->I2Cx, ENABLE);
      I2C_SoftwareResetCmd(Hi2c->I2Cx, DISABLE);
    // ========== 配置I2C参数 + 初始化外设 ==========
    I2C_InitTypeDef I2C_InitStruct;
    I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;                  // 开启应答
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // 7位地址
    I2C_InitStruct.I2C_ClockSpeed = Hi2c->ClockSpeed;         // 速率（由句柄传入）
    I2C_InitStruct.I2C_DutyCycle = (Hi2c->ClockSpeed == 400000) ? I2C_DutyCycle_16_9 : I2C_DutyCycle_2; // 自动适配占空比
    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;                   // 普通I2C模式
    I2C_InitStruct.I2C_OwnAddress1 = 0x00;                    // 主机模式填0

    // 初始化I2C外设
    I2C_Init(Hi2c->I2Cx, &I2C_InitStruct);

    // 开启I2C外设
    I2C_Cmd(Hi2c->I2Cx, ENABLE);
}


/**
 * @brief  I2C等待事件（带超时）
 * @param  I2Cx: I2C外设（I2C1/I2C2/I2C3）
 * @param  Event: 要等待的I2C事件（如I2C_EVENT_MASTER_MODE_SELECT）
 * @param  TimeoutUs: 超时时间（单位：us）
 * @retval 0=成功，1=超时
 * @note   轮询粒度必须远小于I2C单字节传输时间，否则每等一个事件都要多耗掉一个
 *         轮询周期。delay_ms在OS下按tick让出（fac_ms=1时至少1个tick=1ms），
 *         400kHz下一个字节只要22.5us，用毫秒轮询会让每次寄存器读取多花约19ms。
 */
static uint8_t I2C_WaitEvent(I2C_TypeDef* I2Cx, uint32_t Event, uint32_t TimeoutUs)
{
    uint32_t waited_us = 0; // 已等待时间（us）
    while(!I2C_CheckEvent(I2Cx, Event)) // 循环检测事件是否发生
    {
        delay_us(I2C_WAIT_POLL_US); // 微秒级轮询，不主动让出CPU
        waited_us += I2C_WAIT_POLL_US;
        if(waited_us >= TimeoutUs) // 超过设定超时时间，返回失败
        {
            return 1;
        }
    }
    return 0;                     // 事件触发，返回成功
}
/**
 * @brief  I2C发送起始信号
 * @param  I2Cx: I2C外设（I2C1/I2C2/I2C3）
 */
uint8_t I2C_Start(I2C_TypeDef* I2Cx)
{
    I2C_GenerateSTART(I2Cx, ENABLE);                              // 发送起始信号
    return I2C_WaitEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT, I2C_DEFAULT_TIMEOUT_US); // 等待起始信号发送成功
}
/**
 * @brief  I2C发送设备地址（7位）+ 读写位
 * @param  I2Cx: I2C外设
 * @param  DevAddr: 7位设备地址
 * @param  Write: 0=读，1=写
 * @retval 0=成功，1=超时（无应答）
 */
uint8_t I2C_SendAddr(I2C_TypeDef* I2Cx, uint8_t DevAddr, uint8_t Write)
{
    
    uint8_t addr = (DevAddr << 1) | (Write ? 0 : 1); // 7位地址左移+读写位（写=0，读=1）
    
    // 确定通信方向
    uint8_t direction;
    if(Write == 1)
    {
        direction = I2C_Direction_Transmitter;
    }
    else
    {
        direction = I2C_Direction_Receiver;
    }
    I2C_Send7bitAddress(I2Cx, addr, direction);
    
    // 确定等待的事件
    uint32_t wait_event;
    if(Write == 1)
    {
        wait_event = I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED;
    }
    else
    {
        wait_event = I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED;
    }
    // 等待地址发送成功+收到应答
    return I2C_WaitEvent(I2Cx, wait_event, I2C_DEFAULT_TIMEOUT_US);
}

/**
 * @brief  I2C发送单个字节
 * @param  I2Cx: I2C外设
 * @param  Data: 要发送的字节
 * @retval 0=成功，1=超时
 */
uint8_t I2C_SendByte(I2C_TypeDef* I2Cx, uint8_t Data)
{
    I2C_SendData(I2Cx, Data);                                          // 发送字节
    return I2C_WaitEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED, I2C_DEFAULT_TIMEOUT_US); // 等待发送完成
}
/**
 * @brief  I2C接收单个字节
 * @param  I2Cx: I2C外设
 * @param  Ack: 0=非应答（最后一个字节），1=应答（继续接收）
 * @retval 接收的字节（超时返回0xFF）
 */
uint8_t I2C_RecvByte(I2C_TypeDef* I2Cx, uint8_t Ack)
{
    //配置应答模式
    FunctionalState ack_state;
    if(Ack == 1)
    {
        ack_state = ENABLE;
    }
    else
    {
        ack_state = DISABLE;
    }
    I2C_AcknowledgeConfig(I2Cx, ack_state);
    
    // 等待接收完成
    if(I2C_WaitEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED, I2C_DEFAULT_TIMEOUT_US))
        return 0xFF;
    // 读取接收的数据
    return I2C_ReceiveData(I2Cx);
}
/**
 * @brief  I2C发送停止信号
 * @param  I2Cx: I2C外设
 */
void I2C_Stop(I2C_TypeDef* I2Cx)
{
    I2C_GenerateSTOP(I2Cx, ENABLE); // 发送停止信号
}

/**
 * @brief  强制重置I2C总线，解决因异常导致的总线锁死问题
 * @note   1. I2C总线锁死典型场景：通信中断、从机掉电、电磁干扰导致SDA/SCL引脚被拉低且无法释放；
 *         2. 核心原理：将I2C引脚临时切换为GPIO推挽输出模式，手动发送9个SCL时钟脉冲（I2C协议规定），强制从机释放SDA总线，
 *            最后恢复引脚为I2C复用模式，重新使能I2C外设；
 *         3. 该函数为应急修复手段，需在I2C通信失败后调用（如检测到设备无应答时）。
 * @param  GPIOx_SCL: SCL引脚所属的GPIO端口
 * @param  SCL_Pin: SCL引脚的具体编号
 * @param  GPIOx_SDA: SDA引脚所属的GPIO端口
 * @param  SDA_Pin: SDA引脚的具体编号
 * @retval 无
 */
void I2C_ResetBus(I2C_TypeDef* I2Cx, GPIO_TypeDef* GPIOx_SCL, uint16_t SCL_Pin, GPIO_TypeDef* GPIOx_SDA, uint16_t SDA_Pin)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    
    // 先关闭I2C外设
    I2C_Cmd(I2Cx, DISABLE);
    
    // 将SCL/SDA配置为推挽输出
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    
    GPIO_InitStruct.GPIO_Pin = SCL_Pin;
    GPIO_Init(GPIOx_SCL, &GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin = SDA_Pin;
    GPIO_Init(GPIOx_SDA, &GPIO_InitStruct);
    
    // 手动发送时钟，释放总线
    GPIO_SetBits(GPIOx_SDA, SDA_Pin);
    GPIO_SetBits(GPIOx_SCL, SCL_Pin);
    for(uint8_t i = 0; i < I2C_RESET_BUS_CLOCK_NUM; i++)
    {
        GPIO_ResetBits(GPIOx_SCL, SCL_Pin);
        delay_us(I2C_DELAY_BASE_US);
        GPIO_SetBits(GPIOx_SCL, SCL_Pin);
        delay_us(I2C_DELAY_BASE_US);
    }
    
    // 发送停止信号
    GPIO_ResetBits(GPIOx_SDA, SDA_Pin);
    delay_us(I2C_DELAY_BASE_US);
    GPIO_SetBits(GPIOx_SDA, SDA_Pin);
    delay_us(I2C_DELAY_BASE_US);
    
    // 恢复I2C引脚为复用模式
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    
    GPIO_InitStruct.GPIO_Pin = SCL_Pin;
    GPIO_Init(GPIOx_SCL, &GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin = SDA_Pin;
    GPIO_Init(GPIOx_SDA, &GPIO_InitStruct);
    
    // 重新使能I2C
    I2C_Cmd(I2Cx, ENABLE);
    
}//  I2C设备在线检测
uint8_t I2C_CheckDevice(I2C_TypeDef* I2Cx, uint8_t DevAddr)
{
    uint8_t ret = I2C_OP_FAIL;
    I2C_GenerateSTART(I2Cx, ENABLE);
    if(!I2C_WaitEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT, I2C_DEFAULT_TIMEOUT_US))
    {
        I2C_Send7bitAddress(I2Cx, (DevAddr << 1), I2C_Direction_Transmitter);
        if(!I2C_WaitEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, I2C_DEFAULT_TIMEOUT_US))
        {
            ret = I2C_OP_SUCCESS;
        }
    }
    I2C_GenerateSTOP(I2Cx, ENABLE);
    return ret;
}
