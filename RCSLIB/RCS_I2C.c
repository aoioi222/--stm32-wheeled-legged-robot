#include "RCS_I2C.h"
#include "rcs.h"

#define CLOCK_SPEED 10000  // 10kHz

volatile uint8_t isI2Cbusy1;
volatile uint8_t isI2Cbusy2;
volatile uint8_t isI2Csuccess1;
volatile uint8_t isI2Csuccess2;

volatile uint8_t txBuffer_1[BUFFER_SIZE];
volatile uint8_t txIndex_1 = 0;
volatile uint8_t txlength_1 = 0;

volatile uint8_t rxBuffer_1[BUFFER_SIZE];
volatile uint8_t rxIndex_1 = 0;
volatile uint8_t rxlength_1 = 0;

volatile uint8_t slaveAddr_1 = 0x00;
volatile uint8_t I2C_Direction_1;

volatile uint8_t txBuffer_2[BUFFER_SIZE];
volatile uint8_t txIndex_2 = 0;
volatile uint8_t txlength_2 = 0;

volatile uint8_t rxBuffer_2[BUFFER_SIZE];
volatile uint8_t rxIndex_2 = 0;
volatile uint8_t rxlength_2 = 0;

volatile uint8_t slaveAddr_2 = 0x00;
volatile uint8_t I2C_Direction_2;

uint8_t str[10];

void RCS_I2C1_EV_IRQ(void);
void RCS_I2C2_EV_IRQ(void);

void RCS_I2C1_ER_IRQ(void);
void RCS_I2C2_ER_IRQ(void);

inline static void RCC_Configuration(I2C_TypeDef *_I2Cx, GPIO_TypeDef *_GPIOx)
{
    RCC_APB1PeriphClockCmd(GetRCS_RCC_APB2Periph_I2C(_I2Cx), ENABLE);
    RCC_AHB1PeriphClockCmd(GetRCS_RCC_AHB1Periph_GPIO(_GPIOx), ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

}

inline static void GPIO_Configuration(GPIO_TypeDef *_GPIOx, uint32_t _GPIO_PinX_SCL, 
                        uint32_t _GPIO_PinX_SDA, I2C_TypeDef *_I2Cx)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_StructInit( &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin =  _GPIO_PinX_SCL | _GPIO_PinX_SDA;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; // GPIO_OType_OD
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // 
    GPIO_Init(_GPIOx, &GPIO_InitStructure);

    GPIO_PinAFConfig(_GPIOx, GetRCS_GPIO_PinSource(_GPIO_PinX_SCL), GetRCS_I2C_AF(_I2Cx));
    GPIO_PinAFConfig(_GPIOx, GetRCS_GPIO_PinSource(_GPIO_PinX_SDA), GetRCS_I2C_AF(_I2Cx));
}

inline static void NVIC_Configuration(I2C_TypeDef *_I2Cx)
{
    if( _I2Cx == I2C1)
    {
        BSP_IntVectSet(GetBSP_INT_ID_I2C_EV(I2C1), RCS_I2C1_EV_IRQ);
        BSP_IntVectSet(GetBSP_INT_ID_I2C_ER(I2C1), RCS_I2C1_ER_IRQ);
    }
    else if( _I2Cx == I2C2)
    {   
        BSP_IntVectSet(GetBSP_INT_ID_I2C_EV(I2C2), RCS_I2C2_EV_IRQ);
        BSP_IntVectSet(GetBSP_INT_ID_I2C_ER(I2C2), RCS_I2C2_ER_IRQ);
    }

    NVIC_InitTypeDef  NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = GetRCS_I2C_EV_IRQn(_I2Cx);
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = GetRCS_I2C_ER_IRQn(_I2Cx);
    NVIC_Init(&NVIC_InitStructure);
}

void RCS_Init_I2C(I2C_TypeDef *_I2Cx, GPIO_TypeDef *_GPIOx, 
                        uint32_t _GPIO_PinX_SCL, uint32_t _GPIO_PinX_SDA, uint16_t address)
{
    RCC_Configuration(_I2Cx, _GPIOx);
    GPIO_Configuration(_GPIOx, _GPIO_PinX_SCL, _GPIO_PinX_SDA, _I2Cx);

    I2C_DeInit(_I2Cx);
    I2C_InitTypeDef I2C_InitStructure;
    I2C_StructInit(&I2C_InitStructure);
    I2C_InitStructure.I2C_ClockSpeed = CLOCK_SPEED;
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = address;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable; // I2C_Ack_Enable I2C_Ack_Disable
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(_I2Cx, &I2C_InitStructure);  

    I2C_Cmd(_I2Cx, ENABLE); // Enables the specified I2C peripheral.
    // I2C_ITConfig(_I2Cx, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, ENABLE);//原先库
    NVIC_Configuration(_I2Cx);

    if(_I2Cx == I2C1)
        isI2Cbusy1 = 0;
    else if(_I2Cx == I2C2)
        isI2Cbusy2 = 0;
}

bool RCS_I2C_StartSend(I2C_TypeDef *_I2Cx, uint8_t _address)
{
    if( _I2Cx == I2C1)
    {
        slaveAddr_1 = _address;
        I2C_Direction_1 = I2C_Direction_Transmitter;
    }
    else if( _I2Cx == I2C2)
    {
        slaveAddr_2 = _address;
        I2C_Direction_2 = I2C_Direction_Transmitter;
    }
    int count = 0;
    while(I2C_GetFlagStatus(_I2Cx, I2C_FLAG_BUSY))
    {
        count ++;
        if (count > 10000)
        {
            return false;
        }
    }

    if(_I2Cx == I2C1)
    {
        isI2Csuccess1 = 0;
        isI2Cbusy1 = 1;
    }
    else if(_I2Cx == I2C2)
    {
        isI2Csuccess2 = 0;
        isI2Cbusy2 = 1;
    }
    I2C_GenerateSTART(_I2Cx, ENABLE);
    return true;
}

bool RCS_I2C_StartRequest(I2C_TypeDef *_I2Cx, uint8_t _address, uint8_t _length)
{
    if( _I2Cx == I2C1)
    {
        slaveAddr_1 = _address;
        rxlength_1 = _length;
        I2C_Direction_1 = I2C_Direction_Receiver;
    }
    else if( _I2Cx == I2C2)
    {
        slaveAddr_2 = _address;
        rxlength_2 = _length;
        I2C_Direction_2 = I2C_Direction_Receiver;
    }
    int count  = 0;
    while(I2C_GetFlagStatus(_I2Cx, I2C_FLAG_BUSY))
    {
        if(count > 100000)
        {
			I2C_ClearFlag(_I2Cx, I2C_FLAG_BUSY);
            return false;
        }
        count++;
    }

    if(_I2Cx == I2C1)
    {
        isI2Csuccess1 = 0;
        isI2Cbusy1 = 1;
    }
    else if(_I2Cx == I2C2)
    {
        isI2Csuccess2 = 0;
        isI2Cbusy2 = 1;
    }
    I2C_GenerateSTART(_I2Cx, ENABLE);
    return true;
}

void RCS_I2C1_EV_IRQ(void)
{
    uint32_t event;
    event = I2C_GetLastEvent(I2C1);

    switch(event)
    {
        // master
        case I2C_EVENT_MASTER_MODE_SELECT:  // 0x00030001
            I2C_Send7bitAddress(I2C1, slaveAddr_1, I2C_Direction_1);
            break;
        // master transmitter
        case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED:  // 0x00070082
            I2C_SendData(I2C1, txBuffer_1[0]);
            txIndex_1 = 1;
            break;
        case I2C_EVENT_MASTER_BYTE_TRANSMITTING:  // 0x00070080
            break;
        case I2C_EVENT_MASTER_BYTE_TRANSMITTED: // 0x00070084
            if(txIndex_1 < txlength_1)
                I2C_SendData(I2C1, txBuffer_1[txIndex_1]);
            else if(txIndex_1 == txlength_1)
            {
                I2C_GenerateSTOP(I2C1, ENABLE);
                isI2Cbusy1 = 0;
                isI2Csuccess1 = 1;
            }
            ++txIndex_1;
        break;

        // master receiver
        case I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED:
            rxIndex_1 = 0;
            break;
        case I2C_EVENT_MASTER_BYTE_RECEIVED:
            rxBuffer_1[rxIndex_1] = I2C_ReceiveData(I2C1);
            if(rxIndex_1 == (rxlength_1-2))
            {
                I2C_NACKPositionConfig(I2C1, I2C_NACKPosition_Current); 
                I2C_AcknowledgeConfig(I2C1,DISABLE);
                I2C_GenerateSTOP(I2C1, ENABLE);
            }else if(rxIndex_1 == (rxlength_1-1))
            {
                isI2Cbusy1 = 0;
                isI2Csuccess1 = 1;
            }
            ++rxIndex_1;
            break;

        // slave
        case I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED:  // 0x00020002
            rxIndex_1 = 0;
            isI2Cbusy1 = 1;
            break;
        case I2C_EVENT_SLAVE_BYTE_RECEIVED: // 0x00020040
            rxBuffer_1[rxIndex_1] = I2C_ReceiveData(I2C1);
            ++ rxIndex_1;
            break;
        case I2C_EVENT_SLAVE_STOP_DETECTED: // 0x00000010
            rxlength_1 = rxIndex_1;
            I2C_GetITStatus(I2C1, I2C_IT_STOPF);  
            I2C_Cmd(I2C1, ENABLE);
            isI2Cbusy1 = 0;
            for(rxIndex_1 = 0; rxIndex_1<rxlength_1; ++rxIndex_1)
            {
               // sprintf((char*)str, "%d ", rxBuffer_1[rxIndex_1]);
               // RCS_USART_Send_Str(BT_USART, (char*)str);
            }
            RCS_USART_Send_Char(BT_USART, '\n');
        break;
      default:
        break;
    }
  return;
}

void RCS_I2C2_EV_IRQ(void)
{
    uint32_t event;
    event = I2C_GetLastEvent(I2C2);

    switch(event)
    {
        // master
        case I2C_EVENT_MASTER_MODE_SELECT:  // 0x00030001
         
            I2C_Send7bitAddress(I2C2, slaveAddr_2, I2C_Direction_2);
            break;
        // master transmitter
        case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED:  // 0x00070082
          
            I2C_SendData(I2C2, txBuffer_2[0]);
            txIndex_2 = 1;
            break;
        case I2C_EVENT_MASTER_BYTE_TRANSMITTING:  // 0x00070080
            break;
        case I2C_EVENT_MASTER_BYTE_TRANSMITTED: // 0x00070084
            if(txIndex_2 < txlength_2)
                I2C_SendData(I2C2, txBuffer_2[txIndex_2]);
            else if(txIndex_2 == txlength_2)
            {
                I2C_GenerateSTOP(I2C2, ENABLE);
                isI2Cbusy2 = 0;
                isI2Csuccess2 = 1;
            }
            ++txIndex_2;
        break;

        // master receiver
        case I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED:
             rxIndex_2 = 0;
            break;
        case I2C_EVENT_MASTER_BYTE_RECEIVED: 
            rxBuffer_2[rxIndex_2] = I2C_ReceiveData(I2C2);
            if(rxIndex_2 == (rxlength_2-2))
            {
                I2C_NACKPositionConfig(I2C2, I2C_NACKPosition_Current); 
                I2C_AcknowledgeConfig(I2C2,DISABLE);
                I2C_GenerateSTOP(I2C2, ENABLE);
            }else if(rxIndex_2 == (rxlength_2-1))
            {
                I2C_NACKPositionConfig(I2C2, I2C_NACKPosition_Next); 
                I2C_AcknowledgeConfig(I2C2,ENABLE);
                I2C_GenerateSTOP(I2C2, DISABLE);
                isI2Cbusy2 = 0;
                isI2Csuccess2 = 1;
            }
            else
            {
                I2C_NACKPositionConfig(I2C2, I2C_NACKPosition_Next); 
            }
            ++rxIndex_2;
            break;

        // slave transmitter
        case I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED:  // 0x00020002
            rxIndex_2 = 0;
            isI2Cbusy2 = 1;
            break;
        case I2C_EVENT_SLAVE_BYTE_RECEIVED: // 0x00020040
            rxBuffer_2[rxIndex_2] = I2C_ReceiveData(I2C2);
            ++ rxIndex_2;
            break;
        case I2C_EVENT_SLAVE_STOP_DETECTED: // 0x00000010
            rxlength_2 = rxIndex_2;
            I2C_GetITStatus(I2C2, I2C_IT_STOPF);  
            I2C_Cmd(I2C2, ENABLE);
            isI2Cbusy2 = 0;
            for(rxIndex_2 = 0; rxIndex_2<rxlength_2; ++rxIndex_2)
            {
               sprintf((char*)str, "%d ", rxBuffer_2[rxIndex_2]);
               RCS_USART_Send_Str(BT_USART, str);
            }
            RCS_USART_Send_Char(BT_USART, '\n');
            break;    
        default:
            I2C_DeInit(I2C2);
            RCS_Init_I2C(I2C2, GPIOB, GPIO_Pin_10, GPIO_Pin_11, (uint16_t)0x00);
            
            break;
    }
    return;
}

void RCS_I2C1_ER_IRQ()
{
    if(SET == I2C_GetITStatus(I2C1, I2C_IT_TIMEOUT))
    {
        I2C_ClearFlag(I2C1, I2C_IT_TIMEOUT);
        I2C_GenerateSTOP(I2C1, ENABLE);
        isI2Cbusy1 = 0;
    }
    if(SET == I2C_GetITStatus(I2C1, I2C_IT_AF))
    {
        I2C_ClearFlag(I2C1, I2C_IT_AF);
        I2C_GenerateSTOP(I2C1, ENABLE);
        isI2Cbusy1 = 0;
    }
    if(SET == I2C_GetITStatus(I2C1, I2C_IT_BERR))
    {
        I2C_ClearFlag(I2C1, I2C_IT_BERR);
    }
    if(SET == I2C_GetITStatus(I2C1, I2C_IT_ARLO))
    {
        I2C_ClearFlag(I2C1, I2C_IT_ARLO);        
    }
}

void RCS_I2C2_ER_IRQ()
{
    if(SET == I2C_GetITStatus(I2C2, I2C_IT_TIMEOUT))
    {
        I2C_ClearFlag(I2C2, I2C_IT_TIMEOUT);
        I2C_GenerateSTOP(I2C2, ENABLE);
        isI2Cbusy2 = 0;
    }
    if(SET == I2C_GetITStatus(I2C2, I2C_IT_AF))
    {
        I2C_ClearFlag(I2C2, I2C_IT_AF);
        I2C_GenerateSTOP(I2C2, ENABLE);
        isI2Cbusy2 = 0;
    }
    if(SET == I2C_GetITStatus(I2C2, I2C_IT_BERR))
    {
        I2C_ClearFlag(I2C2, I2C_IT_BERR);
    }
    if(SET == I2C_GetITStatus(I2C2, I2C_IT_ARLO))
    {
        I2C_ClearFlag(I2C2, I2C_IT_ARLO);        
    }
}

uint8_t isI2CBusy(I2C_TypeDef *_I2Cx)
{
    if(_I2Cx == I2C1)
    {
        return isI2Cbusy1;
    }else if(_I2Cx == I2C2)
    {
        return isI2Cbusy2;
    }
    else
        return -1;
}

uint8_t isI2CSuccess(I2C_TypeDef *_I2Cx)
{
    if(_I2Cx == I2C1)
    {
        return isI2Csuccess1;
    }else if(_I2Cx == I2C2)
    {
        return isI2Csuccess2;
    }
    else
        return -1;
}

void RCS_I2C2_SetData(uint8_t _number, uint8_t _angle,uint8_t _speed)
{
    txlength_2 = 5;
    txBuffer_2[0] = 0xAA;
    txBuffer_2[1] = _number;
    txBuffer_2[2] = 0xF0 | (_angle >> 4);
    txBuffer_2[3] = 0xF0 | _angle;
    txBuffer_2[4] = _speed;
}






