/**
 * @filename:RCS_CAN.c
 * @brief:为CAN通信提供硬件无关的操作接口
 * @changelog: 2023-11-15 CYK-dot 隔离硬件抽象层与应用层，添加发送拓展数据包的功能
 * @changelog: 2024-2-14 CYK-dot 为CAN报文发送提供软件buffer 
**/

/* =========头文件===================================================*/
#include "RCS_CAN.h"

/* =========接口函数实现===============================================*/

/**
 @name: CAN1_Config
 @brief: 配置CAN1的通信功能，并将CAN1绑定到某个引脚
 @param: _baudRate  设置波特率（波特率需能整除3000000）
**/                    
void RCS_CAN1_Config(RCS_PIN_CAN CANx_MAP,FNCT_VOID _isr, uint32_t _baudRate)//(GPIO_TypeDef *_GPIOx, uint32_t _GPIO_PinX_Rx, uint32_t _GPIO_PinX_Tx, FNCT_VOID _isr, uint32_t _baudRate)
{
    // 定义结构体变量
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    CAN_InitTypeDef CAN_InitStructure;
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;

    RCC_ClocksTypeDef rcc_clocks;                   //can不能用一般与时钟有关
    RCC_GetClocksFreq(&rcc_clocks);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = CANx_MAP.GPIO_Pin_Rx | CANx_MAP.GPIO_Pin_Tx;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_Init(CANx_MAP.GPIOx, &GPIO_InitStructure);
    GPIO_PinAFConfig(CANx_MAP.GPIOx, GetRCS_GPIO_PinSource(CANx_MAP.GPIO_Pin_Rx), GPIO_AF_CAN1);
    GPIO_PinAFConfig(CANx_MAP.GPIOx, GetRCS_GPIO_PinSource(CANx_MAP.GPIO_Pin_Tx), GPIO_AF_CAN1);

    CAN_DeInit(CAN1);
    CAN_StructInit(&CAN_InitStructure);
    CAN_InitStructure.CAN_ABOM = ENABLE;
    CAN_InitStructure.CAN_TXFP = ENABLE;
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
    CAN_InitStructure.CAN_BS1 = CAN_BS1_4tq;
    CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
    CAN_InitStructure.CAN_Prescaler = 6000000 / _baudRate;//波特率计算公式：CAN时钟频率/（(SJW + BS1 + BS2) * Prescaler)
    // CAN_InitStructure.CAN_Prescaler = rcc_clocks.PCLK1_Frequency/ (7 * _baudRate);     //波特率计算公式：CAN时钟频率/（(SJW + BS1 + BS2) * Prescaler)
    //vesc
		CAN_InitStructure.CAN_TTCM=DISABLE; //非时间触发通信模式 
		CAN_InitStructure.CAN_ABOM=ENABLE; //软件自动离线管理 
		CAN_InitStructure.CAN_AWUM=DISABLE;//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
		CAN_InitStructure.CAN_NART=ENABLE; //禁止报文自动传送 
		CAN_InitStructure.CAN_RFLM=DISABLE; //报文不锁定,新的覆盖旧的 
		CAN_InitStructure.CAN_TXFP=ENABLE; //优先级由报文标识符决定 
		CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位
		
		CAN_Init(CAN1, &CAN_InitStructure);
/*******************************CAN的过滤器设置********************************/
    // CAN_FilterInit(&CAN_FilterInitStructure);     //初始化CAN_FilterInitStructrue结构体变量 
    CAN_FilterInitStructure.CAN_FilterNumber = 0; //选择过滤器0，范围为0~13 
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;//设置过滤器组0为屏蔽模式  
    CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;//设置过滤器组0位宽为32位

    //标识位寄存器的设置
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;  //设置标识符寄存器高字节。  
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;   //设置标识符寄存器低字节

    //屏蔽寄存器的设置 
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000; //全部接收是0x0000;屏蔽位的多机通讯ffff
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;//全部接收是0x0000;屏蔽位的多机通讯ffff


    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0; //此过滤器组关联到接收FIFO0  
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活此过滤器组
    CAN_FilterInit(&CAN_FilterInitStructure); // //设置过滤器
/*********************************************************************************/
    BSP_IntVectSet(BSP_INT_ID_CAN1_RX0, _isr);

    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure); 

    // NVIC_InitStructure.NVIC_IRQChannel = CAN1_TX_IRQn;
    // NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    // NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    // NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    // NVIC_Init(&NVIC_InitStructure); 

    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
		
		
		
	//		CAN_FilterInitTypeDef  CAN_FilterInitStructure;
  CAN_FilterInitStructure.CAN_FilterNumber = 14;
  CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;


  CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;

    //????????
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;


  CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;
  CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);
		
		
		#ifdef CAN_DEBUG_PRINT
			RCS_Shell_Logs("RCS_CAN.c: CAN1 peripheral config done");
		#endif
}

/**
 @name: CAN2_Config
 @brief: 配置CAN2的通信功能，并将CAN2绑定到某个引脚
 @param: _baudRate  设置波特率（波特率需能整除3000000）
**/    
void RCS_CAN2_Config(RCS_PIN_CAN CANx_MAP,FNCT_VOID _isr, uint32_t _baudRate)//(GPIO_TypeDef *_GPIOx, uint32_t _GPIO_PinX_Rx, uint32_t _GPIO_PinX_Tx, FNCT_VOID _isr, uint32_t _baudRate)
{
    // 定义结构体变量
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    CAN_InitTypeDef CAN_InitStructure;
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;

    RCC_ClocksTypeDef rcc_clocks;                   //can不能用一般与时钟有关
    RCC_GetClocksFreq(&rcc_clocks);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = CANx_MAP.GPIO_Pin_Rx | CANx_MAP.GPIO_Pin_Tx;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_Init(CANx_MAP.GPIOx, &GPIO_InitStructure);
    GPIO_PinAFConfig(CANx_MAP.GPIOx, GetRCS_GPIO_PinSource(CANx_MAP.GPIO_Pin_Rx), GPIO_AF_CAN2);
    GPIO_PinAFConfig(CANx_MAP.GPIOx, GetRCS_GPIO_PinSource(CANx_MAP.GPIO_Pin_Tx), GPIO_AF_CAN2);

    CAN_DeInit(CAN2);
    CAN_StructInit(&CAN_InitStructure);
    CAN_InitStructure.CAN_ABOM = ENABLE;
    CAN_InitStructure.CAN_TXFP = ENABLE;
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
    CAN_InitStructure.CAN_BS1 = CAN_BS1_4tq;
    CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
    CAN_InitStructure.CAN_Prescaler = 6000000 / _baudRate;//波特率计算公式：CAN时钟频率/（(SJW + BS1 + BS2) * Prescaler)
    // CAN_InitStructure.CAN_Prescaler = rcc_clocks.PCLK1_Frequency/ (7 * _baudRate);     //波特率计算公式：CAN时钟频率/（(SJW + BS1 + BS2) * Prescaler)
		//vesc
		CAN_InitStructure.CAN_TTCM=DISABLE; //非时间触发通信模式 
		CAN_InitStructure.CAN_ABOM=ENABLE; //软件自动离线管理 
		CAN_InitStructure.CAN_AWUM=DISABLE;//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
		CAN_InitStructure.CAN_NART=ENABLE; //禁止报文自动传送 
		CAN_InitStructure.CAN_RFLM=DISABLE; //报文不锁定,新的覆盖旧的 
		CAN_InitStructure.CAN_TXFP=ENABLE; //优先级由报文标识符决定 
		CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位
		CAN_Init(CAN2, &CAN_InitStructure);


/*******************************CAN的过滤器设置********************************/
    // CAN_FilterInit(&CAN_FilterInitStructure);     //初始化CAN_FilterInitStructrue结构体变量 
    CAN_FilterInitStructure.CAN_FilterNumber = 14; //选择过滤器14，范围为14~27 
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;//设置过滤器组0为屏蔽模式  
    CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;//设置过滤器组0位宽为32位

    //标识位寄存器的设置
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;  //设置标识符寄存器高字节。  
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;   //设置标识符寄存器低字节

    //屏蔽寄存器的设置 
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000; //全部接收是0x0000;屏蔽位的多机通讯ffff
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;//全部接收是0x0000;屏蔽位的多机通讯ffff


    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0; //此过滤器组关联到接收FIFO0  
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活此过滤器组
    CAN_FilterInit(&CAN_FilterInitStructure); // //设置过滤器
/*********************************************************************************/


    BSP_IntVectSet(BSP_INT_ID_CAN2_RX0, _isr);

    NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 6;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure); 

    // NVIC_InitStructure.NVIC_IRQChannel = CAN1_TX_IRQn;
    // NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    // NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    // NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    // NVIC_Init(&NVIC_InitStructure); 

    CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);
		
		#ifdef CAN_DEBUG_PRINT
			RCS_Shell_Logs("RCS_CAN.c: CAN2 peripheral config done");
		#endif
}


/**
 @name:RCS_CANx_Recieve
 @brief:CAN接收
 @param:RCS_CAN2B_DATA_FM_RX* 指向接收数据帧结构体的指针
**/
void RCS_CANx_Recieve(RCS_CAN_T* CANx,RCS_CAN2B_DATA_FM_RX* RxMessage)
{
    CAN_Receive(CANx, CAN_FIFO0, RxMessage);
    CAN_FIFORelease(CANx,CAN_FIFO0);
}
/**
 @name:RCS_CANx_Transmit
 @brief:CAN发送
 @param:RCS_CAN2B_DATA_FM_TX* 指向发送数据帧结构体的指针
**/
void RCS_CANx_Transmit(RCS_CAN_T* CANx,RCS_CAN2B_DATA_FM_TX* TxMessage)
{
    CAN_Transmit(CANx,TxMessage);
}




