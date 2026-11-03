/**
 * @name:RCS_CAN_Interface
 * @brief:为CAN总线提供灵活的操作接口(缓冲发送+接收服务注册)
 * @changelog:2024-3-4 CYK-Dot 初次创建
 * @todo:将延时发送改为延时计算,解决时滞问题
*/
/* ======头文件==============================================*/
#include "RCS_CAN_Interface.h"

/* ======私有全局变量声明=====================================*/
RCS_QUENE CAN1_Tx_Buffer,CAN2_Tx_Buffer;//发送队列
RCS_CAN2B_DATA_FM_TX CAN1_Tx_Data_Buffer_Arr[CAN_SOFT_BUFFER_LEN+1]; 
RCS_CAN2B_DATA_FM_TX CAN2_Tx_Data_Buffer_Arr[CAN_SOFT_BUFFER_LEN+1];

uint8_t CAN1_Rx_Handler_Len,CAN2_Rx_Handler_Len;//接收句柄长度
CAN_HANDLER_VOID CAN1_Rx_Data_Handler_Arr[CAN_HANDLER_MAX_LEN];
CAN_HANDLER_VOID CAN2_Rx_Data_Handler_Arr[CAN_HANDLER_MAX_LEN];

/* ======静态函数声明========================================*/
static void RCS_CAN_Send_With_Buffer(RCS_CAN2B_DATA_FM_TX Tx_Msg);
static void RCS_CAN_Send_With_Buffer2(RCS_CAN2B_DATA_FM_TX Tx_Msg);
static void CAN1_Buffer_Tx_Isr(void);
static void CAN2_Buffer_Tx_Isr(void);
void CAN1_Rx_Service_Handler(void);
void CAN2_Rx_Service_Handler(void);

/* ======函数接口定义========================================*/

/**
 @name: RCS_CANx_Config_With_Buffer
 @brief: 配置CANx的发送功能,绑定CANx输出引脚,并提供软件buffer支持
 @param: CANx_MAP   填入CAN1_MAP或是CAN2_MAP,指定CAN外设和输出的管脚
 @param: _baudRate  设置波特率（波特率需能整除3000000）
 @tips:一次主循环内需要发送超过三条CAN报文的,需要使用软件缓冲避免送不出去
 @tips:此方法初始化后,仍然可以用原先的函数直接发送CAN报文,也可以用Send_With_Buffer发送CAN报文
**/
void RCS_CANx_Config_With_Buffer(RCS_PIN_CAN CANx_MAP, uint32_t _baudRate)
{
    if (CANx_MAP.CANx==CAN1)
    {
        //CAN外设初始化
        RCS_CAN1_Config(CANx_MAP,CAN1_Rx_Service_Handler,_baudRate);
			
        //队列初始化
        // CAN1_Tx_Buffer.data=CAN1_Tx_Data_Buffer_Arr;
        // CAN1_Tx_Buffer.len=CAN_SOFT_BUFFER_LEN;
        // CAN1_Tx_Buffer.bottom=0;
        // CAN1_Tx_Buffer.top=0;
        // CAN1_Tx_Buffer.data_size=sizeof(RCS_CAN2B_DATA_FM_TX);
				Quene_Init(&CAN1_Tx_Buffer,CAN1_Tx_Data_Buffer_Arr,CAN_SOFT_BUFFER_LEN,sizeof(RCS_CAN2B_DATA_FM_TX));

        //队列驱动定时器初始化
        InitTimerInt(CAN1_BUFFER_TIMER,CAN_BUFFER_SEND_FRQ,CAN_BUFFER_TIMER_FRQ,&CAN1_Buffer_Tx_Isr,RCS_CAN1_BUFFER_PRI);
        StopTimer(CAN1_BUFFER_TIMER);

        //完成配置	
		#ifdef CAN_DEBUG_PRINT
			RCS_Shell_Logs(&Core407_RCSLIB_Debug,"CAN: CAN1 buffer timer config done");
		#endif
    }
    else if (CANx_MAP.CANx==CAN2)
    {
        //CAN外设初始化
        RCS_CAN2_Config(CANx_MAP,CAN2_Rx_Service_Handler,_baudRate);
		//队列初始化
        // CAN2_Tx_Buffer.data=CAN2_Tx_Data_Buffer_Arr;
        // CAN2_Tx_Buffer.len=CAN_SOFT_BUFFER_LEN;
        // CAN2_Tx_Buffer.bottom=0;
        // CAN2_Tx_Buffer.top=0;
        // CAN2_Tx_Buffer.data_size=sizeof(RCS_CAN2B_DATA_FM_TX);
        Quene_Init(&CAN2_Tx_Buffer,CAN2_Tx_Data_Buffer_Arr,CAN_SOFT_BUFFER_LEN,sizeof(RCS_CAN2B_DATA_FM_TX));

        //队列驱动定时器初始化
        InitTimerInt(CAN2_BUFFER_TIMER,CAN_BUFFER_SEND_FRQ,CAN_BUFFER_TIMER_FRQ,&CAN2_Buffer_Tx_Isr,RCS_CAN2_BUFFER_PRI);
        StopTimer(CAN2_BUFFER_TIMER);
			
        //完成配置	
		#ifdef CAN_DEBUG_PRINT
			RCS_Shell_Logs(&Core407_RCSLIB_Debug,"CAN: CAN2 buffer timer config done");
		#endif
    }
}

/**
 * @name:RCS_CANx_Add_Handler
 * @brief:为CANx增加一个处理报文的服务程序
 * @param:RCS_PIN_CAN CANx_MAP 填入CAN1_MAP或是CAN2_MAP,指定CAN外设
 * @param:CAN_HANDLER_VOID Handler 
*/
void RCS_CANx_Add_Handler(RCS_CAN_T* CANx,CAN_HANDLER_VOID Handler)
{
    if (CANx == CAN1)
    {
        //列表队头+1
        CAN1_Rx_Data_Handler_Arr[CAN1_Rx_Handler_Len]=Handler;
        CAN1_Rx_Handler_Len++;
        
        //超出最大服务数量
        if (CAN1_Rx_Handler_Len >= CAN_HANDLER_MAX_LEN)
        {   
            #ifdef CAN_DEBUG_PRINT
            RCS_Shell_Logs(&Core407_RCSLIB_Debug,"Error:Add Too Many Handler to CAN1!");
            #endif
            uint8_t err=10/0.0f;//进入hardfault
        }
    }
    else if (CANx == CAN2)
    {
        //列表队头
        CAN2_Rx_Data_Handler_Arr[CAN2_Rx_Handler_Len]=Handler;
        CAN2_Rx_Handler_Len++;

        //超出最大服务数量
        if (CAN2_Rx_Handler_Len >= CAN_HANDLER_MAX_LEN)
        {
            #ifdef CAN_DEBUG_PRINT
            RCS_Shell_Logs(&Core407_RCSLIB_Debug,"Error:Add Too Many Handler to CAN2!");
            #endif
            uint8_t err=10/0.0f;//进入hardfault
        }
    }
}

/**
 @name: RCS_CANx_Send_STDID
 @brief: CAN2.0B发送标准帧数据
 @param:CAN_TypeDef *CANx	CAN组
 @param:uint32_t Id	发送给对应ID号
 @param:uint8_t Length 发送数据长度
 @param:uint8_t* sendData  数据
**/
void RCS_CANx_Send_STDID(RCS_CAN_T* CANx,uint32_t Id, uint8_t Length, uint8_t* sendData)
{
	int i;
    RCS_CAN2B_DATA_FM_TX TxMessage;
    TxMessage.IDE = CAN_Id_Standard;//标准ID
    TxMessage.StdId = Id;//标准ID
    TxMessage.ExtId = Id;//拓展ID
    TxMessage.RTR = CAN_RTR_Data;//占据主动权
    TxMessage.DLC = Length;//数据长度，CAN2.0B不允许数据长度大于8

    for(i = 0;i < Length; i++)
    {
        TxMessage.Data[i] = sendData[i];
    }
		
		#ifdef CAN_BUFFER_ALL_FUNCTION
			if (CANx==CAN1)
				RCS_CAN_Send_With_Buffer(TxMessage);
			else if (CANx==CAN2)
				RCS_CAN_Send_With_Buffer2(TxMessage);
		#else
			CAN_Transmit(CANx, &TxMessage);
		#endif
}

/**
 @name: RCS_CANx_Send_EXTID
 @brief: CAN2.0B发送拓展帧数据
 @param:CAN_TypeDef *CANx	CAN组
 @param:uint32_t Id	发送给对应ID号
 @param:uint8_t Length 发送数据长度
 @param:uint8_t* sendData  数据
**/
void RCS_CANx_Send_EXTID(RCS_CAN_T* CANx,uint32_t Id, uint8_t Length, uint8_t* sendData)
{
	int i;
    RCS_CAN2B_DATA_FM_TX TxMessage;
    TxMessage.IDE = CAN_Id_Extended;//拓展ID
    TxMessage.StdId = Id;//标准ID
    TxMessage.ExtId = Id; //拓展ID
    TxMessage.RTR = CAN_RTR_Data;//占据主动权
    TxMessage.DLC = Length;//数据长度，CAN2.0B不允许数据长度大于8

    for(i = 0;i < Length; i++)
    {
        TxMessage.Data[i] = sendData[i];
    }
		
    #ifdef CAN_BUFFER_ALL_FUNCTION
			if (CANx==CAN1)
				RCS_CAN_Send_With_Buffer(TxMessage);
			else if (CANx==CAN2)
				RCS_CAN_Send_With_Buffer2(TxMessage);
		#else
			CAN_Transmit(CANx, &TxMessage);
		#endif
}


/**
 @name: RCS_CANx_Send
 @brief: CAN2.0B常规通信,不考虑STD与EXT的兼容性
 @param:CAN_TypeDef *CANx	CAN组
 @param:uint32_t Id	发送给对应ID号
 @param:uint8_t Length 发送数据长度
 @param:uint8_t* sendData  数据
**/
void RCS_CANx_Send(RCS_CAN_T* CANx,uint32_t Id, uint8_t Length, uint8_t* sendData)
{
    if (IS_CAN_STDID(Id))//区分标准帧和拓展帧
    {
        RCS_CANx_Send_STDID(CANx,Id,Length,sendData);
    }
    else
    {
        RCS_CANx_Send_EXTID(CANx,Id,Length,sendData);
    }
}

/**
 * @name:RCS_CANx_Get_CAN_Service_Len
 * @brief:获取当前的服务数量
*/
uint8_t RCS_Get_CAN_Service_Len(RCS_CAN_T* CANx)
{
    if (CANx==CAN1)      return CAN1_Rx_Handler_Len;
    else if (CANx==CAN2) return CAN2_Rx_Handler_Len;
}

/**
 * @name:RCS_CANx_Get_CAN_Service_Handler
 * @brief:获取CANx上第ID个被加入的服务，ID从0开始
*/
CAN_HANDLER_VOID RCS_Get_CAN_Service_Handler(RCS_CAN_T* CANx,uint8_t ID)
{
    if ((ID > CAN1_Rx_Handler_Len)||(ID >= CAN_HANDLER_MAX_LEN)||(ID < 0))
    {
        #ifdef CAN_DEBUG_PRINT
        RCS_Shell_Logs(&Core407_RCSLIB_Debug,"Warning: attempt to visit out-of-bounds CAN Service");
        #endif
        uint8_t err=10/0.0f;
    }
    else
    {
        if (CANx==CAN1)
        {
            return CAN1_Rx_Data_Handler_Arr[ID];
        }
        else if (CANx==CAN1)
        {
            return CAN2_Rx_Data_Handler_Arr[ID];
        }
    }
}

/**
 * @name:RCS_Judge_CAN_Service
 * @brief:判断这个服务是否已经在队列中
*/
uint8_t RCS_Judge_CAN_Service(RCS_CAN_T* CANx,CAN_HANDLER_VOID Handler)
{
    int visit_i;

	for(visit_i = 0;visit_i < RCS_Get_CAN_Service_Len(CANx); visit_i++)
	{
		if (CANx==CAN1)
			if (RCS_Get_CAN_Service_Handler(CANx,visit_i) == Handler) return 1;
		else if (CANx==CAN2)
			if (RCS_Get_CAN_Service_Handler(CANx,visit_i) == Handler) return 1;
	}
    
    return 0;
}

/* ======静态函数定义========================================*/
/**
 * @name:RCS_CANx_Send_With_Buffer
 * @brief:在CAN1上使用环形队列定时发送CAN报文
 * @param:RCS_CAN2B_DATA_FM_TX Tx_Msg  待发送的报文  
*/
static void RCS_CAN_Send_With_Buffer(RCS_CAN2B_DATA_FM_TX Tx_Msg)
{
	StopTimer(CAN1_BUFFER_TIMER);//避免在Quene_Add_Member未执行完的时候就进定时中断
	TIM_SetCounter(CAN1_BUFFER_TIMER,0);
	Quene_Add_Member(&CAN1_Tx_Buffer,(void*)(&Tx_Msg));
	
	#ifdef CAN_DEBUG_PRINT
		if (CAN1_Tx_Buffer.top == CAN1_Tx_Buffer.bottom) RCS_Shell_Logs(&Core407_RCSLIB_Debug,"RCS_CAN.c:CAN1 Buffer overflow");
	#endif
	
	StartTimer(CAN1_BUFFER_TIMER);
}

/**
 * @name:RCS_CAN_Send_With_Buffer2
 * @brief:在CAN2上使用环形队列定时发送CAN报文
 * @param:RCS_CAN2B_DATA_FM_TX Tx_Msg  待发送的报文  
*/
static void RCS_CAN_Send_With_Buffer2(RCS_CAN2B_DATA_FM_TX Tx_Msg)
{
  StopTimer(CAN2_BUFFER_TIMER);
	TIM_SetCounter(CAN2_BUFFER_TIMER,0);
	
	Quene_Add_Member(&CAN2_Tx_Buffer,(void*)(&Tx_Msg));
	
	#ifdef CAN_DEBUG_PRINT
		if (CAN2_Tx_Buffer.top == CAN2_Tx_Buffer.bottom) RCS_Shell_Logs(&Core407_RCSLIB_Debug,"RCS_CAN.c:CAN2 Buffer overflow");
	#endif
	
	StartTimer(CAN2_BUFFER_TIMER);
}

/**
 * @name:RCS_CAN1_Interface_Rx_Server
 * @brief:依次执行CAN1上已注册的服务
*/
static void RCS_CAN1_Interface_Rx_Server(RCS_CAN2B_DATA_FM_RX Rx_Msg)
{
    for (int i=0; i<CAN1_Rx_Handler_Len; i++)
    {
        CAN1_Rx_Data_Handler_Arr[i](Rx_Msg);
    }
}

/**
 * @name:RCS_CAN2_Interface_Rx_Server
 * @brief:依次执行CAN1上已注册的服务
*/
static void RCS_CAN2_Interface_Rx_Server(RCS_CAN2B_DATA_FM_RX Rx_Msg)
{
    for (int i=0; i<CAN2_Rx_Handler_Len; i++)
    {
        CAN2_Rx_Data_Handler_Arr[i](Rx_Msg);
    }
}

/* ======回调函数定义========================================*/
/**
 * @name:CAN1_Buffer_Tx_Isr
 * @brief:CAN1队列发送定时器回调函数
*/
static void CAN1_Buffer_Tx_Isr(void)
{
    RCS_CAN2B_DATA_FM_TX* CAN1_Tx_Data;
    CAN1_Tx_Data=(RCS_CAN2B_DATA_FM_TX*)Quene_Quit_Member(&CAN1_Tx_Buffer);
	
    if (CAN1_Tx_Data == QUENE_MEMBER_EMPTY)
    {
        StopTimer(CAN1_BUFFER_TIMER);
    }
    else
    {
        CAN_Transmit(CAN1,CAN1_Tx_Data);
    }
    TIM_ClearITPendingBit(CAN1_BUFFER_TIMER,TIM_IT_Update);
}

/**
 * @name:CAN1_Buffer_Tx_Isr
 * @brief:CAN2队列发送定时器回调函数
*/
static void CAN2_Buffer_Tx_Isr(void)
{
    RCS_CAN2B_DATA_FM_TX* CAN2_Tx_Data;
    CAN2_Tx_Data=(RCS_CAN2B_DATA_FM_TX*)Quene_Quit_Member(&CAN2_Tx_Buffer);
	
    if (CAN2_Tx_Data == QUENE_MEMBER_EMPTY)
    {
        StopTimer(CAN2_BUFFER_TIMER);
    }
    else
    {
        CAN_Transmit(CAN2,CAN2_Tx_Data);
    }
    TIM_ClearITPendingBit(CAN2_BUFFER_TIMER,TIM_IT_Update);
}

/**
 * @name:CAN1_Rx_Service_Handler
 * @brief:CAN1中断回调函数
 * @tips:注册的CAN服务会在中断函数中被依次执行
*/
void CAN1_Rx_Service_Handler(void)
{
    RCS_CAN2B_DATA_FM_RX rx_message;
	if (RCS_CAN_RXIT_FLAG(CAN1) != RESET)
    {
        RCS_CANx_Recieve(CAN1, &rx_message);
        RCS_CAN1_Interface_Rx_Server(rx_message);
    }
}

/**
 * @name:CAN2_Rx_Service_Handler
 * @brief:CAN2中断回调函数
 * @tips:注册的CAN服务会在中断函数中被依次执行
*/
void CAN2_Rx_Service_Handler(void)
{
    RCS_CAN2B_DATA_FM_RX rx_message;
	if (RCS_CAN_RXIT_FLAG(CAN2) != RESET)
    {
        RCS_CANx_Recieve(CAN2, &rx_message);
        RCS_CAN2_Interface_Rx_Server(rx_message);
    }
}

