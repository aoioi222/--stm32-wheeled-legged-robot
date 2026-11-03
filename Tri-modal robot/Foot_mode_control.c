/**
 * @filename: Foot_mode_control.c
 * @brief:   轮足机器人足式模式舵机底层控制
 * @details: 幻尔 HTS-35H 串行总线舵机, 半双工USART3通信
 *           接收上位机下发的12关节角度, 转换为舵机位置指令发送
 *           HTS-35H协议: 0x55 0x55 ID Len Cmd Data[] Checksum
 *           采用DMA非阻塞发送, 消除阻塞等待对控制周期的拖累
 */

#include "Foot_mode_control.h"
#include "RCS_Usart.h"

/* 舵机DMA发送: USART3_TX 映射到 DMA1_Stream3_Channel4 */
#define SERVO_DMA_STREAM    DMA1_Stream3
#define SERVO_DMA_CHANNEL   DMA_Channel_4
#define SERVO_DMA_FLAG_TC   DMA_FLAG_TCIF3
#define SERVO_PKT_LEN       10  /* 单帧字节数: 帧头2+id+len+cmd+参数4+校验1 */

/* 阻塞发送的等待限时(us)：SERVO_BAUD=1000000时一个字节10us，
   限时留足余量又不至于让最高优先级任务长时间卡死 */
#define SERVO_TX_POLL_US        5U      /* 标志轮询间隔 */
#define SERVO_TX_BYTE_TIMEOUT_US   500U  /* 单字节最多等 0.5ms */
#define SERVO_TX_IDLE_TIMEOUT_US  2000U  /* 等待总线空闲最多等 2ms */

/* DMA发送缓冲与忙标志 */
static uint8_t s_servo_tx_buf[SERVO_COUNT * SERVO_PKT_LEN];
static volatile uint8_t s_servo_dma_busy = 0;

/* 舵机ID查找表, 对应 joint_angle[0..11] */
static const uint8_t servo_id[SERVO_COUNT] = {
    SERVO_ID_RF_HIP_ABD, SERVO_ID_RF_HIP_ROT, SERVO_ID_RF_KNEE,
    SERVO_ID_LF_HIP_ABD, SERVO_ID_LF_HIP_ROT, SERVO_ID_LF_KNEE,
    SERVO_ID_RB_HIP_ABD, SERVO_ID_RB_HIP_ROT, SERVO_ID_RB_KNEE,
    SERVO_ID_LB_HIP_ABD, SERVO_ID_LB_HIP_ROT, SERVO_ID_LB_KNEE
};

/* 关节类型查找表, 对应 joint_angle[0..11] */
static const JointType_t servo_joint_type[SERVO_COUNT] = {
    JOINT_HIP_ABD, JOINT_HIP_ROT, JOINT_KNEE,
    JOINT_HIP_ABD, JOINT_HIP_ROT, JOINT_KNEE,
    JOINT_HIP_ABD, JOINT_HIP_ROT, JOINT_KNEE,
    JOINT_HIP_ABD, JOINT_HIP_ROT, JOINT_KNEE
};

/**
 * @brief: 计算舵机协议校验和
 * @param: id   舵机ID
 * @param: len  数据长度 (Cmd + 参数)
 * @param: cmd  指令
 * @param: data 参数字节数组
 * @retval: ~(ID+Len+Cmd+Sum(Data)) & 0xFF
 */
static uint8_t servo_checksum(uint8_t id, uint8_t len, uint8_t cmd, const uint8_t *data)
{
    uint8_t sum = id + len + cmd;
    uint8_t i;
    uint8_t param_count = len - 1;  /* 去掉cmd占的1字节 */
    for (i = 0; i < param_count; i++) {
        sum += data[i];
    }
    return ~sum;
}

/**
 * @brief: 限时等待USART发送完成标志
 * @param: flag        要等待的标志 (USART_FLAG_TXE / USART_FLAG_TC)
 * @param: timeout_us  最长等待时间(us)
 * @retval: 0=标志已置位，-1=超时
 * @note:   原实现用裸 while 死等且无超时，位于最高优先级任务内，
 *          一旦标志始终不来(总线被拉低、外设时钟异常)会永久卡死整个系统，
 *          这里改为微秒级限时轮询，超时则放弃本次阻塞发送。
 */
static int servo_wait_flag(uint16_t flag, uint32_t timeout_us)
{
    uint32_t waited_us = 0;

    while (USART_GetFlagStatus(SERVO_USART, flag) == RESET) {
        delay_us(SERVO_TX_POLL_US);
        waited_us += SERVO_TX_POLL_US;
        if (waited_us >= timeout_us)
            return -1;
    }
    return 0;
}

/**
 * @brief: 限时发送一个字节
 * @param: byte  待发送数据
 * @retval: 0=成功，-1=超时
 */
static int servo_write_byte(uint8_t byte)
{
    /* 先等发送数据寄存器空，避免在上一字节未取走时覆盖DR */
    if (servo_wait_flag(USART_FLAG_TXE, SERVO_TX_BYTE_TIMEOUT_US) != 0)
        return -1;
    USART_SendData(SERVO_USART, byte);
    /* 再等发送完成，保证整帧按字节间距发出 */
    return servo_wait_flag(USART_FLAG_TC, SERVO_TX_BYTE_TIMEOUT_US);
}

/**
 * @brief: 阻塞发送一帧舵机指令 (仅紧急卸载时使用)
 * @param: id   舵机ID
 * @param: len  数据长度 (Cmd + 参数)
 * @param: cmd  指令码
 * @param: data 参数字节数组
 * @retval: 0=整帧发送完成，-1=中途超时(已放弃本帧)
 */
static int servo_send_packet(uint8_t id, uint8_t len, uint8_t cmd, const uint8_t *data)
{
    uint8_t i;

    /* 等待总线空闲 */
    if (servo_wait_flag(USART_FLAG_TC, SERVO_TX_IDLE_TIMEOUT_US) != 0)
        return -1;

    /* 帧头 */
    if (servo_write_byte(0x55) != 0) return -1;
    if (servo_write_byte(0x55) != 0) return -1;
    /* ID */
    if (servo_write_byte(id) != 0) return -1;
    /* 长度 */
    if (servo_write_byte(len) != 0) return -1;
    /* 指令 */
    if (servo_write_byte(cmd) != 0) return -1;
    /* 参数 */
    for (i = 0; i < len - 1; i++) {
        if (servo_write_byte(data[i]) != 0) return -1;
    }
    /* 校验和 */
    if (servo_write_byte(servo_checksum(id, len, cmd, data)) != 0) return -1;
    return 0;
}

/**
 * @brief: 构建单舵机位置指令到DMA发送缓冲 (非阻塞)
 * @param: buf 写入缓冲区
 * @param: id  舵机ID
 * @param: pos 目标位置 (0-1000)
 * @retval: 写入字节数
 */
static uint16_t servo_build_packet(uint8_t *buf, uint8_t id, uint16_t pos)
{
    uint8_t data[4];
    uint8_t i;
    uint16_t n = 0;
    data[0] = (uint8_t)(pos & 0xFF);
    data[1] = (uint8_t)((pos >> 8) & 0xFF);
    data[2] = (uint8_t)(SERVO_MOVE_TIME & 0xFF);
    data[3] = (uint8_t)((SERVO_MOVE_TIME >> 8) & 0xFF);

    buf[n++] = 0x55;               /* 帧头 */
    buf[n++] = 0x55;
    buf[n++] = id;                 /* ID */
    buf[n++] = 5;                  /* len = cmd(1) + 参数(4) */
    buf[n++] = 1;                  /* cmd=1 位置指令 */
    for (i = 0; i < 4; i++)
        buf[n++] = data[i];        /* 位置2字节 + 时间2字节 */
    buf[n++] = servo_checksum(id, 5, 1, data);
    return n;
}

/**
 * @brief: 发送单舵机卸载指令 (阻塞, 紧急停止用)
 * @param: id    舵机ID
 * @param: load  0=卸载, 1=加载
 * @retval: 0=发送完成，-1=超时
 */
static int servo_set_load(uint8_t id, uint8_t load)
{
    uint8_t data[1];
    data[0] = load;
    return servo_send_packet(id, 2, 31, data);  /* len=2: cmd(1) + param(1) */
}

/**
 * @brief: 关节角度转舵机位置值
 * @param: angle 关节角度
 * @retval: 舵机位置 (0-1000)
 */
static uint16_t angle_to_position(float angle)
{
    float pos = SERVO_POS_MID + (angle / (SERVO_ANGLE_RANGE * 0.5f)) * (SERVO_POS_RANGE * 0.5f);
    if (pos < 0.0f)    pos = 0.0f;
    if (pos > 1000.0f) pos = 1000.0f;
    return (uint16_t)pos;
}

/**
 * @brief: 初始化舵机DMA发送通道 (USART3_TX = DMA1_Stream3_Channel4)
 */
static void servo_dma_init(void)
{
    DMA_InitTypeDef dma;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    DMA_DeInit(SERVO_DMA_STREAM);
    while (DMA_GetCmdStatus(SERVO_DMA_STREAM) != DISABLE) {}

    DMA_StructInit(&dma);
    dma.DMA_Channel = SERVO_DMA_CHANNEL;
    dma.DMA_PeripheralBaseAddr = (uint32_t)&(SERVO_USART->DR);
    dma.DMA_Memory0BaseAddr = (uint32_t)s_servo_tx_buf;
    dma.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    dma.DMA_BufferSize = 0;                 /* 每次发送前动态设置 */
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Normal;
    dma.DMA_Priority = DMA_Priority_High;
    dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(SERVO_DMA_STREAM, &dma);

    USART_DMACmd(SERVO_USART, USART_DMAReq_Tx, ENABLE);
}

/**
 * @brief: 启动DMA后台发送
 * @param: len 发送字节数
 */
static void servo_dma_send(uint16_t len)
{
    DMA_Cmd(SERVO_DMA_STREAM, DISABLE);
    while (DMA_GetCmdStatus(SERVO_DMA_STREAM) != DISABLE) {}
    DMA_SetCurrDataCounter(SERVO_DMA_STREAM, len);
    DMA_ClearFlag(SERVO_DMA_STREAM, SERVO_DMA_FLAG_TC);
    USART_DMACmd(SERVO_USART, USART_DMAReq_Tx, ENABLE);
    DMA_Cmd(SERVO_DMA_STREAM, ENABLE);
    s_servo_dma_busy = 1;
}

/**
 * @name: Foot_Mode_Init
 * @brief: 初始化舵机总线USART3 (半双工, 115200bps) 及DMA发送通道
 */
void Foot_Mode_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    /* 时钟使能 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    /* TX pin (PD8): 推挽复用 */
    GPIO_InitStructure.GPIO_Pin   = SERVO_PIN_TX;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SERVO_GPIO, &GPIO_InitStructure);

    /* RX pin (PD9): 半双工总线监听, 开漏不拉 */
    GPIO_InitStructure.GPIO_Pin   = SERVO_PIN_RX;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SERVO_GPIO, &GPIO_InitStructure);

    GPIO_PinAFConfig(SERVO_GPIO, GPIO_PinSource8, GPIO_AF_USART3);
    GPIO_PinAFConfig(SERVO_GPIO, GPIO_PinSource9, GPIO_AF_USART3);

    /* USART3 配置: 仅发送 */
    USART_StructInit(&USART_InitStructure);
    USART_InitStructure.USART_BaudRate            = SERVO_BAUD;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Tx;
    USART_Init(SERVO_USART, &USART_InitStructure);
    USART_Cmd(SERVO_USART, ENABLE);

    /* 初始化DMA发送通道 */
    servo_dma_init();
}

/**
 * @name: Foot_Mode_Control
 * @brief: 足式模式舵机控制主函数, 每个控制周期调用, DMA非阻塞发送
 * @param: joint_angle[12] 12个关节目标角度
 *         顺序: RF(HipAbd,HipRot,Knee), LF(...), RB(...), LB(...)
 */
void Foot_Mode_Control(const float joint_angle[SERVO_COUNT])
{
    uint8_t i;
    uint16_t tx_len = 0;

    /* 上一批DMA未发完则跳过本周期, 舵机靠move_time插值保持 */
    if (s_servo_dma_busy) {
        if (DMA_GetFlagStatus(SERVO_DMA_STREAM, SERVO_DMA_FLAG_TC) == RESET)
            return;
        DMA_ClearFlag(SERVO_DMA_STREAM, SERVO_DMA_FLAG_TC);
        s_servo_dma_busy = 0;
    }

    /* 构建12路舵机指令到发送缓冲 */
    for (i = 0; i < SERVO_COUNT; i++) {
        float clamped = clamp_joint_angle(servo_joint_type[i], joint_angle[i]);
        uint16_t pos   = angle_to_position(clamped);
        tx_len += servo_build_packet(&s_servo_tx_buf[tx_len], servo_id[i], pos);
    }

    /* 启动DMA后台发送, CPU立即返回, 不阻塞控制周期 */
    servo_dma_send(tx_len);
}

/**
 * @name: Foot_Mode_Stop
 * @brief: 紧急停止, 停DMA后阻塞卸载所有舵机扭矩
 * @note:  12路舵机逐个限时发送，某一帧超时只放弃该帧并继续下一路，
 *         避免因单路总线异常卡死最高优先级任务
 */
void Foot_Mode_Stop(void)
{
    uint8_t i;
    uint32_t waited_us = 0;

    /* 停止DMA后台发送, 改阻塞方式紧急卸载 */
    DMA_Cmd(SERVO_DMA_STREAM, DISABLE);
    USART_DMACmd(SERVO_USART, USART_DMAReq_Tx, DISABLE);
    s_servo_dma_busy = 0;

    /* 限时等待在途数据发完, 防止阻塞发送插进DMA帧中间造成总线错帧 */
    while (USART_GetFlagStatus(SERVO_USART, USART_FLAG_TC) == RESET) {
        delay_us(SERVO_TX_POLL_US);
        waited_us += SERVO_TX_POLL_US;
        if (waited_us >= SERVO_TX_IDLE_TIMEOUT_US)
            break;
    }

    for (i = 0; i < SERVO_COUNT; i++) {
        servo_set_load(servo_id[i], 0);
    }
}

/**
 * @name: clamp_joint_angle
 * @brief: 关节角度限幅
 * @param: joint_type  关节类型
 * @param: input_angle 原始角度
 * @retval: 限幅后的角度
 */
float clamp_joint_angle(JointType_t joint_type, float input_angle)
{
    float clamped = input_angle;

    switch (joint_type) {
        case JOINT_HIP_ABD:
            if (input_angle > HIP_ABD_MAX) clamped = HIP_ABD_MAX;
            if (input_angle < HIP_ABD_MIN) clamped = HIP_ABD_MIN;
            break;
        case JOINT_HIP_ROT:
            if (input_angle > HIP_ROT_MAX) clamped = HIP_ROT_MAX;
            if (input_angle < HIP_ROT_MIN) clamped = HIP_ROT_MIN;
            break;
        case JOINT_KNEE:
            if (input_angle > KNEE_MAX) clamped = KNEE_MAX;
            if (input_angle < KNEE_MIN) clamped = KNEE_MIN;
            break;
        default:
            break;
    }

    return clamped;
}
