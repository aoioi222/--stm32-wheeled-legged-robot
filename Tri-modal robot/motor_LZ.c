/**
 *@filename:  motor_LZ.c
 *@author     适配RCS框架
 *@brief:     CAN网络上的灵足(LZ)电机服务
 *
 *@changlog:  2026-02-18 适配RCS_CAN框架，参考RCS_MOTOR.c设计风格
 *@brief:     支持LZ-02/LZ-05型号电机
 *@brief:     灵足电机使用CAN2.0B扩展帧，与RM电机标准帧无ID冲突
 *@brief:     LZ-02参数范围：角度±12.57rad、速度±44rad/s、力矩±17N·m
 *@brief:     LZ-05参数范围：角度±12.57rad、速度±50rad/s、力矩±5.5N·m
 *
 *@changelog: 2026-02-18 提供统一的防呆框架，适配RCS_CAN回调机制
 *@brief:     建议调用LZMotor_Update和LZMotor_Execute发送指令
*/

/* ===============头文件=========================================*/
#include "motor_LZ.h"

/* ===============私有全局变量===================================*/
// 电机基础配置
static Motor_LZ_Init g_motor_LZ_config[MAX_LZMOTOR_ON_CAN] = {
    {2, MOTOR_LZ_02}  // 默认配置：ID2，LZ-02型号
};

// 发送参数缓存（CAN1/CAN2适配）
static user_send_Lz g_motor_LZ_send_param[MAX_LZMOTOR_ON_CAN][CAN_PERI_COUNT] = {0};
// 接收参数缓存（CAN1/CAN2适配）
static user_recv_Lz g_motor_LZ_recv_data[MAX_LZMOTOR_ON_CAN][CAN_PERI_COUNT] = {0};
// 电机使能状态
static uint8_t g_motor_LZ_enable[MAX_LZMOTOR_ON_CAN][CAN_PERI_COUNT] = {0};
// 电机ID映射表（物理ID→数组索引）
static uint8_t g_motor_LZ_id_seat[MAX_LZMOTOR_ON_CAN] = {0};

/* ===============静态函数声明===================================*/
static void LZ_Motor_Param_Init(CAN_TypeDef* CANx);
static void LZ_Motor_Send_Handle(uint8_t idx, uint8_t can_chose);
static void LZ_Motor_Recv_Parse(uint8_t idx, uint8_t can_chose, uint32_t ext_id, uint8_t* data);


/**
 * @name:LZMotor_CAN_Init
 * @brief:初始化灵足电机（适配CAN1/CAN2）
 * @param:CAN_TypeDef* CANx - CAN1/CAN2外设
*/
void LZMotor_CAN_Init(CAN_TypeDef* CANx)
{
    // 1. 初始化电机参数
    LZ_Motor_Param_Init(CANx);
    
    // 2. 配置CAN接收回调服务
    if (CANx == CAN1)
    {
        if (RCS_Judge_CAN_Service(CAN1, LZ_Motor_Data_Filter1) == 0)
            RCS_CANx_Add_Handler(CAN1, LZ_Motor_Data_Filter1);
    }
    else if (CANx == CAN2)
    {
        if (RCS_Judge_CAN_Service(CAN2, LZ_Motor_Data_Filter2) == 0)
            RCS_CANx_Add_Handler(CAN2, LZ_Motor_Data_Filter2);
    }

    // 3. 配置CAN外设（引脚映射+波特率）
    if (CANx == CAN1)
    {
        RCS_CANx_Config_With_Buffer(CAN1_MAP, CAN1_BAUD);
    }
    else if (CANx == CAN2)
    {
        RCS_CANx_Config_With_Buffer(CAN2_MAP, CAN2_BAUD);
    }

    // 4. 等待电机反馈，初始化使能状态
    delay_ms(10);
    uint8_t can_chose = (CANx == CAN1) ? 0 : 1;
    for (uint8_t i = 0; i < MAX_LZMOTOR_ON_CAN; i++)
    {
        uint8_t motor_id = g_motor_LZ_config[i].id;
        if (motor_id == 0) continue; // 跳过未配置ID
        
        // 等待温度反馈（确认电机在线）
        while (g_motor_LZ_recv_data[i][can_chose].Temp == 0)
        {
            LZMotor_Enable(CANx, motor_id);
            delay_ms(1);
        }
        
        // 开启主动上报
        LZMotor_Active_Recv(CANx, motor_id, 1);
        g_motor_LZ_enable[i][can_chose] = 1;
        
        // 初始化角度为当前反馈值
        g_motor_LZ_send_param[i][can_chose].Angle = g_motor_LZ_recv_data[i][can_chose].Angle;
    }
}

/**
 * @name:LZMotor_Update_Param
 * @brief:更新灵足电机控制参数（角度/速度/Kp/Kd/力矩）
 * @param:CAN_TypeDef* CANx - CAN1/CAN2外设
 * @param:uint8_t id - 电机物理ID
 * @param:user_send_Lz* param - 控制参数指针
*/
void LZMotor_Update_Param(CAN_TypeDef* CANx, uint8_t id, user_send_Lz* param)
{
    if (id >= MAX_LZMOTOR_ON_CAN) return; // 防呆：ID越界检查
    uint8_t can_chose = (CANx == CAN1) ? 0 : 1;
    uint8_t idx = g_motor_LZ_id_seat[id];
    
    // 参数限幅（灵足电机硬件量程）
    switch (g_motor_LZ_config[idx].model)
    {
        case MOTOR_LZ_02:
            param->Angle = (param->Angle > 12.57f) ? 12.57f : (param->Angle < -12.57f ? -12.57f : param->Angle);
            param->W = (param->W > 44.0f) ? 44.0f : (param->W < -44.0f ? -44.0f : param->W);
            param->T = (param->T > 17.0f) ? 17.0f : (param->T < -17.0f ? -17.0f : param->T);
            param->Kp = (param->Kp > 500.0f) ? 500.0f : (param->Kp < 0.0f ? 0.0f : param->Kp);
            param->Kd = (param->Kd > 5.0f) ? 5.0f : (param->Kd < 0.0f ? 0.0f : param->Kd);
            break;
        case MOTOR_LZ_05:
            param->Angle = (param->Angle > 12.57f) ? 12.57f : (param->Angle < -12.57f ? -12.57f : param->Angle);
            param->W = (param->W > 50.0f) ? 50.0f : (param->W < -50.0f ? -50.0f : param->W);
            param->T = (param->T > 5.5f) ? 5.5f : (param->T < -5.5f ? -5.5f : param->T);
            param->Kp = (param->Kp > 500.0f) ? 500.0f : (param->Kp < 0.0f ? 0.0f : param->Kp);
            param->Kd = (param->Kd > 5.0f) ? 5.0f : (param->Kd < 0.0f ? 0.0f : param->Kd);
            break;
        default:
            return;
    }
    
    // 更新缓存参数
    g_motor_LZ_send_param[idx][can_chose] = *param;
}

/**
 * @name:LZMotor_Execute
 * @brief:发送灵足电机控制指令（对齐RCS_MOTOR的Excute风格）
 * @param:CAN_TypeDef* CANx - CAN1/CAN2外设
 * @param:uint8_t id - 电机物理ID
*/
void LZMotor_Execute(CAN_TypeDef* CANx, uint8_t id)
{
    if (id >= MAX_LZMOTOR_ON_CAN) return;
    uint8_t can_chose = (CANx == CAN1) ? 0 : 1;
    uint8_t idx = g_motor_LZ_id_seat[id];
    
    // 未使能则不发送
    if (g_motor_LZ_enable[idx][can_chose] == 0) return;
    
    // 处理并发送指令
    LZ_Motor_Send_Handle(idx, can_chose);
}

/**
 * @name:LZMotor_Enable
 * @brief:使能灵足电机
 * @param:CAN_TypeDef* CANx - CAN1/CAN2外设
 * @param:uint8_t id - 电机物理ID
*/
void LZMotor_Enable(CAN_TypeDef* CANx, uint8_t id)
{
    if (id >= MAX_LZMOTOR_ON_CAN) return;
    uint8_t can_chose = (CANx == CAN1) ? 0 : 1;
    uint8_t idx = g_motor_LZ_id_seat[id];
    
    // 构造使能指令帧
    RCS_CAN2B_DATA_FM_TX tx_msg = {0};
    tx_msg.IDE = CAN_Id_Extended;  // 灵足电机用扩展帧
    tx_msg.RTR = CAN_RTR_Data;
    tx_msg.DLC = 8;
    tx_msg.ExtId = (CANCOM_MOTOR_IN << 24) | (motor_LZ_user_id << 8) | id;
    
    // 发送使能指令
    RCS_CANx_Send_EXTID(CANx, tx_msg.ExtId, 8, tx_msg.Data);
    g_motor_LZ_enable[idx][can_chose] = 1;
}

/**
 * @name:LZMotor_Disable
 * @brief:失能灵足电机
 * @param:CAN_TypeDef* CANx - CAN1/CAN2外设
 * @param:uint8_t id - 电机物理ID
*/
void LZMotor_Disable(CAN_TypeDef* CANx, uint8_t id)
{
    if (id >= MAX_LZMOTOR_ON_CAN) return;
    uint8_t can_chose = (CANx == CAN1) ? 0 : 1;
    uint8_t idx = g_motor_LZ_id_seat[id];
    
    // 构造失能指令帧
    RCS_CAN2B_DATA_FM_TX tx_msg = {0};
    tx_msg.IDE = CAN_Id_Extended;
    tx_msg.RTR = CAN_RTR_Data;
    tx_msg.DLC = 8;
    tx_msg.ExtId = (CANCOM_MOTOR_RESET << 24) | (motor_LZ_user_id << 8) | id;
    
    // 发送失能指令
    RCS_CANx_Send_EXTID(CANx, tx_msg.ExtId, 8, tx_msg.Data);
    g_motor_LZ_enable[idx][can_chose] = 0;
}

/**
 * @name:LZMotor_Active_Recv
 * @brief:开启/关闭灵足电机主动上报
 * @param:CAN_TypeDef* CANx - CAN1/CAN2外设
 * @param:uint8_t id - 电机物理ID
 * @param:uint8_t en - 1开启/0关闭
*/
void LZMotor_Active_Recv(CAN_TypeDef* CANx, uint8_t id, uint8_t en)
{
    if (id >= MAX_LZMOTOR_ON_CAN) return;
    
    // 构造主动上报指令帧
    RCS_CAN2B_DATA_FM_TX tx_msg = {0};
    tx_msg.IDE = CAN_Id_Extended;
    tx_msg.RTR = CAN_RTR_Data;
    tx_msg.DLC = 8;
    tx_msg.ExtId = (CANCOM_MODE_ACTIVE_RECV << 24) | (motor_LZ_user_id << 8) | id;
    
    // 填充数据域
    for (uint8_t i = 0; i < 6; i++) tx_msg.Data[i] = i + 1;
    tx_msg.Data[6] = (en == 1) ? 1 : 0;
    tx_msg.Data[7] = 0;
    
    // 发送指令
    RCS_CANx_Send_EXTID(CANx, tx_msg.ExtId, 8, tx_msg.Data);
}

/**
 * @name:LZMotor_Set_Zero
 * @brief:设置灵足电机机械零点
 * @param:CAN_TypeDef* CANx - CAN1/CAN2外设
 * @param:uint8_t id - 电机物理ID
*/
void LZMotor_Set_Zero(CAN_TypeDef* CANx, uint8_t id)
{
    if (id >= MAX_LZMOTOR_ON_CAN) return;
    
    // 先失能
    LZMotor_Disable(CANx, id);
    
    // 构造零点设置指令帧
    RCS_CAN2B_DATA_FM_TX tx_msg = {0};
    tx_msg.IDE = CAN_Id_Extended;
    tx_msg.RTR = CAN_RTR_Data;
    tx_msg.DLC = 8;
    tx_msg.ExtId = (CANCOM_MOTOR_ZERO << 24) | (motor_LZ_user_id << 8) | id;
    tx_msg.Data[0] = 1;
    
    // 发送指令
    RCS_CANx_Send_EXTID(CANx, tx_msg.ExtId, 8, tx_msg.Data);
    
    // 重新使能
    LZMotor_Enable(CANx, id);
}

/**
 * @name:LZMotor_Get_Recv_Data
 * @brief:获取灵足电机反馈数据
 * @param:CAN_TypeDef* CANx - CAN1/CAN2外设
 * @param:uint8_t id - 电机物理ID
 * @retval:user_recv_Lz* - 反馈数据指针（NULL表示ID无效）
*/
user_recv_Lz* LZMotor_Get_Recv_Data(CAN_TypeDef* CANx, uint8_t id)
{
    if (id >= MAX_LZMOTOR_ON_CAN) return NULL;
    uint8_t can_chose = (CANx == CAN1) ? 0 : 1;
    uint8_t idx = g_motor_LZ_id_seat[id];
    
    return &g_motor_LZ_recv_data[idx][can_chose];
}

/* ===============静态函数定义（内部使用）===================================*/

/**
 * @name:LZ_Motor_Param_Init
 * @brief:初始化电机参数缓存和ID映射
 * @param:CAN_TypeDef* CANx - CAN1/CAN2外设
*/
static void LZ_Motor_Param_Init(CAN_TypeDef* CANx)
{
    uint8_t can_chose = (CANx == CAN1) ? 0 : 1;
    
    // 初始化参数缓存和ID映射
    for (uint8_t i = 0; i < MAX_LZMOTOR_ON_CAN; i++)
    {
        uint8_t motor_id = g_motor_LZ_config[i].id;
        if (motor_id == 0) continue;
        
        // ID映射：物理ID→数组索引
        g_motor_LZ_id_seat[motor_id] = i;
        
        // 初始化发送参数
        g_motor_LZ_send_param[i][can_chose].mode = CANCOM_MOTOR_CTRL;
        g_motor_LZ_send_param[i][can_chose].Angle = 0.0f;
        g_motor_LZ_send_param[i][can_chose].W = 0.0f;
        g_motor_LZ_send_param[i][can_chose].T = 0.0f;
        g_motor_LZ_send_param[i][can_chose].Kp = 0.0f;
        g_motor_LZ_send_param[i][can_chose].Kd = 0.0f;
        
        // 初始化接收参数
        memset(&g_motor_LZ_recv_data[i][can_chose], 0, sizeof(user_recv_Lz));
        
        // 初始化使能状态
        g_motor_LZ_enable[i][can_chose] = 0;
    }
}

/**
 * @name:LZ_Motor_Send_Handle
 * @brief:处理控制参数并发送CAN指令（内部函数）
 * @param:uint8_t id - 数组索引
 * @param:uint8_t can_chose - 0=CAN1/1=CAN2
*/
static void LZ_Motor_Send_Handle(uint8_t idx, uint8_t can_chose)
{
    RCS_CAN2B_DATA_FM_TX tx_msg = {0};
    uint16_t motor_data[5] = {0};
    Motor_LZ_Model model = g_motor_LZ_config[idx].model;
    uint8_t motor_id = g_motor_LZ_config[idx].id;
    user_send_Lz* param = &g_motor_LZ_send_param[idx][can_chose];
    
    // 1. 参数量化（浮点→电机识别的16位整数）
    switch (model)
    {
        case MOTOR_LZ_02:
            motor_data[0] = (uint16_t)((param->Angle + 12.57f) * 65535 / 25.14f);
            motor_data[1] = (uint16_t)((param->W + 44.0f) * 65535 / 88.0f);
            motor_data[2] = (uint16_t)((param->T + 17.0f) * 65535 / 34.0f);
            motor_data[3] = (uint16_t)(param->Kp * 65535 / 500.0f);
            motor_data[4] = (uint16_t)(param->Kd * 65535 / 5.0f);
            break;
        case MOTOR_LZ_05:
            motor_data[0] = (uint16_t)((param->Angle + 12.57f) * 65535 / 25.14f);
            motor_data[1] = (uint16_t)((param->W + 50.0f) * 65535 / 100.0f);
            motor_data[2] = (uint16_t)((param->T + 5.5f) * 65535 / 11.0f);
            motor_data[3] = (uint16_t)(param->Kp * 65535 / 500.0f);
            motor_data[4] = (uint16_t)(param->Kd * 65535 / 5.0f);
            break;
        default:
            return;
    }
    
    // 2. 配置CAN扩展帧ID
    tx_msg.IDE = CAN_Id_Extended;
    tx_msg.RTR = CAN_RTR_Data;
    tx_msg.DLC = 8;
    tx_msg.ExtId = (CANCOM_MOTOR_CTRL << 24) | (motor_data[2] << 8) | motor_id;
    
    // 3. 填充8字节数据域
    tx_msg.Data[0] = motor_data[0] >> 8;
    tx_msg.Data[1] = motor_data[0] & 0xFF;
    tx_msg.Data[2] = motor_data[1] >> 8;
    tx_msg.Data[3] = motor_data[1] & 0xFF;
    tx_msg.Data[4] = motor_data[3] >> 8;
    tx_msg.Data[5] = motor_data[3] & 0xFF;
    tx_msg.Data[6] = motor_data[4] >> 8;
    tx_msg.Data[7] = motor_data[4] & 0xFF;
    
    // 4. 发送CAN指令（区分CAN1/CAN2）
    if (can_chose == 0)
        RCS_CANx_Send_EXTID(CAN1, tx_msg.ExtId, 8, tx_msg.Data);
    else
        RCS_CANx_Send_EXTID(CAN2, tx_msg.ExtId, 8, tx_msg.Data);
}

/**
 * @name:LZ_Motor_Recv_Parse
 * @brief:解析电机反馈数据（内部函数）
 * @param:uint8_t id - 数组索引
 * @param:uint8_t can_chose - 0=CAN1/1=CAN2
 * @param:uint32_t ext_id - CAN扩展ID
 * @param:uint8_t* data - 8字节反馈数据
*/
static void LZ_Motor_Recv_Parse(uint8_t idx, uint8_t can_chose, uint32_t ext_id, uint8_t* data)
{
    user_recv_Lz* recv = &g_motor_LZ_recv_data[idx][can_chose];
    Motor_LZ_Model model = g_motor_LZ_config[idx].model;
    
    // 解析故障码和模式
    recv->MError = (ext_id >> 16) & 0x3F;
    recv->mode = (ext_id >> 22) & 0x03;
    
    // 解析角度（通用）
    recv->Angle = (float)((data[0] << 8) | data[1]) * 12.57f / 32768 - 12.57f;
    
    // 解析速度和力矩（区分型号）
    switch (model)
    {
        case MOTOR_LZ_02:
            recv->W = (float)((data[2] << 8) | data[3]) * 11.0f / 8192 - 44.0f;
            recv->T = (float)((data[4] << 8) | data[5]) * 17.0f / 32768 - 17.0f;
            break;
        case MOTOR_LZ_05:
            recv->W = (float)((data[2] << 8) | data[3]) * 12.5f / 8192 - 50.0f;
            recv->T = (float)((data[4] << 8) | data[5]) * 5.5f / 32768 - 5.5f;
            break;
        default:
            break;
    }
    
    // 解析温度
    recv->Temp = (float)((data[6] << 8) | data[7]) / 10.0f;
}

/* ===============CAN回调过滤函数（对齐RCS_MOTOR）===================================*/

/**
 * @name:LZ_Motor_Data_Filter1
 * @brief:CAN1灵足电机数据过滤回调
 * @param:RCS_CAN2B_DATA_FM_RX rx_message - 接收报文
*/
void LZ_Motor_Data_Filter1(RCS_CAN2B_DATA_FM_RX rx_message)
{
    // 仅处理扩展帧、数据帧、8字节长度
    if (rx_message.IDE != CAN_Id_Extended || rx_message.RTR != CAN_RTR_Data || rx_message.DLC != 8)
        return;
    
    // 解析电机ID和指令模式
    uint8_t motor_id = rx_message.ExtId & 0xFF;
    uint8_t mode = (rx_message.ExtId >> 24) & 0x1F;
    
    // 仅处理主动上报/反馈帧
    if (mode != CANCOM_MODE_ACTIVE_RECV && mode != CANCOM_MOTOR_FEEDBACK)
        return;
    
    // 验证ID有效性
    if (motor_id >= MAX_LZMOTOR_ON_CAN || g_motor_LZ_id_seat[motor_id] >= MAX_LZMOTOR_ON_CAN)
        return;
    
    // 解析反馈数据
    uint8_t idx = g_motor_LZ_id_seat[motor_id];
    LZ_Motor_Recv_Parse(idx, 0, rx_message.ExtId, rx_message.Data);
}

/**
 * @name:LZ_Motor_Data_Filter2
 * @brief:CAN2灵足电机数据过滤回调
 * @param:RCS_CAN2B_DATA_FM_RX rx_message - 接收报文
*/
void LZ_Motor_Data_Filter2(RCS_CAN2B_DATA_FM_RX rx_message)
{
    // 仅处理扩展帧、数据帧、8字节长度
    if (rx_message.IDE != CAN_Id_Extended || rx_message.RTR != CAN_RTR_Data || rx_message.DLC != 8)
        return;
    
    // 解析电机ID和指令模式
    uint8_t motor_id = rx_message.ExtId & 0xFF;
    uint8_t mode = (rx_message.ExtId >> 24) & 0x1F;
    
    // 仅处理主动上报/反馈帧
    if (mode != CANCOM_MODE_ACTIVE_RECV && mode != CANCOM_MOTOR_FEEDBACK)
        return;
    
    // 验证ID有效性
    if (motor_id >= MAX_LZMOTOR_ON_CAN || g_motor_LZ_id_seat[motor_id] >= MAX_LZMOTOR_ON_CAN)
        return;
    
    // 解析反馈数据
    uint8_t idx = g_motor_LZ_id_seat[motor_id];
    LZ_Motor_Recv_Parse(idx, 1, rx_message.ExtId, rx_message.Data);
}