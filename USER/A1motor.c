#include "A1motor.h"
#include "rcs.h"
#include "Gyro.h"
#include "RCS_Usart.h"
#include <time.h>  

/*---------------全局变量-------------*/
static MotorRecvState current_state = STATE_IDLE;  // 当前状态
static MotorData* motor_data_ptr = NULL;         // 数据指针
static uint8_t recv_buffer[128]= {0};     // 接收缓冲区
static uint16_t recv_index = 0;                  // 接收索引
static uint32_t crc_recv_ptr[4]= {0};      // CRC接收缓冲区
static uint16_t crc_index = 0;                  // CRC索引
RCS_PIN_USART A1_USART_MAP;
static time_t last_recv_time;  // 用于记录上次接收数据的时间
#define TIMEOUT_SEC 1  // 超时时间，单位：秒

/*---------------静态函数-------------*/
static void A1_Interrupt(void);
static int check_timeout();

/*------------------------------------*/

/**
    @name: Struct_MotorCmd
    @brief:初始化A1MotorCmd结构体
**/
void Struct_MotorCmd(MotorCmd* motor_cmd) {
    if (motor_cmd == NULL) {
        return;  
    }
    motor_cmd->motorType = A1_Motor;
    motor_cmd->hex_len = 0;
    motor_cmd->id = 0;
    motor_cmd->mode = 0;
    motor_cmd->tau = 0.0f;
    motor_cmd->dq = 0.0f;
    motor_cmd->q = 0.0f;
    motor_cmd->kp = 0.0f;
    motor_cmd->kd = 0.0f;
    
    motor_cmd->A1B1_motor_send_data.head.start[0] = 0xFE;
    motor_cmd->A1B1_motor_send_data.head.start[1] = 0xEE;
    motor_cmd->A1B1_motor_send_data.head.motorID = 0;
    motor_cmd->A1B1_motor_send_data.head.reserved = 0;
    motor_cmd->A1B1_motor_send_data.Mdata.mode = 0;
    motor_cmd->A1B1_motor_send_data.Mdata.ModifyBit = 0xFF;
    motor_cmd->A1B1_motor_send_data.Mdata.ReadBit = 0;
    motor_cmd->A1B1_motor_send_data.Mdata.reserved = 0;
    memset(&motor_cmd->A1B1_motor_send_data.Mdata.Modify, 0, sizeof(COMData32));
    motor_cmd->A1B1_motor_send_data.Mdata.T = 0;
    motor_cmd->A1B1_motor_send_data.Mdata.W = 0;
    motor_cmd->A1B1_motor_send_data.Mdata.Pos = 0;
    motor_cmd->A1B1_motor_send_data.Mdata.K_P = 0;
    motor_cmd->A1B1_motor_send_data.Mdata.K_W = 0;
    motor_cmd->A1B1_motor_send_data.Mdata.LowHzMotorCmdIndex = 0;
    motor_cmd->A1B1_motor_send_data.Mdata.LowHzMotorCmdByte = 0;
    memset(&motor_cmd->A1B1_motor_send_data.Mdata.Res, 0, sizeof(COMData32));
    memset(&motor_cmd->A1B1_motor_send_data.CRCdata, 0, sizeof(COMData32));
}

/**
    @name: Input_MotorCmd
    @brief:数据导入A1MotorCmd结构体
**/

void Input_MotorCmd(MotorCmd* motor_cmd,int len,unsigned short id,unsigned short mode,float kp,float kd ) {
    if (motor_cmd == NULL) {
        return;  
    }
    motor_cmd->motorType = A1_Motor;
    motor_cmd->hex_len = len;
    motor_cmd->id = id;
    motor_cmd->mode = mode;
    motor_cmd->tau = 0.0f;
    motor_cmd->dq = 0.0f;
    motor_cmd->q = 0.0f;
    motor_cmd->kp = kp;
    motor_cmd->kd = kd;
    
        
    motor_cmd->A1B1_motor_send_data.head.start[0] = 0xFE;
    motor_cmd->A1B1_motor_send_data.head.start[1] = 0xEE;
    motor_cmd->A1B1_motor_send_data.head.motorID = id;
    motor_cmd->A1B1_motor_send_data.head.reserved = 0;
    motor_cmd->A1B1_motor_send_data.Mdata.mode = mode;
    motor_cmd->A1B1_motor_send_data.Mdata.ModifyBit = 0xFF;
    motor_cmd->A1B1_motor_send_data.Mdata.ReadBit = 0;
    motor_cmd->A1B1_motor_send_data.Mdata.reserved = 0;
    memset(&motor_cmd->A1B1_motor_send_data.Mdata.Modify, 0, sizeof(COMData32));
    motor_cmd->A1B1_motor_send_data.Mdata.T = 0;
    motor_cmd->A1B1_motor_send_data.Mdata.W = 0;
    motor_cmd->A1B1_motor_send_data.Mdata.Pos = 0;
    motor_cmd->A1B1_motor_send_data.Mdata.K_P = (q15_t) kp;
    motor_cmd->A1B1_motor_send_data.Mdata.K_W =(q15_t) kd;
    motor_cmd->A1B1_motor_send_data.Mdata.LowHzMotorCmdIndex = 0;
    motor_cmd->A1B1_motor_send_data.Mdata.LowHzMotorCmdByte = 0;
    memset(&motor_cmd->A1B1_motor_send_data.Mdata.Res, 0, sizeof(COMData32));
    memset(&motor_cmd->A1B1_motor_send_data.CRCdata, 0, sizeof(COMData32));
}

/**
    @name: Struct_MotorData
    @brief:初始化A1MotorData接收结构体
**/
void Struct_MotorData(MotorData* motor_data) {
    if (motor_data == NULL) {
        return;  
    }
    motor_data->motorType = A1_Motor;
    motor_data->hex_len = 0;
    motor_data->motor_id = 0;
    motor_data->mode = 0;
    motor_data->Temp = 0;
    motor_data->merror = 0;
    motor_data->footForce = 0;
    motor_data->Acc = 0;
    motor_data->correct = 0;
    motor_data->tau = 0.0f;
    motor_data->dq = 0.0f;
    motor_data->q = 0.0f;
    motor_data->LW = 0.0f;

    for (int i = 0; i < 3; i++) {
        motor_data->gyro[i] = 0.0f;
        motor_data->acc[i] = 0.0f;
    }
    // 初始化结构体成员
    memset(&motor_data->A1B1_motor_recv_data, 0, sizeof(ServoComdDataV3)); 
}

/**
    @name: get_motorA1_send_data
    @brief:发送数据到A1电机
**/
 void get_motorA1_send_data(MotorCmd* motor_cmd) {
    if (motor_cmd == NULL) return;

    uint8_t* send_ptr = (uint8_t*)&motor_cmd->A1B1_motor_send_data;
    uint32_t data_length = sizeof(MasterComdDataV3);

    // 填充固定帧头
    send_ptr[0] = 0xFE;
    send_ptr[1] = 0xEE;

    // 填充基础参数
    send_ptr[2] = motor_cmd->id;
    send_ptr[3] = 0;
    send_ptr[4] = motor_cmd->mode;
    send_ptr[5] = 0xFF;
    send_ptr[6] = 0;

    // 填充32位参数
    *(uint32_t*)(send_ptr + 8) = 0;
    *(uint32_t*)(send_ptr + 12) = 0;
    *(uint32_t*)(send_ptr + 16) = 0;
    *(uint32_t*)(send_ptr + 20) = 0;

    // 填充浮点参数（小端序）
    float_to_bytes(motor_cmd->q, send_ptr + 24);
    float_to_bytes(motor_cmd->tau, send_ptr + 28);

    // 计算CRC（跳过CRC字段本身）
    uint32_t crc = calculate_crc(send_ptr, data_length - sizeof(COMData32));
    *(uint32_t*)(send_ptr + data_length - 4) = crc;

    USART_TypeDef* USARTx = USART1;

    // 发送数据（轮询模式优化）
    for (uint32_t i = 0; i < data_length; ++i) {
        while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);
        USART_SendData(USARTx, send_ptr[i]);
    }
}

/**
    @name: get_motorA1_recv_data
    @brief:获取从A1电机接收到的数据的指针
**/
void get_motorA1_recv_data(MotorData* motor_data,RCS_PIN_USART USARTx_MAP) {
    A1_USART_MAP = USARTx_MAP;
    uint8_t receive_char;
    if(USART_GetITStatus(A1_USART_MAP.USARTx, USART_IT_RXNE) == SET)
    {
        last_recv_time = time(NULL);  // 更新接收时间
        receive_char = RCS_USART_Accept_Char(A1_USART_MAP.USARTx);
        A1Motor_exam(&motor_data);
    }
    USART_ClearITPendingBit(A1_USART_MAP.USARTx, USART_IT_RXNE);
}

/**
    @name: A1Motor_exam
    @brief:接收数据处理
**/
void A1Motor_exam(MotorData* motor_data) {
    if (motor_data == NULL) return;

    motor_data_ptr = motor_data;  // 保存数据指针

    // 触发初始化请求事件
    A1_Interrupt();
}

/**
    @name: A1_Init
    @brief: A1初始化
**/
void A1_Init(RCS_PIN_USART USARTx_MAP)
{
    A1_USART_MAP = USARTx_MAP;
    RCS_USART_Config(USARTx_MAP.USARTx, USARTx_MAP.GPIOx,USARTx_MAP.GPIO_Pin_Tx,USARTx_MAP.GPIO_Pin_Rx,A1_Interrupt ,A1_BAUD, A1_PRI);
    delay_ms(100);
}

/**
    @name: A1_Interrupt
    @brief: 中断接受函数(使用时映射成对应USART的中断函数)
**/
static void A1_Interrupt() {
    MotorRecvEvent event;

    if (check_timeout()) {
        current_state = STATE_IDLE;
        recv_index = 0;
        crc_index = 0;
        return;
    }

    switch (current_state) {
        case STATE_IDLE:
            if (event == EVENT_FRAME_START) {
                current_state = STATE_RECEIVE_HEADER;
                recv_index = 0;
                crc_index = 0;
                // 触发接收帧头事件
                event = EVENT_DATA_RECEIVED;
            }
            break;

        case STATE_RECEIVE_HEADER:
            if (event == EVENT_DATA_RECEIVED) {
                if (recv_buffer[recv_index] == 0xFE) {
                    recv_buffer[recv_index++] = 0xFE;
                    current_state = STATE_RECEIVE_DATA;
                    // 触发接收帧头事件
                    event = EVENT_DATA_RECEIVED;
                } else {
                    current_state = STATE_IDLE;
                }
            }
            break;

        case STATE_RECEIVE_DATA:
            if (event == EVENT_DATA_RECEIVED) {
                recv_buffer[recv_index++] = RCS_USART_Accept_Char(A1_USART_MAP.USARTx);
                
                // 检查帧尾
                if (recv_buffer[recv_index-1] == 0xEE) {
                    // 完整数据帧接收
                    if (recv_index >= sizeof(ServoComdDataV3) + 4) {
                        // 提取CRC字段
                        uint32_t received_crc = *(uint32_t*)(recv_buffer + recv_index - 4);
                        
                        // 计算CRC
                        uint32_t computed_crc = calculate_crc(recv_buffer, recv_index - 4);
                        
                        if (received_crc == computed_crc) {
                            // 解析数据
                            ServoComdDataV3* resp_data = (ServoComdDataV3*)recv_buffer;
                    
                            // 基础参数解析
                            motor_data_ptr->motorType =  A1_Motor;                          // 电机类型固定为A1
                            motor_data_ptr->motor_id = resp_data->head.motorID;       // 从帧头获取电机ID
                            motor_data_ptr->mode = resp_data->Mdata.mode;           // 命令模式
                            motor_data_ptr->Temp = resp_data->Mdata.Temp;// 温度
                            motor_data_ptr->merror = resp_data->Mdata.Merror;              // 错误标志
                            
                            // 浮点参数解析（需字节序转换）
                            uint8_t tau_bytes[2];
                            tau_bytes[0] = (resp_data->Mdata.T >> 0) & 0xFF;
                            tau_bytes[1] = (resp_data->Mdata.T >> 8) & 0xFF;
                            motor_data_ptr->tau = bytes_to_float(tau_bytes);   // 扭矩指令
                                                        
                            uint8_t q_bytes[2];
                            q_bytes[0] = (resp_data->Mdata.Pos >> 0) & 0xFF;
                            q_bytes[1] = (resp_data->Mdata.Pos >> 8) & 0xFF;
                            motor_data_ptr->q = bytes_to_float(q_bytes);              // 位置指令
                                                        
                            uint8_t LW_bytes[4];  
                            uint32_t lw_val = *(uint32_t*)&resp_data->Mdata.LW;  // 将float转换为uint32_t
                            LW_bytes[0] = (lw_val >> 0) & 0xFF;
                            LW_bytes[1] = (lw_val >> 8) & 0xFF;
                            LW_bytes[2] = (lw_val >> 16) & 0xFF;
                            LW_bytes[3] = (lw_val >> 24) & 0xFF;
                            motor_data_ptr->LW = resp_data->Mdata.LW;             // 低频速度
                        
                            // 整型参数解析
                            motor_data_ptr->Acc = resp_data->Mdata.Acc;                 // 加速度
                        
                            int16_t force_high = resp_data->Mdata.Force16;
                            int8_t  force_low  = resp_data->Mdata.Force8;
                            motor_data_ptr->footForce = (force_high << 8) | force_low;      // 足端力
                            
                            // 数组参数解析（需逐个元素转换）
                            for (int i = 0; i < 3; ++i) {
                                uint8_t Fgyro_bytes[2];
                                Fgyro_bytes[0] = (resp_data->Mdata.Fgyro[i] >> 8) & 0xFF;  // 高8位
                                Fgyro_bytes[1] = resp_data->Mdata.Fgyro[i] & 0xFF;        // 低8位
                                motor_data_ptr->gyro[i] = bytes_to_float(Fgyro_bytes);
                                  
                                uint8_t Facc_bytes[2];
                                Facc_bytes[0] = (resp_data->Mdata.Facc[i] >> 8) & 0xFF;  // 高8位
                                Facc_bytes[1] = resp_data->Mdata.Facc[i] & 0xFF;        // 低8位
                                motor_data_ptr->acc[i] = bytes_to_float(Facc_bytes);
                            }
                            
                            // 32位参数解析（直接取整数值）
                            motor_data_ptr->tau_int = resp_data->Mdata.T ;          // 扭矩整数部分
                            motor_data_ptr->q_int = resp_data->Mdata.Pos;             // 位置整数部分
                            
                            // 校验位解析
                            motor_data_ptr->correct = resp_data->Mdata.correct;          // 校验标志
                                
                            // 重新初始化接收状态
                            current_state = STATE_IDLE;
                            crc_index = 0;
                            recv_index = 0;
                        } else {
                            // CRC错误，丢弃数据
                            current_state = STATE_IDLE;
                            crc_index = 0;
                            recv_index = 0;
                        }
                    }
                }
            }
            break;

        default:
            current_state = STATE_IDLE;
            break;
    }
	}

	/**
	@name: calculate_crc
	@brief: CRC校验
**/
uint32_t calculate_crc(uint8_t* data, uint32_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for(uint32_t i=0; i<len; i++) {
        crc ^= data[i];
        for(int j=0; j<8; j++) {
            crc = (crc >> 1) ^ ((crc & 1) ? 0xEDB88320 : 0);
        }
    }
    return ~crc;
}


/**
	@name: float_to_bytes
	@brief: 将float转换为4字节字节数组（小端序）
**/
void float_to_bytes(float f, uint8_t* bytes) {
    uint32_t temp;
    memcpy(&temp, &f, sizeof(float));
    bytes[0] = (temp >> 0) & 0xFF;
    bytes[1] = (temp >> 8) & 0xFF;
    bytes[2] = (temp >> 16) & 0xFF;
    bytes[3] = (temp >> 24) & 0xFF;
}


/**
	@name: bytes_to_float
	@brief:将4字节字节数组转换为float（小端序）
**/
float bytes_to_float(uint8_t* bytes) {
    uint32_t temp = 0;
    temp |= (uint32_t)bytes[0] << 0;
    temp |= (uint32_t)bytes[1] << 8;
    temp |= (uint32_t)bytes[2] << 16;
    temp |= (uint32_t)bytes[3] << 24;
    float f;
    memcpy(&f, &temp, sizeof(float));
    return f;
}

/**
    @name: check_timeout
    @brief: 检查是否超时
**/
static int check_timeout() {
    time_t now = time(NULL);
    return (now - last_recv_time) > TIMEOUT_SEC;
}

//  MotorCmd test;
//	Struct_MotorCmd(&test);
//	Input_MotorCmd(&test,sizeof(MasterComdDataV3),1,FOC,10.0f,0.5f);
//	test.q = 1.57f;
//	get_motorA1_send_data(&test);        发送测试样例



