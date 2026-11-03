#ifndef A1MOTOR_H_
#define A1MOTOR_H_

#include "rcs.h"
#define A1_BAUD				  	4800000
#define TIMEOUT_MS 50 

// 发送用单个数据数据结构
typedef union{
        int32_t           L1;
        uint8_t       u8[4];
       uint16_t      u16[2];
       uint32_t         u32;
          float           F1;
}COMData32;

typedef struct {
    // 定义 数据包头
    unsigned char  start[2];     // 包头
    unsigned char  motorID;      // 电机ID  0,1,2,3 ...   0xBB 表示向所有电机广播（此时无返回）
    unsigned char  reserved;
}COMHead;


typedef struct {  // 以 4个字节一组排列 ，不然编译器会凑整
    // 定义 数据
    uint8_t  mode;        // 关节模式选择
    uint8_t  ModifyBit;   // 电机控制参数修改位
    uint8_t  ReadBit;     // 电机控制参数发送位
    uint8_t  reserved;

    COMData32  Modify;     // 电机参数修改 的数据 
                           //实际给FOC的指令力矩为：
                            //K_P*delta_Pos + K_W*delta_W + T
    q15_t     T;      // 期望关节的输出力矩（电机本身的力矩）x256, 7 + 8 描述
    q15_t     W;      // 期望关节速度 （电机本身的速度） x128,       8 + 7描述	
    int32_t   Pos;      // 期望关节位置 x 16384/6.2832, 14位编码器（主控0点修正，电机关节还是以编码器0点为准）

    q15_t    K_P;      // 关节刚度系数 x2048  4+11 描述
    q15_t    K_W;      // 关节速度系数 x1024  5+10 描述

    uint8_t LowHzMotorCmdIndex;     // 电机低频率控制命令的索引, 0-7, 分别代表LowHzMotorCmd中的8个字节
    uint8_t LowHzMotorCmdByte;      // 电机低频率控制命令的字节
    
     COMData32  Res[1];    // 通讯 保留字节  用于实现别的一些通讯内容
     int8_t  temp;
}MasterComdV3;   // 加上数据包的包头 和CRC 34字节

typedef struct {
    // 定义 电机控制命令数据包	
    COMHead head;    
    MasterComdV3 Mdata;
    COMData32 CRCdata;
}MasterComdDataV3;//返回数据

typedef struct {  // 以 4个字节一组排列 ，不然编译器会凑整
    // 定义 数据
    uint8_t  mode;        // 当前关节模式
    uint8_t  ReadBit;     // 电机控制参数修改     是否成功位
    int8_t  Temp;        // 电机当前平均温度   
    uint8_t  Merror;      // 电机错误 标识
    COMData32  Read;     // 读取的当前 电机 的控制数据 
    int16_t     T;      // 当前实际电机输出力矩       7 + 8 描述
    int16_t     W;      // 当前实际电机速度（高速）   8 + 7 描述
    float      LW;      // 当前实际电机速度（低速）   
    int16_t     W2;      // 当前实际关节速度（高速）   8 + 7 描述
    float      LW2;      // 当前实际关节速度（低速）   
    int16_t    Acc;           // 电机转子加速度       15+0 描述  惯量较小
    int16_t    OutAcc;        // 输出轴加速度         12+3 描述  惯量较大
    int32_t   Pos;      // 当前电机位置（主控0点修正，电机关节还是以编码器0点为准）
    int32_t   Pos2;     // 关节编码器位置(输出编码器)
    int16_t     gyro[3];  // 电机驱动板6轴传感器数据
    int16_t     acc[3];   
    int16_t     Fgyro[3];    // 力传感器的数据   
    int16_t     Facc[3];
    int16_t     Fmag[3];
    uint8_t     Ftemp;     // 8位表示的温度  7位（-28~100度）  1位0.5度分辨率
    int16_t     Force16;   // 力传感器高16位数据
    int8_t      Force8;    // 力传感器低8位数据
    uint8_t     FError;    //  足端传感器错误标识
		int32_t  correct;  
    int8_t      Res[1];    // 通讯 保留字节
    
}ServoComdV3;  // 加上数据包的包头 和CRC 78字节（4+70+4）

typedef struct {
    // 定义 电机控制命令数据包	
    COMHead        head;
    ServoComdV3      Mdata;
    COMData32    CRCdata;

}ServoComdDataV3;	//发送数据

typedef enum {
    STATE_IDLE,          // 等待帧头
    STATE_RECEIVE_HEADER, // 接收帧头（0xFE 0xEE）
    STATE_RECEIVE_DATA,   // 接收数据主体
    STATE_VERIFY_CRC      // 校验CRC
} MotorRecvState;


typedef enum {
    EVENT_FRAME_START,   // 接收到0xFE
    EVENT_FRAME_END,     // 接收到0xEE
    EVENT_DATA_RECEIVED, // 接收到有效数据
    EVENT_CRC_COMPLETE   // CRC接收完成
} MotorRecvEvent;


typedef enum {
    A1_Motor, // 4.8M baudrate
    B1_Motor, // 6.0M baudrate
    GO_M8010_6
} MotorType;


typedef enum {
    BRAKE, 
    FOC, 
    CALIBRATE
} MotorMode;

// 结构体定义
typedef struct MotorCmd {
    MotorType motorType;
    int hex_len;                   
    unsigned short id;              
    unsigned short mode;           
    float tau;                        
    float dq;                       
    float q;                     
    float kp;                     
    float kd;                     
     COMData32 Res;                
     MasterComdDataV3  A1B1_motor_send_data; 
} MotorCmd;

typedef struct MotorData {
    MotorType motorType;
    int hex_len;                   
    unsigned char motor_id;         
    unsigned char mode;             
    int Temp;                      
    int merror;          
    float tau;  
    float tau_int;  	
    float dq;                       
    float q;    
    float q_int;  	
    int footForce;
    float LW;                      
    int Acc;                      
    float gyro[3];                  
    float acc[3];
    int correct;    
     ServoComdDataV3 A1B1_motor_recv_data;     
} MotorData;


void Struct_MotorCmd(MotorCmd* motor_cmd);
void modify_data(MotorCmd* motor_s, MotorCmd* motor_cmd); 
void get_motorA1_send_data(MotorCmd* motor_cmd);
void Struct_MotorData(MotorData* motor_data);
void get_motorA1_recv_data(MotorData* motor_data,RCS_PIN_USART USARTx_MAP);
void A1Motor_exam(MotorData* motor_data); 
void A1_Init(RCS_PIN_USART USARTx_MAP);
void Input_MotorCmd(MotorCmd* motor_cmd,int len,unsigned short id,unsigned short mode,float kp,float kd );
uint32_t calculate_crc(uint8_t* data, uint32_t length);
void float_to_bytes(float f, uint8_t* bytes);
float bytes_to_float(uint8_t* bytes);
/////////////////////////////////////////////////////////////////

#endif