//#ifndef LINUX_ORANGEPI_H
//#define LINUX_ORANGEPI_H

//#include <stdint.h>

//#define UART_DEV    "/dev/ttyAMA1"  // OrangePi 设备路径
//#define BAUDRATE    4800000         // 4.8Mbps波特率
//#define FRAME_SIZE  32              

//#pragma pack(push, 1)
//typedef struct {
//    uint8_t motor_id;   // 电机ID
//    float q;            // 位置（弧度）
//    float tau;          // 扭矩（Nm）
//    int Temp;           // 温度（℃）
//    int merror;         // 错误标志位
//    int footForce;      // 足端力
//    int Acc;            // 加速度
//    int correct;        // 校验标志
//    float LW;           // 低频速度
//    float gyro[3];      // 陀螺仪数据
//    float acc[3];       // 加速度计数据
//} MotorData;

//typedef struct {
//    uint8_t header[2];  // 0xFE 0xEE 
//    uint8_t motor_id;   // 电机ID 
//    uint8_t mode;       // 命令模式（FOC/CALIBRATE）
//    uint8_t modify_bit; // 参数修改标志位 
//    uint8_t read_bit;   // 参数读取标志位 
//    uint8_t reserved[2]; // 保留字节 
//    
//    // 运动参数（4字节对齐）
//    int32_t position;   // 位置（Q14.17格式）
//    int16_t torque;     // 扭矩（Q7.8格式）
//    int16_t kp;         // 刚度系数（Q11.4格式）
//    int16_t kd;         // 阻尼系数（Q10.5格式）
//    int32_t LW;         // 低频速度（Q14.17格式）
//    
//    // 传感器数据
//    int16_t Fgyro[3];     // 陀螺仪数据（±2000dps）
//    int16_t Facc[3];     // 加速度计数据（±16g）
//    int8_t Temp;        // 温度（℃）
//    uint8_t Merror;     // 电机错误标识
//    int16_t Force16;    // 力传感器高16位数据 
//    int8_t  Force8;     // 力传感器低8位数据 
//    uint8_t FError;     // 足端传感器错误标识
//    int32_t correct;    // 校验标志
//    
//    uint32_t crc32;     // CRC32校验值 
//} MotorFrame;

//#pragma pack(pop)

///* 函数声明 */
//int Linux_connet_init(const char* device, int baudrate);
//void Linux_connet_close(int fd);
//int Linux_send(int fd, uint8_t id, uint8_t mode, float position, float torque, float kp, float kd);
//int Linux_recv(int fd, MotorData* data);

//#endif    