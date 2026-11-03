#ifndef UART_COMM_H
#define UART_COMM_H

#include "RCS_Pin_Mapping.h"
#include "RCS_Usart.h"
#include "crc8.h"

/************************ 协议常量定义 ************************/
#define FRAME_HEAD1         0xAA
#define FRAME_HEAD2         0x55
#define FRAME_TAIL          0x0A
#define FRAME_ID_CTRL       0x01    // 下行：树莓派→单片机 控制指令
#define FRAME_ID_FB         0x02    // 上行：单片机→树莓派 反馈数据
#define CTRL_PAYLOAD_LEN    64      // 下行载荷长度：12*4 + 4*4 = 64字节

/************************ 数据结构体定义 ************************/
// 遥控指令结构体（与题目完全一致，紧凑排列）
typedef struct __attribute__((packed))
{
    float cmd_vel_x;
    float cmd_vel_y;
    float cmd_vel_yaw;
    uint8_t jump_trigger;
    uint8_t gait_switch;
    uint8_t emergency_stop;
    uint8_t shutdown_trigger;
    uint8_t fold_trigger;
    uint8_t stand_trigger;
    uint8_t gait_start_trigger;
    uint8_t mode_switch;
    int8_t  pitch_adjust;
} RemoteCommand_t;

// IMU 欧拉角数据
typedef struct __attribute__((packed))
{
    float roll;
    float pitch;
    float yaw;
} ImuData_t;

// 下行控制数据：12关节角度 + 4足端轮速
typedef struct __attribute__((packed))
{
    float joint_angle[12];
    float wheel_speed[4];
} ControlData_t;

// 上行反馈数据：IMU + 遥控指令
typedef struct __attribute__((packed))
{
    ImuData_t imu;
    RemoteCommand_t remote;
} FeedbackData_t;

/************************ 外部全局变量 ************************/
extern ControlData_t g_ctrl_data;     // 最新解析到的控制指令
extern uint8_t g_ctrl_updated;        // 控制指令更新标志（置1表示有新数据，使用后请清零）

/************************ 函数接口 ************************/
/**
 * @brief  初始化串口通信模块
 * @param  pin_map   管脚映射结构体（对应主控板串口接口）
 * @param  baudrate  波特率（如 115200）
 * @param  pri       中断优先级（同 RCS_USART_Config 的 pri 参数）
 */
void uart_comm_init(const RCS_PIN_USART *pin_map, uint32_t baudrate, uint8_t pri);

/**
 * @brief  向上位机（树莓派）发送一帧反馈数据
 * @param  fb  反馈数据结构体指针
 * @note   阻塞发送，调用后等待发送完成再返回
 */
void uart_send_feedback(const FeedbackData_t *fb);

/**
 * @brief  串口接收单字节处理函数（中断内调用）
 * @param  byte  接收到的单字节数据
 */
void uart_recv_byte_isr(uint8_t byte);

#endif
