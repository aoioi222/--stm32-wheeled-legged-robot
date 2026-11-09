#ifndef __MOTOR_LZ_H__
#define __MOTOR_LZ_H__

/* ===============依赖头文件===================================*/
#include "stm32f4xx_can.h"
#include "RCS_CAN.h"
#include "rcs.h"
#include <string.h>

/* ===============核心配置宏===================================*/
// 最大支持的灵足电机数量（对齐MAX_RMESC_ON_CAN）
#define MAX_LZMOTOR_ON_CAN 8
// CAN外设数量（0=CAN1/1=CAN2）
#define CAN_PERI_COUNT 2
// 电机用户ID（灵足协议要求）
#define motor_LZ_user_id 0

/* ===============电机型号枚举===================================*/
typedef enum {
    MOTOR_LZ_00 = 0,  // 预留
    MOTOR_LZ_01,      // LZ-01
    MOTOR_LZ_02,      // LZ-02（常用）
    MOTOR_LZ_03,
    MOTOR_LZ_04,
    MOTOR_LZ_05,      // LZ-05
} Motor_LZ_Model;

/* ===============电机配置结构体===================================*/
typedef struct {
    uint8_t id;            // 电机物理ID
    Motor_LZ_Model model;  // 电机型号
} Motor_LZ_Init;

/* ===============CAN指令集枚举===================================*/
typedef enum {
    CANCOM_ANNOUNCE_DEVID   = 0,    // 设备ID广播
    CANCOM_MOTOR_CTRL       = 1,    // 电机控制指令
    CANCOM_MOTOR_FEEDBACK   = 2,    // 电机反馈数据
    CANCOM_MOTOR_IN         = 3,    // 电机使能
    CANCOM_MOTOR_RESET      = 4,    // 电机失能
    CANCOM_MOTOR_CALI       = 5,    // 电机标定
    CANCOM_MOTOR_ZERO       = 6,    // 设置机械零点
    CANCOM_MOTOR_ID         = 7,    // 修改CAN ID
    CANCOM_PARA_WRITE       = 8,    // 写入参数
    CANCOM_PARA_READ        = 9,    // 读取参数
    CANCOM_CALI_ING         = 10,   // 标定中
    CANCOM_CALI_RST         = 11,   // 标定复位
    CANCOM_PARA_STR_INFO    = 12,   // 参数结构体信息
    CANCOM_MOTOR_BRAKE      = 13,   // 电机刹车
    CANCOM_FAULT_WARN       = 14,   // 故障警告
    CANCOM_MODE_TOTAL       = 15,   // 模式总数
    CANCOM_MODE_Bd          = 16,   // 波特率设置
    CANCOM_MODE_ACTIVE_RECV = 0x18, // 主动上报
    CANCOM_MODE_AGREEMENT   = 0x19, // 协议版本
} CANCOM_Mode;

/* ===============控制/反馈参数结构体===================================*/
// 发送参数（给电机）
typedef struct {
    CANCOM_Mode mode;    // 控制模式
    float T;             // 力矩 (N·m)
    float Angle;         // 角度 (rad)
    float W;             // 速度 (rad/s)
    float Kp;            // 位置环增益
    float Kd;            // 速度环增益
} user_send_Lz;

// 接收参数（电机反馈）
typedef struct {
    uint8_t MError;      // 故障码（0=无故障）
    uint8_t mode;        // 反馈模式
    float T;             // 实际力矩 (N·m)
    float Angle;         // 实际角度 (rad)
    float W;             // 实际速度 (rad/s)
    float Temp;          // 电机温度 (℃)
} user_recv_Lz;


/* ===============对外函数声明===================================*/

void LZMotor_CAN_Init(CAN_TypeDef* CANx);
void LZMotor_Update_Param(CAN_TypeDef* CANx, uint8_t id, user_send_Lz* param);
void LZMotor_Execute(CAN_TypeDef* CANx, uint8_t id);
void LZMotor_Enable(CAN_TypeDef* CANx, uint8_t id);
void LZMotor_Disable(CAN_TypeDef* CANx, uint8_t id);
void LZMotor_Active_Recv(CAN_TypeDef* CANx, uint8_t id, uint8_t en);
void LZMotor_Set_Zero(CAN_TypeDef* CANx, uint8_t id);
user_recv_Lz* LZMotor_Get_Recv_Data(CAN_TypeDef* CANx, uint8_t id);

/* ===============CAN回调函数声明（供RCS_CAN调用）===================================*/
void LZ_Motor_Data_Filter1(RCS_CAN2B_DATA_FM_RX rx_message);
void LZ_Motor_Data_Filter2(RCS_CAN2B_DATA_FM_RX rx_message);

#endif /* __MOTOR_LZ_H__ */