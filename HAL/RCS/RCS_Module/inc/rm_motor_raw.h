/**
 *@filename rm_motor_raw.h
 *@brief 大疆电机原始控制接口(直接封装CAN报文)
 *@author CYK-Dot@2025-05-20
 *--------------------------------------------------------------------------------------------------
**/

#pragma once    

#ifdef __cplusplus
    extern "C" {
#endif

/* 头文件依赖 -------------------------------------------------------------------------------------*/

#include "rcs_private_config.h"
#include "rcs_types.h"

#include "rcs_can.h"

/* 配置宏定义 --------------------------------------------------------------------------------------*/


/* 导出函数声明 -----------------------------------------------------------------------------------*/

// 让CAN总线支持大疆电机的原始控制
rcs_err_t rm_motor_init(CAN_HandleTypeDef *hcan,rcs_can_t RCS_CAN_x);

// 控制大疆电机
rcs_err_t rm_motor_send_current_rm3508_front(rcs_can_t RCS_CAN_x, int16_t current1, int16_t current2, int16_t current3, int16_t current4, TickType_t timeout);
rcs_err_t rm_motor_send_current_gm6020_front(rcs_can_t RCS_CAN_x, int16_t current1, int16_t current2, int16_t current3, int16_t current4, TickType_t timeout);
rcs_err_t rm_motor_send_voltage_gm6020_front(rcs_can_t RCS_CAN_x, int16_t voltage1, int16_t voltage2, int16_t voltage3, int16_t voltage4, TickType_t timeout);
rcs_err_t rm_motor_send_current_rm3508_rear(rcs_can_t RCS_CAN_x, int16_t current5, int16_t current6, int16_t current7, int16_t current8, TickType_t timeout);
rcs_err_t rm_motor_send_current_gm6020_rear(rcs_can_t RCS_CAN_x, int16_t current5, int16_t current6, int16_t current7, TickType_t timeout);
rcs_err_t rm_motor_send_voltage_gm6020_rear(rcs_can_t RCS_CAN_x, int16_t voltage5, int16_t voltage6, int16_t voltage7, TickType_t timeout);
#define rm_motor_send_current_rm2006_front rm_motor_send_current_rm3508_front
#define rm_motor_send_current_rm2006_rear rm_motor_send_current_rm3508_rear

// 获取大疆电机角度和速度
float rm_motor_get_angle_rm3508_deg(rcs_can_t RCS_CAN_x,uint16_t id);
float rm_motor_get_speed_rm3508_rpm(rcs_can_t RCS_CAN_x,uint16_t id);
float rm_motor_get_speed_gm6020_rpm(rcs_can_t RCS_CAN_x,uint16_t id);
float rm_motor_get_angle_gm6020_deg(rcs_can_t RCS_CAN_x,uint16_t id);
#define rm_motor_get_angle_rm2006_deg rm_motor_get_angle_rm3508_deg
#define rm_motor_get_speed_rm2006_rpm rm_motor_get_speed_rm3508_rpm

#ifdef __cplusplus
    }
#endif






