/**
 * @file rcs_gyro.h
 * @author 陈煜楷
 * @brief 维特智能陀螺仪的驱动程序，使用面向对象封装，允许同时使用多个陀螺仪
 *        适用范围：维特智能的产品，例如HWT101CT(只能获取yaw轴，但精度高)、JY901(可以获取多维数据，但精度较低)等陀螺仪
 * @version 0.1
 * @date 2025-06-05
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include <vector>

#include "rcs_private_config.h"
#include "rcs_types.h"
#include "rcs_uart.h"

class rcs_jy901
{
    public:
        // 对象公开方法1：初始化
        rcs_err_t init(UART_HandleTypeDef *huartx,rcs_uart_t RCS_UART_NUM_x);
        // 对象公开方法2：获取数据
        float get_yaw_deg();
        float get_spd_x();
        float get_spd_y();
        float get_spd_z();
        // 对象公开成员1：所属串口
        uint8_t rcs_id;
        rcs_uart_t uart;
        
    private:
        // 对象私有成员：协议状态机
        uint8_t my_wit_s;
        int16_t my_wit_temp[10];
        float my_wit_yaw;
        float my_wit_acce[3];
        float my_wit_spd[3];
        float my_wit_g_start[3];
        uint8_t my_wit_start_flag;
		uint8_t sum,flag;
		float gyro_z_anglespeed;
		float gyro_z_angle;
        // 类私有成员1：当前所有陀螺仪的链表
        static std::vector<rcs_jy901*> gyro_list;
        // 类私有方法1：陀螺仪协议的UART回调接收函数
        static size_t gyro_uart_rx_callback(rcs_uart_t uart,const uint8_t* data, size_t len);
};