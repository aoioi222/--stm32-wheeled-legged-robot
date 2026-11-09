#pragma once


/* 头文件引入 ------------------------------------------------------------*/

#include "rcs_private_config.h"
#include "rcs_types.h"
#include "rcs_uart.h"

#include "wit_c_sdk.h"
#include <vector>

/* 配置宏定义 ------------------------------------------------------------*/

#define RCS_WIT_GYRO_MALLOC pvPortMalloc
#define RCS_WIT_GYRO_FREE vPortFree

#define RCS_WIT_GYRO_TX_BUFFER_SIZE 256
#define RCS_WIT_GYRO_RX_BUFFER_SIZE 1

#define RCS_WIT_GYRO_TX_MODE MODE_IT
#define RCS_WIT_GYRO_RX_MODE MODE_IT

/**
 *@brief 维特智能陀螺仪类
 */
class rcs_wit_gyro
{
    public:
        // 构造与析构函数
        rcs_wit_gyro();
        ~rcs_wit_gyro();
        rcs_err_t init(UART_HandleTypeDef *huart,int wit_protocol,uint8_t wit_addr);
        rcs_err_t deinit();
        
        // 数据获取接口
        float get_pitch() const;
        float get_roll() const;
        float get_yaw() const;
        float get_temperature() const;
        float get_pressure() const;
        float get_altitude() const;
        float get_velocity() const;
        
        // 配置接口
        rcs_err_t set_baud(int32_t baud_index);
        rcs_err_t set_bandwidth(int32_t bandwidth);
        rcs_err_t set_output_rate(int32_t rate);
        rcs_err_t set_content(int32_t content);
        rcs_err_t save_parameter();
        rcs_err_t reset();
        rcs_err_t calibrate_reference_angle();
        
        // 校准接口
        rcs_err_t start_acc_calibration();
        rcs_err_t stop_acc_calibration();
        rcs_err_t start_mag_calibration();
        rcs_err_t stop_mag_calibration();

    private:
        rcs_uart_t uart;
        wit_gyro_t wit_handle;
        static std::vector<rcs_uart_t> uart_list;
        static std::vector<rcs_wit_gyro*> gyro_list;
        static wit_gyro_t prv_get_gyro_from_uart(rcs_uart_t handle);
        static void uart_rx_callback(rcs_uart_t handle, uint8_t *data, size_t size);
        static void reg_update_callback(wit_gyro_t handle, uint32_t reg, uint32_t reg_num);
        static void delay_ms(uint16_t ms);
        static void serial_write(wit_gyro_t handle, uint8_t *data, uint32_t len);
};

