/**
 *@file rcs_wit_gyro.cpp
 *@brief 维特智能陀螺仪类实现
 *@author CYK-Dot
 *@date 2025-05-23
 */

/* 头文件引入 ------------------------------------------------------------*/
#include "rcs_wit_gyro.h"

#include "FreeRTOS.h"
#include "task.h"
#include "rcs_uart.h"
#include "rcs_types.h"


/* 静态成员变量 ------------------------------------------------------------*/

std::vector<rcs_uart_t> uart_list;
std::vector<rcs_wit_gyro*> gyro_list;

/* 成员函数 -----------------------------------------------------------------*/

/**
 *@brief 维特智能陀螺仪类构造函数
 *@note  由于RCS_UART使用了RTOS功能，无法在main函数之前初始化，因此不提供构造函数的实现
 */
rcs_wit_gyro::rcs_wit_gyro() : uart(nullptr), wit_handle(nullptr)
{
    
}

/**
 *@brief 维特智能陀螺仪类析构函数
 *@note  本函数用于析构维特智能陀螺仪类，是使用意义上的析构函数
 */
rcs_wit_gyro::~rcs_wit_gyro()
{
    deinit();
    
}

/* 初始化函数 ----------------------------------------------------------------*/

/**
 *@brief 维特智能陀螺仪类初始化函数
 *@param huart 串口句柄
 *@retval 错误码
 *
 *@note  本函数用于初始化维特智能陀螺仪类，是使用意义上的构造函数
 *@note 请先在CubeMX中配置好串口的波特率，维特智能陀螺仪一般使用115200波特率
 *
 *@warning 本函数不允许在RTOS启动前调用
 *@warning 请不要让多个串口设备占用同一个串口，如果出现了占用情况，本函数将会返回RCS_ERR_NOT_ALLOWED
 */
rcs_err_t rcs_wit_gyro::init(UART_HandleTypeDef *huart,int wit_protocol,uint8_t wit_addr)
{
    if (!huart) return RCS_ERR_INVALID_PARAM;
    
    // 创建UART对象
    rcs_uart_conf_t uart_conf = {
        .huartx = huart,
        .txBufferSize = RCS_WIT_GYRO_TX_BUFFER_SIZE,
        .rxBufferSize = RCS_WIT_GYRO_RX_BUFFER_SIZE,
        .txTimeout = pdMS_TO_TICKS(1),
        .rxMode = RCS_WIT_GYRO_RX_MODE,
        .txMode = RCS_WIT_GYRO_TX_MODE,
        .rxCallback = uart_rx_callback
    };
    uart = rcs_uart_create(&uart_conf);
    if (!uart) return RCS_ERR_NOT_ALLOWED;
    
    // 创建WIT对象
    wit_handle = (wit_gyro_t)RCS_WIT_GYRO_MALLOC(sizeof(wit_gyro_internal_t));
    if (!wit_handle) {
        return RCS_ERR_NO_MEM;
    }
    
    // 初始化WIT对象
    if (WitInit(wit_handle, wit_protocol, wit_addr, this) != WIT_HAL_OK) {
        RCS_WIT_GYRO_FREE(wit_handle);
        return RCS_ERR_HW;
    }
    
    // 注册回调函数
    if (WitRegisterCallBack(wit_handle, reg_update_callback) != WIT_HAL_OK) {
        WitDeInit(wit_handle);
        RCS_WIT_GYRO_FREE(wit_handle);
        return RCS_ERR_HW;
    }
    
    // 注册串口写入函数
    if (WitSerialWriteRegister(wit_handle, serial_write) != WIT_HAL_OK) {
        WitDeInit(wit_handle);
        RCS_WIT_GYRO_FREE(wit_handle);
        return RCS_ERR_HW;
    }
    
    // 注册延时函数
    if (WitDelayMsRegister(wit_handle, delay_ms) != WIT_HAL_OK) {
        WitDeInit(wit_handle);
        RCS_WIT_GYRO_FREE(wit_handle);
        return RCS_ERR_HW;
    }

    // 将uart和gyro添加到列表中
    uart_list.push_back(uart);
    gyro_list.push_back(this);
    
    return RCS_OK;
}

/**
 *@brief 维特智能陀螺仪类反初始化函数
 *@note  本函数用于反初始化维特智能陀螺仪类，是使用意义上的析构函数
 *@warning 单片机一般不会动态申请资源，因此本函数在大多数时候不应被调用
 */
rcs_err_t rcs_wit_gyro::deinit()
{
    if (wit_handle) {
        WitDeInit(wit_handle);
        RCS_WIT_GYRO_FREE(wit_handle);
        wit_handle = nullptr;
    }
    
    if (uart) {
        uart = nullptr;
    }
    
    return RCS_OK;
}


/* 类静态函数定义 ------------------------------------------------------------*/

/**
 *@brief 从uart向量和gyro向量中获取对应的维特智能陀螺仪对象
 *@param handle 串口句柄
 *@return 维特智能陀螺仪对象
 */
wit_gyro_t rcs_wit_gyro::prv_get_gyro_from_uart(rcs_uart_t handle)
{
    for (size_t i = 0; i < uart_list.size(); i++) {
        if (uart_list[i] == handle) {
            return gyro_list[i];
        }
    }
    return nullptr;
}

/**
 *@brief 维特智能陀螺仪类串口接收回调函数
 *@param handle 串口句柄
 *@param data 数据
 *@param size 数据大小
 */
void rcs_wit_gyro::uart_rx_callback(rcs_uart_t handle, uint8_t *data, size_t size)
{
    rcs_wit_gyro gyro = prv_get_gyro_from_uart(handle);
    if (!gyro) return;
    
    for (size_t i = 0; i < size; i++) {
        WitSerialDataIn(gyro->wit_handle, data[i]);
    }
}

/**
 *@brief 维特智能陀螺仪类寄存器更新回调函数
 *@param handle 维特智能陀螺仪句柄
 *@param reg 寄存器
 *@param reg_num 寄存器数量
 */
void rcs_wit_gyro::reg_update_callback(wit_gyro_t handle, uint32_t reg, uint32_t reg_num)
{
    if (!handle || !handle->wit_handle) return;
    
    // 这里可以添加数据更新处理
}

/**
 *@brief 维特智能陀螺仪类延时函数
 *@param ms 延时时间
 */
void rcs_wit_gyro::delay_ms(uint16_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

/**
 *@brief 维特智能陀螺仪类串口写入函数
 *@param handle 维特智能陀螺仪句柄
 *@param data 数据
 *@param len 数据长度
 */
void rcs_wit_gyro::serial_write(wit_gyro_t handle, uint8_t *data, uint32_t len)
{
    if (!handle || !handle->uart) return;
    rcs_uart_send(handle->uart, data, len, portMAX_DELAY);
}

/* 成员函数定义 ------------------------------------------------------------*/

/**
 *@brief 维特智能陀螺仪类获取俯仰角函数
 *@return 俯仰角
 */
float rcs_wit_gyro::get_pitch() const
{
    if (!wit_handle) return 0.0f;
    return wit_handle->sReg[Pitch] / 32768.0f * 180.0f;
}

/**
 *@brief 维特智能陀螺仪类获取滚转角函数
 *@return 滚转角
 */
float rcs_wit_gyro::get_roll() const
{
    if (!wit_handle) return 0.0f;
    return wit_handle->sReg[Roll] / 32768.0f * 180.0f;
}

/**
 *@brief 维特智能陀螺仪类获取偏航角函数
 *@return 偏航角
 */
float rcs_wit_gyro::get_yaw() const
{
    if (!wit_handle) return 0.0f;
    return wit_handle->sReg[Yaw] / 32768.0f * 180.0f;
}

/**
 *@brief 维特智能陀螺仪类获取温度函数
 *@return 温度
 */
float rcs_wit_gyro::get_temperature() const
{
    if (!wit_handle) return 0.0f;
    return wit_handle->sReg[TEMP] / 100.0f;
}

/**
 *@brief 维特智能陀螺仪类获取气压函数
 *@return 气压
 */
float rcs_wit_gyro::get_pressure() const
{
    if (!wit_handle) return 0.0f;
    return (wit_handle->sReg[PressureH] << 16 | wit_handle->sReg[PressureL]) / 100.0f;
}

/**
 *@brief 维特智能陀螺仪类获取高度函数
 *@return 高度
 */
float rcs_wit_gyro::get_altitude() const
{
    if (!wit_handle) return 0.0f;
    return (wit_handle->sReg[HeightH] << 16 | wit_handle->sReg[HeightL]) / 100.0f;
}

/**
 *@brief 维特智能陀螺仪类获取速度函数
 *@return 速度
 */
float rcs_wit_gyro::get_velocity() const
{
    if (!wit_handle) return 0.0f;
    return (wit_handle->sReg[GPSVH] << 16 | wit_handle->sReg[GPSVL]) / 100.0f;
}

/**
 *@brief 维特智能陀螺仪类设置波特率函数
 *@param baud_index 波特率索引
 *@return 错误码
 */
rcs_err_t rcs_wit_gyro::set_baud(int32_t baud_index)
{
    if (!wit_handle) return RCS_ERR_NOT_ALLOWED;
    return WitSetUartBaud(wit_handle, baud_index) == WIT_HAL_OK ? RCS_OK : RCS_ERR_HW;
}

/**
 *@brief 维特智能陀螺仪类设置带宽函数
 *@param bandwidth 带宽
 *@return 错误码
 */
rcs_err_t rcs_wit_gyro::set_bandwidth(int32_t bandwidth)
{
    if (!wit_handle) return RCS_ERR_NOT_ALLOWED;
    return WitSetBandwidth(wit_handle, bandwidth) == WIT_HAL_OK ? RCS_OK : RCS_ERR_HW;
}

/**
 *@brief 维特智能陀螺仪类设置输出速率函数
 *@param rate 输出速率
 *@return 错误码
 */
rcs_err_t rcs_wit_gyro::set_output_rate(int32_t rate)
{
    if (!wit_handle) return RCS_ERR_NOT_ALLOWED;
    return WitSetOutputRate(wit_handle, rate) == WIT_HAL_OK ? RCS_OK : RCS_ERR_HW;
}

/**
 *@brief 维特智能陀螺仪类设置内容函数
 *@param content 内容
 *@return 错误码
 */
rcs_err_t rcs_wit_gyro::set_content(int32_t content)
{
    if (!wit_handle) return RCS_ERR_NOT_ALLOWED;
    return WitSetContent(wit_handle, content) == WIT_HAL_OK ? RCS_OK : RCS_ERR_HW;
}

/**
 *@brief 维特智能陀螺仪类保存参数函数
 *@return 错误码
 */
rcs_err_t rcs_wit_gyro::save_parameter()
{
    if (!wit_handle) return RCS_ERR_NOT_ALLOWED;
    return WitSaveParameter(wit_handle) == WIT_HAL_OK ? RCS_OK : RCS_ERR_HW;
}

/**
 *@brief 维特智能陀螺仪类重置函数
 *@return 错误码
 */
rcs_err_t rcs_wit_gyro::reset()
{
    if (!wit_handle) return RCS_ERR_NOT_ALLOWED;
    return WitSetForReset(wit_handle) == WIT_HAL_OK ? RCS_OK : RCS_ERR_HW;
}

/**
 *@brief 维特智能陀螺仪类校准参考角度函数
 *@return 错误码
 */
rcs_err_t rcs_wit_gyro::calibrate_reference_angle()
{
    if (!wit_handle) return RCS_ERR_NOT_ALLOWED;
    return WitCaliRefAngle(wit_handle) == WIT_HAL_OK ? RCS_OK : RCS_ERR_HW;
}

/**
 *@brief 维特智能陀螺仪类开始加速度校准函数
 *@return 错误码
 */
rcs_err_t rcs_wit_gyro::start_acc_calibration()
{
    if (!wit_handle) return RCS_ERR_NOT_ALLOWED;
    return WitStartAccCali(wit_handle) == WIT_HAL_OK ? RCS_OK : RCS_ERR_HW;
}

/**
 *@brief 维特智能陀螺仪类停止加速度校准函数
 *@return 错误码
 */
rcs_err_t rcs_wit_gyro::stop_acc_calibration()
{
    if (!wit_handle) return RCS_ERR_NOT_ALLOWED;
    return WitStopAccCali(wit_handle) == WIT_HAL_OK ? RCS_OK : RCS_ERR_HW;
}

/**
 *@brief 维特智能陀螺仪类开始磁力计校准函数
 *@return 错误码
 */
rcs_err_t rcs_wit_gyro::start_mag_calibration()
{
    if (!wit_handle) return RCS_ERR_NOT_ALLOWED;
    return WitStartMagCali(wit_handle) == WIT_HAL_OK ? RCS_OK : RCS_ERR_HW;
}

/**
 *@brief 维特智能陀螺仪类停止磁力计校准函数
 *@return 错误码
 */
rcs_err_t rcs_wit_gyro::stop_mag_calibration()
{
    if (!wit_handle) return RCS_ERR_NOT_ALLOWED;
    return WitStopMagCali(wit_handle) == WIT_HAL_OK ? RCS_OK : RCS_ERR_HW;
}
