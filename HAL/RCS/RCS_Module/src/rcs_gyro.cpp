/**
 * @file rcs_gyro.cpp
 * @author 陈煜楷
 * @brief 维特智能陀螺仪的驱动程序，使用面向对象封装，允许同时使用多个陀螺仪
 *        适用范围：维特智能的产品，例如HWT101CT(只能获取yaw轴，但精度高)、JY901(可以获取多维数据，但精度较低)等陀螺仪
 * @version 0.1
 * @date 2025-06-05
 * 
 * @copyright Copyright (c) 2025
 * 
 */

/* 头文件引入 -----------------------------------------------------------------------------------*/
#include "rcs_os.h"
#include "rcs_gyro.h"

/* 类私有成员定义 -------------------------------------------------------------------------------*/

std::vector<rcs_jy901*> rcs_jy901::gyro_list;

/* 对象公开方法定义 ------------------------------------------------------------------------------*/

/**
 * @brief 陀螺仪对象初始化
 * 
 * @param huartx 哪一个HAL_UART对象，请务必使用CubeMX自动生成的HAL_UART对象，比如&huart1。不要传入自己定义的UART_HandleTypeDef对象
 * @param RCS_UART_NUM_x 哪一路串口
 * @return rcs_err_t 
 */
rcs_err_t rcs_jy901::init(UART_HandleTypeDef *huartx,rcs_uart_t RCS_UART_NUM_x)
{
    rcs_kprintf("rcs_hwt101:start init on UART%d\n",portMAX_DELAY,RCS_UART_NUM_x+1);
    // 检查这一路串口是否有使用过
    for(auto &gyro_ptr : gyro_list) {
        if(gyro_ptr->uart == RCS_UART_NUM_x) {
            rcs_kprintf("rcs_hwt101: init failed,UART%d has been used\n",portMAX_DELAY);
            return RCS_ERR_MULTIPLE_USED;
        }
    }
    // 初始化串口
    const rcs_uart_conf_t conf = {
        .huartx = huartx,

        .txMode = MODE_NONE,
        .txBufferSize = 0,
        .txTimeout = 0,

        .rxMode = MODE_IT,
        .rxBufferSize = 1,
        .rxCallback = rcs_jy901::gyro_uart_rx_callback,
    };
    if (rcs_uart_driver_install(&conf,RCS_UART_NUM_x) != RCS_OK) {
        rcs_kprintf("rcs_hwt101: init failed\n",portMAX_DELAY);
        return RCS_FAIL;
    }
    // 注册到陀螺仪链表
    gyro_list.push_back(this);
    this->uart = RCS_UART_NUM_x;
    // 结束
    rcs_kprintf("rcs_hwt101: init OK on UART%d\n",portMAX_DELAY,RCS_UART_NUM_x);
    return RCS_OK;
}

/**
 * @brief 获取陀螺仪的偏航角，单位为度
 * 
 * @return float 
 */
float rcs_jy901::get_yaw_deg()
{
    return this->my_wit_yaw;
}

/**
 * @brief 获取陀螺仪的x轴速度
 * 
 * @warning 本函数不适用于不存在x轴传感器的陀螺仪，例如HWT101CT
 * 
 * @return float 
 */
float rcs_jy901::get_spd_x()
{
    return this->my_wit_spd[0];
}
/**
 * @brief 获取陀螺仪的y轴速度
 * 
 * @warning 本函数不适用于不存在y轴传感器的陀螺仪，例如HWT101CT
 * 
 * @return float 
 */
float rcs_jy901::get_spd_y()
{
    return this->my_wit_spd[1];
}
/**
 * @brief 获取陀螺仪的z轴速度
 * 
 * @note 陀螺仪的z轴是平动方向，和我们一般说的z轴不太一样。我们说的z轴其实是yaw轴
 * @warning 本函数不适用于不存在z轴传感器的陀螺仪，例如HWT101CT
 * 
 * @return float 
 */
float rcs_jy901::get_spd_z()
{
    return this->my_wit_spd[2];
}

/* 类私有方法定义 --------------------------------------------------------------------------------*/

/**
 * @brief 陀螺仪的串口回调函数(直接移植自老代码，质量不高难以阅读，不应作为学习对象)
 * 
 * @param uart 
 * @param data 
 * @param len 
 */
size_t rcs_jy901::gyro_uart_rx_callback(rcs_uart_t uart,const uint8_t* data, size_t len)
{
    uint8_t receive_char;
    // 遍历陀螺仪链表
    for(auto &gyro_ptr : gyro_list) {
        if(gyro_ptr->uart == uart) {
            // 逐个将受到的数据送入陀螺仪接收状态机
            for(uint8_t i=0;i<len;i++) {
                receive_char = data[i];
                switch(gyro_ptr->my_wit_s)
                {
                    case 0:
                        if (receive_char == 0x55) gyro_ptr->my_wit_s++;
                    break;

                    case 1:
                        if (receive_char == 0x53)      {gyro_ptr->my_wit_s=2;gyro_ptr->sum=0x55+0x53;gyro_ptr->flag=0;memset(gyro_ptr->my_wit_temp,0,10);}
                        else if (receive_char == 0x51) {gyro_ptr->my_wit_s=3;gyro_ptr->sum=0x55+0x51;gyro_ptr->flag=0;memset(gyro_ptr->my_wit_temp,0,10);}
                    break;

                    case 2:
                        if (gyro_ptr->flag != 8) {
                            gyro_ptr->sum+=receive_char;
                            gyro_ptr->flag++;
                            if (gyro_ptr->flag == 5)      gyro_ptr->my_wit_temp[0]|=(uint16_t)receive_char;
                            else if (gyro_ptr->flag == 6) gyro_ptr->my_wit_temp[0]|=((uint16_t)receive_char)<<8;
                        }
                        else {
                            if (gyro_ptr->sum == receive_char) {
                                gyro_ptr->my_wit_yaw = (float)gyro_ptr->my_wit_temp[0]/32768.0f*180.0f;
                                gyro_ptr->my_wit_yaw = gyro_ptr->my_wit_yaw>180.0f   ?gyro_ptr->my_wit_yaw-360.0f:gyro_ptr->my_wit_yaw;
                                gyro_ptr->my_wit_yaw = gyro_ptr->my_wit_yaw<=-180.0f ?gyro_ptr->my_wit_yaw+360.0f:gyro_ptr->my_wit_yaw;
                                gyro_ptr->gyro_z_anglespeed = gyro_ptr->my_wit_yaw - gyro_ptr->gyro_z_angle;
                                gyro_ptr->gyro_z_angle=gyro_ptr->my_wit_yaw;
                            }
                            gyro_ptr->flag=0;
                            gyro_ptr->my_wit_s=0;
                        }
                    break;

                    case 3:
                        if (gyro_ptr->flag != 8) {
                            gyro_ptr->sum+=receive_char;
                            gyro_ptr->flag++;
                            if (gyro_ptr->flag == 1)      gyro_ptr->my_wit_temp[0]|=(int16_t)receive_char;
                            else if (gyro_ptr->flag == 2) gyro_ptr->my_wit_temp[0]|=((int16_t)receive_char)<<8;
                            else if (gyro_ptr->flag == 3) gyro_ptr->my_wit_temp[1]|=((int16_t)receive_char);
                            else if (gyro_ptr->flag == 4) gyro_ptr->my_wit_temp[1]|=((int16_t)receive_char)<<8;
                            else if (gyro_ptr->flag == 5) gyro_ptr->my_wit_temp[2]|=((int16_t)receive_char);
                            else if (gyro_ptr->flag == 6) gyro_ptr->my_wit_temp[2]|=((int16_t)receive_char)<<8;
                            else if (gyro_ptr->flag == 7) gyro_ptr->my_wit_temp[3]|=((int16_t)receive_char);
                            else if (gyro_ptr->flag == 8) gyro_ptr->my_wit_temp[3]|=((int16_t)receive_char)<<8;
                        }
                        else {
                            if (gyro_ptr->sum == receive_char) {
                                    gyro_ptr->my_wit_acce[0] = (float)gyro_ptr->my_wit_temp[0]/32768.0f*16.0f*9.8f;
                                    gyro_ptr->my_wit_acce[1] = (float)gyro_ptr->my_wit_temp[1]/32768.0f*16.0f*9.8f;
                                    gyro_ptr->my_wit_acce[2] = (float)gyro_ptr->my_wit_temp[2]/32768.0f*16.0f*9.8f;
                                if (!gyro_ptr->my_wit_start_flag) {
                                    gyro_ptr->my_wit_g_start[0]=gyro_ptr->my_wit_acce[0];
                                    gyro_ptr->my_wit_g_start[1]=gyro_ptr->my_wit_acce[1];
                                    gyro_ptr->my_wit_g_start[2]=gyro_ptr->my_wit_acce[2];
                                gyro_ptr->my_wit_start_flag=1;
                                }
                                else {
                                    gyro_ptr->my_wit_spd[0]+= (gyro_ptr->my_wit_acce[0]-gyro_ptr->my_wit_g_start[0]);
                                    gyro_ptr->my_wit_spd[1]+= (gyro_ptr->my_wit_acce[1]-gyro_ptr->my_wit_g_start[1]);///200.0f;
                                    gyro_ptr->my_wit_spd[2]+= (gyro_ptr->my_wit_acce[2]-gyro_ptr->my_wit_g_start[2]);///200.0f;
                                }
                            }
                            gyro_ptr->my_wit_s=0;
                            gyro_ptr->flag=0;
                        }
                    break;
                }
            }
        }
    }
    return len;
}