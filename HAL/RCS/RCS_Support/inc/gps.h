#pragma once

#include "rcs_private_config.h"
#include "rcs_types.h"

#include <FreeRTOS.h>
#include <timers.h>

#define RCS_GPS_TASK_PRIO 2

class gps
{
    public:
		// 导出类型
        using port_fptr = double (*)(void);
		
		// 构造函数
        gps(double TRANSFORM_X,double TRANSFORM_Y, double ANGLE_DX, double ANGLE_DY, double TRANSFORM_AX, double TRANSFORM_AY, uint8_t CAC_FREQ);
        rcs_err_t init(port_fptr x_encoder, port_fptr y_encoder,port_fptr z_gyro);
		
		// 坐标获取
        double get_pos_x(void);
        double get_pos_y(void);
        double get_pos_z(void);
		// 坐标修正
		void set_pos_x(double x);
		void set_pos_y(double y);
    
    private:
        // 硬件接口函数
        port_fptr get_encoder_x = nullptr;
        port_fptr get_encoder_y = nullptr;
        port_fptr get_gyro_z = nullptr;

        // 参数
        double transform_x;
        double transform_y;
        double angle_dx;
        double angle_dy;
        double transform_ax;
        double transform_ay;
        uint8_t cac_freq;

        // 状态变量
        double last_org_x, last_org_y;
        double last_zangle;
        double raw_gps_x, raw_gps_y;
        int cycle_z;

        // 当前位置
        double gps_x, gps_y, gps_z;
		
		// 解算任务
		static void gps_task(void *args);
};
