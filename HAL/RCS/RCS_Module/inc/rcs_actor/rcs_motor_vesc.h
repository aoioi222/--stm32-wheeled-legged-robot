#pragma once

#include "rcs_private_config.h"
#include "rcs_types.h"
#include "rcs_motorlist.h"
#include <cstdint>

/**
 * @brief VESC电调类
 * @param id 电机ID
 * @note VESC的PID需要在上位机中配置，不由主控控制
 */
class rcs_motor_vesc : public rcs_motor_base
{
    public:
        // 构造函数和初始化
        rcs_motor_vesc(uint16_t id);
        rcs_err_t set_port(rcs_can_t can);

        // 电机控制
        rcs_err_t update_current(float current);
        rcs_err_t update_rpm(float rpm);
        rcs_err_t update_position(float position);

        // 对接电机链表
        static excute_fptr get_excute_fptr() const override;
		static reset_fptr get_reset_fptr() const override;
        static rcs_err_t excute(rcs_can_t can);
        static rcs_err_t reset(rcs_can_t can);
    
    private:
        // 对象成员
        uint16_t id;
        rcs_can_t can;

        // 类成员
        static std::vector<uint16_t> vesc_id;
        static std::vector<rcs_can_t> vesc_can;
};
