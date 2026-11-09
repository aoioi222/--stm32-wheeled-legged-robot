#pragma once

#include "rcs_private_config.h"
#include "rcs_types.h"

#include "rcs_can.h"
#include "rcs_uart.h"
#include "rcs_actor_bus.h"
#include "sync_pid.h"

#include <cstdint>

#define RCS_MOTOR_RM3508_RESET_THRESHOLD 50    ///< 多少轮循环没有更新过，就复位。(一轮5ms,50轮即250ms)

/**
 * @brief 3508或2006电机类
 * @param id 电机ID
 * @param speed_pid 速度PID控制器
 * @param position_pid 位置PID控制器
 */
class rcs_motor_rm3508 : public rcs_actor 
{

    public:
        using ctrl_function = float(*)(rcs_motor_rm3508*);
        enum ctrl_type {
            CTRL_CURRENT,
            CTRL_SPEED,
            CTRL_POSITION,
            CTRL_USER,
        };
        enum watch_type {
            WATCH_CURRENT_TARGET,
            WATCH_SPEED_TARGET,
            WATCH_POS_TARGET,
            WATCH_SPEED,
            WATCH_POS,
        };
    
    public:
        // 类公开方法1：供rcs_actor_bus调用
        static rcs_err_t excute(void);
        static rcs_err_t reset(void);
        static float monitor(uint8_t channel);
        // 对象公开成员
        sync_pid* speed_pid = nullptr;
        sync_pid* angle_pid = nullptr;
    
    public:
        // 对象公开构造函数
        rcs_err_t init(CAN_HandleTypeDef *hcan,rcs_can_t RCS_CAN_x,uint8_t motor_id,sync_pid *speed_pid,sync_pid *angle_pid);
        // 对象公开方法1：更新
        rcs_err_t set_mode(ctrl_type mode);
        rcs_err_t set_mode_user(ctrl_type mode,ctrl_function function);
        rcs_err_t update(float target);
        // 对象公开方法2：获取
        float get_target_current();
        float get_target_user();
        float get_target_speed();
        float get_target_position();
        float get_real_speed();   
        float get_real_position();
        // 对象公开方法3：波形观察
        rcs_err_t watch(watch_type type,uint8_t channel);

    public:
        // 对象公开成员1：在电机链表的下标
        uint8_t actor_list_index;
        // 对象公开成员2：在执行链表的下标
        uint8_t excute_list_index;
        // 对象公开成员3：在观测链表的下标
        uint8_t monitor_list_index;    
        // 对象公开成员4：电机状态
        float update_target = 0;
        uint8_t excute_counter = 0;
        // 对象私有成员5：控制模式和控制函数
        ctrl_type ctrl_mode = CTRL_CURRENT;
        ctrl_function usr_ctrl_fptr = nullptr;

    private:
        enum frame_type{
            FRAME_ONLY_FRONT = 0,
            FRAME_ONLY_REAR,
        };
        // 类私有成员1：电机链表
        static std::vector<rcs_motor_rm3508*> actor_list;

        // 类私有成员2：执行链表(vector共享下标)
        static std::vector<rcs_can_t> excute_list_can;                       ///< 这条控制报文在哪一路CAN
        static std::vector<frame_type> excute_list_type;                    ///< 这条控制报文的类型
        static std::vector<std::array<uint8_t,4>>excute_list_motor;          ///< 这条控制报文属于哪些电机(用电机链表中的ID索引)

        // 类私有成员3：观测链表(vector共享下标)
        static std::vector<uint8_t> monitor_list_channel;          ///< 这个观测通道是多少
        static std::vector<watch_type> monitor_list_type;          ///< 这个观测通道观测的是什么东西
        static std::vector<uint8_t> monitor_list_motor;            ///< 这个观测通道属于哪个电机(用电机链表中的ID索引)

        // 类私有方法1：三种基本的控制函数
        static float ctrl_current(rcs_motor_rm3508 *motor);
        static float ctrl_speed(rcs_motor_rm3508 *motor);
        static float ctrl_position(rcs_motor_rm3508 *motor);

        // 对象私有成员1：目标值
        float target_current = 0;
        float target_speed = 0;
        float target_position = 0;
        float target_user = 0;
        // 对象私有成员2：电机信息
        rcs_can_t can;
        int8_t id;
};