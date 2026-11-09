#pragma once

#include "rcs_private_config.h"
#include "rcs_types.h"
#include "rcs_actor_bus.h"

/**
 * @brief 不带传感器的启停执行器，可用于气缸(推/拉)、有刷真空泵(抽/停)、灯条(开/关)等。
 * @param port GPIO端口号
 * @param pin GPIO引脚号
 * @param is_high_push 高电平是推出气缸还是缩回气缸
 * @param is_reset_pull 复位后气缸是否缩回
 * @param maybe_arrive_time 可能到达时间(因为气缸没有传感器，不可能准确估计它什么时候抵达，所以需要设置一个可能的时间，用于判断是否到达)
 * 
 * @note 由于气缸用得比较多，所以就拿气缸命名了。
 *       但要知道，只要是无传感器的启停执行器，都用这个。
 * 
 * @note 本类在实现时更重视可读性而非代码体积。
 *       所以请尽可能使用Flash大的STM32，比如STM32F40VGT6(1MB)而不是VET6(512KB)
 */
class cylinder_no_sensor : public rcs_actor 
{
    public:
        // 类公开类型定义
        enum stat_t{
            PULLED  = 0,
            PUSHING = 1,
            PUSHED  = 2,
            PULLING = 3,
        };
        enum ctrl_t{
            RESET = 0,
            PULL = 1,
            PUSH = 2,
        };
        struct cylinder_conf {
            GPIO_TypeDef *port;
            uint16_t pin;
            bool is_high_push;
            bool is_reset_pull;
            TickType_t maybe_arrive_time;
        };

        // 类公开方法：供rcs_actor_bus调用
        static rcs_err_t excute(void);
        static rcs_err_t reset(void);
        static float monitor(uint8_t channel);

        // 对象公开方法：供用户调用
        void init(GPIO_TypeDef *GPIOx, uint16_t GPIO_PIN_x,bool isHighPush,bool isResetPull,TickType_t arriveTime);
        void update(ctrl_t input);
        stat_t get(void);
        void watch(uint8_t channel_target,uint8_t channel_current);

    //private:
        // 类私有成员1：气缸的链表
        static std::vector<cylinder_conf> conf_list;
        static std::vector<cylinder_no_sensor*> obj_list;
        // 类私有成员2：气缸控制相关的链表
        static std::vector<TickType_t> last_push_time;
        static std::vector<TickType_t> last_pull_time;
        static std::vector<ctrl_t> input;
		static std::vector<stat_t> status;
        // 类私有成员3：气缸观测相关的链表
        static std::vector<int16_t> monitor_target;
        static std::vector<int16_t> monitor_current;
        // 对象私有成员：气缸在链表中的下标
        uint8_t index;
};