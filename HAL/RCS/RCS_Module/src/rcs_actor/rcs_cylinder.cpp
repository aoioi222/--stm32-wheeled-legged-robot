/**
 * @file rcs_cylinder.cpp
 * @author 陈煜楷
 * @brief 部署气缸执行器，提高代码可读性，同时从制度上避免一些常见的错误写法(例如把GPIO当作气缸状态)
 * @version 0.1
 * @date 2025-06-01
 * 
 * @copyright Copyright XMU-RCS (c) 2025
 * 
 */

/* 头文件引入 --------------------------------------------------------------------------------------*/
#include "rcs_cylinder.h"
#include <limits>
#include "rcs_os.h"

/* 私有宏定义 --------------------------------------------------------------------------------------*/
#define RCS_CYLINDER_MONITOR_NOT_ENABLE -1

/* 类私有成员 --------------------------------------------------------------------------------------*/

std::vector<cylinder_no_sensor::cylinder_conf> cylinder_no_sensor::conf_list;
std::vector<cylinder_no_sensor*> cylinder_no_sensor::obj_list;
std::vector<TickType_t> cylinder_no_sensor::last_push_time;
std::vector<TickType_t> cylinder_no_sensor::last_pull_time;
std::vector<cylinder_no_sensor::ctrl_t> cylinder_no_sensor::input;
std::vector<cylinder_no_sensor::stat_t> cylinder_no_sensor::status;
std::vector<int16_t> cylinder_no_sensor::monitor_target;
std::vector<int16_t> cylinder_no_sensor::monitor_current;

/* 类公开方法 --------------------------------------------------------------------------------------*/

/**
 * @brief 获取执行器的监控数据，由actor_bus调用
 * @param channel 
 * @return float 
 * @note 如果需要获取气缸当前所处的位置，请使用 @ref cylinder_no_sensor::get_status，这个函数不是给用户准备的
 */
float cylinder_no_sensor::monitor(uint8_t channel)
{
    for(int i=0;i<conf_list.size();i++) {
        if (monitor_target[i] == channel) {
            return (float)input[i];
        }
        else if (monitor_current[i] == channel) {
            return (float)status[i];
        }
    }
	rcs_kprintf("cylinder:monitor channel not found\n",portMAX_DELAY);
	return std::numeric_limits<float>::quiet_NaN();
}

/**
 * @brief 执行气缸执行器的输出状态，由actor_bus调用
 * @note 执行器都应具备同样的逻辑：更新状态和执行状态分离，因此本函数只负责控制外设，而不负责更新状态
 * @return rcs_err_t 
 * @note 这个函数不是给用户准备的，但可以用来学习如何使用状态机一箭双雕地实现气缸抵达估计+控制
 */
rcs_err_t cylinder_no_sensor::excute(void)
{
    for(int i=0;i<conf_list.size();i++) {
        switch(status[i]) {
            case PULLED:
                // 状态函数
                if (conf_list[i].is_high_push) 
                    HAL_GPIO_WritePin(conf_list[i].port,conf_list[i].pin,GPIO_PIN_RESET);
                else 
                    HAL_GPIO_WritePin(conf_list[i].port,conf_list[i].pin,GPIO_PIN_SET);
                // 转移条件+转移动作
                if ((input[i] == PUSH)||((input[i] == RESET)&&(!conf_list[i].is_reset_pull))) {
                    status[i] = PUSHING;
                    last_push_time[i] = xTaskGetTickCount();
                }
            break;

            case PUSHING:
                // 状态函数
                if (conf_list[i].is_high_push) 
                    HAL_GPIO_WritePin(conf_list[i].port,conf_list[i].pin,GPIO_PIN_SET);
                else 
                    HAL_GPIO_WritePin(conf_list[i].port,conf_list[i].pin,GPIO_PIN_RESET);
                // 转移条件
                if (xTaskGetTickCount() - last_push_time[i] > conf_list[i].maybe_arrive_time) {
                    status[i] = PUSHED;
                }
				if ((input[i] == PULL)||((input[i] == RESET)&&(conf_list[i].is_reset_pull))) {
                    status[i] = PULLING;
                    last_pull_time[i] = xTaskGetTickCount();
                }
            break;

            case PUSHED:
                // 状态函数
                if (conf_list[i].is_high_push) 
                    HAL_GPIO_WritePin(conf_list[i].port,conf_list[i].pin,GPIO_PIN_SET);
                else 
                    HAL_GPIO_WritePin(conf_list[i].port,conf_list[i].pin,GPIO_PIN_RESET);
                // 转移条件+转移动作
                if ((input[i] == PULL)||((input[i] == RESET)&&(conf_list[i].is_reset_pull))) {
                    status[i] = PULLING;
                    last_pull_time[i] = xTaskGetTickCount();
                }
            break;

            case PULLING:
                // 状态函数
                if (conf_list[i].is_high_push) 
                    HAL_GPIO_WritePin(conf_list[i].port,conf_list[i].pin,GPIO_PIN_RESET);
                else 
                    HAL_GPIO_WritePin(conf_list[i].port,conf_list[i].pin,GPIO_PIN_SET);
                // 转移条件
                if (xTaskGetTickCount() - last_pull_time[i] > conf_list[i].maybe_arrive_time) {
                    status[i] = PULLED;
                }
				if ((input[i] == PUSH)||((input[i] == RESET)&&(!conf_list[i].is_reset_pull))) {
                    status[i] = PUSHING;
                    last_push_time[i] = xTaskGetTickCount();
                }
            break;
        }
    }
    return RCS_OK;
}

/**
 * @brief 重置所有气缸执行器的状态，由actor_bus调用
 * @return rcs_err_t 
 * @note 这个函数不是给用户准备的
 */
rcs_err_t cylinder_no_sensor::reset(void)
{
    // 气缸需要时间运动到位，
    // 如果一个时刻没有控制气缸，气缸不应复位，而应维持上一个状态。
    // 所以这里不重置状态，而是直接短路。
    return RCS_OK;
}

/* 对象公开方法 ------------------------------------------------------------------------------------*/

/**
 * @brief 启停执行器的构造函数
 * 
 * @param port 使用哪一个GPIO端口
 * @param pin 使用哪一个GPIO引脚
 * @param is_high_push 高电平推？
 * @param is_reset_pull 复位状态下是拉？
 * @param maybe_arrive_time 从发起控制到气缸到达目标位置的时间，没法准确获取，请用估的。
 */
void cylinder_no_sensor::init(GPIO_TypeDef *GPIOx, uint16_t GPIO_PIN_x,bool isHighPush,bool isResetPull,TickType_t arriveTime)
{
    // 生成一个新的气缸节点
    cylinder_conf conf = {
        .port = GPIOx,
        .pin = GPIO_PIN_x,
        .is_high_push = isHighPush,
        .is_reset_pull = isResetPull,
        .maybe_arrive_time = arriveTime,
    };
    // 将气缸节点加入气缸链表
    conf_list.push_back(conf);
    obj_list.push_back(this);
    // 配置气缸节点的控制属性
    last_push_time.push_back(0);
    last_pull_time.push_back(0);
    input.push_back(RESET);
	if (isResetPull) status.push_back(PULLED); else status.push_back(PUSHED);
	// 配置气缸节点的观测属性
	monitor_target.push_back(-1);
	monitor_current.push_back(-1);
    // 获取气缸节点在气缸链表中的ID
    this->index = cylinder_no_sensor::conf_list.size() - 1;
    rcs_kprintf("rcs_cylinder: No.%d cylinder created on Port %X,Pin %X\n",portMAX_DELAY,this->index,GPIOx,GPIO_PIN_x);
}

/**
 * @brief 更新气缸执行器的输出状态
 * @param output 详见 @ref cylinder_no_sensor::ctrl
 * @note 执行器都应具备同样的逻辑：更新状态和执行状态分离，因此本函数只是对状态进行更新，不执行任何操作
 */
void cylinder_no_sensor::update(cylinder_no_sensor::ctrl_t updateInput)
{
    input[this->index] = updateInput;
}

/**
 * @brief 获取气缸当前所处的位置
 * @retval cylinder_no_sensor::stat 气缸当前所处的位置
 * 
 * @note 让我看看还有谁想通过读GPIO电平得知气缸状态？
 *       气缸是有惯性的，读GPIO电平肯定不对，正确的做法是估时间。
 */
cylinder_no_sensor::stat_t cylinder_no_sensor::get(void)
{
    return status[this->index];
}

/**
 * @brief 配置监控通道
 * 
 * @param channel_target 目标通道
 * @param channel_current 当前通道
 * 
 * @note 目标通道和当前通道是两个独立的通道，用于监控气缸状态
 *       目标通道用于记录用户希望气缸到达的位置
 *       当前通道用于记录气缸当前所处的位置
 * 
 * @warning 使用者必须确保通道不超出actor_bus设置的最大值，以及不和其他执行器干涉
 */
void cylinder_no_sensor::watch(uint8_t channel_target,uint8_t channel_current)
{
    monitor_target[this->index] = channel_target;
    monitor_current[this->index] = channel_current;
    rcs_kprintf("rcs_cylinder: No.%d cylinder watched on channel %d-%d\n",portMAX_DELAY,this->index,channel_target,channel_current);
}

