/**
 * @file rcs_motor_gm6020.cpp
 * @author 陈煜楷
 * @brief gm6020电机节点类实现，提供安全且易用的电机节点
 * @version 0.1
 * @date 2025-05-30
 * 
 * @copyright XMU-RCS Copyright (c) 2025
 * @note 代码编写逻辑为一般到特殊：类静态成员--类静态方法--类重载静态方法--对象方法
 * 
 */


/* 导入头文件 ------------------------------------------------------------------------*/

#include <FreeRTOS.h>
#include <task.h>

#include "rcs_motor_gm6020.h"
#include "rm_motor_raw.h"
#include "rcs_os.h"

/* 类静态成员定义 --------------------------------------------------------------------*/

// 类私有成员1：电机链表
std::vector<rcs_motor_gm6020*> rcs_motor_gm6020::actor_list;

// 类私有成员2：执行链表(vector共享下标)
std::vector<rcs_can_t> rcs_motor_gm6020::excute_list_can;                      
std::vector<rcs_motor_gm6020::frame_type> rcs_motor_gm6020::excute_list_type;                   
std::vector<std::array<uint8_t,4>> rcs_motor_gm6020::excute_list_motor;         

// 类私有成员3：观测链表(vector共享下标)
std::vector<uint8_t> rcs_motor_gm6020::monitor_list_channel;         
std::vector<rcs_motor_gm6020::watch_type> rcs_motor_gm6020::monitor_list_type;        
std::vector<uint8_t> rcs_motor_gm6020::monitor_list_motor;            


/* 类静态方法定义 -----------------------------------------------------------------*/

/**
 * @brief 3508电机链表执行函数，供actor_bus使用
 * 
 * @return rcs_err_t 
 */
rcs_err_t rcs_motor_gm6020::excute(void)
{
    rcs_err_t retval = RCS_OK;
    int16_t current[4] = {0};
    // 遍历执行链表
    for(uint8_t i = 0; i < excute_list_can.size(); i++)
    {
        // 根据指定的执行函数计算输出，并减少更新计数器
        memset(current, 0, sizeof(current));
        for(uint8_t j = 0; j < 4; j++) {
            if (actor_list[excute_list_motor[i][j]]->usr_ctrl_fptr != nullptr)
                current[j] = actor_list[excute_list_motor[i][j]]->usr_ctrl_fptr(actor_list[excute_list_motor[i][j]]);
            if (actor_list[excute_list_motor[i][j]]->excute_counter > 0)
                actor_list[excute_list_motor[i][j]]->excute_counter--;
			else
				rcs_kprintf("actor_gm6020: Motor(ID%hhd)on CAN%d timeout\n",portMAX_DELAY,actor_list[excute_list_motor[i][j]]->id,actor_list[excute_list_motor[i][j]]->can);
        }
        // 根据指定的执行函数类型发送数据
        switch(excute_list_type[i]) {
            case FRAME_ONLY_FRONT:
                rm_motor_send_current_gm6020_front(
                    excute_list_can[i],
                    current[0],
                    current[1],
                    current[2],
                    current[3],
                    pdMS_TO_TICKS(0)
                );
            break;

            case FRAME_ONLY_REAR:
                rm_motor_send_current_gm6020_rear(
                    excute_list_can[i],
                    current[0],
                    current[1],
                    current[2],
                    pdMS_TO_TICKS(0)
                );
            break;
        }
    }
    return retval;
}

/**
 * @brief 3508电机链表输出复位函数，供actor_bus使用
 * 
 * @return rcs_err_t 
 */
rcs_err_t rcs_motor_gm6020::reset(void)
{
    // 遍历电机链表
    for(uint8_t i = 0; i < actor_list.size(); i++)
    {
        // 如果超时抵达，强制切回电流模式，并将输出重置为0
        if (actor_list[i]->excute_counter == 0) {
            actor_list[i]->ctrl_mode = CTRL_CURRENT;
            actor_list[i]->update_target = 0;
        }
    }
    return RCS_OK;
}

/**
 * @brief 3508电机观测链表获取函数，供actor_bus使用
 * 
 * @param channel 
 * @return float 
 */
float rcs_motor_gm6020::monitor(uint8_t channel)
{
    // 遍历观测链表，寻找channel
    for(uint8_t i = 0; i < monitor_list_motor.size(); i++)
    {
        if (monitor_list_channel[i] == channel)
        {
            // 找到channel后，根据类型返回不同的观测值
            switch(monitor_list_type[i]) {
                case WATCH_CURRENT_TARGET:
                    return actor_list[monitor_list_motor[i]]->target_current;
                break;

                case WATCH_SPEED_TARGET:
                    return actor_list[monitor_list_motor[i]]->target_speed;
                break;

                case WATCH_POS_TARGET:
                    return actor_list[monitor_list_motor[i]]->target_position;
                break;

                case WATCH_SPEED:
                    return rm_motor_get_speed_gm6020_rpm(actor_list[monitor_list_motor[i]]->can,actor_list[monitor_list_motor[i]]->id);
                break;

                case WATCH_POS:
                    return rm_motor_get_angle_gm6020_deg(actor_list[monitor_list_motor[i]]->can,actor_list[monitor_list_motor[i]]->id);
                break;
            }
        }
    }
	return 0;
}

/**
 * @brief 默认的力矩/电流控制器
 * 
 * @param motor 
 * @return float 
 */
float rcs_motor_gm6020::ctrl_current(rcs_motor_gm6020 *motor)
{
    return motor->update_target;
}

/**
 * @brief 默认的速度控制器
 * 
 * @param motor 
 * @return float 
 */
float rcs_motor_gm6020::ctrl_speed(rcs_motor_gm6020 *motor)
{
    if (motor->speed_pid == nullptr) {
        rcs_kprintf("actor_gm6020: Motor(ID%hhd)on CAN%d speed pid is nullptr\n",portMAX_DELAY,motor->id,motor->can);
        return 0;
    }
    motor->target_current = motor->speed_pid->calculate(motor->update_target,motor->get_real_speed());
    return motor->target_current;
}

/**
 * @brief 默认的角度控制器
 * 
 * @param motor 
 * @return float 
 */
float rcs_motor_gm6020::ctrl_position(rcs_motor_gm6020 *motor)
{
    if (motor->angle_pid == nullptr || motor->speed_pid == nullptr) 
        return motor->update_target;
    motor->target_speed = motor->angle_pid->calculate(motor->update_target,motor->get_real_position());
    motor->target_current = motor->speed_pid->calculate(motor->target_speed,motor->get_real_speed());
    return motor->target_current;
}

/* 对象方法定义 ----------------------------------------------------------------------*/

/**
 * @brief 3508电机节点初始化函数
 * 
 * @param hcan 3508挂在哪一个HAL_CAN句柄上
 * @param RCS_CAN_x 3508挂在哪一个RCS_CAN句柄上
 * @param motor_id 电机的ID，即灯闪烁的次数
 * @param speed_pid 速度环PID，如果不使用可以填nullptr
 * @param angle_pid 角度环PID，如果不使用可以填nullptr
 * @return rcs_err_t 
 * 
 * @warning 请勿在init尚未执行前就调用其他的方法。受限于时间，没有对这种误操作进行保护和提醒。
 */
rcs_err_t rcs_motor_gm6020::init(CAN_HandleTypeDef *hcan,rcs_can_t RCS_CAN_x,uint8_t motor_id,sync_pid *speed_pid,sync_pid *angle_pid)
{
    rcs_kprintf("actor_gm6020: init gm6020(id%hhd) on CAN%hhd\n",portMAX_DELAY,motor_id,RCS_CAN_x+1);
    // 参数检查
    if (hcan == NULL || RCS_CAN_x>=RCS_CAN_COUNT) {
        rcs_kprintf("actor_gm6020: init failed, invalid param\n",portMAX_DELAY);
        return RCS_ERR_INVALID_PARAM;
    }
    // 初始化CAN总线
    if (rm_motor_init(hcan,RCS_CAN_x) != RCS_OK) {
        rcs_kprintf("actor_gm6020: init failed, can driver install failed\n",portMAX_DELAY);
        return RCS_FAIL;
    }

    // 将对象自己的成员进行初始化
    rcs_kprintf("actor_gm6020: step1--init self\n",portMAX_DELAY);
    this->speed_pid = speed_pid;
    this->angle_pid = angle_pid;
    this->target_current = 0;
    this->target_speed = 0;
    this->target_position = 0;
    this->target_user = 0;
    this->ctrl_mode = CTRL_CURRENT;
    this->usr_ctrl_fptr = nullptr;
    this->excute_counter = 0;
    this->can = RCS_CAN_x;
    this->id = motor_id;
    rcs_kprintf("actor_gm6020: step1--OK\n",portMAX_DELAY);

    // 电机链表新增一个成员，并获取电机链表下标
    rcs_kprintf("actor_gm6020: step2--update actorList\n",portMAX_DELAY);
    actor_list.push_back(this);
    this->actor_list_index = actor_list.size()-1;
    rcs_kprintf("actor_gm6020: step2--OK,be No%hhd of actorList\n",portMAX_DELAY,this->actor_list_index);

    // 获取该电机所述的执行报文类型
    rcs_kprintf("actor_gm6020: step3--update excuteList\n",portMAX_DELAY);
    frame_type excute_frame = (motor_id <= 4) ? FRAME_ONLY_FRONT : FRAME_ONLY_REAR;
    // 检查执行链表中是否右报文类型和CAN均一致的成员
    for(uint8_t i = 0; i < excute_list_type.size(); i++)
    {
        // 如果一致，则更新excute_list_motor中的下标，然后获取执行链表的下标
        if (excute_list_type[i] == excute_frame && excute_list_can[i] == RCS_CAN_x){
            if (motor_id <= 4)  
                excute_list_motor[i][motor_id-1] = this->actor_list_index;
            else
                excute_list_motor[i][motor_id-5] = this->actor_list_index;
            this->excute_list_index = i;
            rcs_kprintf("actor_gm6020: step3--early OK,find No%hhd in excuteList\n",portMAX_DELAY,this->excute_list_index);
            return RCS_OK;
        }
    }
    // 执行链表中不存在本电机的执行组，因此新增一个成员
    excute_list_type.push_back(excute_frame);
    excute_list_can.push_back(RCS_CAN_x);
    excute_list_motor.push_back({0, 0, 0, 0});
    // 更新excute_list_motor中的下标，然后获取执行链表的下标
    if (motor_id <= 4)
        excute_list_motor[excute_list_motor.size()-1][motor_id-1] = this->actor_list_index;
    else
        excute_list_motor[excute_list_motor.size()-1][motor_id-5] = this->actor_list_index;
    this->excute_list_index = excute_list_type.size()-1;
    rcs_kprintf("actor_gm6020: step3--OK,be No%hhd of excuteList\n",portMAX_DELAY,this->excute_list_index);

    // 成功
    rcs_kprintf("actor_gm6020: init All OK.\n",portMAX_DELAY);
    return RCS_OK;
}

/**
 * @brief 设置电机的控制模式
 * 
 * @param mode 详见 @ref: rcs_motor_gm6020::ctrl_type
 * @return rcs_err_t 
 */
rcs_err_t rcs_motor_gm6020::set_mode(ctrl_type mode)
{
    this->ctrl_mode = mode;
    switch(this->ctrl_mode)
    {
        case CTRL_CURRENT:
            this->usr_ctrl_fptr = &rcs_motor_gm6020::ctrl_current;
        break;
        case CTRL_SPEED:
            this->usr_ctrl_fptr = &rcs_motor_gm6020::ctrl_speed;
        break;
        case CTRL_POSITION:
            this->usr_ctrl_fptr = &rcs_motor_gm6020::ctrl_position;
        break;
        case CTRL_USER:
            this->usr_ctrl_fptr = nullptr;
            rcs_kprintf("actor_gm6020: set_mode_user failed, please set a function\n",portMAX_DELAY);
            return RCS_ERR_INVALID_PARAM;
        break;
    }
    return RCS_OK;
}

/**
 * @brief 设置电机的控制模式，并绑定用户自定义的控制函数
 * 
 * @param mode 详见 @ref: rcs_motor_gm6020::ctrl_type
 * @param function 详见 @ref: rcs_motor_gm6020::ctrl_function
 * @return rcs_err_t 
 */
rcs_err_t rcs_motor_gm6020::set_mode_user(ctrl_type mode,ctrl_function function)
{
    this->ctrl_mode = mode;
    this->usr_ctrl_fptr = function;
    return RCS_OK;
}

/**
 * @brief 更新目标值
 * 
 * @param target 目标值。具体更新的什么，取决于模式。
 * @return rcs_err_t 
 */
rcs_err_t rcs_motor_gm6020::update(float target)
{
    switch(this->ctrl_mode)
    {
        case CTRL_CURRENT:
            this->target_current = target;
        break;
        case CTRL_SPEED:
            this->target_speed = target;
        break;
        case CTRL_POSITION:
            this->target_position = target;
        break;
        case CTRL_USER:
            this->target_user = target;
        break;
    }
    this->excute_counter = RCS_MOTOR_GM6020_RESET_THRESHOLD;
    this->update_target = target;
    return RCS_OK;
}

/**
 * @brief 获取最近一次下发给电机的电流
 * 
 * @return float 
 */
float rcs_motor_gm6020::get_target_current()
{
    return this->target_current;
}

/**
 * @brief 获取最近一次下发给电机的速度
 * 
 * @return float 
 */
float rcs_motor_gm6020::get_target_speed()
{
    return this->target_speed;
}

/**
 * @brief 获取最近一次下发给电机的位置
 * 
 * @return float 
 */
float rcs_motor_gm6020::get_target_position()
{
    return this->target_position;
}

/**
 * @brief 获取最近一次下发给电机的用户自定义值
 * 
 * @return float 
 */
float rcs_motor_gm6020::get_target_user()
{
    return this->target_user;
}

/**
 * @brief 获取电机当前的实际速度
 * 
 * @return float 
 */
float rcs_motor_gm6020::get_real_speed()
{
    return rm_motor_get_speed_gm6020_rpm(this->can,this->id);
}

/**
 * @brief 获取电机当前的实际位置
 * 
 * @return float 
 */
float rcs_motor_gm6020::get_real_position()
{
    return rm_motor_get_angle_gm6020_deg(this->can,this->id);
}

/**
 * @brief 将电机的某个参数加入观测波形
 * 
 * @param type 要观测什么东西
 * @param channel 要加入哪一个波形通道
 * @return rcs_err_t 
 * 
 * @warning 受限于时间，这里没有检查通道是否被其他人占有，请自行确保没有其他执行器占有该通道
 */
rcs_err_t rcs_motor_gm6020::watch(watch_type type,uint8_t channel)
{
    monitor_list_channel.push_back(channel);
    monitor_list_type.push_back(type);
    monitor_list_motor.push_back(this->actor_list_index);
    return RCS_OK;
}

