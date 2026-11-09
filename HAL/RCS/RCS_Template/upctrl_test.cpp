/**
 * @file upctrl_test.cpp
 * @author 陈煜楷
 * @brief upctrl示范代码，展示如何完成机器人的上层机构控制和上层逻辑控制
 * @version 0.1
 * @date 2025-06-04
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "rcs_actor_bus.h"
#include "rcs_motor_rm3508.h"
#include "rcs_cylinder.h"
#include "rm_motor_raw.h"

#include "sync_pid.h"
#include "rcs_can.h"
#include "rcs_uart.h"
#include "rcs_os.h"

#include <FreeRTOS.h>
#include <task.h>

#include "upctrl_test.h"

// CubeMX生成的HAL对象
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern UART_HandleTypeDef huart1;

// 外设对象
static rcs_can_t g_fiction_can = RCS_CAN_NUM_MAX; // 全局变量需要初始化，避免和别人冲突
static rcs_uart_t g_monitor_uart = RCS_UART_NUM_MAX;

// PID对象
static sync_pid fictionL_spd_pid(RCS_ASYN_PID_DEFAULT_RM3508_SPPED_PID); 
static sync_pid fictionR_spd_pid(RCS_ASYN_PID_DEFAULT_RM3508_SPPED_PID);
static sync_pid picker_spd_pid(RCS_ASYN_PID_DEFAULT_RM3508_SPPED_PID); 
static sync_pid craw_spd_pid(RCS_ASYN_PID_DEFAULT_RM2006_SPPED_PID); 
static sync_pid picker_pos_pid(20.0f,0.0f,0.0f,1000.0f,1000.0f,0.0f ,0.0f ,0.0f);

// 执行器对象
static rcs_motor_rm3508 fictionL; // 左摩擦轮电机(3508)
static rcs_motor_rm3508 fictionR; // 右摩擦轮电机(3508)
static rcs_motor_rm3508 picker;   // 抓手大臂电机(3508)
static rcs_motor_rm3508 craw;     // 抓手电机(2006)
static cylinder_no_sensor pusher; // 上弹气缸
static cylinder_no_sensor pump;   // 抓手吸盘

// 传感器对象
//(还没实现，等待补充....)

// 静态函数
uint32_t upctrl(float spd_fiction,float pos_picker,float spd_craw,bool is_load,bool is_suck);

// 事件定义
#define UPCTRL_FICTION_ARRIVE (1 << 0)
#define UPCTRL_PICKER_ARRIVE  (1 << 1)
#define UPCTRL_LOADER_ARRIVE  (1 << 2)


/**
 * @brief 编写业务逻辑的任务
 * 
 * @param args 未使用
 *
 * @note 这里为了演示，把业务逻辑任务和upctrl代码放到了一起，这其实不是好习惯
 *       建议在自己编写代码时，将uptrl_init和upctrl放到upctrl.cpp，upctrl_task放到upctrl_fsm.cpp
 *       upctrl.cpp专注于控制机器人的机构，upctrl_fsm.cpp专注于控制机器人的自动/半自动运行逻辑
 */
static void upctrl_test_task(void *args)
{
	static uint8_t state = 0;
	static uint32_t retval;
    while(1) {	
		switch(state) {
			// 吸球
			case 0:
				retval = upctrl(0.0f,0.0f,0.0f,false,true);
				if (rcs_count_delay(0,50)) state = 1;
			break;
			
			// 抓球
			case 1:
				retval = upctrl(0.0f,0.0f,1000.0f,false,true);
				if (rcs_count_delay(0,50)) state = 2;
			break;
			
			// 举球
			case 2:
				retval = upctrl(0.0f,90.0f*19.0f*3.0f,1000.0f,false,true);//假设外部齿轮减速比3，而19是3508的减速比
				if (retval & UPCTRL_PICKER_ARRIVE) state = 3;
			break;
			
			// 松球
			case 3:
				retval = upctrl(5000.0f,90.0f*19.0f*3.0f,-500.0f,false,false);
				if (rcs_count_delay(0,80)) state = 4;
			break;
			
			// 射球+大臂复位
			case 4:
				retval = upctrl(5000.0f,0.0f,-500.0f,false,false);
				if ((retval & UPCTRL_FICTION_ARRIVE) && (retval & UPCTRL_PICKER_ARRIVE)) state = 0;
			break;
		}
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/**
 * @brief 初始化观测通道
 * 
 * @param huartx 
 * @param RCS_UART_NUM_x 
 *
 * @note 对标老代码的ble_init
 */
void upctrl_test_watch_init(UART_HandleTypeDef *huartx,rcs_uart_t RCS_UART_NUM_x)
{
    // ----------初始化串口，用于波形输出-----------------------
	
    const rcs_uart_conf_t monitor_uart_conf = {
        .huartx = huartx,
        .txMode = MODE_IT,
        .txBufferSize = 64,
        .txTimeout = pdMS_TO_TICKS(2),
        .rxMode = MODE_NONE,
    };
    rcs_uart_driver_install(&monitor_uart_conf,RCS_UART_NUM_x);
    g_monitor_uart = RCS_UART_NUM_x;

    // ----------初始化执行器总线的观测器------------------------
	
    rcs_actor_bus::monitor_setup(g_monitor_uart,3);

    // ----------向执行器总线申请观测器通道----------------------
	
    fictionL.watch(rcs_motor_rm3508::WATCH_SPEED,0);
    fictionL.watch(rcs_motor_rm3508::WATCH_SPEED_TARGET,1);
    fictionL.watch(rcs_motor_rm3508::WATCH_CURRENT_TARGET,2);
    rcs_actor_bus::monitor_add(&fictionL.monitor,0,2);

    // ------------启动执行器总线的观测器------------------------
	
    rcs_actor_bus::monitor_start();
}

/**
 * @brief 完整初始化上层机构
 * 
 * @param hcanx 
 * @param RCS_CAN_NUM_x 
 * @param GPIOx 
 * @param GPIO_Pin_x 
 *
 * @note 对标老代码的upctrl_init
 */
void upctrl_test_init(void)
{
    // -------------初始化keil::EventRecorder-----------------------------
	
    rcs_kprintf_init();

    // ----------------初始化各个执行器节点-------------------------------
	
    fictionL.init(
        &hcan1,            // 电机所在的HAL_CAN句柄
        RCS_CAN_NUM_1,     // 电机所在的CAN线标号
        1,                 // 电机ID
        &fictionL_spd_pid, // 电机速度环PID
        nullptr            // 电机角度环PID
    );
    fictionR.init(
        &hcan1,            // 电机所在的HAL_CAN句柄
        RCS_CAN_NUM_1,     // 电机所在的CAN线标号
        2,                 // 电机ID
        &fictionR_spd_pid, // 电机速度环PID
        nullptr            // 电机角度环PID
    );
	picker.init(
        &hcan1,            // 电机所在的HAL_CAN句柄
        RCS_CAN_NUM_1,     // 电机所在的CAN线标号
        5,                 // 电机ID
        &picker_spd_pid,   // 电机速度环PID
        &picker_pos_pid    // 电机角度环PID
    );
	craw.init(
        &hcan1,            // 电机所在的HAL_CAN句柄
        RCS_CAN_NUM_1,     // 电机所在的CAN线标号
        6,                 // 电机ID
        &craw_spd_pid,     // 电机速度环PID
        nullptr            // 电机角度环PID
    );
		
    pusher.init(
        GPIOC,             // 气缸所在的GPIO端口
        GPIO_PIN_3,        // 气缸所在的GPIO引脚
        false,             // 高电平是否为推
        false,             // 复位状态是否为拉
        pdMS_TO_TICKS(500) // 大概要多久的时间才能完成一次动作
    );
	pump.init(
        GPIOB,             // 真空泵所在的GPIO端口
        GPIO_PIN_2,        // 真空泵所在的GPIO引脚
        false,             // 真空泵高电平是否为推
        false,             // 真空泵复位状态是否为拉
        pdMS_TO_TICKS(1200)// 真空泵大概要多久的时间才能完成一次动作
    );

    // -----------将各个执行器节点加入总线----------------------------
		
    rcs_actor_bus::actor_add(&fictionL.excute,&fictionL.reset);
    rcs_actor_bus::actor_add(&fictionR.excute,&fictionR.reset);
    rcs_actor_bus::actor_add(&pusher.excute,&pusher.reset);
	
	// -----创建业务逻辑任务，用于更新各个执行器的输出-----------------
		
    xTaskCreate(
		upctrl_test_task,
		"upctrl_test_task",
		512,
		NULL,
		6,
		NULL
	);
		
    // ------------启动执行器总线(必须放在最后)-------------------------
	
    rcs_actor_bus::actor_start();

    // ------------------配置观测器(可选)-------------------------------
	
    upctrl_test_watch_init(&huart1,RCS_UART_NUM_1);
}

/**
 * @brief upctrl函数，一个函数控制所有执行机构
 *
 * @param spd_fiction 摩擦轮转速
 * @param pos_picker  大臂角度
 * @param spd_craw    爪子速度
 * @param is_load     true代表气缸推出，可以把球推到摩擦轮上发射
 * @param is_suck     true代表真空泵启动，可以把球先吸到爪子上
 *
 * @retval int 是否完成
 *
**/
uint32_t upctrl(float spd_fiction,float pos_picker,float spd_craw,bool is_load,bool is_suck)
{
	// 更新电机模式
	fictionL.set_mode(rcs_motor_rm3508::CTRL_SPEED);
	fictionR.set_mode(rcs_motor_rm3508::CTRL_SPEED);
	picker.set_mode(rcs_motor_rm3508::CTRL_POSITION);
	craw.set_mode(rcs_motor_rm3508::CTRL_SPEED);
	
	// 下发电机指令
	fictionL.update(spd_fiction);
	fictionR.update(-spd_fiction);
	picker.update(pos_picker);
	craw.update(spd_craw);
	
	// 下发气缸指令
	if (is_load)
		pusher.update(cylinder_no_sensor::PUSH);
	else 
		pusher.update(cylinder_no_sensor::PULL);
	if (is_suck)
		pump.update(cylinder_no_sensor::PUSH);
	else 
		pump.update(cylinder_no_sensor::PULL);
	
	// 判断动作是否完成
	uint32_t retval = 0;
	if (fabsf(fictionL.get_real_speed()-spd_fiction) < 10.0f)
		retval |= UPCTRL_FICTION_ARRIVE;
	
	if (fabsf(picker.get_real_position()-pos_picker) < NEAR_ZERO)
		retval |= UPCTRL_PICKER_ARRIVE;
	
	if (pusher.get()==cylinder_no_sensor::PUSHED && is_load) 
		retval |= UPCTRL_LOADER_ARRIVE;
	
	if (pusher.get()==cylinder_no_sensor::PULLED && !is_load) 
		retval |= UPCTRL_LOADER_ARRIVE;
	
	return retval;
}
