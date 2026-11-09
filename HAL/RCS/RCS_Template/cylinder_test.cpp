/**
 * @file cylinder_test.c
 * @brief 执行器总线测试代码(以气缸+真空泵为例)
 * @author CYK-Dot
 * @date 2025-05-27
 * ------------------------------------------------------------------------------------------
 * @note 评估方式：
 * 
 * step1:硬件连接和外设配置
 * 1. 利用DAPLink接上单片机的SWD调试口和UART1串口。
 * 2. 在CubeMX中配置UART1外设的参数，并启用UART1的中断。
 * 3. 在keil的包管理器中启用EventRecorder插件，然后将Printf重定向到DAP。
 * 
 * step2:调用评估代码
 * 3. 在你需要的位置(例如app_main)调用cylinder_test_init()
 * 
 * step3:观察现象
 * 4. 进入keil调试，开启Debug(printf)Viewer。
 * 5. 运行代码，通过Debug(printf)Viewer查看日志输出，应能看到全部OK。
 * 6. 打开VOFA串口上位机，将数据模式更改为JustFloat，然后打开串口示波器，观察波形。
 * 
 * ------------------------------------------------------------------------------------------
 * @note 借鉴方式：
 * 1. rcs_kprintf用于简单的前期调试，比如检查代码是否正确初始化。
 * 2. 本代码提供了控制+监测的代码实现，如果不需要监测，可以不加。
 */

#include "cylinder_test.h"

#include "rcs_private_config.h"
#include "rcs_types.h"
#include "rcs_cylinder.h"
#include "rcs_actor_bus.h"
#include "rcs_uart.h"

#include "rcs_os.h"

extern UART_HandleTypeDef huart1;

static cylinder_no_sensor ball_loader;
static cylinder_no_sensor pump;
static rcs_uart_t monitor_uart;

/**
 * @brief 仅负责业务逻辑的状态机任务函数
 * @note 在执行器总线的加持下，业务逻辑完全和控制代码分离了，干净简洁，不易犯错！
**/
static void cylinder_test_task(void *pvParameters)
{
    static volatile uint8_t fsm = 0;
    while (1)
    {
        switch(fsm)
        {
			// 状态0：上弹气缸推，吸球气泵关
            case 0:
                ball_loader.update(cylinder_no_sensor::PUSH);
                pump.update(cylinder_no_sensor::PULL);
			
                if (ball_loader.get() == cylinder_no_sensor::PUSHED) {
					fsm = 1;
				}
            break;
				
			// 状态1：上弹气缸拉，吸球气泵开
            case 1:
                ball_loader.update(cylinder_no_sensor::PULL);
                pump.update(cylinder_no_sensor::PUSH);
			
                if (ball_loader.get() == cylinder_no_sensor::PULLED) {
					fsm = 0;
				}
            break;	
        }
        // actor_bus内部的任务已经负责了执行器的5ms控制，
		// 因此状态机的频率可以随意给，不再一定是5ms，完全看你。
		// 当然，不是越快越好，要考虑性能，只要响应得过来就行，比如20ms，几乎是一眨眼的事
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/**
 * @brief 使用执行器总线控制气缸的代码参考
 * @note 本函数对标upctrl_init
**/
void cylinder_test_init(void)
{
    rcs_kprintf_init();
    rcs_err_t err;
    
    /* 执行器控制初始化 ---------------------------------------------------------------*/
    rcs_kprintf("cylinder_test: actor-bus init controller.\n",portMAX_DELAY);

	// step1: 使用执行器自己的函数，初始化执行器
	ball_loader.init(GPIOC,GPIO_PIN_5,false,false,500);
	pump.init(GPIOB,GPIO_PIN_2,false,false,500);

    // step2: 将执行器按分类加入总线
    err = rcs_actor_bus::actor_add(&cylinder_no_sensor::excute,&cylinder_no_sensor::reset);
	if (err != RCS_OK) {
        rcs_kprintf("cylinder_test: failed to add cylinder to actor-bus,code=%d\n",portMAX_DELAY,err);
        return;
    }

    // step3: 启动执行器任务
    err = rcs_actor_bus::actor_start();
    if (err != RCS_OK) {
        rcs_kprintf("cylinder_test: failed to start ctrl service on actor-bus,code=%d\n",portMAX_DELAY,err);
        return;
    }

    /* 执行器观测初始化 ---------------------------------------------------------------*/
    rcs_kprintf("cylinder_test: actor-bus init monitor.\n",portMAX_DELAY);

    // step1: 初始化UART
    const rcs_uart_conf_t monitor_uart_conf = {
        .huartx = &huart1,
        .txMode = MODE_IT,
        .txBufferSize = 64,
        .txTimeout = pdMS_TO_TICKS(2),
        .rxMode = MODE_NONE,
    };
    if (rcs_uart_driver_install(&monitor_uart_conf,RCS_UART_NUM_1) != RCS_OK) {
        rcs_kprintf("cylinder_test: failed to create rcs_uart\n",portMAX_DELAY);
        return;
    }
	monitor_uart = RCS_UART_NUM_1;

    // step2: 初始化执行器总线的观测器
    err = rcs_actor_bus::monitor_setup(monitor_uart,4);
    if (err != RCS_OK) {
        rcs_kprintf("cylinder_test: failed to setup monitor service on actor-bus,code=%d\n",portMAX_DELAY,err);
        return;
    }

    // step3: 执行器向总线申请观测通道
    ball_loader.watch(2,3);//上弹气缸使用0和1通道
    pump.watch(0,1);       //吸球气泵使用2和3通道
    err = rcs_actor_bus::monitor_add(cylinder_no_sensor::monitor,0,3);//所有气缸类的执行器共占有0~4通道
    if (err != RCS_OK) {
        rcs_kprintf("cylinder_test: failed to register monitor to actor-bus,code=%d\n",portMAX_DELAY,err);
        return;
    }

    // step4: 启动执行器总线的观测任务
    err = rcs_actor_bus::monitor_start();
    if (err != RCS_OK) {
        rcs_kprintf("cylinder_test: failed to start monitor service on actor-bus,code=%d\n",portMAX_DELAY,err);
        return;
    }

    /* 业务逻辑初始化 ---------------------------------------------------------------*/
    rcs_kprintf("cylinder_test: init user functions.\n",portMAX_DELAY);

    // 创建一个任务，用于编写业务逻辑
    if (xTaskCreate(cylinder_test_task, "cylinder_test_task", 128, NULL, 5, NULL)!=pdPASS) {
        rcs_kprintf("cylinder_test: user task init failed.\n",portMAX_DELAY);
    }

    rcs_kprintf("cylinder_test: init done.\n",portMAX_DELAY);
	vTaskDelay(100);
}