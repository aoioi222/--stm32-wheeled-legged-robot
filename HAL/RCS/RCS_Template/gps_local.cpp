/**
 * @brief 在STM32F407上利用TIM外设读取正交编码轮，部署全场定位算法，不利用STM32G431中转
 * @author 陈煜楷
 *
 * @note 使用方法
 * 1. 找一套现成的全场定位模块，将XY轴的正交编码器分别接到主控板的定时器接口上。
 *    注意，一个TIM只能接一个编码器，一个正交编码器需要2个通道。因此需要两路TIM，每路TIM需要提供2路通道，共四路。
 * 2. 在CubeMX上配置TIM外设，让其工作在编码器模式，并开启TIM中断。
 * 3. 在CubeMX上配置UART外设，波特率取决于你把陀螺仪配成了什么，然后开启UART中断。
 * 4. 在app_main中调用gps_local_test_init函数。编译、烧录。
 * 5. 进入keil调试，打开Debug(printf)Viewer
 * 6. 让程序全速运行，观察控制台输出，如果输出正常，则说明全场定位算法已经部署成功。
**/
#include <FreeRTOS.h>
#include <task.h>

#include "gps.h"
#include "rcs_tim.h"
#include "rcs_gyro.h"
#include "rcs_os.h"

#include "gps_local.h"

// CubeMX生成的外设对象
extern TIM_HandleTypeDef htim1; // 如果linking报错Undefined symbol，那就是你在CubeMX中没有配置tim1，导致htim1没有生成
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;

// 正交编码器对象
rcs_tim_encoder_t encoder_x;
rcs_tim_encoder_t encoder_y;

// 陀螺仪对象
rcs_jy901 jy901;

// GPS对象
gps local_gps(0.68288f,0.68288f,2.35619449019f,2.35619449019f,0.0f,3600.0f,5);

/**
 * @brief GPS接口函数：获取全场定位坐标系下的X轴里程
 */
static double gps_local_get_encoder_x(void)
{
	return rcs_tim_get_encoder(encoder_x);
}
/**
 * @brief GPS接口函数：获取全场定位坐标系下的Y轴里程
 */
static double gps_local_get_encoder_y(void)
{
	return rcs_tim_get_encoder(encoder_y);
}
/**
 * @brief GPS接口函数：获取全场定位坐标系下的Yaw轴里程
 */
static double gps_local_get_gyro_z(void)
{
	return jy901.get_yaw_deg()*DEG2RAD;
}
/**
 * @brief 测试任务，向keil调试控制台打印xyz坐标
 * 
 * @param args 
 */
static void gps_test_printf_task(void *args)
{
	while(1) {
		rcs_kprintf("x:%f y:%f z:%f\n",portMAX_DELAY,local_gps.get_pos_x(),local_gps.get_pos_y(),local_gps.get_pos_z());
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}
/**
 * @brief 调用此函数，启动全场定位算法测试，此函数会在keil调试的控制台持续输出xyz坐标
 * 
 */
void gps_local_test_init(void)
{
	// step1:初始化编码器
	//encoder_x = rcs_tim_create_encoder(&htim1);
	//encoder_y = rcs_tim_create_encoder(&htim2);
	// step2:初始化陀螺仪
	jy901.init(&huart1,RCS_UART_NUM_1);
	// step3:初始化GPS任务
    local_gps.init(&gps_local_get_encoder_x, &gps_local_get_encoder_y,&gps_local_get_gyro_z);
	// 可选:初始化打印任务
    xTaskCreate(gps_test_printf_task,"gps_test_printf_task",512,nullptr,1,nullptr);
}