/**
 * @file rcs_tim.c
 * @author 陈煜楷
 * @brief 实现面向对象的定时器接口(暂时只实现了编码器和PWM，基本定时功能请直接使用FreeRTOS软件定时器)
 * @version 0.1
 * @date 2025-06-04
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "rcs_tim.h"
#include "rcs_os.h"

/* 私有类型定义 ----------------------------------------------------------------------------------*/

typedef struct {
    TIM_HandleTypeDef *htim;
    int32_t cycle;
} rcs_tim_encoder_impl_t;

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
} rcs_tim_pwm_impl_t;

/* 全局变量定义 ----------------------------------------------------------------------------------*/

// 编码器类的私有成员
static rcs_tim_encoder_impl_t *g_encoder[RCS_TIM_ENCODER_MAX];
static uint8_t g_encoder_count = 0;

// PWM类的私有成员
static rcs_tim_pwm_impl_t *g_pwm[RCS_TIM_PWM_MAX];
static uint8_t g_pwm_count = 0;

/* 导出函数定义 ----------------------------------------------------------------------------------*/

/**
 * @brief 创建编码器对象，使用TIM的编码器模式读取正交编码器值
 * 
 * @param htim 哪一路HAL_TIM对象，请务必使用CubeMX生成的HAL_TIM对象，如&htim1，不要传入自己定义的TIM_HandleTypeDef变量
 * @return rcs_tim_encoder_t 
 * 
 * @note 如果之前有创建过相同htim的编码器对象，则返回之前创建的对象，不会重新创建
 */
rcs_tim_encoder_t rcs_tim_create_encoder(TIM_HandleTypeDef *htim) 
{
    rcs_kprintf("rcs_tim: create encoder on %X\n",portMAX_DELAY,htim->Instance);
    // 检查此前是否已经创建过该定时器的编码器对象
	for(uint8_t i=0;i<g_encoder_count;i++) {
		if (g_encoder[i]->htim == htim) {
            rcs_kprintf("rcs_tim: early OK,already created before\n",portMAX_DELAY);
            return g_encoder[i];
        }
	}
	// 创建编码器对象
    rcs_tim_encoder_impl_t *encoder = RCS_TIM_MALLOC(sizeof(rcs_tim_encoder_impl_t));
    if (!encoder) return NULL;
    encoder->htim = htim;
    encoder->cycle = 0;
	
    // 编码器链表新增
	g_encoder[g_encoder_count] = encoder;
	g_encoder_count++;

    HAL_TIM_Encoder_Start_IT(htim, TIM_CHANNEL_ALL);
    rcs_kprintf("rcs_tim: OK\n",portMAX_DELAY);
    return (rcs_tim_encoder_t)encoder;
}

/**
 * @brief 获取编码器当前值
 * 
 * @param handle 
 * @return int32_t 
 */
int32_t rcs_tim_get_encoder(rcs_tim_encoder_t handle) 
{
    rcs_tim_encoder_impl_t *encoder = (rcs_tim_encoder_impl_t *)handle;
    return encoder->cycle*0xffff+__HAL_TIM_GetCounter(encoder->htim);
}

/**
 * @brief 创建PWM对象
 * 
 * @param htim 哪一路HAL_TIM对象，请务必使用CubeMX生成的HAL_TIM对象，不要自己随便定义一个TIM_HandleTypeDef变量
 * @param channel 定时器的哪一路通道，比如 TIM_CHANNEL_1
 * @return rcs_tim_pwm_t 
 * 
 * @note 如果之前有创建过相同htim和channel的PWM对象，则返回之前创建的对象，不会重新创建
 */
rcs_tim_pwm_t rcs_tim_create_pwm(TIM_HandleTypeDef *htim, uint32_t channel) 
{
	for(uint8_t i=0;i<g_pwm_count;i++) {
		if (g_pwm[i]->htim == htim) return g_pwm[i];
	}
	
    rcs_tim_pwm_impl_t *pwm = RCS_TIM_MALLOC(sizeof(rcs_tim_pwm_impl_t));
    if (!pwm) return NULL;

    pwm->htim = htim;
    pwm->channel = channel;
	
	g_pwm[g_pwm_count] = pwm;
	g_pwm_count++;

    HAL_TIM_PWM_Start(htim, channel);
    return (rcs_tim_pwm_t)pwm;
}

/**
 * @brief 设置PWM占空比
 * 
 * @param handle 
 * @param duty 
 */
void rcs_time_set_duty(rcs_tim_pwm_t handle, float duty) 
{
    rcs_tim_pwm_impl_t *pwm = (rcs_tim_pwm_impl_t *)handle;

    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;

    uint32_t period = pwm->htim->Init.Period;
    __HAL_TIM_SET_COMPARE(pwm->htim, pwm->channel, (uint32_t)(duty * period));
}

/**
 * @brief HAL库定时器回调函数代理
 * 
 * @param htim 
 * 
 * @warning CubeMX把HAL_TIM_PeriodElapsedCallback直接定义在main.c里了，导致我们无法重新定义
 *          因此需要各位手动把此函数放到main.c的HAL_TIM_PeriodElapsedCallback函数里调用
 *          如果使用的是F407模板代码，已经把这件事做好了
 */
void RCS_TIM_PeriodElapsedCallback_Proxy(TIM_HandleTypeDef *htim)
{
	for(uint8_t i=0;i<g_encoder_count;i++) {
		if (g_encoder[i]->htim == htim) {
			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
				g_encoder[i]->cycle--;
			else
				g_encoder[i]->cycle++;
		}
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

}


