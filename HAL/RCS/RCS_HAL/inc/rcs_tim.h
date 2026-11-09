/**
 * @file rcs_tim.h
 * @author 陈煜楷
 * @brief 实现面向对象的定时器接口(暂时只实现了编码器和PWM，基本定时功能请直接使用FreeRTOS软件定时器)
 * @version 0.1
 * @date 2025-06-04
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once

/* 头文件引入---------------------------------------------------------------*/

#include "rcs_private_config.h"
#include "rcs_types.h"

/* 配置宏定义---------------------------------------------------------------*/

#define RCS_TIM_MALLOC pvPortMalloc
#define RCS_TIM_FREE   vPortFree

#define RCS_TIM_COUNT 12

#define RCS_TIM_ENCODER_MAX 14
#define RCS_TIM_PWM_MAX 16

/* C++兼容支持--------------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

/* 导出类型定义--------------------------------------------------------------*/

typedef enum 
{
    RCS_TIM_NUM_1 = 0,
    RCS_TIM_NUM_2,
    RCS_TIM_NUM_3,
    RCS_TIM_NUM_4,
    RCS_TIM_NUM_5,
    RCS_TIM_NUM_6,
    RCS_TIM_NUM_7,
    RCS_TIM_NUM_8,
    RCS_TIM_NUM_9,
    RCS_TIM_NUM_10,
    RCS_TIM_NUM_11,
    RCS_TIM_NUM_12,
    RCS_TIM_NUM_13,
    RCS_TIM_NUM_14,
    RCS_TIM_NUM_15,
    RCS_TIM_NUM_16,
    RCS_TIM_NUM_17,
    RCS_TIM_NUM_18,
    RCS_TIM_NUM_19,
    RCS_TIM_NUM_20,
    RCS_TIM_NUM_21,
    RCS_TIM_NUM_22,
    RCS_TIM_NUM_23,
    RCS_TIM_NUM_24,
    RCS_TIM_NUM_25,
    RCS_TIM_NUM_26,
    RCS_TIM_NUM_MAX
}rcs_tim_t;

typedef struct 
{
    TIM_HandleTypeDef *htim;
    uint16_t callback_max_count;
}rcs_tim_capture_conf_t;

typedef void* rcs_tim_encoder_t;
typedef void* rcs_tim_pwm_t;

typedef void (*rcs_tim_callback_fptr)(TIM_TypeDef *TIMx,uint32_t TIM_CHANNEL_x);

/* 导出函数声明--------------------------------------------------------------*/

// TIM功能1：利用基本定时器实现us级别的计时
rcs_err_t rcs_tim_driver_install_stamper(TIM_HandleTypeDef *htim,rcs_tim_t RCS_TIM_NUM_x);
rcs_err_t rcs_tim_stamp_get(rcs_tim_t RCS_TIM_NUM_x);
rcs_err_t rcs_tim_stamp_reset(rcs_tim_t RCS_TIM_NUM_x);

// TIM功能2：利用输入捕获(IC)实现用户自定义功能
rcs_err_t rcs_tim_driver_install_capture(const rcs_tim_capture_conf_t *conf,rcs_tim_t RCS_TIM_NUM_x);
rcs_err_t rcs_tim_capture_register_callback(rcs_tim_t RCS_TIM_NUM_x,uint32_t TIM_CHANNEL_x,rcs_tim_callback_fptr callback);

// TIM功能3：利用输出比较(OC)实现PWM
rcs_err_t rcs_tim_driver_install_pwm(TIM_HandleTypeDef *htim,rcs_tim_t RCS_TIM_NUM_x);
rcs_tim_pwm_t rcs_tim_pwm_open(rcs_tim_t RCS_TIM_NUM_x,uint32_t TIM_CHANNEL_x);
void rcs_time_set_duty(rcs_tim_pwm_t pwm,float duty);

// TIM功能4：读取正交编码器
rcs_err_t rcs_tim_driver_install_encoder(TIM_HandleTypeDef *htim,rcs_tim_t RCS_TIM_NUM_x);
rcs_tim_encoder_t rcs_tim_encoder_open(rcs_tim_t RCS_TIM_NUM_x);
int32_t rcs_tim_get_encoder(rcs_tim_encoder_t encoder);

// 底层代码对接
void RCS_TIM_PeriodElapsedCallback_Proxy(TIM_HandleTypeDef *htim);

/* C++兼容支持--------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif