#include "rcs_soft_encoder.h"
#include <stdlib.h>
#include <string.h>

typedef enum {
    RCS_ENCODER_MODE_EXTI,
    RCS_ENCODER_MODE_TIM
} rcs_encoder_mode_t;

typedef struct {
    rcs_encoder_mode_t mode;
    int32_t count;
    
    union {
        struct {
            GPIO_TypeDef *A_GPIOx;
            uint16_t A_GPIO_Pin;
            GPIO_TypeDef *B_GPIOx;
            uint16_t B_GPIO_Pin;
        } exti;

        struct {
            TIM_HandleTypeDef *htim;
            uint32_t channel;
            uint32_t last_capture;
            GPIO_TypeDef *B_GPIOx;
            uint16_t B_GPIO_Pin;
        } tim;
    } config;
} rcs_soft_encoder_internal_t;


static rcs_soft_encoder_internal_t g_encoder_list[14];

static void encoder_exti_callback(rcs_soft_encoder_internal_t *enc)
{
    GPIO_PinState b = HAL_GPIO_ReadPin(enc->config.exti.B_GPIOx, enc->config.exti.B_GPIO_Pin);
    enc->count += (b == GPIO_PIN_SET) ? 1 : -1;
}

static void encoder_tim_callback(rcs_soft_encoder_internal_t *enc, uint32_t capture)
{
    GPIO_PinState b = HAL_GPIO_ReadPin(enc->config.tim.B_GPIOx, enc->config.tim.B_GPIO_Pin);
    enc->count += (b == GPIO_PIN_SET) ? 1 : -1;
    enc->config.tim.last_capture = capture;
}

rcs_soft_encoder_t rcs_soft_encoder_create_exti(const rcs_soft_encoder_exti_conf_t *conf)
{
    rcs_soft_encoder_internal_t *enc = malloc(sizeof(rcs_soft_encoder_internal_t));
    if (!enc) return NULL;
    memset(enc, 0, sizeof(*enc));
    
    enc->mode = RCS_ENCODER_MODE_EXTI;
    enc->config.exti.A_GPIOx = conf->A_GPIOx;
    enc->config.exti.A_GPIO_Pin = conf->A_GPIO_Pin;
    enc->config.exti.B_GPIOx = conf->B_GPIOx;
    enc->config.exti.B_GPIO_Pin = conf->B_GPIO_Pin;

    // EXTI 配置应由用户完成，EXTI ISR 中需调用 encoder_exti_callback()

    return (rcs_soft_encoder_t)enc;
}

rcs_soft_encoder_t rcs_soft_encoder_create_tim(const rcs_soft_encoder_tim_conf_t *conf)
{
    rcs_soft_encoder_internal_t *enc = malloc(sizeof(rcs_soft_encoder_internal_t));
    if (!enc) return NULL;
    memset(enc, 0, sizeof(*enc));

    enc->mode = RCS_ENCODER_MODE_TIM;
    enc->config.tim.htim = (TIM_HandleTypeDef *)conf->A_TIMx;
    enc->config.tim.channel = conf->A_TIM_Channel;
    enc->config.tim.B_GPIOx = conf->B_GPIOx;
    enc->config.tim.B_GPIO_Pin = conf->B_GPIO_Pin;

    // TIM IC 启动应由用户完成，回调中需调用 encoder_tim_callback()

    return (rcs_soft_encoder_t)enc;
}

int32_t rcs_soft_encoder_get(rcs_soft_encoder_t encoder)
{
    return ((rcs_soft_encoder_internal_t *)encoder)->count;
}

void rcs_soft_encoder_reset(rcs_soft_encoder_t encoder)
{
    ((rcs_soft_encoder_internal_t *)encoder)->count = 0;
}

// 用户需手动在 EXTI 中断中调用该函数：
void rcs_soft_encoder_exti_irq(rcs_soft_encoder_t encoder)
{
    if (!encoder) return;
    rcs_soft_encoder_internal_t *enc = (rcs_soft_encoder_internal_t *)encoder;
    if (enc->mode == RCS_ENCODER_MODE_EXTI)
        encoder_exti_callback(enc);
}

// 用户需手动在 TIM IC 回调中调用该函数：
void rcs_soft_encoder_tim_irq(rcs_soft_encoder_t encoder, uint32_t capture)
{
    if (!encoder) return;
    rcs_soft_encoder_internal_t *enc = (rcs_soft_encoder_internal_t *)encoder;
    if (enc->mode == RCS_ENCODER_MODE_TIM)
        encoder_tim_callback(enc, capture);
}
