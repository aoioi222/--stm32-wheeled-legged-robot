#pragma once

#include "rcs_private_config.h"
#include "rcs_types.h"

#include <vector>

class soft_encoder 
{
    public:
        rcs_err_t init(TIM_HandleTypeDef *A_htimx,uint32_t A_channel,GPIO_TypeDef *B_GPIOx,uint16_t B_GPIO_Pin);
        rcs_err_t init(GPIO_TypeDef *A_GPIOx,uint16_t A_GPIO_Pin,GPIO_TypeDef *B_GPIOx,uint16_t B_GPIO_Pin);
        int32_t get_count(void);
        void reset_count(void);
    private:
        static void soft_encoder_exti_callback(GPIO_TypeDef *GPIOx, uint32_t pin);
        static void soft_encoder_tim_ic_callback(TIM_HandleTypeDef *htim);
};

