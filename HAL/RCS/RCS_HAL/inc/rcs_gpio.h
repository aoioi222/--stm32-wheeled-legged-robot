#pragma once

#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "rcs_private_config.h"
#include "rcs_types.h"

#ifdef __cplusplus
	extern "C" {
#endif

void rcs_gpio_set(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void rcs_gpio_reset(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void rcs_gpio_write(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin,bool state);
void rcs_gpio_toggle(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
uint8_t rcs_gpio_read(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void rcs_gpio_output_init_legacy(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

#ifdef __cplusplus
	}
#endif
