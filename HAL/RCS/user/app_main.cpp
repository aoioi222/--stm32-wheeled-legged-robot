#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include <EventRecorder.h>

#include "main.h"

#include "rmmotor_test.h"
#include "rm_motor_raw.h"
#include "freertos_kprintf_test.h"
#include "can_test.h"
#include "cylinder_test.h"
#include "upctrl_test.h"

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"

extern "C" void app_main(void *argument)
{
	upctrl_test_init();

    // 将自己永远挂起，以供其他任务占有CPU
    vTaskDelay(pdMS_TO_TICKS(portMAX_DELAY));
}
