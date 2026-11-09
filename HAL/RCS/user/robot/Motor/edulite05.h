#ifndef EDULITE05_H
#define EDULITE05_H

#include <stdbool.h>
#include "stm32f4xx_hal.h"

#define EDULITE05_MAX_MOTORS 12u
#define EDULITE05_POSITION_MAX_RAD 12.57f

typedef struct {
    uint8_t host_id;
    float position_kp;
    float position_kd;
    float max_position_step_rad;
} EduLite05Config_t;

/* 初始化一组电机并独占所选 CAN 的收发中断。ID 数组和配置会被复制。 */
bool EduLite05_Init(CAN_HandleTypeDef *can, const uint8_t ids[], uint8_t count,
                    const EduLite05Config_t *config);

/* 按初始化时的 ID 顺序周期下发电机侧目标角度。返回 true 表示全部已就绪。 */
bool EduLite05_PositionControl(const float position_rad[]);

/* 停止期间周期调用，直到后台发送完成。下一次位置控制可重新启动。 */
void EduLite05_Stop(void);

#endif
