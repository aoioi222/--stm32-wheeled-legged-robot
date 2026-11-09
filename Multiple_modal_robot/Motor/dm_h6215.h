#ifndef DM_H6215_H
#define DM_H6215_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx.h"

#define DM_H6215_MAX_MOTORS 12u

typedef struct {
    uint16_t motor_id;
    uint16_t feedback_id;
} DM_H6215Address_t;

typedef struct {
    uint16_t service_id_offset;
    float speed_limit_rad_s;
} DM_H6215Config_t;

/* 初始化一组电机。CAN 需已用 RCS 的发送缓冲初始化，地址数组和配置会被复制。 */
bool DM_H6215_Init(CAN_TypeDef *can, const DM_H6215Address_t addresses[], uint8_t count,
                   const DM_H6215Config_t *config);

/* 电机须设为速度模式并开启通信超时。目标转速按地址表顺序传入，单位 rad/s。 */
void DM_H6215_VelocityControl(const float speed_rad_s[]);

/* 停止期间周期调用，直到收到失能反馈。下一次速度控制可重新启动。 */
void DM_H6215_Stop(void);

#endif
