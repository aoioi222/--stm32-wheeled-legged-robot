#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* 头文件依赖 -------------------------------------------------------------------------------------*/

#include "rcs_private_config.h"
#include "rcs_types.h"

#include "rcs_can.h"

/* 配置宏定义 --------------------------------------------------------------------------------------*/


/* 导出函数声明 -----------------------------------------------------------------------------------*/

// 让CAN总线支持VESC电调
rcs_err_t vesc_motor_init(const rcs_can_conf_t *conf,rcs_can_t *handle);

// VESC电调操作
rcs_err_t vesc_motor_send_current(rcs_can_t handle,uint16_t id,int32_t current,TickType_t timeout);
rcs_err_t vesc_motor_send_rpm(rcs_can_t handle,uint16_t id,int32_t erpm,TickType_t timeout);
rcs_err_t vesc_motor_send_pos(rcs_can_t handle,uint16_t id,float pos,TickType_t timeout);

#ifdef __cplusplus
}
#endif