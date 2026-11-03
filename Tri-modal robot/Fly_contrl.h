/**
 ******************************************************************************
 * @file    Fly_contrl.h
 * @brief   四轴旋翼飞行控制模块
 * @details 角度-角速度串级PID + X型混控 + 50Hz航模PWM输出
 *          控制链路: SBUS摇杆→期望姿态→角度环P→期望角速度→角速度环PID→
 *                   混控矩阵→4路PWM→ESC→无刷电机
 ******************************************************************************
 */

#ifndef FLY_CONTRL_H
#define FLY_CONTRL_H

#include "stm32f4xx.h"
#include "rcs.h"
#include "robot_state.h"
#include "RCS_PIDctrl.h"

/* ============================ 电机编号(X型布局) ============================ */

enum {
    FLY_MOTOR_FR = 0,  /* 右前 CW  */
    FLY_MOTOR_FL = 1,  /* 左前 CCW */
    FLY_MOTOR_RR = 2,  /* 右后 CCW */
    FLY_MOTOR_RL = 3,  /* 左后 CW  */
    FLY_MOTOR_COUNT = 4
};

/* ============================ ESC PWM引脚映射 ============================ */

#define FLY_ESC1_TIM        TIM2
#define FLY_ESC1_CH         3
#define FLY_ESC1_PORT       GPIOB
#define FLY_ESC1_PIN        GPIO_Pin_10

#define FLY_ESC2_TIM        TIM12
#define FLY_ESC2_CH         1
#define FLY_ESC2_PORT       GPIOB
#define FLY_ESC2_PIN        GPIO_Pin_15

#define FLY_ESC3_TIM        TIM2
#define FLY_ESC3_CH         4
#define FLY_ESC3_PORT       GPIOB
#define FLY_ESC3_PIN        GPIO_Pin_11

#define FLY_ESC4_TIM        TIM12
#define FLY_ESC4_CH         2
#define FLY_ESC4_PORT       GPIOB
#define FLY_ESC4_PIN        GPIO_Pin_14

/* ============================ PWM参数 ============================ */

#define FLY_PWM_FREQ_HZ     50       /* 航模ESC标准50Hz             */
#define FLY_PWM_CLK_HZ      1000000  /* 定时器计数频率1MHz          */
#define FLY_PWM_ARR          20000   /* ARR = 1MHz / 50Hz           */
#define FLY_PWM_DUTY_MIN    0.05f   /* 1ms = 5%  (最小油门)        */
#define FLY_PWM_DUTY_MAX    0.10f   /* 2ms = 10% (最大油门)        */
#define FLY_PWM_DUTY_IDLE   0.05f   /* 怠速 = 最小油门              */
#define FLY_PWM_DUTY_STOP   0.00f   /* 停转                         */

/* ============================ 飞行PID参数(预留调参) ============================ */

/* 角度环 P (deg → target_rate_deg/s) */
#define FLY_ANGLE_KP_ROLL    5.0f
#define FLY_ANGLE_KP_PITCH   5.0f
#define FLY_ANGLE_KP_YAW     3.0f

/* 角速度环 PID */
#define FLY_RATE_KP_ROLL     0.15f
#define FLY_RATE_KI_ROLL     0.02f
#define FLY_RATE_KD_ROLL     0.01f
#define FLY_RATE_LIMIT_ROLL  100.0f
#define FLY_RATE_INTLIM_ROLL 50.0f

#define FLY_RATE_KP_PITCH    0.15f
#define FLY_RATE_KI_PITCH    0.02f
#define FLY_RATE_KD_PITCH    0.01f
#define FLY_RATE_LIMIT_PITCH 100.0f
#define FLY_RATE_INTLIM_PITCH 50.0f

#define FLY_RATE_KP_YAW      0.20f
#define FLY_RATE_KI_YAW      0.05f
#define FLY_RATE_KD_YAW      0.01f
#define FLY_RATE_LIMIT_YAW   100.0f
#define FLY_RATE_INTLIM_YAW  50.0f

/* 角速度环 D项低通滤波 (抑制陀螺仪噪声在微分项中的放大) */
#define FLY_RATE_DTERM_LPF_HZ 50.0f  /* D项截止频率 (Hz)，越小越平滑、相位滞后越大 */

/* ============================ 飞行限制 ============================ */

#define FLY_MAX_ANGLE_DEG     10.0f  /* 最大允许倾角 平飞平移不过10° */
#define FLY_MAX_RATE_DEG_S    150.0f /* 最大角速度限制 (deg/s)      */
#define FLY_MAX_YAW_RATE_DEG  60.0f  /* 偏航最大角速度 (deg/s)      */
#define FLY_MAX_THROTTLE      0.90f  /* 油门上限 (0~1)              */
#define FLY_MIN_THROTTLE      0.05f  /* 油门下限 (0~1，即怠速)      */
#define FLY_HOVER_THROTTLE    0.55f  /* 悬停油门估计值(预留)        */

/* ============================ 机械参数(预留) ============================ */

#define FLY_ARM_LENGTH_M      0.30f  /* 轴臂长度 电机到重心 (m)     */
#define FLY_MASS_KG           7.5f   /* 起飞重量 5~10kg取中值       */
#define FLY_MOTOR_KV          500    /* 电机KV值 (RPM/V) 6S+大桨    */
#define FLY_BAT_VOLTAGE       22.2f  /* 电池电压 (6S LiPo)          */

/* ============================ 公开接口 ============================ */

/**
 * @brief  飞行控制初始化
 * @note   初始化4路ESC PWM输出、PID控制器，配置电调停机状态
 */
void Fly_Control_Init(void);

/**
 * @brief  飞行控制主更新函数
 * @param  state  全局机器人状态(含IMU姿态角)
 * @param  ch     指向SBUS 16通道原始数据的指针
 * @param  dt     控制周期 (s)
 * @note   每个控制周期调用一次，完成姿态控制→混控→PWM输出
 */
void Fly_Control_Update(const RobotGlobalState_t *state,
                        const uint16_t *ch, float dt);

/**
 * @brief  停机 所有电机PWM输出置零
 */
void Fly_Motor_Stop(void);

/**
 * @brief  解锁 电机输出怠速PWM
 */
void Fly_Motor_Arm(void);

/**
 * @brief  加锁 电机停转
 */
void Fly_Motor_Disarm(void);

/**
 * @brief  获取当前电机输出值
 * @param  throttle  输出数组[4] 每路0~1
 */
void Fly_Get_Motor_Output(float throttle[FLY_MOTOR_COUNT]);

#endif /* FLY_CONTRL_H */
