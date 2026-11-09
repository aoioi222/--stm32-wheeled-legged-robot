/**
 ******************************************************************************
 * @file    Fly_contrl.c
 * @brief   四轴旋翼飞行控制实现
 * @details 角度-角速度串级PID + X型混控 + 50Hz航模PWM输出
 ******************************************************************************
 */

#include "Fly_contrl.h"
#include "RCS_Pwm.h"
#include "I2C_ICM_42688_P_control.h"

/* ============================ SBUS摇杆原始范围 ============================ */

#define SBUS_MIN  192
#define SBUS_MID  992
#define SBUS_MAX  1792
#define SBUS_DEAD 40

/* ============================ 私有全局变量 ============================ */

/* 三个轴的角速度环PID */
static PID_Struct fly_rate_pid_roll;
static PID_Struct fly_rate_pid_pitch;
static PID_Struct fly_rate_pid_yaw;

/* 电机油门输出 0~1 */
static float motor_throttle[FLY_MOTOR_COUNT];

/* 解锁状态 */
static uint8_t fly_armed = 0;

/* D项一阶低通滤波状态 (每轴独立，存上一拍滤波输出) */
static float fly_dterm_roll_prev  = 0.0f;
static float fly_dterm_pitch_prev = 0.0f;
static float fly_dterm_yaw_prev   = 0.0f;

/* ============================ 私有函数声明 ============================ */

static float sbus_to_norm(uint16_t raw);
static float sbus_to_angle(uint16_t raw, float max_deg);
static float clamp_f(float val, float min, float max);
static void fly_pid_reset(void);
static void esc_write(uint8_t motor_idx, float throttle);
static float fly_rate_pid_filtered(float desired, float actual,
                                   PID_Struct *pid, float *dterm_prev, float dt);

/* ============================ 公开函数实现 ============================ */

/**
 * @brief  飞行控制初始化
 */
void Fly_Control_Init(void)
{
    /* 初始化4路ESC PWM输出 */
    PWMInit(FLY_ESC1_TIM, FLY_ESC1_CH, FLY_ESC1_PORT, FLY_ESC1_PIN,
            FLY_PWM_CLK_HZ, FLY_PWM_FREQ_HZ);
    PWMInit(FLY_ESC2_TIM, FLY_ESC2_CH, FLY_ESC2_PORT, FLY_ESC2_PIN,
            FLY_PWM_CLK_HZ, FLY_PWM_FREQ_HZ);
    PWMInit(FLY_ESC3_TIM, FLY_ESC3_CH, FLY_ESC3_PORT, FLY_ESC3_PIN,
            FLY_PWM_CLK_HZ, FLY_PWM_FREQ_HZ);
    PWMInit(FLY_ESC4_TIM, FLY_ESC4_CH, FLY_ESC4_PORT, FLY_ESC4_PIN,
            FLY_PWM_CLK_HZ, FLY_PWM_FREQ_HZ);

    /* 电调上电默认输出停止信号 */
    Fly_Motor_Stop();

    /* 初始化角速度环PID */
    fly_rate_pid_roll.P = FLY_RATE_KP_ROLL;
    fly_rate_pid_roll.I = FLY_RATE_KI_ROLL;
    fly_rate_pid_roll.D = FLY_RATE_KD_ROLL;
    fly_rate_pid_roll.Limit_Output = FLY_RATE_LIMIT_ROLL;
    fly_rate_pid_roll.Limit_Integral = FLY_RATE_INTLIM_ROLL;

    fly_rate_pid_pitch.P = FLY_RATE_KP_PITCH;
    fly_rate_pid_pitch.I = FLY_RATE_KI_PITCH;
    fly_rate_pid_pitch.D = FLY_RATE_KD_PITCH;
    fly_rate_pid_pitch.Limit_Output = FLY_RATE_LIMIT_PITCH;
    fly_rate_pid_pitch.Limit_Integral = FLY_RATE_INTLIM_PITCH;

    fly_rate_pid_yaw.P = FLY_RATE_KP_YAW;
    fly_rate_pid_yaw.I = FLY_RATE_KI_YAW;
    fly_rate_pid_yaw.D = FLY_RATE_KD_YAW;
    fly_rate_pid_yaw.Limit_Output = FLY_RATE_LIMIT_YAW;
    fly_rate_pid_yaw.Limit_Integral = FLY_RATE_INTLIM_YAW;

    fly_pid_reset();
    fly_armed = 0;
}

/**
 * @brief  飞行控制主更新函数
 * @param  state  全局机器人状态(含IMU姿态角 rad)
 * @param  ch     SBUS 16通道原始数据
 * @param  dt     控制周期 (s)
 */
void Fly_Control_Update(const RobotGlobalState_t *state,
                        const uint16_t *ch, float dt)
{
    if (!fly_armed)
        return;

    /* 获取实测姿态角 (rad → deg) 和角速度 (deg/s) */
    float actual_roll  = state->body_roll  * RAD2DEG;
    float actual_pitch = state->body_pitch * RAD2DEG;
    float actual_yaw   = state->body_yaw   * RAD2DEG;

    ICM42688P_Data_t *imu = ICM42688P_GetData();
    float actual_rate_roll  = imu->gyro[0];
    float actual_rate_pitch = imu->gyro[1];
    float actual_rate_yaw   = imu->gyro[2];

    /* SBUS摇杆 → 期望值 */
    float cmd_roll  = sbus_to_angle(ch[0], FLY_MAX_ANGLE_DEG);
    float cmd_pitch = sbus_to_angle(ch[1], FLY_MAX_ANGLE_DEG);
    float cmd_yaw_rate = sbus_to_angle(ch[3], FLY_MAX_YAW_RATE_DEG);
    float cmd_throttle = sbus_to_norm(ch[2]);

    /* 油门映射 0~1 → 最小~最大油门 */
    float throttle_base = FLY_MIN_THROTTLE +
                          cmd_throttle * (FLY_MAX_THROTTLE - FLY_MIN_THROTTLE);

    /* ================= 角度环 P (误差deg → 期望角速度deg/s) ================= */

    float angle_err_roll  = cmd_roll  - actual_roll;
    float angle_err_pitch = cmd_pitch - actual_pitch;
    /* 偏航: 期望角速度直接给定,不做角度闭环 */

    float desired_rate_roll  = FLY_ANGLE_KP_ROLL  * angle_err_roll;
    float desired_rate_pitch = FLY_ANGLE_KP_PITCH * angle_err_pitch;
    float desired_rate_yaw   = cmd_yaw_rate;

    /* 角速度限幅 */
    desired_rate_roll  = clamp_f(desired_rate_roll,  -FLY_MAX_RATE_DEG_S, FLY_MAX_RATE_DEG_S);
    desired_rate_pitch = clamp_f(desired_rate_pitch, -FLY_MAX_RATE_DEG_S, FLY_MAX_RATE_DEG_S);

    /* ================= 角速度环 PID ================= */

    float roll_out  = fly_rate_pid_filtered(desired_rate_roll,  actual_rate_roll,  &fly_rate_pid_roll,  &fly_dterm_roll_prev,  dt);
    float pitch_out = fly_rate_pid_filtered(desired_rate_pitch, actual_rate_pitch, &fly_rate_pid_pitch, &fly_dterm_pitch_prev, dt);
    float yaw_out   = fly_rate_pid_filtered(desired_rate_yaw,   actual_rate_yaw,   &fly_rate_pid_yaw,   &fly_dterm_yaw_prev,   dt);

    /* ================= X型混控 ================= */
    /*
     * M1(右前 CW)  = T + R + P - Y
     * M2(左前 CCW) = T - R + P + Y
     * M3(右后 CCW) = T + R - P + Y
     * M4(左后 CW)  = T - R - P - Y
     */

    float m[FLY_MOTOR_COUNT];
    m[FLY_MOTOR_FR] = throttle_base + roll_out + pitch_out - yaw_out;
    m[FLY_MOTOR_FL] = throttle_base - roll_out + pitch_out + yaw_out;
    m[FLY_MOTOR_RR] = throttle_base + roll_out - pitch_out + yaw_out;
    m[FLY_MOTOR_RL] = throttle_base - roll_out - pitch_out - yaw_out;

    /* 油门限幅 */
    for (int i = 0; i < FLY_MOTOR_COUNT; i++)
    {
        m[i] = clamp_f(m[i], FLY_MIN_THROTTLE, FLY_MAX_THROTTLE);
        motor_throttle[i] = m[i];
        esc_write(i, m[i]);
    }
}

/**
 * @brief  停机 所有电机PWM输出置零
 */
void Fly_Motor_Stop(void)
{
    fly_armed = 0;
    fly_pid_reset();

    for (int i = 0; i < FLY_MOTOR_COUNT; i++)
    {
        motor_throttle[i] = 0.0f;
        esc_write(i, 0.0f);
    }
}

/**
 * @brief  解锁 电机输出怠速PWM
 */
void Fly_Motor_Arm(void)
{
    fly_pid_reset();

    for (int i = 0; i < FLY_MOTOR_COUNT; i++)
    {
        motor_throttle[i] = FLY_PWM_DUTY_IDLE;
        esc_write(i, FLY_PWM_DUTY_IDLE);
    }

    fly_armed = 1;
}

/**
 * @brief  加锁 电机停转
 */
void Fly_Motor_Disarm(void)
{
    Fly_Motor_Stop();
}

/**
 * @brief  获取当前电机输出值
 * @param  throttle  输出数组[4] 每路0~1
 */
void Fly_Get_Motor_Output(float throttle[FLY_MOTOR_COUNT])
{
    for (int i = 0; i < FLY_MOTOR_COUNT; i++)
        throttle[i] = motor_throttle[i];
}

/* ============================ 私有函数实现 ============================ */

/**
 * @brief  SBUS原始值 → 归一化 0~1 (含死区)
 */
static float sbus_to_norm(uint16_t raw)
{
    if (raw >= (SBUS_MID - SBUS_DEAD) && raw <= (SBUS_MID + SBUS_DEAD))
        return 0.0f;

    if (raw > SBUS_MID)
    {
        float t = (float)(raw - SBUS_MID - SBUS_DEAD)
                / (float)(SBUS_MAX - SBUS_MID - SBUS_DEAD);
        return t;
    }
    else
    {
        float t = (float)(SBUS_MID - SBUS_DEAD - raw)
                / (float)(SBUS_MID - SBUS_DEAD - SBUS_MIN);
        return -t;
    }
}

/**
 * @brief  SBUS原始值 → 期望角度 deg (含死区)
 */
static float sbus_to_angle(uint16_t raw, float max_deg)
{
    return sbus_to_norm(raw) * max_deg;
}

/**
 * @brief  浮点数限幅
 */
static float clamp_f(float val, float min, float max)
{
    if (val > max) return max;
    if (val < min) return min;
    return val;
}

/**
 * @brief  带D项一阶低通滤波的角速度环PID
 * @note   逻辑与 PID_Normal_Ctrl 一致(位置式: P*err + I*积分 + D项)，
 *         唯一区别：D项(误差差分)先乘 D 增益，再经一阶低通滤波后进入输出，
 *         抑制陀螺仪噪声在微分项中的放大。滤波截止频率 FLY_RATE_DTERM_LPF_HZ。
 * @param  desired     期望角速度 (deg/s)
 * @param  actual      实测角速度 (deg/s)
 * @param  pid         PID 参数与状态 (P/I/D、积分、上次误差、限幅)
 * @param  dterm_prev  D项滤波上一拍输出 (每轴独立，由调用方维护)
 * @param  dt          控制周期 (s)
 * @return float       控制器输出
 */
static float fly_rate_pid_filtered(float desired, float actual,
                                   PID_Struct *pid, float *dterm_prev, float dt)
{
    float err = desired - actual;

    /* 积分 + 抗饱和 (与 PID_Normal_Ctrl 相同) */
    pid->Integral += err;
    if (pid->Integral >= pid->Limit_Integral)
        pid->Integral = pid->Limit_Integral;
    else if (pid->Integral <= -pid->Limit_Integral)
        pid->Integral = -pid->Limit_Integral;

    /* D项 = D * 误差差分，再做一阶低通滤波: y = y_prev + α*(x - y_prev) */
    float d_raw = pid->D * (err - pid->Last_Error);
    float rc = 1.0f / (2.0f * pi * FLY_RATE_DTERM_LPF_HZ);
    float alpha = dt / (dt + rc);
    float d_filt = *dterm_prev + alpha * (d_raw - *dterm_prev);
    *dterm_prev = d_filt;

    float output = pid->P * err + pid->I * pid->Integral + d_filt;

    /* 输出限幅 (与 PID_Normal_Ctrl 相同) */
    if (output >= pid->Limit_Output)
        output = pid->Limit_Output;
    else if (output <= -pid->Limit_Output)
        output = -pid->Limit_Output;

    pid->Last_Error = err;
    return output;
}

/**
 * @brief  复位所有PID积分和上次误差
 */
static void fly_pid_reset(void)
{
    fly_rate_pid_roll.Integral    = 0.0f;
    fly_rate_pid_roll.Last_Error  = 0.0f;
    fly_rate_pid_pitch.Integral   = 0.0f;
    fly_rate_pid_pitch.Last_Error = 0.0f;
    fly_rate_pid_yaw.Integral     = 0.0f;
    fly_rate_pid_yaw.Last_Error   = 0.0f;

    /* D项滤波状态一并清零，避免解锁瞬间用旧值 */
    fly_dterm_roll_prev  = 0.0f;
    fly_dterm_pitch_prev = 0.0f;
    fly_dterm_yaw_prev   = 0.0f;
}

/**
 * @brief  单路ESC PWM输出
 * @param  motor_idx  电机编号 0-3
 * @param  throttle   油门值 0~1, 内部转换为5%~10%占空比(1ms~2ms)
 */
static void esc_write(uint8_t motor_idx, float throttle)
{
    /* throttle=0 → 停机(0%), throttle>0 → 映射到5%~10% */
    float duty;
    if (throttle <= 0.001f)
        duty = FLY_PWM_DUTY_STOP;
    else
        duty = FLY_PWM_DUTY_MIN + throttle * (FLY_PWM_DUTY_MAX - FLY_PWM_DUTY_MIN);

    if (duty > 1.0f)  duty = 1.0f;
    if (duty < 0.0f)  duty = 0.0f;

    switch (motor_idx)
    {
    case FLY_MOTOR_FR:
        PWMOutput(FLY_ESC1_TIM, FLY_ESC1_CH, duty);
        break;
    case FLY_MOTOR_FL:
        PWMOutput(FLY_ESC2_TIM, FLY_ESC2_CH, duty);
        break;
    case FLY_MOTOR_RR:
        PWMOutput(FLY_ESC3_TIM, FLY_ESC3_CH, duty);
        break;
    case FLY_MOTOR_RL:
        PWMOutput(FLY_ESC4_TIM, FLY_ESC4_CH, duty);
        break;
    default:
        break;
    }
}

/* ============================ 以下代码仅在编译环境缺少clamp_f时启用 ============================ */
#ifdef FLY_SELF_TEST
#include <stdio.h>
void Fly_SelfTest(void)
{
    printf("Fly_contrl self-test: PID structs initialized, PWM pins configured.\n");
    printf("  ESC1: TIM2_CH3 (PB10) M1 FR\n");
    printf("  ESC2: TIM12_CH1 (PB15) M2 FL\n");
    printf("  ESC3: TIM2_CH4 (PB11) M3 RR\n");
    printf("  ESC4: TIM12_CH2 (PB14) M4 RL\n");
}
#endif
