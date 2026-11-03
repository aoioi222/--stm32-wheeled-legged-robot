/**
 * @filename: Wheel_mode_contrl.c
 * @brief:    轮足机器人轮式模式底层电机控制
 * @details:  四轮差速驱动，上位机解算轮速后下发，STM32做PID速度环控制
 *            CAN1挂载4个RM3508电机，ID: 1=RF, 2=LF, 3=RB, 4=LB
 * @note:     控制流参考仿真 my_controller.c 中的 wheel_motors_update()
 *            轮式相关参数全部取自 robot_config.h 的“十二、轮式模式参数”
 */

#include "Wheel_mode_contrl.h"
#include "RCS_PIDctrl.h"

/* CAN ID 查找表，按 wheel[RF,LF,RB,LB] 顺序 */
static const uint8_t wheel_can_id[WHEEL_COUNT] = {
    WHEEL_CAN_ID_RF,
    WHEEL_CAN_ID_LF,
    WHEEL_CAN_ID_RB,
    WHEEL_CAN_ID_LB
};

static PID_Struct wheel_pid[WHEEL_COUNT];
static float current_rpm[WHEEL_COUNT];

/**
 * @name: Wheel_Mode_Init
 * @brief: 初始化4个轮子电机的PID控制器
 */
void Wheel_Mode_Init(void)
{
    uint8_t i;
    for (i = 0; i < WHEEL_COUNT; i++) {
        wheel_pid[i] = PID_Get_RM2006_Speed_Pid();
        current_rpm[i] = 0.0f;
    }
}

/**
 * @name: Wheel_Mode_Control
 * @brief: 轮式模式速度控制主函数，每个控制周期调用一次
 * @param: wheel_speed[4]  四个轮子的目标角速度 (rad/s)，顺序: RF, LF, RB, LB
 * @note:  rad/s → rpm 换算: rpm = rad/s * 60 / (2*PI)
 */
void Wheel_Mode_Control(const float wheel_speed[WHEEL_COUNT])
{
    uint8_t i;
    int16_t target_rpm[WHEEL_COUNT];
    int16_t output_current[WHEEL_COUNT];

    /* 1. 读取当前转速 (rpm) */
    for (i = 0; i < WHEEL_COUNT; i++) {
        current_rpm[i] = (float)RM3508_Get_Speed_Rpm(CAN1, wheel_can_id[i]);
    }

    /* 2. 目标转速: rad/s → rpm，限幅 */
    for (i = 0; i < WHEEL_COUNT; i++) {
        float rpm = wheel_speed[i] * 9.5493f;
        if (rpm > WHEEL_MAX_RPM)  rpm = WHEEL_MAX_RPM;
        if (rpm < -WHEEL_MAX_RPM) rpm = -WHEEL_MAX_RPM;
        target_rpm[i] = (int16_t)rpm;
    }

    /* 3. PID速度环计算 */
    for (i = 0; i < WHEEL_COUNT; i++) {
        float out = PID_Normal_Ctrl((float)target_rpm[i], current_rpm[i], &wheel_pid[i]);
        if (out > 20000.0f)  out = 20000.0f;
        if (out < -20000.0f) out = -20000.0f;
        output_current[i] = (int16_t)out;
    }

    /* 4. 写入CAN发送缓冲区并执行 */
    RM3508_Update_Current(CAN1, WHEEL_CAN_ID_RF, output_current[0]);
    RM3508_Update_Current(CAN1, WHEEL_CAN_ID_LF, output_current[1]);
    RM3508_Update_Current(CAN1, WHEEL_CAN_ID_RB, output_current[2]);
    RM3508_Update_Current(CAN1, WHEEL_CAN_ID_LB, output_current[3]);
    RM3508_Excute_Front_Current(CAN1);
}

/**
 * @name: Wheel_Mode_Stop
 * @brief: 紧急停止，所有轮子电流清零并复位PID积分
 */
void Wheel_Mode_Stop(void)
{
    uint8_t i;

    RM3508_Update_Current(CAN1, WHEEL_CAN_ID_RF, 0);
    RM3508_Update_Current(CAN1, WHEEL_CAN_ID_LF, 0);
    RM3508_Update_Current(CAN1, WHEEL_CAN_ID_RB, 0);
    RM3508_Update_Current(CAN1, WHEEL_CAN_ID_LB, 0);
    RM3508_Excute_Front_Current(CAN1);

    for (i = 0; i < WHEEL_COUNT; i++) {
        wheel_pid[i].Integral = 0.0f;
        wheel_pid[i].Last_Error = 0.0f;
        current_rpm[i] = 0.0f;
    }
}
